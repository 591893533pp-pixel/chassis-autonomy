#!/usr/bin/env python3
"""Bounded MID-360S startup recovery. No ROS, raw sockets, CAN writes or mode changes."""
import argparse
import binascii
import contextlib
import fcntl
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import time
import zlib

ROOT = Path(__file__).resolve().parents[1]
FAULTS = {0x02160003, 0x02100003}
KEYS = (0x8000, 0x8006, 0x8008, 0x8011)
CONFLICTS = {'livox_ros_driver2_node', 'fastlio_mapping', 'fusion_node.py',
             'run_mapping.py', 'run_static.py', 'run_reposition.py', 'remote_session.py',
             'run_remote.py', 'guarded_remote.py', 'can_bridge', 'robstride_can_bridge',
             'remote_bridge', 'controller_server'}


def packet(command, sequence, payload, kind=0, sender=0):
    header = struct.pack('<BBHIHBB6s', 0xaa, 0, 24 + len(payload), sequence,
                         command, kind, sender, bytes(6))
    return header + struct.pack('<HI', binascii.crc_hqx(header, 0xffff),
                                zlib.crc32(payload)) + payload


def unpack(data, command, sequence):
    if len(data) < 24 or data[:2] != b'\xaa\x00':
        raise ValueError('invalid SDK header')
    length, seq, cmd, kind, sender = struct.unpack_from('<HIHBB', data, 2)
    if (length, seq, cmd, kind, sender) != (len(data), sequence, command, 1, 1):
        raise ValueError('unmatched SDK response')
    crc16, crc32 = struct.unpack_from('<HI', data, 18)
    if crc16 != binascii.crc_hqx(data[:18], 0xffff) or crc32 != zlib.crc32(data[24:]):
        raise ValueError('SDK CRC mismatch')
    return data[24:]


def diagnostic(payload):
    # SDK2 LivoxLidarDiagInternalInfoResponse: uint8 ret, uint16 count, TLVs.
    if len(payload) < 3 or payload[0] != 0:
        raise ValueError('diagnostic query rejected/truncated')
    count = struct.unpack_from('<H', payload, 1)[0]
    values = {}
    offset = 3
    for _ in range(count):
        if offset + 4 > len(payload):
            raise ValueError('truncated TLV')
        key, size = struct.unpack_from('<HH', payload, offset)
        offset += 4
        if key in values or offset + size > len(payload):
            raise ValueError('duplicate/truncated TLV')
        values[key] = payload[offset:offset + size]
        offset += size
    if offset != len(payload) or not set(KEYS) <= values.keys():
        raise ValueError('incomplete diagnostic response')
    if any(len(values[k]) != size for k, size in ((0x8006, 1), (0x8008, 4), (0x8011, 32))):
        raise ValueError('invalid diagnostic field length')
    sn = values[0x8000].rstrip(b'\0').decode('ascii')
    if not sn or len(sn) > 16:
        raise ValueError('invalid serial')
    return {'serial': sn, 'work_state': values[0x8006][0],
            'powerup_count': struct.unpack('<I', values[0x8008])[0],
            'hms': [x for x in struct.unpack('<8I', values[0x8011]) if x]}


class Transport:
    def __init__(self, sock):
        self.sock = sock
        self.sequence = secrets.randbelow(65000)

    def request(self, ip, command, payload):
        self.sequence = (self.sequence + 1) & 0xffff
        seq = self.sequence
        self.sock.sendto(packet(command, seq, payload), (ip, 56100))
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            self.sock.settimeout(max(.001, end - time.monotonic()))
            data, source = self.sock.recvfrom(65535)
            if source != (ip, 56100):
                continue
            try:
                return unpack(data, command, seq)
            except ValueError:
                continue
        raise TimeoutError('no matching ACK')

    def query(self, ip):
        return diagnostic(self.request(ip, 0x0101, struct.pack('<HH4H', 4, 0, *KEYS)))

    def reboot(self, ip):
        # SDK CommandImpl::LivoxLidarReboot: uint16 timeout=100. Never resend on ACK loss.
        ack = self.request(ip, 0x0200, struct.pack('<H', 100))
        if ack != b'\x00':
            raise ValueError('reboot rejected: ' + ack.hex())


def atomic_json(path, data):
    temp = path.with_suffix('.tmp')
    with temp.open('w') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class Budget:
    """Consume before send, persist across service restart; only a new host boot resets it."""
    def __init__(self, path, boot):
        self.path = path
        if path.exists():
            saved = json.loads(path.read_text())  # Corruption fails closed, never resets silently.
            if (saved.get('schema') != 1 or not isinstance(saved.get('attempts'), dict)
                    or not isinstance(saved.get('boot_id'), str) or not saved['boot_id']):
                raise ValueError('invalid recovery budget')
            self.data = saved if saved.get('boot_id') == boot else {'schema': 1, 'boot_id': boot, 'attempts': {}}
        else:
            self.data = {'schema': 1, 'boot_id': boot, 'attempts': {}}
        atomic_json(self.path, self.data)

    def used(self, serial):
        return serial in self.data['attempts']

    def consume(self, serial, before):
        if self.used(serial):
            raise RuntimeError('automatic attempt budget exhausted')
        self.data['attempts'][serial] = {'utc': time.time(), 'before': before, 'result': 'reserved_before_send'}
        atomic_json(self.path, self.data)

    def finish(self, serial, result):
        self.data['attempts'][serial]['result'] = result
        atomic_json(self.path, self.data)


class Debounce:
    def __init__(self):
        self.since = {}

    def reset(self):
        self.since.clear()

    def observe(self, device, row, now):
        sn = device['serial']
        # Require BOTH known scan errors; any additional Error/Fatal prevents this workaround.
        errors = {x for x in row['hms'] if (x & 0xff) >= 3}
        match = (row['serial'] == sn and row['work_state'] == 4 and errors == FAULTS)
        if not match:
            self.since.pop(sn, None)
            return False
        old = self.since.get(sn)
        if old is None or now - old[1] > 10 or old[2] != row['powerup_count']:
            old = (now, now, row['powerup_count'])
        self.since[sn] = (old[0], now, old[2])
        return now - old[0] >= 30


def blockers(config):
    """Conservative guard: UP CAN is blocked even if a controller claims to be idle."""
    interface = Path('/sys/class/net') / config['interface']
    try:
        if (interface / 'carrier').read_text().strip() != '1':
            return 'WAIT_LINK'
    except OSError:
        return 'WAIT_LINK'
    for entry in Path('/sys/class/net').glob('can*'):
        if int((entry / 'flags').read_text().strip(), 16) & 1:
            return 'BLOCKED_CAN_UP:' + entry.name
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            names = {Path(x.decode(errors='replace')).name for x in (proc / 'cmdline').read_bytes().split(b'\0') if x}
        except FileNotFoundError:
            continue
        except PermissionError:
            return 'BLOCKED_UNREADABLE_PROCESS'
        if names & CONFLICTS:
            return 'BLOCKED_ACTIVE_PROCESS:' + proc.name
    return None


@contextlib.contextmanager
def ownership(config):
    # The same locks used by mapping/calibration and by the real motor control entry.
    with contextlib.ExitStack() as stack:
        for path in (Path('/home/pc/lidar test/logs/driver.lock'), Path(f'/tmp/poke-remote-{os.getuid()}.lock')):
            path.parent.mkdir(parents=True, exist_ok=True)
            f = stack.enter_context(path.open('a'))
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sockets = []
        for port in (56101, 56201, 56301, 56401):
            sock = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            sock.bind((config['host_ip'], port))  # No SO_REUSEADDR; never share SDK ownership.
            sockets.append(sock)
        yield Transport(sockets[0])


class Guard:
    def __init__(self, config, directory):
        self.config = config
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        self.budget = Budget(directory / 'budget.json', Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        self.debounce = Debounce()
        self.last_status = None

    def event(self, kind, **fields):
        row = {'utc': time.time(), 'monotonic': time.monotonic(), 'event': kind, **fields}
        with (self.directory / 'events.jsonl').open('a') as f:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')
            f.flush()
            os.fsync(f.fileno())
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def status(self, phase, **fields):
        row = {'utc': time.time(), 'phase': phase, **fields}
        atomic_json(self.directory / 'status.json', row)
        if phase != self.last_status:
            self.event(phase, **fields)
            self.last_status = phase

    def verify(self, transport, device, before):
        # ACK is not proof of recovery: require a changed boot counter AND 10s SAMPLING.
        end = time.monotonic() + 90
        stable = None
        last = None
        while time.monotonic() < end:
            block = blockers(self.config)
            if block:
                self.event('VERIFY_INTERRUPTED', serial=device['serial'], reason=block)
                return False
            try:
                last = transport.query(device['ip'])
                good = (last['serial'] == device['serial'] and last['work_state'] == 1
                        and last['powerup_count'] == (before['powerup_count'] + 1) % (2**32)
                        and not any((x & 0xff) >= 3 for x in last['hms']))
                stable = (stable if stable is not None else time.monotonic()) if good else None
                if stable is not None and time.monotonic() - stable >= 10:
                    self.event('RECOVERED_SAMPLING', serial=device['serial'], after=last)
                    return True
            except (OSError, ValueError):
                stable = None
            time.sleep(2)
        self.event('RECOVERY_UNCONFIRMED', serial=device['serial'], after=last)
        return False

    def cycle(self):
        block = blockers(self.config)
        if block:
            self.debounce.reset()
            self.status(block)
            return
        rows = {}
        try:
            with ownership(self.config) as transport:
                for device in self.config['devices']:
                    # A controller could have started after the previous process scan.
                    block = blockers(self.config)
                    if block:
                        self.debounce.reset()
                        self.status(block)
                        return
                    try:
                        row = transport.query(device['ip'])
                    except (OSError, ValueError) as error:
                        self.debounce.since.pop(device['serial'], None)
                        rows[device['name']] = {'query_error': str(error)}
                        continue
                    rows[device['name']] = row
                    eligible = self.debounce.observe(device, row, time.monotonic())
                    if row['serial'] != device['serial']:
                        rows[device['name']]['blocked'] = 'IDENTITY_MISMATCH'
                        continue
                    if self.budget.used(device['serial']):
                        rows[device['name']]['automatic_attempt_used'] = True
                    if not eligible or self.budget.used(device['serial']):
                        continue
                    block = blockers(self.config)
                    if block:
                        self.debounce.reset()
                        self.status(block)
                        return
                    self.event('REBOOT_EVIDENCE', device=device, before=row)
                    self.budget.consume(device['serial'], row)
                    self.status('REBOOTING', device=device)
                    try:
                        transport.reboot(device['ip'])  # Exactly one send; ACK loss never retries.
                        self.event('REBOOT_ACK_OK', serial=device['serial'])
                    except (OSError, ValueError) as error:
                        self.event('REBOOT_ACK_UNCONFIRMED', serial=device['serial'], error=str(error))
                    recovered = self.verify(transport, device, row)
                    self.budget.finish(device['serial'], 'recovered_sampling' if recovered else 'unconfirmed_no_retry')
                    self.debounce.reset()
                    # Return before the next device: its full fault debounce starts afresh.
                    self.status('RECOVERED' if recovered else 'RECOVERY_FAILED_NO_RETRY', device=device)
                    return
            self.status('OBSERVING', devices=rows)
        except OSError as error:
            self.debounce.reset()
            self.status('WAIT_OWNERSHIP_OR_ADDRESS', error=str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'mapping_trial/config/devices.json')
    parser.add_argument('--state-dir', type=Path, default=Path.home() / '.local/state/chassis-lidar-boot-guard')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if len(config['devices']) != 2 or len({d['serial'] for d in config['devices']}) != 2:
        raise ValueError('exactly two distinct configured sensors required')
    args.state_dir.mkdir(parents=True, exist_ok=True)
    with (args.state_dir / 'owner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        guard = Guard(config, args.state_dir)
        while True:
            guard.cycle()
            time.sleep(5)


if __name__ == '__main__':
    main()
