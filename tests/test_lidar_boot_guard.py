"""No hardware: fault histories, crash budgets, transport corruption and serialized recovery."""
import contextlib
import json
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import patch

from recovery import lidar_boot_guard as guard

DEVICES = [{'name': 'rear', 'ip': '192.0.2.1', 'serial': 'TEST_REAR'},
           {'name': 'front', 'ip': '192.0.2.2', 'serial': 'TEST_FRONT'}]


def row(device=DEVICES[0], state=4, codes=None, count=7):
    return {'serial': device['serial'], 'work_state': state, 'powerup_count': count,
            'hms': sorted(guard.FAULTS) if codes is None else codes}


class Clock:
    now = 0.

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeTransport:
    def __init__(self, recover=True, ack_lost=False):
        self.sent = []
        self.recover = recover
        self.ack_lost = ack_lost

    def query(self, ip):
        d = next(d for d in DEVICES if d['ip'] == ip)
        if ip in self.sent and self.recover:
            return row(d, 1, [0x04050002], 8)
        return row(d)

    def reboot(self, ip):
        self.sent.append(ip)
        if self.ack_lost:
            raise socket.timeout('lost ACK')


class TriggerTests(unittest.TestCase):
    def test_sustained_fault_required(self):
        d = guard.Debounce()
        for t in range(0, 30, 5):
            self.assertFalse(d.observe(DEVICES[0], row(), t))
        self.assertTrue(d.observe(DEVICES[0], row(), 30))

    def test_ptp_missing_single_code_other_fault_and_wrong_identity_do_not_trigger(self):
        samples = [row(state=1), row(codes=[0x04050002]), row(codes=[0x02160003]),
                   row(codes=[*guard.FAULTS, 0x01160003]), row(codes=[]),
                   {**row(), 'serial': 'WRONG'}]
        for sample in samples:
            d = guard.Debounce()
            for t in range(0, 100, 5):
                self.assertFalse(d.observe(DEVICES[0], sample, t))

    def test_recovery_data_gap_and_power_cycle_reset_debounce(self):
        for interruption in ('good', 'gap', 'counter'):
            d = guard.Debounce()
            for t in range(0, 30, 5):
                d.observe(DEVICES[0], row(), t)
            if interruption == 'good':
                d.observe(DEVICES[0], row(state=1, codes=[]), 27)
                self.assertFalse(d.observe(DEVICES[0], row(), 30))
            elif interruption == 'gap':
                self.assertFalse(d.observe(DEVICES[0], row(), 40))
            else:
                self.assertFalse(d.observe(DEVICES[0], row(count=8), 30))


class BudgetTests(unittest.TestCase):
    def test_reserved_attempt_survives_crash_and_new_boot_resets(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'budget.json'
            a = guard.Budget(path, 'boot1')
            a.consume('TEST_REAR', row())
            b = guard.Budget(path, 'boot1')
            self.assertTrue(b.used('TEST_REAR'))
            with self.assertRaises(RuntimeError):
                b.consume('TEST_REAR', row())
            self.assertFalse(guard.Budget(path, 'boot2').used('TEST_REAR'))

    def test_corrupt_state_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'budget.json'
            for content in ('{', '{}', '{"schema": 1, "attempts": []}', '{"schema": 1, "attempts": {}}'):
                path.write_text(content)
                with self.assertRaises(ValueError):
                    guard.Budget(path, 'boot1')


class ProtocolTests(unittest.TestCase):
    def test_reboot_wire_layout_and_crc_corruption(self):
        request = guard.packet(0x0200, 42, struct.pack('<H', 100))
        self.assertEqual(request[:18].hex(), 'aa001a002a00000000020000000000000000')
        self.assertEqual(request[24:], b'\x64\x00')
        reply = guard.packet(0x0200, 42, b'\x00', kind=1, sender=1)
        self.assertEqual(guard.unpack(reply, 0x0200, 42), b'\x00')
        for index in (0, 2, 4, 8, 10, 11, 18, 20, 24):
            bad = bytearray(reply)
            bad[index] ^= 1
            with self.assertRaises(ValueError):
                guard.unpack(bytes(bad), 0x0200, 42)

    def test_query_response_tlv_validation(self):
        values = ((0x8000, b'TEST_REAR\0'), (0x8006, b'\x04'),
                  (0x8008, struct.pack('<I', 7)),
                  (0x8011, struct.pack('<8I', *sorted(guard.FAULTS), *([0]*6))))
        data = b'\0' + struct.pack('<H', len(values))
        data += b''.join(struct.pack('<HH', k, len(v)) + v for k, v in values)
        self.assertEqual(guard.diagnostic(data), row())
        for bad in (data[:-1], data + b'\0', b'\1' + data[1:], b'\0\0\0'):
            with self.assertRaises(ValueError):
                guard.diagnostic(bad)

    def test_transport_ignores_foreign_reply_and_sends_once(self):
        class Sock:
            sent = []

            def sendto(self, data, dest):
                self.sent.append((data, dest))

            def settimeout(self, seconds):
                pass

            def recvfrom(self, size):
                return replies.pop(0)
        sock = Sock()
        transport = guard.Transport(sock)
        transport.sequence = 20
        valid = guard.packet(0x0200, 21, b'\0', 1, 1)
        replies = [(valid, ('192.0.2.2', 56100)),
                   (guard.packet(0x0200, 20, b'\0', 1, 1), ('192.0.2.1', 56100)),
                   (valid, ('192.0.2.1', 56100))]
        transport.reboot('192.0.2.1')
        self.assertEqual(len(sock.sent), 1)
        self.assertEqual(sock.sent[0][1], ('192.0.2.1', 56100))


class CycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = Clock()
        self.clock.now = 0.
        self.transport = FakeTransport()
        self.instance = guard.Guard({'devices': DEVICES}, Path(self.temp.name))
        for patcher in (patch.object(guard.time, 'monotonic', self.clock.time),
                        patch.object(guard.time, 'sleep', self.clock.sleep),
                        patch.object(guard, 'blockers', return_value=None),
                        patch.object(guard, 'ownership', side_effect=lambda _: contextlib.nullcontext(self.transport)),
                        patch('builtins.print')):
            patcher.start()
            self.addCleanup(patcher.stop)

    def run_cycles(self, count):
        for _ in range(count):
            self.instance.cycle()
            self.clock.sleep(5)

    def test_dual_fault_recovers_serially_and_only_once(self):
        self.run_cycles(7)
        self.assertEqual(self.transport.sent, ['192.0.2.1'])
        self.run_cycles(6)
        self.assertEqual(self.transport.sent, ['192.0.2.1'])
        self.run_cycles(10)
        self.assertEqual(self.transport.sent, ['192.0.2.1', '192.0.2.2'])
        saved = json.loads((Path(self.temp.name) / 'budget.json').read_text())
        self.assertTrue(all(v['result'] == 'recovered_sampling' for v in saved['attempts'].values()))

    def test_lost_ack_does_not_repeat_command_and_recovery_can_still_be_verified(self):
        self.transport.ack_lost = True
        self.run_cycles(7)
        self.assertEqual(self.transport.sent, ['192.0.2.1'])
        self.assertEqual(self.instance.budget.data['attempts']['TEST_REAR']['result'], 'recovered_sampling')

    def test_failed_recovery_is_bounded_and_no_retry_after_restart(self):
        self.transport.recover = False
        self.run_cycles(7)
        self.assertEqual(self.instance.budget.data['attempts']['TEST_REAR']['result'], 'unconfirmed_no_retry')
        self.instance = guard.Guard({'devices': DEVICES}, Path(self.temp.name))
        self.run_cycles(30)
        self.assertEqual(self.transport.sent.count('192.0.2.1'), 1)
        self.assertEqual(self.transport.sent.count('192.0.2.2'), 1)

    def test_link_down_motion_or_busy_ownership_cannot_send(self):
        for block in ('WAIT_LINK', 'BLOCKED_CAN_UP:can0', 'BLOCKED_ACTIVE_PROCESS:123'):
            with patch.object(guard, 'blockers', return_value=block):
                self.run_cycles(10)
        with patch.object(guard, 'ownership', side_effect=BlockingIOError('busy')):
            self.run_cycles(10)
        self.assertEqual(self.transport.sent, [])

    def test_ack_without_new_boot_counter_is_not_success(self):
        with patch.object(self.transport, 'query', return_value=row(state=1, codes=[], count=7)):
            self.assertFalse(self.instance.verify(self.transport, DEVICES[0], row()))
        self.assertGreaterEqual(self.clock.now, 90)

    def test_verification_rejects_wrong_device_and_new_error(self):
        for invalid in ({**row(state=1, codes=[], count=8), 'serial': 'WRONG'},
                        row(state=1, codes=[0x01160003], count=8)):
            with patch.object(self.transport, 'query', return_value=invalid):
                self.assertFalse(self.instance.verify(self.transport, DEVICES[0], row()))


if __name__ == '__main__':
    unittest.main()
