#!/usr/bin/env python3
"""Bounded lift-only PV commissioning. No wheel frames. Hardware requires explicit flag."""
import argparse, datetime, fcntl, hashlib, json, os, select, signal, socket, struct
import threading, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
COUNTS_MM = 800 * 7 / 48
REQUESTED_SPEED_MM_S = 10.0
SPEED = round(REQUESTED_SPEED_MM_S * COUNTS_MM)
OVERSPEED_COUNTS_S = max(500, 2*SPEED)
END_MARGIN_MM = 0.5
APPROACH_TIME_S = 0.5


def operation_enabled(sw):
    return sw & 0x6f == 0x27


def neutral(axes, buttons, allow_start=False):
    return (all(abs(axes[i]) < .12 for i in (0, 1, 2, 3, 6, 7))
            and all(axes[i] > .8 for i in (4, 5))
            and not any(v for i, v in enumerate(buttons) if not (allow_start and i == 11)))


class Gate:
    """Hold to run; conflicting inputs and direction changes require release."""
    def __init__(self):
        self.direction = 0
        self.blocked = False

    def target(self, buttons, position, origin, now):
        if not buttons[3] and not buttons[4]:
            self.direction = 0
            self.blocked = False
            return 0
        direction = int(bool(buttons[3])) - int(bool(buttons[4]))
        if not direction or self.blocked:
            self.blocked = True
            return 0
        if self.direction and direction != self.direction:
            self.blocked = True
            return 0
        self.direction = direction
        if not 0 <= position <= 250:
            raise RuntimeError('absolute position boundary')
        distance = 250-position if direction > 0 else position
        if distance <= END_MARGIN_MM:
            self.blocked = True
            return 0
        # Reduce the target near either endpoint; reserve stopping margin.
        cap = round((distance-END_MARGIN_MM)/APPROACH_TIME_S*COUNTS_MM)
        return direction*min(SPEED, max(0, cap))



def record_and_validate(feedback, event, state, target, origin, speed_reference=None):
    reference = target if speed_reference is None else speed_reference
    # Preserve the failing sample BEFORE raising. Target is the preceding
    # successfully sent command, not a future command calculated from this sample.
    feedback.write(json.dumps(dict(t=time.monotonic(),
        target_counts_s=target, sample_phase='before_next_command', **state))+'\n')
    reason = None
    if not operation_enabled(state['statusword']):
        reason = f"lost OperationEnabled {state['statusword']:#x}"
    elif state['mode'] != 3:
        reason = 'mode changed'
    elif not 0 <= state['position_mm'] <= 250:
        reason = 'position boundary exceeded'
    elif abs(state['velocity_counts_s']) > max(500, 2*abs(reference)):
        reason = 'overspeed feedback'
    if reason:
        event('fault_snapshot', reason=reason, target_counts_s=target,
              overspeed_limit_counts_s=max(500, 2*abs(reference)), **state)
        raise RuntimeError(reason)


class Joystick:
    def __init__(self, folder, layout=None):
        self.mutex = threading.Lock()
        self.sample = None
        self.error = None
        self.done = threading.Event()
        self.folder = folder
        self.layout = layout
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            fd = os.open('/dev/input/js0', os.O_RDONLY | os.O_NONBLOCK)
            try:
                na, nb = bytearray(1), bytearray(1)
                fcntl.ioctl(fd, 0x80016a11, na, True)
                fcntl.ioctl(fd, 0x80016a12, nb, True)
                if self.layout is not None:
                    self.layout.validate_fd(fd)
                elif na[0] != 8 or nb[0] < 12:
                    raise RuntimeError('unverified joystick layout')
                axes, buttons = [0.]*na[0], [0]*nb[0]
                ia, ib = set(), set()
                with (self.folder/'joystick.jsonl').open('w', buffering=1) as log:
                    while not self.done.is_set():
                        if select.select([fd], [], [], .02)[0]:
                            data = os.read(fd, 1024)
                            if not data or len(data)%8:
                                raise RuntimeError('joystick disconnected')
                            for _, v, kind, i in struct.iter_unpack('IhBB', data):
                                kind &= 0x7f
                                if kind == 2:
                                    axes[i] = max(-1., min(1., -v/32767.)); ia.add(i)
                                elif kind == 1:
                                    buttons[i] = int(v); ib.add(i)
                            log.write(json.dumps(dict(t=time.monotonic(), axes=axes, buttons=buttons))+'\n')
                        with self.mutex:
                            self.sample = (time.monotonic(), axes[:], buttons[:], len(ia)==len(axes) and len(ib)==len(buttons))
            finally:
                os.close(fd)
        except Exception as e:
            with self.mutex:
                self.error = repr(e)

    def get(self):
        with self.mutex:
            if self.error:
                raise RuntimeError(self.error)
            sample = self.sample
        if sample is None:
            return None
        if time.monotonic()-sample[0] > .15:
            raise RuntimeError('joystick reader heartbeat stale')
        stop_pressed = False
        if sample[3]:
            if self.layout is not None:
                stop_pressed = self.layout.dual_trigger(sample[1], sample[2])
            else:
                stop_pressed = ((sample[1][4] < -.6 and sample[1][5] < -.6)
                                or (sample[2][8] and sample[2][9]))
        if stop_pressed:
            raise RuntimeError('dual trigger stop')
        return sample


class Bus:
    def __init__(self, folder):
        self.log = (folder/'can.jsonl').open('w', buffering=1)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', 0, 0xC0000000))
        self.sock.bind(('can0',))
        self.sock.setblocking(False)

    def record(self, direction, cid, data):
        self.log.write(json.dumps(dict(t=time.monotonic(), direction=direction, can_id=hex(cid), data=data.hex()))+'\n')

    def send(self, cid, data):
        if cid not in (0, 0x601):
            raise RuntimeError('non-lift frame forbidden')
        self.sock.send(struct.pack('=IB3x8s', cid, len(data), data.ljust(8,b'\0')))
        self.record('tx', cid, data)

    def sdo(self, idx, value=None, size=4, signed=False):
        cmd = 0x40 if value is None else {1:0x2f,2:0x2b,4:0x23}[size]
        req = bytes([cmd, idx&255, idx>>8, 0]) + (0 if value is None else value & 0xffffffff).to_bytes(4,'little')
        self.send(0x601, req)
        deadline = time.monotonic()+.1
        while time.monotonic() < deadline:
            if not select.select([self.sock], [], [], max(0,deadline-time.monotonic()))[0]:
                break
            cid, n, data = struct.unpack('=IB3x8s', self.sock.recv(16))
            self.record('rx', cid, data[:n])
            if cid != 0x581 or n != 8 or data[1:4] != req[1:4]:
                continue
            if data[0] == 0x80:
                raise RuntimeError(f'SDO {idx:04x} abort {data[4:].hex()}')
            if value is not None and data[0] == 0x60:
                return
            if value is None and data[0] in (0x43,0x47,0x4b,0x4f):
                length = 4-((data[0]>>2)&3)
                return int.from_bytes(data[4:4+length], 'little', signed=signed)
            raise RuntimeError(f'SDO {idx:04x} malformed response')
        raise RuntimeError(f'SDO {idx:04x} timeout')

    def position(self):
        return self.sdo(0x6064, signed=True)/COUNTS_MM

    def snapshot(self):
        return dict(position_mm=self.position(), velocity_counts_s=self.sdo(0x606c, signed=True),
                    statusword=self.sdo(0x6041), mode=self.sdo(0x6061, signed=True))

    def zero(self):
        self.sdo(0x60ff, 0)


def enable_pv(bus, check_input):
    bus.zero()
    bus.sdo(0x6060, 3, 1)
    if bus.sdo(0x6061, signed=True) != 3:
        raise RuntimeError('PV mode readback mismatch')
    bus.send(0, bytes([1,1]))
    deadline = time.monotonic()+2
    while time.monotonic() < deadline:
        check_input()
        sw = bus.sdo(0x6041)
        if operation_enabled(sw):
            return
        if sw & 0x4f == 0x40:
            cw = 6
        elif sw & 0x6f == 0x21:
            cw = 7
        elif sw & 0x6f == 0x23:
            cw = 15
        else:
            raise RuntimeError(f'unexpected enable state {sw:#x}; no automatic fault reset')
        bus.sdo(0x6040, cw, 2)
        time.sleep(.02)
    raise RuntimeError('enable timeout')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hardware', action='store_true')
    args=ap.parse_args()
    if not args.hardware:
        ap.error('No hardware requested; use test_lift_pv_remote.py for offline verification')
    lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a')
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=ROOT/'chassis_lift_control/logs'/('lift-pv-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    folder.mkdir()
    (folder/'script_used.py').write_bytes(Path(__file__).read_bytes())
    event_file=(folder/'events.jsonl').open('w',buffering=1)
    def event(name, **kw):
        row=dict(t=time.monotonic(), event=name, **kw)
        event_file.write(json.dumps(row,ensure_ascii=False)+'\n'); print(name,kw,flush=True)
    print('LOG',folder,flush=True)
    bus=joy=None; touched=False; exit_code=0
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        bus=Bus(folder)
        before=bus.snapshot()
        (folder/'before.json').write_text(json.dumps(before,indent=2))
        if operation_enabled(before['statusword']) or abs(before['velocity_counts_s'])>10:
            raise RuntimeError('drive already enabled/moving; ownership not established')
        if not 0 <= before['position_mm'] <= 250:
            raise RuntimeError('initial position outside 0–250 mm')
        joy=Joystick(folder)
        event('waiting_start', **before, speed_counts_s=SPEED, speed_mm_s=SPEED/COUNTS_MM,
              requested_speed_mm_s=REQUESTED_SPEED_MM_S,
              hold_limit_s=None, absolute_limits_mm=[0,250], session_guard_mm=None,
              endpoint_margin_mm=END_MARGIN_MM, approach_time_s=APPROACH_TIME_S,
              full_speed_feedback_limit_counts_s=OVERSPEED_COUNTS_S)
        ready=False; previous_start=False; enabled=False; last_target=0; gate=Gate()
        last_tick=time.monotonic(); stop_deadline=None; stopping_target=0; last_notice=None
        with (folder/'feedback.jsonl').open('w',buffering=1) as feedback:
            while True:
                now=time.monotonic()
                if enabled and now-last_tick > .3:
                    raise RuntimeError('control loop overrun')
                last_tick=now
                sample=joy.get()
                if sample is None or not sample[3]:
                    time.sleep(.02); continue
                _,axes,buttons,_=sample
                start=bool(buttons[11]); rising=start and not previous_start; previous_start=start
                if not enabled:
                    if neutral(axes,buttons):ready=True
                    elif not neutral(axes,buttons,True):ready=False
                    if ready and rising and neutral(axes,buttons,True):
                        def check_neutral():
                            s=joy.get()
                            if not s or not s[3] or not neutral(s[1],s[2],True):
                                raise RuntimeError('controls changed during enable')
                        touched=True
                        enable_pv(bus,check_neutral)
                        enabled=True
                        last_tick=time.monotonic()
                        event('enabled', message='PV ready; X up / Y down; release zero; dual triggers exit')
                    time.sleep(.02);continue
                state=bus.snapshot()
                record_and_validate(feedback,event,state,last_target,before['position_mm'],
                                    stopping_target if stop_deadline is not None else None)
                sample=joy.get(); _,axes,buttons,_=sample
                target=gate.target(buttons,state['position_mm'],before['position_mm'],time.monotonic())
                # Sticks/D-pad never command wheels; accidental non-lift input stops lift.
                if any(abs(axes[i])>.12 for i in (0,1,2,3,6,7)) or any(v for i,v in enumerate(buttons) if i not in (3,4)):
                    target=0;gate.blocked=True
                if stop_deadline is not None:
                    if abs(state['velocity_counts_s'])<=10:
                        event('zero_verified', **state); stop_deadline=None
                    elif time.monotonic()>stop_deadline:
                        raise RuntimeError('zero speed not reached within 0.5s')
                    else:
                        target=0
                notice = ('release_required_or_limit' if gate.blocked else
                          'endpoint_slowdown' if target and abs(target)<SPEED else 'ready')
                if notice!=last_notice:
                    event('motion_gate',reason=notice,position_mm=state['position_mm'])
                    last_notice=notice
                if target!=last_target:
                    bus.sdo(0x60ff,target)
                    event('velocity_target', counts_s=target, previous_counts_s=last_target,
                          position_mm=state['position_mm'])
                    if last_target and not target:
                        stopping_target=last_target;stop_deadline=time.monotonic()+.5
                    last_target=target
                time.sleep(.02)
        event('session_timeout')
    except KeyboardInterrupt:
        event('operator_exit')
    except Exception as e:
        exit_code=1;event('fault',message=str(e))
    finally:
        if bus and touched:
            for name,action in [('zero',bus.zero),('disable',lambda:bus.sdo(0x6040,0,2)),('preop',lambda:bus.send(0,bytes([128,1])))]:
                try:
                    action()
                    if name=='zero':time.sleep(.15)
                    event('cleanup_'+name)
                except Exception as e:
                    exit_code=1;event('cleanup_failed',action=name,message=str(e))
            try:
                final=bus.snapshot()
                event('final_readback',**final)
                if operation_enabled(final['statusword']) or abs(final['velocity_counts_s'])>10:
                    exit_code=1;event('cleanup_unconfirmed',message='use physical stop; enabled or moving feedback')
            except Exception as e:event('final_readback_failed',message=str(e));exit_code=1
        if joy:joy.done.set();joy.thread.join(timeout=1)
        if bus:bus.sock.close();bus.log.close()
        event('session_end',exit_code=exit_code)
        event_file.close()
        (folder/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()},indent=2))
    return exit_code

if __name__=='__main__':
    raise SystemExit(main())
