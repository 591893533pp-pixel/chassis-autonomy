#!/usr/bin/env python3
"""Standalone DINPUT joystick reader using the copied navslam state machine."""
import argparse
import dataclasses
import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import time
from vendor.navslam_core import Config, TeleopCore
from button_core import ButtonCore

ROOT = Path(__file__).resolve().parents[1]

def manual_config(maximum=.5, initial_gear=1):
    steps = round(maximum / .05)
    if not 1 <= steps <= 30 or abs(maximum - steps * .05) > 1e-9:
        raise ValueError('maximum must be a 0.05 m/s increment in [0.05, 1.5]')
    if not 1 <= initial_gear <= steps:
        raise ValueError('initial gear exceeds selected maximum')
    return Config(speed_levels=tuple(i / steps for i in range(1, steps + 1)),
                  max_linear_speed=maximum, max_angular_speed=.4,
                  stick_yaw_independent_gear=True,
                  initial_speed_level=initial_gear-1, button_auto=-1)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--buttons', action='store_true', help='fixed-speed D-pad/LB/RB hold-to-run; sticks disabled')
    p.add_argument('--initial-gear', type=int, choices=range(1,15), default=1, help='starting gear at 0.05 m/s increments; defaults to lowest')
    p.add_argument('--max-linear-speed', type=float, default=.5, help='manual ceiling in m/s, 0.05..1.5 in 0.05 increments; default 0.5')
    p.add_argument('--device', default='/dev/input/js0')
    p.add_argument('--hardware', action='store_true', help='connect real CAN; default is dry run')
    p.add_argument('--can-interface', help='SocketCAN interface, e.g. can0 for XCAN CH1')
    p.add_argument('--serial', help='verified CANable serial; XCAN uses --can-interface')
    p.add_argument('--lift', action='store_true', help='include lift (0–250 mm, 3 mm/s)')
    p.add_argument('--lift-zero-verified', action='store_true', help='operator has verified lift persistent zero and count scale')
    args = p.parse_args()
    if not .05 <= args.max_linear_speed <= 1.5:
        p.error('--max-linear-speed must be finite and in [0.05, 1.5]')
    try:
        manual_cfg = manual_config(args.max_linear_speed, args.initial_gear)
    except ValueError as exc:
        p.error(str(exc))
    if args.buttons and args.initial_gear != 1:
        p.error('--buttons uses fixed speed; omit --buttons to use gears')
    if args.buttons and args.lift:
        p.error('--buttons is chassis direction verification only; omit --lift')
    if args.can_interface and args.serial:
        p.error('choose --can-interface or --serial, not both')
    if args.hardware and not (args.serial or args.can_interface):
        p.error('--hardware requires --can-interface can0 (XCAN) or --serial (CANable)')
    if args.can_interface:
        import re
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', args.can_interface):
            p.error('invalid CAN interface name')
    transport = 'socketcan:' + args.can_interface if args.can_interface else args.serial
    if args.hardware and args.lift and not args.lift_zero_verified:
        p.error('lift requires --lift-zero-verified after physical zero/count-scale verification')
    # One session per local user. All robot CAN applications must still have exclusive ownership.
    lock = open(f'/tmp/poke-remote-{os.getuid()}.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error('another remote session is already running')
    if not Path(args.device).exists():
        p.error(f'joystick not found: {args.device}; connect the DINPUT controller first')
    folder = ROOT / 'logs' / ('remote-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    folder.mkdir(parents=True)
    latest = ROOT / 'logs' / 'remote-latest'
    temp = latest.with_name('.remote-latest-temp')
    temp.unlink(missing_ok=True)
    temp.symlink_to(folder.name)
    temp.replace(latest)
    cfg = manual_cfg
    core = ButtonCore() if args.buttons else TeleopCore(cfg)
    cfg = core.config
    (folder / 'session.json').write_text(json.dumps({
        'args': vars(args), 'config': dataclasses.asdict(cfg),
        'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source': '/home/pc/navslam/src/poke_teleop/poke_teleop/core.py',
        'axes_conversion': 'Linux js int16 -> -value/32767 (joy_linux convention)',
        'lift_policy': 'optional; chassis/lift motion interlocked; 0.6 mm PP segments',
        'chassis_mapping': 'physical vx/vy -> legacy -vx/-vy; yaw unchanged; final_front_20260917',
        'body_control': {'hold_enabled': True, 'reason': 'bounded hold correction; ground stability not yet validated', 'linear_accel_m_s2': 1.0, 'angular_accel_rad_s2': .30, 'crawl_speed_m_s': .08, 'crawl_cross_trim_cap_m_s': .002, 'hold_planar_cap_m_s': .005, 'crawl_hold_planar_cap_m_s': .002, 'hold_planar_cap_scope': 'total hold correction before acceleration shaping and wheel PI; revision 20260921', 'crawl_yaw_trim_cap_rad_s': .006, 'feedback': 'segment-local wheel odometry only'},
        'wheel_balance': {'enabled': True, 'normal_kp': .2, 'normal_ki': .3, 'normal_max_fraction': .25, 'crawl_kp': .12, 'crawl_ki': .12, 'crawl_max_fraction': .15, 'crawl_threshold_rad_s': .80, 'filter_tau_s': .12, 'error_definition': 'matched filtered target minus filtered velocity; revision 20260921', 'feedback': 'wheel encoders, not SLAM'},
        'feedback_is_real': args.hardware}, ensure_ascii=False, indent=2))
    print(f'日志目录: {folder}', flush=True)
    print('模式: ' + ('实机 CAN' if args.hardware else '手柄检查，未连接CAN'), flush=True)
    print('松开全部按键、摇杆及扳机后按START；双扳机停止；Ctrl+C退出。', flush=True)
    if args.buttons:
        print('按钮模式：方向键平移，LB左转/RB右转；摇杆无效；按住持续运动，松开停止；平移固定5cm/s，旋转0.04rad/s。', flush=True)
    if not args.buttons:
        print(f'完整手动模式：方向键/左摇杆平移，右摇杆旋转；LB降档/RB升档（1–{len(cfg.speed_levels)}档，每次0.05m/s）；松开运动输入停止。平移请求上限{args.max_linear_speed:.2f}m/s，一档0.05m/s，右摇杆旋转上限{cfg.max_angular_speed:.2f}rad/s（独立于平移档位）。', flush=True)
    command_log = open(folder/'commands.jsonl', 'w', buffering=1)
    event_log = open(folder/'events.jsonl', 'w', buffering=1)
    raw_log = open(folder/'joystick.jsonl', 'w', buffering=1)
    bridge_log = open(folder/'bridge.log', 'w', buffering=1)
    def event(kind, **details):
        entry = dict(monotonic_s=time.monotonic(), utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), event=kind, **details)
        event_log.write(json.dumps(entry, ensure_ascii=False)+'\n')
        print(kind, details, flush=True)
    cmd = [str(ROOT/'build/remote_bridge'), '--log', str(folder/'feedback.csv')]
    if args.hardware: cmd += ['--hardware', transport]
    if args.lift: cmd += ['--lift']
    process = None
    joy = None
    exit_code = 0
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        joy = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
        axes_count, buttons_count = bytearray(1), bytearray(1)
        fcntl.ioctl(joy, 0x80016a11, axes_count, True)
        fcntl.ioctl(joy, 0x80016a12, buttons_count, True)
        if axes_count[0] < core.required_axes or buttons_count[0] < core.required_buttons:
            raise RuntimeError(f'wrong DINPUT layout: axes={axes_count[0]}, buttons={buttons_count[0]}')
        axes, buttons = [0.] * axes_count[0], [0] * buttons_count[0]
        initialized_axes, initialized_buttons = set(), set()
        process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=bridge_log, stderr=bridge_log, text=True, start_new_session=True)
        last_state = None
        tick = time.monotonic()
        while True:
            if process.poll() is not None:
                raise RuntimeError(f'CAN bridge exited: {process.returncode}; inspect bridge.log')
            ready, _, _ = select.select([joy], [], [], max(0, tick-time.monotonic()))
            if ready:
                data = os.read(joy, 8 * 128)
                if not data or len(data) % 8:
                    raise RuntimeError('joystick disconnected / malformed event')
                for _, value, kind, number in struct.iter_unpack('IhBB', data):
                    kind &= 0x7f
                    if kind == 2:
                        if number >= len(axes): raise RuntimeError('invalid axis index')
                        axes[number] = max(-1., min(1., -value/32767.))
                        initialized_axes.add(number)
                    elif kind == 1:
                        if number >= len(buttons): raise RuntimeError('invalid button index')
                        buttons[number] = int(value)
                        initialized_buttons.add(number)
                raw_log.write(json.dumps(dict(monotonic_s=time.monotonic(), axes=axes, buttons=buttons))+'\n')
            now = time.monotonic()
            if now < tick: continue
            # Like joy_linux autorepeat: device connectivity is checked, not RF link health.
            # Do not fabricate trigger defaults; wait for all JS_EVENT_INIT values.
            initialized = len(initialized_axes)==len(axes) and len(initialized_buttons)==len(buttons)
            if initialized:
                # Let core see elapsed silence before feeding the next cached sample.
                core.output(now)
                core.receive_joy(axes, buttons, now)
            command = core.output(now)
            if not args.lift: command = dataclasses.replace(command, lift=0.)
            status = core.status(now)
            state = (status['enabled'], status['reason'], status['speed_level'], status.get('button_reason'))
            if state != last_state:
                event('teleop_state', **status);last_state=state
            process.stdin.write(f'{now:.9f} {int(core.enabled)} {command.x:.9f} {command.y:.9f} {command.yaw:.9f} {command.lift:.9f}\n')
            process.stdin.flush()
            command_log.write(json.dumps(dict(monotonic_s=now, command=dataclasses.asdict(command), status=status, initialized=initialized))+'\n')
            tick = now + 1/30
    except KeyboardInterrupt:
        event('operator_exit')
    except Exception as exc:
        exit_code = 1
        event('fault', message=str(exc))
        # Surface the actual CAN failure, rather than only the parent pipe error.
        if process is not None:
            try:
                details = (folder/'bridge.log').read_text(errors='replace').splitlines()[-12:]
                event('bridge_diagnostics', lines=details)
            except OSError:
                pass
    finally:
        if joy is not None: os.close(joy)
        if process is not None:
            if process.poll() is None:
                try:
                    process.stdin.write(f'{time.monotonic():.9f} 0 0 0 0 0\n');process.stdin.flush()
                except (BrokenPipeError, OSError): pass
                process.terminate()
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    # SIGKILL would bypass drive cleanup; report rather than pretending stopped.
                    event('stop_timeout', pid=process.pid, message='CAN stop unconfirmed; use physical emergency stop')
                    exit_code=1
            if process.returncode not in (None, 0): exit_code=1
        event('session_end', exit_code=exit_code)
        for f in (command_log,event_log,raw_log,bridge_log): f.close()
    return exit_code

if __name__ == '__main__':
    raise SystemExit(main())
