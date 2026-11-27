"""Bounded ground commissioning using the installed controller's cleanup path."""
import argparse
import dataclasses
import datetime
import json
from pathlib import Path
import signal
import sys

ROOT = Path('/home/pc/rk3588 link/remote control')
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'program/remote'), str(ROOT/'lift_reference')]


def limited_config(factory, maximum, initial_gear):
    if maximum != .05 or initial_gear != 1:
        raise ValueError('Ground commissioning requires 0.05 m/s, gear 1')
    config = dataclasses.replace(factory(maximum, initial_gear), max_angular_speed=.04)
    config.validate()
    return config


def stationary_core_class(base):
    class StationaryCore(base):
        def output(self, now):
            command = super().output(now)
            return dataclasses.replace(command, x=0., y=0., yaw=0., lift=0.)
    return StationaryCore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=('stationary', 'motion'), default='stationary')
    parser.add_argument('--seconds', type=int, default=180)
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 180:
        parser.error('seconds must be 10..180')
    import remote_session as session
    factory = session.manual_config
    config = limited_config(factory, .05, 1)
    setup = dict(phase=args.phase, limit_linear_m_s=.05, limit_yaw_rad_s=.04,
                 timeout_seconds=args.seconds, automatic_enable=False,
                 motion_commands_locked=args.phase == 'stationary')
    print(json.dumps(setup, ensure_ascii=False), flush=True)
    if args.preview:
        print(json.dumps(vars(config), ensure_ascii=False, indent=2))
        return 0
    session.manual_config = lambda maximum, initial_gear: limited_config(factory, maximum, initial_gear)
    if args.phase == 'stationary':
        session.TeleopCore = stationary_core_class(session.TeleopCore)
    folder = ROOT/'logs'/('ground_'+args.phase+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    sys.argv = ['remote_session.py', '--chassis-only', '--max-linear-speed', '.05',
                '--initial-gear', '1', '--log-dir', str(folder)]
    def expired(*_):
        print('地面验收时间到，执行停车及失能清理。', flush=True)
        raise KeyboardInterrupt
    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, args.seconds)
    try:
        return session.main()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        if folder.exists():
            (folder/'ground_check.json').write_text(json.dumps(setup, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
