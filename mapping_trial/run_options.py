"""Select a bounded experiment without importing ROS or opening hardware."""
import argparse


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=30)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--motion', action='store_true')
    mode.add_argument('--sensors-only', action='store_true',
                      help='Static LiDAR/IMU/SLAM check; never starts the control bridge')
    args = parser.parse_args(argv)
    if not 30 <= args.seconds <= 900:
        parser.error('seconds must be 30..900')
    return args


def scope(args):
    if args.sensors_only:
        return 'STATIC_SENSOR_MAPPING_CHECK'
    return 'MANUAL_MAPPING_TRIAL' if args.motion else 'MAPPING_LOCKED_CONTROL_CHECK'
