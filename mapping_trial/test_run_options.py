import contextlib
import io
import unittest

from run_options import parse_args, scope


class RunOptions(unittest.TestCase):
    def test_sensor_check_has_no_motion(self):
        args = parse_args(['--sensors-only', '--seconds', '90'])
        self.assertFalse(args.motion)
        self.assertTrue(args.sensors_only)
        self.assertEqual(scope(args), 'STATIC_SENSOR_MAPPING_CHECK')

    def test_sensor_mode_cannot_be_combined_with_motion(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['--sensors-only', '--motion'])

    def test_existing_modes_preserved(self):
        self.assertEqual(scope(parse_args([])), 'MAPPING_LOCKED_CONTROL_CHECK')
        self.assertEqual(scope(parse_args(['--motion'])), 'MANUAL_MAPPING_TRIAL')

    def test_duration_is_bounded(self):
        for duration in ['0', '29', '901']:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--sensors-only', '--seconds', duration])


if __name__ == '__main__':
    unittest.main()
