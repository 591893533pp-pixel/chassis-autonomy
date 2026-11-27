"""Fixed-speed, single-direction hold-to-run control; no stick motion input."""
from vendor.navslam_core import Command, Config, TeleopCore

class ButtonCore(TeleopCore):
    def __init__(self):
        super().__init__(Config(axis_forward=-1, axis_left=-1, axis_yaw=-1,
            button_auto=-1, button_slower=-1, button_faster=-1,
            max_linear_speed=.05, max_angular_speed=.04, initial_speed_level=0))
        self.pulse_start = None
        self.release_required = True
        self.active_direction = None
        self.button_reason = 'release_buttons'

    def _neutral(self):
        return super()._neutral() and not self._button(6) and not self._button(7)

    def output(self, now):
        self._check_joy_timeout(now)
        if not self.axes or not self.buttons:
            return Command()
        x, y = self._raw(7), self._raw(6)
        directions = []
        if abs(x) > .5: directions.append(('forward' if x > 0 else 'reverse', Command(x=.05 if x > 0 else -.05)))
        if abs(y) > .5: directions.append(('left' if y > 0 else 'right', Command(y=.05 if y > 0 else -.05)))
        if self._button(6): directions.append(('turn_left', Command(yaw=.04)))
        if self._button(7): directions.append(('turn_right', Command(yaw=-.04)))
        # Triggers and lift buttons never change speed or issue motion in this mode.
        blocked = (self._trigger(4) > .12 or self._trigger(5) > .12
                   or any(self._button(i) for i in (3,4,8,9)))
        if not self.enabled:
            self.release_required = True
            self.pulse_start = None
            self.button_reason = 'disabled'
            return Command()
        if not directions and not blocked:
            self.release_required = False
            self.pulse_start = None
            self.active_direction = None
            self.button_reason = 'ready'
            return Command()
        if blocked or len(directions) != 1:
            self.release_required = True
            self.button_reason = 'conflicting_buttons_release_all'
            return Command()
        direction, command = directions[0]
        if self.release_required:
            self.button_reason = 'release_buttons'
            return Command()
        if self.pulse_start is None:
            self.pulse_start = now
            self.active_direction = direction
        if direction != self.active_direction:
            self.release_required = True
            self.button_reason = 'direction_changed_release_buttons'
            return Command()
        self.button_reason = direction
        return command

    def status(self, now):
        s = super().status(now)
        s.update(control_mode='buttons', button_reason=self.button_reason,
                 pulse_limit_s=None, fixed_linear_m_s=.05, fixed_yaw_rad_s=.04)
        return s
