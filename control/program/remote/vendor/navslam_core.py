"""ROS-independent joystick state machine; all times are monotonic seconds."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


@dataclass
class Config:
    axis_forward: int = 1
    axis_left: int = 0
    axis_yaw: int = 2
    axis_yaw_secondary: int = -1
    axis_dpad_forward: int = 7
    axis_dpad_left: int = 6
    axis_rt: int = 4
    axis_lt: int = 5
    sign_forward: float = 1.0
    sign_left: float = 1.0
    sign_yaw: float = 1.0
    sign_yaw_secondary: float = 1.0
    trigger_rest: float = 1.0
    trigger_pressed: float = -1.0
    button_auto: int = 0
    button_manual: int = 1
    button_lift_up: int = 3
    button_lift_down: int = 4
    button_slower: int = 6
    button_faster: int = 7
    button_lt: int = 8
    button_rt: int = 9
    button_enable: int = 11
    deadzone: float = 0.10
    neutral_threshold: float = 0.12
    emergency_trigger_threshold: float = 0.80
    max_linear_speed: float = 0.60
    max_angular_speed: float = 1.20
    stick_yaw_independent_gear: bool = False
    auto_forward_speed: float = 0.70
    auto_reverse_speed: float = 0.25
    auto_lateral_speed: float = 0.25
    auto_angular_speed: float = 0.40
    speed_levels: tuple = (0.20, 0.40, 0.60, 0.80, 1.0)
    initial_speed_level: int = 1
    rt_boost: float = 0.50
    lt_reduction: float = 0.50
    arc_yaw_fraction: float = 0.60
    joy_timeout: float = 0.50
    auto_timeout: float = 0.50

    def validate(self):
        for name, value in vars(self).items():
            if name.startswith(('axis_', 'button_')):
                if type(value) is not int or value < -1:
                    raise ValueError(f'{name} must be an index >= 0 or -1 to disable')
            elif isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.button_enable < 0:
            raise ValueError('button_enable cannot be disabled')
        if not 0 <= self.deadzone < self.neutral_threshold < 1:
            raise ValueError('Require 0 <= deadzone < neutral_threshold < 1')
        if not self.neutral_threshold < self.emergency_trigger_threshold <= 1:
            raise ValueError('emergency_trigger_threshold must exceed neutral_threshold')
        if self.max_linear_speed <= 0 or self.max_angular_speed <= 0:
            raise ValueError('Speed limits must be positive')
        if any(getattr(self,k)<=0 for k in ('auto_forward_speed','auto_reverse_speed','auto_lateral_speed','auto_angular_speed')):
            raise ValueError('Automatic speed limits must be positive')
        if self.joy_timeout <= 0 or self.auto_timeout <= 0:
            raise ValueError('Timeouts must be positive')
        if self.trigger_rest == self.trigger_pressed:
            raise ValueError('Trigger rest and pressed values must differ')
        if not self.speed_levels or any(
                not math.isfinite(x) or not 0 < x <= 1 for x in self.speed_levels):
            raise ValueError('speed_levels must contain finite fractions in (0, 1]')
        if list(self.speed_levels) != sorted(self.speed_levels):
            raise ValueError('speed_levels must be increasing')
        if not 0 <= self.initial_speed_level < len(self.speed_levels):
            raise ValueError('initial_speed_level outside speed_levels')
        if not 0 <= self.lt_reduction <= 1 or self.rt_boost < 0:
            raise ValueError('Require lt_reduction in [0, 1], rt_boost >= 0')
        if not 0 <= self.arc_yaw_fraction <= 1:
            raise ValueError('arc_yaw_fraction must be in [0, 1]')


@dataclass(frozen=True)
class Command:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    lift: float = 0.0


class TeleopCore:
    def __init__(self, config: Optional[Config] = None):
        self.config = config or Config()
        self.config.validate()
        self.enabled = False
        self.estop_latched = False
        self.mode = 'manual'
        self.speed_level = self.config.initial_speed_level
        self.reason = 'waiting_for_joystick'
        self.neutral_ready = False
        self.last_joy: Optional[float] = None
        self.last_auto: Optional[float] = None
        self.axes = []
        self.buttons = []
        self.previous_buttons = None
        self.auto = Command()
        self.required_axes = 1 + max(
            value for name, value in vars(self.config).items() if name.startswith('axis_'))
        self.required_buttons = 1 + max(
            value for name, value in vars(self.config).items() if name.startswith('button_'))

    def _button(self, index):
        return index >= 0 and bool(self.buttons[index])

    def _edge(self, index):
        return (index >= 0 and self._button(index)
                and self.previous_buttons is not None and not self.previous_buttons[index])

    def _raw(self, index):
        return self.axes[index] if index >= 0 else 0.0

    def _axis(self, index):
        value = self._raw(index)
        if abs(value) <= self.config.deadzone:
            return 0.0
        return math.copysign((abs(value) - self.config.deadzone)
                             / (1.0 - self.config.deadzone), value)

    def _trigger(self, index):
        if index < 0:
            return 0.0
        c = self.config
        value = (self.axes[index] - c.trigger_rest) / (c.trigger_pressed - c.trigger_rest)
        return max(0.0, min(1.0, value))

    def _neutral(self):
        c = self.config
        indices = (c.axis_forward, c.axis_left, c.axis_yaw, c.axis_yaw_secondary,
                   c.axis_dpad_forward, c.axis_dpad_left)
        return (all(abs(self._raw(i)) <= c.neutral_threshold for i in indices)
                and self._trigger(c.axis_lt) <= c.neutral_threshold
                and self._trigger(c.axis_rt) <= c.neutral_threshold
                and not any(self._button(i) for i in
                            (c.button_lt, c.button_rt, c.button_lift_up, c.button_lift_down)))

    def _disarm(self, reason, forget_joystick=False):
        self.enabled = False
        self.neutral_ready = False
        self.last_auto = None
        self.auto = Command()
        self.reason = reason
        if forget_joystick:
            self.last_joy = None
            self.previous_buttons = None

    def _check_joy_timeout(self, now):
        if self.last_joy is not None and now - self.last_joy > self.config.joy_timeout:
            self._disarm('joystick_timeout', forget_joystick=True)

    def receive_joy(self, axes: Sequence[float], buttons: Sequence[int], now: float):
        self._check_joy_timeout(now)
        if (len(axes) < self.required_axes or len(buttons) < self.required_buttons
                or not all(math.isfinite(a) and -1.05 <= a <= 1.05 for a in axes)
                or not all(b in (0, 1) for b in buttons)):
            self._disarm('invalid_joystick_layout', forget_joystick=True)
            return
        self.axes = [max(-1.0, min(1.0, a)) for a in axes]
        self.buttons = list(buttons)
        self.last_joy = now
        c = self.config
        lt = self._button(c.button_lt) or self._trigger(c.axis_lt) >= c.emergency_trigger_threshold
        rt = self._button(c.button_rt) or self._trigger(c.axis_rt) >= c.emergency_trigger_threshold
        if lt and rt:
            self.estop_latched = True
            self._disarm('emergency_stop')
            self.previous_buttons = self.buttons.copy()
            return

        if self._edge(c.button_enable):
            if self.enabled:
                self._disarm('disabled_by_start')
            elif self.neutral_ready and self._neutral():
                self.enabled = True
                self.estop_latched = False
                self.last_auto = None
                self.reason = 'enabled'
            else:
                self.reason = 'release_controls_then_press_start'

        if self._edge(c.button_manual):
            self.mode = 'manual'
            self.last_auto = None
        elif self._edge(c.button_auto):
            self.mode = 'auto'
            self.last_auto = None
        slower = self._edge(c.button_slower)
        faster = self._edge(c.button_faster)
        if slower != faster:
            self.speed_level = max(0, min(len(c.speed_levels) - 1,
                                         self.speed_level + (1 if faster else -1)))

        if not self.enabled:
            self.neutral_ready = self._neutral() and not self._button(c.button_enable)
            if self.reason == 'waiting_for_joystick':
                self.reason = 'press_start' if self.neutral_ready else 'release_controls_then_press_start'
        self.previous_buttons = self.buttons.copy()

    def receive_auto(self, x: float, y: float, yaw: float, now: float):
        self._check_joy_timeout(now)
        if not self.enabled or self.mode != 'auto':
            return
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.last_auto = None
            self.auto = Command()
            self.reason = 'invalid_auto_command'
            return
        self.auto = Command(max(-self.config.auto_reverse_speed,min(self.config.auto_forward_speed,x)),
                            max(-self.config.auto_lateral_speed,min(self.config.auto_lateral_speed,y)),
                            max(-self.config.auto_angular_speed,min(self.config.auto_angular_speed,yaw)))
        self.last_auto = now

    def _limit(self, x, y, yaw, lift=0.0):
        c = self.config
        magnitude = math.hypot(x, y)
        if magnitude > c.max_linear_speed:
            x *= c.max_linear_speed / magnitude
            y *= c.max_linear_speed / magnitude
        return Command(x, y, max(-c.max_angular_speed, min(c.max_angular_speed, yaw)), lift)

    def output(self, now: float) -> Command:
        self._check_joy_timeout(now)
        if not self.enabled:
            return Command()
        if self.mode == 'auto':
            if self.last_auto is None or now - self.last_auto > self.config.auto_timeout:
                self.reason = 'waiting_for_fresh_auto_command'
                return Command()
            self.reason = 'auto_active'
            return self.auto

        c = self.config
        dpad_x, dpad_y = self._axis(c.axis_dpad_forward), self._axis(c.axis_dpad_left)
        x = dpad_x if dpad_x else self._axis(c.axis_forward) * c.sign_forward
        y = dpad_y if dpad_y else self._axis(c.axis_left) * c.sign_left
        yaw = self._axis(c.axis_yaw) * c.sign_yaw
        if not yaw:
            yaw = self._axis(c.axis_yaw_secondary) * c.sign_yaw_secondary
        lt, rt = self._trigger(c.axis_lt), self._trigger(c.axis_rt)
        left_arc = self._button(c.button_lt)
        right_arc = self._button(c.button_rt)
        arc_active = not yaw and bool(dpad_x) and left_arc != right_arc
        if arc_active:
            yaw = c.arc_yaw_fraction * dpad_x * (1.0 if left_arc else -1.0)
        magnitude = math.hypot(x, y)
        if magnitude > 1.0:
            x /= magnitude
            y /= magnitude
        speed = min(1.0, c.speed_levels[self.speed_level]
                    * (1.0 + c.rt_boost * rt) * (1.0 - c.lt_reduction * lt))
        if arc_active:
            # Mirror the established RT arc response for LT; LT braking would
            # otherwise make the same left arc three times slower at full pull.
            pull = lt if left_arc else rt
            speed = min(1.0, c.speed_levels[self.speed_level] * (1.0 + c.rt_boost * pull))
        yaw_speed = speed
        if c.stick_yaw_independent_gear and not arc_active:
            # Stick rotation uses its own ceiling; retain LT braking and RT cap.
            yaw_speed = min(1.0, (1.0 + c.rt_boost * rt) * (1.0 - c.lt_reduction * lt))
        lift = float(self._button(c.button_lift_up)) - float(self._button(c.button_lift_down))
        self.reason = 'manual_active'
        return self._limit(x * c.max_linear_speed * speed, y * c.max_linear_speed * speed,
                           yaw * c.max_angular_speed * yaw_speed, lift)

    def status(self, now: float):
        return {
            'enabled': self.enabled,
            'mode': self.mode,
            'estop_latched': self.estop_latched,
            'neutral_ready': self.neutral_ready,
            'speed_level': self.speed_level + 1,
            'speed_fraction': self.config.speed_levels[self.speed_level],
            'reason': self.reason,
            'joy_age_seconds': None if self.last_joy is None else round(now - self.last_joy, 3),
            'auto_age_seconds': None if self.last_auto is None else round(now - self.last_auto, 3),
            'required_axes': self.required_axes,
            'required_buttons': self.required_buttons,
        }
