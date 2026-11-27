"""Safety behavior and tutorial controls tested without ROS or a real controller."""

import math

import pytest

from navslam_core import Command, Config, TeleopCore


NEUTRAL = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]


def joy(core, now, axes=None, pressed=()):
    buttons = [0] * 20
    for index in pressed:
        buttons[index] = 1
    core.receive_joy(NEUTRAL.copy() if axes is None else axes, buttons, now)


def arm(core, now=0.0):
    joy(core, now)
    joy(core, now + 0.01, pressed=(11,))
    joy(core, now + 0.02)
    assert core.enabled


def axis(index, value):
    axes = NEUTRAL.copy()
    axes[index] = value
    return axes


def test_start_held_at_startup_does_not_arm_and_start_is_edge_triggered():
    core = TeleopCore()
    joy(core, 0.0, pressed=(11,))
    joy(core, 0.01, pressed=(11,))
    assert not core.enabled
    joy(core, 0.02)
    joy(core, 0.03, pressed=(11,))
    joy(core, 0.04, pressed=(11,))
    assert core.enabled
    joy(core, 0.05)
    joy(core, 0.06, pressed=(11,))
    assert core.output(0.06) == Command()
    assert not core.enabled


def test_non_neutral_start_rejected_even_after_previous_neutral_packet():
    core = TeleopCore()
    joy(core, 0.0)
    joy(core, 0.01, axis(1, 1.0), (11,))
    assert not core.enabled
    joy(core, 0.02, axis(1, 1.0))
    joy(core, 0.03, axis(1, 1.0), (11,))
    assert core.output(0.03) == Command()
    arm(core, 0.04)


def test_translation_yaw_and_dpad_priority_and_diagonal_limit():
    core = TeleopCore()
    arm(core)
    axes = NEUTRAL.copy()
    axes[0], axes[1], axes[2] = 1.0, -1.0, 1.0
    axes[7] = 1.0  # Dpad overrides backwards stick on the forward coordinate.
    joy(core, 0.1, axes)
    cmd = core.output(0.1)
    assert cmd.x == pytest.approx(0.24 / math.sqrt(2))
    assert cmd.y == pytest.approx(cmd.x)
    assert math.hypot(cmd.x, cmd.y) == pytest.approx(0.24)
    assert cmd.yaw == pytest.approx(0.48)


def test_secondary_yaw_is_opt_in_and_horizontal_has_priority():
    core = TeleopCore()
    arm(core)
    joy(core, 0.1, axis(3, 1.0))
    assert core.output(0.1).yaw == 0.0
    core = TeleopCore(Config(axis_yaw_secondary=3))
    arm(core)
    joy(core, 0.1, axis(3, 1.0))
    assert core.output(0.1).yaw > 0.0
    axes = axis(3, 1.0)
    axes[2] = -1.0
    joy(core, 0.2, axes)
    assert core.output(0.2).yaw < 0.0


def test_speed_buttons_do_not_repeat_while_held():
    core = TeleopCore()
    arm(core)
    for i in range(10):
        joy(core, 0.03 + i * 0.01, pressed=(7,))
    assert core.speed_level == 2
    joy(core, 0.2)
    joy(core, 0.21, pressed=(6,))
    assert core.speed_level == 1
    joy(core, 0.22)
    joy(core, 0.23, pressed=(6, 7))
    assert core.speed_level == 1


@pytest.mark.parametrize('use_buttons', [False, True])
def test_dual_trigger_brake_latches_until_neutral_and_new_start(use_buttons):
    core = TeleopCore()
    arm(core)
    joy(core, 0.1, axis(1, 1.0))
    assert core.output(0.1).x > 0
    axes = NEUTRAL.copy()
    if not use_buttons:
        axes[4] = axes[5] = -1.0
    joy(core, 0.2, axes, (8, 9, 11) if use_buttons else (11,))
    assert core.estop_latched
    assert core.output(0.2) == Command()
    joy(core, 0.3)
    joy(core, 0.31, axis(1, 1.0))
    assert core.output(0.31) == Command()
    arm(core, 0.32)
    assert not core.estop_latched


def test_joystick_timeout_requires_new_enable_and_discards_auto():
    core = TeleopCore()
    arm(core)
    joy(core, 0.1, pressed=(0,))
    core.receive_auto(0.2, 0.0, 0.0, 0.11)
    assert core.output(0.12).x == 0.2
    assert core.output(0.61) == Command()
    assert not core.enabled
    assert core.last_auto is None
    joy(core, 0.62, pressed=(11,))  # START held on reconnect cannot enable.
    assert not core.enabled
    arm(core, 0.63)
    assert core.output(0.66) == Command()
    core.receive_auto(0.2, 0.0, 0.0, 0.67)
    assert core.output(0.68).x == 0.2


def test_timeout_detected_when_next_joy_arrives_before_timer():
    core = TeleopCore()
    arm(core)
    joy(core, 1.0, axis(1, 1.0), (11,))
    assert not core.enabled
    assert core.output(1.0) == Command()


def test_auto_needs_post_switch_command_and_stops_on_auto_timeout():
    core = TeleopCore()
    arm(core)
    core.receive_auto(0.4, 0.0, 0.0, 0.03)
    joy(core, 0.04, pressed=(0,))
    assert core.output(0.04) == Command()
    core.receive_auto(100.0, 100.0, -100.0, 0.05)
    cmd = core.output(0.05)
    assert (cmd.x,cmd.y,cmd.yaw) == (0.7,0.25,-0.4)
    joy(core, 0.4)
    assert core.output(0.56) == Command()
    assert core.enabled
    core.receive_auto(0.3, 0.0, 0.0, 0.57)
    assert core.output(0.57).x == 0.3
    joy(core, 0.58, pressed=(1,))
    assert core.mode == 'manual'
    joy(core, 0.59)
    joy(core, 0.60, pressed=(0,))
    assert core.output(0.60) == Command()


@pytest.mark.parametrize('bad_axes,bad_buttons', [
    ([0.0], [0] * 20),
    (NEUTRAL, [0] * 3),
    (axis(1, float('nan')), [0] * 20),
    (axis(1, float('inf')), [0] * 20),
    (axis(1, 2.0), [0] * 20),
    (NEUTRAL, [2] * 20),
])
def test_invalid_joy_disarms(bad_axes, bad_buttons):
    core = TeleopCore()
    arm(core)
    core.receive_joy(bad_axes, bad_buttons, 0.1)
    assert core.output(0.1) == Command()
    assert not core.enabled
    assert core.reason == 'invalid_joystick_layout'


def test_nonfinite_auto_stops_immediately():
    core = TeleopCore()
    arm(core)
    joy(core, 0.1, pressed=(0,))
    core.receive_auto(0.2, 0.0, 0.0, 0.11)
    core.receive_auto(float('nan'), 0.0, 0.0, 0.12)
    assert core.output(0.12) == Command()


def test_lift_only_operates_while_enabled_in_manual():
    core = TeleopCore()
    joy(core, 0.0, pressed=(3,))
    assert core.output(0.0).lift == 0.0
    arm(core, 0.01)
    joy(core, 0.1, pressed=(3,))
    assert core.output(0.1).lift == 1.0
    joy(core, 0.2, pressed=(4,))
    assert core.output(0.2).lift == -1.0
    joy(core, 0.3, pressed=(3, 4))
    assert core.output(0.3).lift == 0.0
    joy(core, 0.4, pressed=(0, 3))
    assert core.output(0.4).lift == 0.0


def test_trigger_scaling_and_arc_precedence():
    core = TeleopCore()
    arm(core)
    axes = axis(7, 1.0)
    axes[5] = -1.0  # LT arc now mirrors RT arc speed.
    joy(core, 0.1, axes, (8,))
    command = core.output(0.1)
    assert command.x == pytest.approx(0.36)
    assert command.yaw == pytest.approx(0.432)
    axes[7] = -1.0
    joy(core, 0.2, axes, (8,))
    assert core.output(0.2).yaw < 0
    axes[2] = 1.0  # Right joystick rotation overrides the arc.
    joy(core, 0.3, axes, (8,))
    assert core.output(0.3).yaw == pytest.approx(0.24)
    axes = axis(1, 1.0)
    axes[4] = -1.0
    joy(core, 0.4, axes, (9,))
    assert core.output(0.4).x == pytest.approx(0.36)
    assert core.output(0.4).yaw == 0.0  # Arc requires forward/back Dpad.


@pytest.mark.parametrize('direction', [-1., 1.])
@pytest.mark.parametrize('pull', [0., .25, .5, 1.])
def test_left_right_arcs_are_mirrored(direction, pull):
    commands = []
    for trigger_axis, button in [(5, 8), (4, 9)]:
        core = TeleopCore()
        arm(core)
        axes = axis(7, direction)
        axes[trigger_axis] = 1. - 2. * pull
        joy(core, .1, axes, (button,))
        commands.append(core.output(.1))
    assert commands[0].x == pytest.approx(commands[1].x)
    assert commands[0].yaw == pytest.approx(-commands[1].yaw)


def test_lt_still_brakes_normal_stick_motion():
    core = TeleopCore()
    arm(core)
    axes = axis(1, 1.)
    axes[5] = -1.
    joy(core, .1, axes, (8,))
    assert core.output(.1).x == pytest.approx(.12)


@pytest.mark.parametrize('config', [
    Config(button_enable=-1), Config(axis_yaw=-2), Config(joy_timeout=0.0),
    Config(deadzone=0.2), Config(max_linear_speed=-1.0),
    Config(speed_levels=[0.5, 0.1]), Config(trigger_rest=-1.0),
])
def test_invalid_configuration_rejected(config):
    with pytest.raises(ValueError):
        TeleopCore(config)


def test_auto_cruise_is_not_clamped_by_manual_speed_limit():
    core=TeleopCore();arm(core);joy(core,.04,pressed=(0,))
    core.receive_auto(.7,.25,.4,.05)
    assert core.output(.05)==Command(.7,.25,.4)
    core.receive_auto(-.7,-.7,-.7,.06)
    assert core.output(.06)==Command(-.25,-.25,-.4)
