"""Offline checks of ground commissioning limits and stationary interlock."""
from test_handset import armed, controls, feed, layout
from run_remote import manual_config
from vendor.navslam_core import TeleopCore
from run_ground_check import limited_config, stationary_core_class
import pytest


def test_ground_caps_cannot_be_raised_by_gear_or_trigger():
    core = TeleopCore(layout().chassis_config(limited_config(manual_config, .05, 1)))
    feed(core, 0);feed(core, .01, pressed=(7,));feed(core, .02)
    for i in range(12):
        feed(core, .03+i*.02)
        command = feed(core, .04+i*.02, {0: -32767, 1: -32767, 3: -32767, 5: 32767}, (5,))
        assert (command.x**2+command.y**2)**.5 <= .050000001
        assert abs(command.yaw) <= .040000001
    with pytest.raises(ValueError):limited_config(manual_config, .1, 1)


def test_stationary_phase_allows_enable_but_locks_motion_and_keeps_stop():
    core = stationary_core_class(TeleopCore)(layout().chassis_config(limited_config(manual_config, .05, 1)))
    feed(core, 0);feed(core, .01, pressed=(7,));feed(core, .02)
    assert core.enabled
    command = feed(core, .1, {0: -32767, 1: -32767, 3: -32767, 5: 32767})
    assert core.enabled and not any(vars(command).values())
    command = feed(core, .2, {2: 32767, 5: 32767})
    assert not core.enabled and core.estop_latched and not any(vars(command).values())
