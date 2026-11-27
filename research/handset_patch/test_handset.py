"""Offline commissioning checks; never open CAN or launch a hardware bridge."""
import json
import os
from pathlib import Path
import sys
import threading
import time

import pytest

ROOT = Path(os.environ['HANDSET_REMOTE_ROOT'])
# Mapping retains its original .2 rad/s ceiling scaled by the .5 first gear;
# independent teleop retains its original .4 rad/s stick yaw at every gear.
YAW_AT_FIRST_GEAR = .1 if ROOT.name == 'remote_control' else .4
sys.path[:0] = [str(ROOT/'program/remote'), str(ROOT/'lift_reference'), str(ROOT/'scripts')]
from joystick_layout import identify, XBOX_AXES, XBOX_BUTTONS, wait_initialized
from run_remote import manual_config
from vendor.navslam_core import TeleopCore
from lift_pv_remote import Joystick


def layout():
    return identify('Microsoft X-Box 360 pad', '045e', '028e', XBOX_AXES, XBOX_BUTTONS)


def controls(raw=None, pressed=()):
    # Physical Linux js samples, as measured on this controller.
    axes = [0, 0, -32767, 0, 0, -32767, 0, 0]
    for index, value in (raw or {}).items():
        axes[index] = value
    buttons = [int(i in pressed) for i in range(11)]
    return [-v/32767 for v in axes], buttons


def feed(core, t, raw=None, pressed=()):
    core.receive_joy(*controls(raw, pressed), t)
    return core.output(t)


def armed():
    core = TeleopCore(layout().chassis_config(manual_config(.1, 1)))
    feed(core, 0)
    feed(core, .01, pressed=(7,))
    feed(core, .02)
    assert core.enabled
    return core


@pytest.mark.parametrize('axis,raw,component,expected', [
    (1, -32767, 'x', .05), (1, 32767, 'x', -.05),
    (0, -32767, 'y', .05), (0, 32767, 'y', -.05),
    (3, -32767, 'yaw', YAW_AT_FIRST_GEAR), (3, 32767, 'yaw', -YAW_AT_FIRST_GEAR),
    (7, -32767, 'x', .05), (7, 32767, 'x', -.05),
    (6, -32767, 'y', .05), (6, 32767, 'y', -.05),
])
def test_physical_direction_and_release(axis, raw, component, expected):
    core = armed()
    command = feed(core, .1, {axis: raw})
    assert getattr(command, component) == pytest.approx(expected)
    assert all(getattr(command, k) == 0 for k in ('x', 'y', 'yaw', 'lift') if k != component)
    assert vars(feed(core, .2)) == dict(x=0., y=0., yaw=0., lift=0.)


def test_start_requires_release_and_neutral():
    core = TeleopCore(layout().chassis_config(manual_config(.1, 1)))
    feed(core, 0, pressed=(7,))
    feed(core, .01, pressed=(7,))
    assert not core.enabled
    feed(core, .02)
    feed(core, .03, {1: -32767}, (7,))
    assert not core.enabled
    feed(core, .04)
    feed(core, .05, pressed=(7,))
    assert core.enabled
    feed(core, .06)
    assert feed(core, .07, {1: -32767}, (7,)).x == 0
    assert not core.enabled


def test_gears_are_edges_and_bounded():
    core = armed()
    assert feed(core, .1, {1: -32767}, (5,)).x == pytest.approx(.1)
    assert feed(core, .2, {1: -32767}, (5,)).x == pytest.approx(.1)
    feed(core, .3)
    assert feed(core, .4, {1: -32767}, (4,)).x == pytest.approx(.05)
    assert feed(core, .5, {1: -32767}, (4,)).x == pytest.approx(.05)


def test_single_triggers_do_not_rotate_or_stop_both_stops():
    core = armed()
    for t, axis in ((.1, 2), (.2, 5)):
        command = feed(core, t, {axis: 32767})
        assert core.enabled and not core.estop_latched
        assert vars(command) == dict(x=0., y=0., yaw=0., lift=0.)
    command = feed(core, .3, {1: -32767, 2: 32767, 5: 32767})
    assert not core.enabled and core.estop_latched
    assert vars(command) == dict(x=0., y=0., yaw=0., lift=0.)
    feed(core, .4)
    assert not core.enabled  # Releasing the stop does not resume motion.


def test_ignored_buttons_do_not_enable_stop_or_lift():
    core = armed()
    for i, button in enumerate((1, 2, 3, 6, 8, 9, 10)):
        command = feed(core, .03+i*.02, pressed=(button,))
        assert core.enabled and not core.estop_latched
        assert vars(command) == dict(x=0., y=0., yaw=0., lift=0.)
    feed(core, .3, pressed=(9, 10))
    assert core.enabled  # Stick clicks are not legacy LT/RT digital buttons.


def test_timeout_and_mapping_gate_require_rearming():
    core = armed()
    feed(core, .1, {1: -32767})
    assert core.output(.7).x == 0 and not core.enabled
    feed(core, .71, pressed=(7,))
    assert not core.enabled
    core = armed()
    core._disarm('mapping_static_or_not_ready')
    feed(core, .1, {1: -32767})
    assert not core.enabled
    feed(core, .2)
    feed(core, .3, pressed=(7,))
    assert core.enabled


def test_layout_rejects_unknown_or_changed_xbox_descriptor():
    with pytest.raises(RuntimeError):
        identify('unknown', '1234', '5678', XBOX_AXES, XBOX_BUTTONS)
    with pytest.raises(RuntimeError):
        identify('Microsoft X-Box 360 pad', '045e', '028e', XBOX_AXES, XBOX_BUTTONS+(319,))
    with pytest.raises(RuntimeError):
        identify('Microsoft X-Box 360 pad', '045e', '028e', (1, 0, *XBOX_AXES[2:]), XBOX_BUTTONS)


def test_legacy_dinput_mapping_preserved():
    profile = identify('legacy', '1234', '5678', range(8), range(12))
    before = manual_config(.1, 1)
    after = profile.chassis_config(before)
    for name, value in vars(before).items():
        if name not in ('button_lift_up', 'button_lift_down'):
            assert getattr(after, name) == value
    assert after.button_lift_up == after.button_lift_down == -1


def test_live_reader_stop_and_error_paths_without_device():
    reader = Joystick.__new__(Joystick)
    reader.mutex = threading.Lock()
    reader.layout = layout()
    reader.error = None
    reader.sample = (time.monotonic(), *controls(pressed=(9, 10)), True)
    assert reader.get() is not None
    reader.sample = (time.monotonic(), *controls({2: 32767, 5: 32767}), True)
    with pytest.raises(RuntimeError, match='dual trigger'):
        reader.get()
    reader.sample = (time.monotonic()-1, *controls(), True)
    with pytest.raises(RuntimeError, match='heartbeat'):
        reader.get()
    reader.error = 'joystick disconnected'
    with pytest.raises(RuntimeError, match='disconnected'):
        reader.get()


def test_initialization_failure_never_starts_can_bridge(monkeypatch, tmp_path):
    import remote_session as session
    class BadJoystick:
        def __init__(self, *args, **kwargs):
            self.done = threading.Event()
            self.thread = threading.Thread(target=lambda: None)
            self.thread.start()
        def get(self):
            raise RuntimeError('injected incomplete device')
    monkeypatch.setattr(session, 'ROOT', tmp_path)
    (tmp_path/'config').mkdir()
    (tmp_path/'config/defaults.json').write_bytes((ROOT/'config/defaults.json').read_bytes())
    (tmp_path/'program/build').mkdir(parents=True)
    (tmp_path/'program/build/remote_bridge').touch()
    original_exists = Path.exists
    monkeypatch.setattr(Path, 'exists', lambda p: True if str(p)=='/dev/input/js0' else original_exists(p))
    monkeypatch.setattr(session, 'detect_device', layout)
    monkeypatch.setattr(session, 'Joystick', BadJoystick)
    monkeypatch.setattr(session.subprocess, 'Popen', lambda *a, **k: pytest.fail('CAN bridge must not start'))
    monkeypatch.setattr(session.signal, 'signal', lambda *a: None)
    monkeypatch.setattr(sys, 'argv', ['remote_session.py', '--chassis-only', '--log-dir', str(tmp_path/'logs')])
    assert session.main() == 1
    assert json.loads((tmp_path/'logs/exit.json').read_text())['exit_code'] == 1


def test_initialization_timeout():
    class NoInput:
        def get(self): return None
    with pytest.raises(RuntimeError, match='初始化超时'):
        wait_initialized(NoInput(), timeout=.01)
