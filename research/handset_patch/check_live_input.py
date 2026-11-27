"""Use the installed production input reader, without creating any CAN bridge."""
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

root = Path(sys.argv[1])
sys.path[:0] = [str(root/'program/remote'), str(root/'lift_reference')]
from joystick_layout import detect_device, wait_initialized
from lift_pv_remote import Joystick
from run_remote import manual_config
from vendor.navslam_core import TeleopCore

def reject(*args, **kwargs):
    raise AssertionError('No sockets or subprocesses are allowed during this check')

socket.socket = reject
subprocess.Popen = reject
lock = open(f'/tmp/poke-remote-{os.getuid()}.lock', 'a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
layout = detect_device()
folder = Path('/home/pc/handset_patch_20261124') / (
    'live_mapping' if root.name == 'remote_control' else 'live_independent')
folder.mkdir(exist_ok=True)
config = layout.chassis_config(manual_config(.1, 1))
core = TeleopCore(config)
joy = Joystick(folder, layout=layout)
try:
    sample = wait_initialized(joy)
    # No enable injection: the real device must remain physically neutral.
    for _ in range(100):
        sample = joy.get()
        core.receive_joy(sample[1], sample[2], time.monotonic())
        command = core.output(time.monotonic())
        assert not core.enabled, 'START was pressed during the neutral check'
        assert not any(vars(command).values())
        time.sleep(.02)
    report = dict(layout=vars(layout), config=vars(config),
                  status=core.status(time.monotonic()), axes=sample[1], buttons=sample[2],
                  hardware_control_started=False)
    assert report['status']['reason'] == 'press_start'
    assert report['status']['neutral_ready']
    (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
finally:
    joy.done.set()
    joy.thread.join(timeout=1)
    assert not joy.thread.is_alive()
