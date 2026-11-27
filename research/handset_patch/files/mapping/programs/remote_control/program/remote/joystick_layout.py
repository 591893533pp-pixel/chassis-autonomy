"""Verified chassis joystick layouts. No CAN access or motion side effects."""
import array
import dataclasses
import fcntl
import os
import time
from pathlib import Path

XBOX_AXES = (0, 1, 2, 3, 4, 5, 16, 17)
XBOX_BUTTONS = (304, 305, 307, 308, 310, 311, 314, 315, 316, 317, 318)


@dataclasses.dataclass(frozen=True)
class Layout:
    profile: str
    name: str
    vendor: str
    product: str
    axes: tuple
    buttons: tuple

    def validate_fd(self, fd):
        if detect_fd(fd) != self:
            raise RuntimeError('手柄设备或布局在启动过程中发生变化，请重新检查')

    def chassis_config(self, config):
        overrides = dict(button_auto=-1, button_lift_up=-1, button_lift_down=-1)
        if self.profile == 'flydigi_xinput':
            overrides.update(axis_yaw=3, axis_lt=2, axis_rt=5,
                             button_enable=7, button_slower=4, button_faster=5,
                             button_manual=0, button_lt=-1, button_rt=-1)
        # Readers already convert Linux js axes to -value/32767. Keep trigger
        # rest=+1, pressed=-1; a second inversion would reverse the stop input.
        result = dataclasses.replace(config, **overrides)
        result.validate()
        return result

    def dual_trigger(self, axes, buttons):
        if self.profile == 'flydigi_xinput':
            return axes[2] < -.6 and axes[5] < -.6
        return ((axes[4] < -.6 and axes[5] < -.6)
                or (buttons[8] and buttons[9]))


def identify(name, vendor, product, axes, buttons):
    axes, buttons = tuple(axes), tuple(buttons)
    if (vendor, product) == ('045e', '028e'):
        if (name != 'Microsoft X-Box 360 pad'
                or axes != XBOX_AXES or buttons != XBOX_BUTTONS):
            raise RuntimeError('Xbox 手柄描述符与已验证布局不符，拒绝套用映射')
        profile = 'flydigi_xinput'
    elif len(axes) == 8 and len(buttons) >= 12:
        # Retain the existing DINPUT acceptance rule for original hardware.
        profile = 'legacy_dinput'
    else:
        raise RuntimeError(f'未验证的手柄布局：{name}, {len(axes)}轴/{len(buttons)}键')
    return Layout(profile, name, vendor, product, axes, buttons)


def detect_fd(fd):
    na, nb, name = bytearray(1), bytearray(1), bytearray(128)
    axes, buttons = bytearray(64), array.array('H', [0] * 512)
    for code, buf in ((0x80016a11, na), (0x80016a12, nb), (0x80806a13, name),
                      (0x80406a32, axes), (0x84006a34, buttons)):
        fcntl.ioctl(fd, code, buf, True)
    dev = os.fstat(fd).st_rdev
    identity = Path(f'/sys/dev/char/{os.major(dev)}:{os.minor(dev)}/device/id')
    return identify(name.split(b'\0')[0].decode('utf-8', 'replace'),
                    (identity/'vendor').read_text().strip().lower(),
                    (identity/'product').read_text().strip().lower(),
                    axes[:na[0]], buttons[:nb[0]])


def detect_device(path='/dev/input/js0'):
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        return detect_fd(fd)
    finally:
        os.close(fd)


def wait_initialized(joystick, timeout=2.0):
    """Fail before creating a CAN bridge if input is missing or malformed."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = joystick.get()
        if sample is not None and sample[3]:
            return sample
        time.sleep(.01)
    raise RuntimeError('手柄初始化超时，未启动底盘控制桥')
