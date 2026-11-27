"""Offline preparation: expiring permission, NOT a motor controller.

The future supervisor must refresh this only after all live health checks pass.
The control loop must check it before every command and disarm on False.
Not connected to the deployed controller yet; never use file existence alone.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import time

TTL_SECONDS = 0.25


def publish(path, session, boot_id, *, now=None):
    """Atomically refresh an already health-qualified session's lease."""
    now = time.monotonic() if now is None else now
    if not session or not boot_id or not math.isfinite(now) or now < 0:
        raise ValueError('Invalid session, boot ID, or monotonic time')
    path = Path(path)
    payload = {'version': 1, 'session': session, 'boot_id': boot_id,
               'monotonic': now, 'allowed': True}
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(payload, stream, allow_nan=False)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


class MotionLease:
    """Fail closed and latch any failure after the first valid permission.

    Recovery requires a new session; this gate never enables motors or replaces
    neutral/START logic, CAN feedback checks, or the bridge's input watchdog.
    """
    def __init__(self, path, session, boot_id):
        if not session or not boot_id:
            raise ValueError('Session and boot ID are required')
        self.path, self.session, self.boot_id = Path(path), session, boot_id
        self.qualified = False
        self.latched = False
        self.reason = 'waiting_for_health'

    def check(self, *, now=None):
        if self.latched:
            return False
        now = time.monotonic() if now is None else now
        reason = 'invalid_lease'
        try:
            with self.path.open() as stream:
                encoded = stream.read(4097)
            if len(encoded) > 4096:
                raise ValueError('Oversized lease')
            data = json.loads(encoded)
            if not isinstance(data, dict):
                raise ValueError('Invalid object')
            stamp = data.get('monotonic')
            valid_stamp = (type(stamp) in (int, float) and math.isfinite(stamp)
                           and math.isfinite(now) and stamp >= 0
                           and 0 <= now - stamp < TTL_SECONDS)
            if (type(data.get('version')) is int and data['version'] == 1
                    and data.get('session') == self.session
                    and data.get('boot_id') == self.boot_id
                    and data.get('allowed') is True and valid_stamp):
                self.qualified = True
                self.reason = 'ok'
                return True
            reason = 'expired_or_mismatched_lease'
        except (OSError, ValueError, TypeError, OverflowError):
            pass
        self.reason = reason
        self.latched = self.qualified
        return False
