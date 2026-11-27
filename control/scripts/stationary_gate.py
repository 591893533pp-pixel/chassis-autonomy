"""Bounded, fresh multi-sample zero-motion evidence; no hardware access.

Retain the original mean-speed limits (wheels 0.1 rad/s, lift 10 counts/s).
A single quantized sample is not proof of motion. Average absolute speeds
over at most 200 ms, requiring 150 ms of evidence. Never cancel opposite
velocities. Three times the mean limit rejects immediately. Invalid data,
stale data and any nonzero command clear all evidence. An optional release
limit adds hysteresis only after zero motion has been established.
"""
from collections import deque
import math


class StationaryGate:
    def __init__(self, limit, dimensions=1, window=.20, minimum_span=.15, release_limit=None):
        self.limit = limit
        self.release_limit = limit if release_limit is None else release_limit
        self.settled = False
        self.dimensions = dimensions
        self.window = window
        self.minimum_span = minimum_span
        self.samples = deque()
        self.reason = 'awaiting_feedback'

    def reset(self, reason='nonzero_command'):
        self.samples.clear()
        self.settled = False
        self.reason = reason
        return False

    def observe(self, stamp, velocities, now, valid=True, zero_command=True):
        if (not valid or len(velocities) != self.dimensions
                or not all(math.isfinite(v) for v in (stamp, now, *velocities))):
            return self.reset('invalid_feedback')
        if not zero_command:
            return self.reset('nonzero_command')
        if not 0 <= now - stamp < self.window:
            return self.reset('stale_feedback')
        speeds = tuple(abs(v) for v in velocities)
        if max(speeds) >= 3 * self.limit:
            return self.reset('large_motion_sample')
        if self.samples and stamp < self.samples[-1][0]:
            return self.reset('timestamp_regression')
        if not self.samples or stamp > self.samples[-1][0]:
            self.samples.append((stamp, speeds))
        # Evidence span follows sensor samples; polling the same fresh sample
        # must not alternately expire/recreate the settled state.
        while self.samples and stamp - self.samples[0][0] > self.window:
            self.samples.popleft()
        if len(self.samples) < 3 or self.samples[-1][0] - self.samples[0][0] < self.minimum_span:
            self.settled = False
            self.reason = 'settling'
            return False
        mean = max(sum(s[1][i] for s in self.samples) / len(self.samples)
                   for i in range(self.dimensions))
        threshold = self.release_limit if self.settled else self.limit
        self.settled = mean <= threshold
        self.reason = 'stationary' if self.settled else 'sustained_motion'
        return self.reason == 'stationary'
