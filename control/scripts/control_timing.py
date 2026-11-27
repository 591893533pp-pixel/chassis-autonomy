"""Monotonic deadlines, skipping missed slots instead of issuing catch-up bursts."""
import time

CONTROL_HZ = 100
PERIOD_S = 1 / CONTROL_HZ

class Periodic:
    def __init__(self, period=PERIOD_S, clock=time.monotonic, sleep=time.sleep, late_tolerance=0.):
        if not 0 <= late_tolerance < period:
            raise ValueError("late tolerance must be below one period")
        self.late_tolerance = late_tolerance
        self.late_cycles = 0
        self.period = period
        self.clock = clock
        self.sleep = sleep
        self.next = clock() + period
        self.missed = 0

    def wait(self):
        now = self.clock()
        if now > self.next:
            self.late_cycles += 1
        if now > self.next + self.late_tolerance:
            skipped = int((now - self.next) / self.period) + 1
            self.missed += skipped
            self.next += skipped * self.period
        self.sleep(max(0., self.next - now))
        self.next += self.period
