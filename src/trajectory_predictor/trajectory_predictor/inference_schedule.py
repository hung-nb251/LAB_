"""Deadline-based inference scheduling for periodically sampled observations."""

import math


class InferenceSchedule:
    """Limit requests without discarding every slightly early sensor callback.

    Use monotonic seconds. Small early arrivals consume the upcoming deadline;
    the next deadline remains on the same grid. A late callback sends at most
    one request and skips missed deadlines, never replaying a backlog.
    """

    def __init__(self, rate_hz):
        rate_hz = float(rate_hz)
        if not math.isfinite(rate_hz) or rate_hz < 0.0:
            raise ValueError('inference_rate_hz must be finite and non-negative')
        self.period = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self.tolerance = min(0.005, self.period * 0.1)
        self.reset()

    def reset(self):
        self.next_due = None

    def ready(self, now):
        if self.period == 0.0:
            return True  # Historical camera profile: every input sample.
        if self.next_due is None:
            self.next_due = now + self.period
            return True
        if now < self.next_due - self.tolerance:
            return False
        if now >= self.next_due + self.period:
            self.next_due = now + self.period
        else:
            self.next_due += self.period
        return True
