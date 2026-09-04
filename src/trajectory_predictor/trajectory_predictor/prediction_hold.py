"""Time-based stationary detector for prediction HOLD output.

The robot-EE callback rate is not guaranteed to match the model rate.  Using a
fixed number of callbacks therefore makes HOLD timing change with the source
frequency.  This helper uses elapsed time and physical displacement instead.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class TimeBasedPredictionHold:
    """Detect a stationary XYZ signal without depending on callback rate."""

    def __init__(self, window_sec, max_motion_m, release_distance_m):
        self.window_sec = float(window_sec)
        self.max_motion_m = float(max_motion_m)
        self.release_distance_m = float(release_distance_m)
        if self.window_sec <= 0.0:
            raise ValueError('window_sec must be positive')
        if self.max_motion_m <= 0.0:
            raise ValueError('max_motion_m must be positive')
        if self.release_distance_m <= self.max_motion_m:
            raise ValueError('release_distance_m must exceed max_motion_m')
        self._samples = deque()
        self.active = False
        self.reference = None
        self.target = None

    def reset(self):
        self._samples.clear()
        self.active = False
        self.reference = None
        self.target = None

    def update(self, position, timestamp_sec):
        """Return ``(state, target)`` where state is tracking/entered/hold/released."""
        position = np.asarray(position, dtype=float)
        if position.shape != (3,):
            raise ValueError('position must contain XYZ values')
        now = float(timestamp_sec)
        self._samples.append((now, position.copy()))
        cutoff = now - self.window_sec
        while len(self._samples) > 1 and self._samples[0][0] < cutoff:
            self._samples.popleft()

        if self.active:
            deviation = float(np.linalg.norm(position - self.reference))
            if deviation > self.release_distance_m:
                self.active = False
                self.reference = None
                self.target = None
                self._samples.clear()
                self._samples.append((now, position.copy()))
                return 'released', None
            return 'hold', self.target.copy()

        if len(self._samples) < 2:
            return 'tracking', None
        covered = self._samples[-1][0] - self._samples[0][0]
        if covered < self.window_sec * 0.9:
            return 'tracking', None

        points = np.asarray([sample[1] for sample in self._samples])
        center = np.mean(points, axis=0)
        max_motion = float(np.max(np.linalg.norm(points - center, axis=1)))
        if max_motion > self.max_motion_m:
            return 'tracking', None

        # HOLD must represent where the robot actually is, not the previous
        # model output that may already be ahead of the robot.
        self.active = True
        self.reference = position.copy()
        self.target = position.copy()
        return 'entered', self.target.copy()
