"""Condition a learned nominal before coupling it to admittance feedback.

This does not change or relabel the raw model output.  A geometric lead bound
alone does not limit the bandwidth of a robot-EE-fed prediction loop.
"""
import math

import numpy as np


class PredictionReference:
    """First-order nominal tracking; zero time constant is legacy passthrough.

    Call with monotonic seconds, once per control tick. Reset on run/role
    realignment and freeze during force HOLD. This is not a safety limiter;
    workspace, lead, velocity and force watchdogs remain downstream.
    """

    def __init__(self, time_constant_sec=0.0):
        self.time_constant = float(time_constant_sec)
        if not math.isfinite(self.time_constant) or self.time_constant < 0:
            raise ValueError('prediction_reference_tau_sec must be finite and non-negative')
        self.position = None
        self.time = None

    def reset(self, position, now):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('Prediction reference must contain finite XYZ')
        if not math.isfinite(now):
            raise ValueError('Reference time must be finite')
        self.position = position.copy()
        self.time = float(now)

    def step(self, desired, now):
        desired = np.asarray(desired, dtype=float)
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            raise ValueError('Prediction reference must contain finite XYZ')
        if not math.isfinite(now):
            raise ValueError('Reference time must be finite')
        if self.position is None or self.time_constant == 0.0:
            self.reset(desired, now)
            return self.position.copy()
        if now <= self.time:
            return self.position.copy()
        # Do not jump across a stalled executor interval.
        dt = min(float(now) - self.time, 0.1)
        alpha = -math.expm1(-dt / self.time_constant)
        self.position += alpha * (desired - self.position)
        self.time = float(now)
        return self.position.copy()
