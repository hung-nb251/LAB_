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

    def __init__(self, time_constant_sec=0.0, lead_sec=0.0, max_lead_m=0.02,
                 velocity_tau_sec=0.15):
        self.time_constant = float(time_constant_sec)
        if not math.isfinite(self.time_constant) or self.time_constant < 0:
            raise ValueError('prediction_reference_tau_sec must be finite and non-negative')
        self.lead_sec = float(lead_sec)
        self.max_lead_m = float(max_lead_m)
        self.velocity_tau_sec = float(velocity_tau_sec)
        if (not all(math.isfinite(x) for x in
                    (self.lead_sec, self.max_lead_m, self.velocity_tau_sec))
                or self.lead_sec < 0 or self.max_lead_m < 0 or self.velocity_tau_sec <= 0):
            raise ValueError('Lead/distance must be non-negative; velocity tau must be positive')
        self.position = None
        self.time = None
        self.velocity = np.zeros(3)
        self.output = None

    def reset(self, position, now):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('Prediction reference must contain finite XYZ')
        if not math.isfinite(now):
            raise ValueError('Reference time must be finite')
        self.position = position.copy()
        self.time = float(now)
        self.velocity[:] = 0.0
        self.output = self.position.copy()

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
            return self.output.copy()
        # Do not jump across a stalled executor interval.
        elapsed = float(now) - self.time
        dt = min(elapsed, 0.1)
        alpha = -math.expm1(-dt / self.time_constant)
        increment = alpha * (desired - self.position)
        self.position += increment
        # Estimate velocity from the filtered trajectory, never raw GRU
        # differences. A second low-pass limits noise in the lead correction.
        beta = -math.expm1(-dt / self.velocity_tau_sec)
        self.velocity += beta * (increment / dt - self.velocity)
        if elapsed > 0.1:
            self.velocity[:] = 0.0
        correction = self.lead_sec * self.velocity
        length = float(np.linalg.norm(correction))
        if length > self.max_lead_m:
            correction *= self.max_lead_m / length
        # A stopped/reversed nominal cannot be overshot by the lead term.
        # Downstream command velocity/acceleration/lead bounds still apply.
        residual = desired - self.position
        correction = np.clip(correction, np.minimum(0.0, residual),
                             np.maximum(0.0, residual))
        self.output = self.position + correction
        self.time = float(now)
        return self.output.copy()
