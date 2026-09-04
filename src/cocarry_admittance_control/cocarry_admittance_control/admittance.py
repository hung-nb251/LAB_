"""Pure numerical core for Cartesian admittance control."""

from __future__ import annotations

import numpy as np


def critical_damping(mass, stiffness):
    """Return D = 2*sqrt(M*K) for diagonal M and K."""
    mass = np.asarray(mass, dtype=float)
    stiffness = np.asarray(stiffness, dtype=float)
    if mass.shape != stiffness.shape:
        raise ValueError('Mass and stiffness must have the same shape')
    if np.any(mass <= 0.0) or np.any(stiffness < 0.0):
        raise ValueError('Mass must be positive and stiffness non-negative')
    return 2.0 * np.sqrt(mass * stiffness)


def nominal_reference(capture_position, mode, predicted_relative=None):
    """Select x_d from trajectory mode without changing admittance gains."""
    if capture_position is None:
        return None
    capture = np.asarray(capture_position, dtype=float)
    if capture.shape != (3,):
        raise ValueError('Captured position must contain XYZ values')
    if mode == 'ground_truth':
        return capture.copy()
    if mode == 'prediction':
        if predicted_relative is None:
            return None
        prediction = np.asarray(predicted_relative, dtype=float)
        if prediction.shape != (3,):
            raise ValueError('Predicted relative position must contain XYZ values')
        return capture + prediction
    raise ValueError(f'Unsupported trajectory mode: {mode}')


def minimum_safe_ee_z(minimum_tip_z, downward_tool_length):
    """Convert a floor clearance for a downward tool into an EE Z bound."""
    minimum_tip_z = float(minimum_tip_z)
    downward_tool_length = float(downward_tool_length)
    if downward_tool_length < 0.0:
        raise ValueError('Downward tool length must be non-negative')
    return minimum_tip_z + downward_tool_length


def commanded_position(nominal, admittance_error, leader=False):
    """Select the Cartesian command for FOLLOWER or direct-MJM LEADER."""
    nominal = np.asarray(nominal, dtype=float)
    error = np.asarray(admittance_error, dtype=float)
    if nominal.shape != (3,) or error.shape != (3,):
        raise ValueError('Nominal and admittance error must contain XYZ values')
    return nominal.copy() if leader else nominal + error


def fresh_mjm_sample(source, sample_time, now, timeout):
    """Return whether an MJM sample is fresh enough for atomic role handoff."""
    timeout = float(timeout)
    if timeout <= 0.0:
        raise ValueError('timeout must be positive')
    return (
        str(source).strip().lower() == 'mjm'
        and float(sample_time) > 0.0
        and 0.0 <= float(now) - float(sample_time) <= timeout
    )


def limit_position_lead(desired, actual, maximum_distance):
    """Limit a desired XYZ point to a sphere around measured XYZ."""
    desired = np.asarray(desired, dtype=float)
    actual = np.asarray(actual, dtype=float)
    maximum_distance = float(maximum_distance)
    if desired.shape != (3,) or actual.shape != (3,):
        raise ValueError('Desired and actual positions must contain XYZ values')
    if maximum_distance <= 0.0:
        raise ValueError('maximum_distance must be positive')
    delta = desired - actual
    distance = float(np.linalg.norm(delta))
    if distance <= maximum_distance or distance == 0.0:
        return desired.copy(), False, distance
    return actual + delta * (maximum_distance / distance), True, distance


def force_watchdog_action(age_sec, stale_hold_sec, timeout_sec):
    """Classify force freshness as ``fresh``, ``hold`` or ``fault``."""
    age_sec = float(age_sec)
    stale_hold_sec = float(stale_hold_sec)
    timeout_sec = float(timeout_sec)
    if stale_hold_sec <= 0.0 or timeout_sec <= stale_hold_sec:
        raise ValueError('Require 0 < stale_hold_sec < timeout_sec')
    if age_sec > timeout_sec:
        return 'fault'
    if age_sec > stale_hold_sec:
        return 'hold'
    return 'fresh'


class CartesianAdmittance:
    """Solve M*e_ddot + D*e_dot + K*e = F for diagonal 3D gains."""

    def __init__(self, mass, damping, stiffness, max_velocity, max_acceleration):
        self.mass = np.asarray(mass, dtype=float)
        self.damping = np.asarray(damping, dtype=float)
        self.stiffness = np.asarray(stiffness, dtype=float)
        if self.mass.shape != (3,) or self.damping.shape != (3,) or self.stiffness.shape != (3,):
            raise ValueError('Mass, damping and stiffness must contain XYZ values')
        if np.any(self.mass <= 0.0):
            raise ValueError('Virtual mass must be positive')
        if np.any(self.damping < 0.0) or np.any(self.stiffness < 0.0):
            raise ValueError('Damping and stiffness must be non-negative')
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.error = np.zeros(3, dtype=float)
        self.error_velocity = np.zeros(3, dtype=float)

    @staticmethod
    def _limit_norm(vector, maximum):
        vector = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(vector))
        if maximum > 0.0 and norm > maximum:
            return vector * (maximum / norm)
        return vector

    def reset(self, error=None):
        self.error = (
            np.zeros(3, dtype=float)
            if error is None else np.asarray(error, dtype=float).copy())
        if self.error.shape != (3,):
            raise ValueError('Initial error must contain XYZ values')
        self.error_velocity.fill(0.0)

    def step(self, force, dt):
        force = np.asarray(force, dtype=float)
        if force.shape != (3,):
            raise ValueError('Force must contain XYZ values')
        # 15 Hz -> dt ~= 0.0667 s. Clamp delayed executor ticks so one late
        # callback cannot create a large integration jump.
        dt = float(np.clip(dt, 1e-4, 0.1))
        acceleration = (
            force - self.damping * self.error_velocity - self.stiffness * self.error
        ) / self.mass
        acceleration = self._limit_norm(acceleration, self.max_acceleration)
        self.error_velocity += acceleration * dt
        self.error_velocity = self._limit_norm(
            self.error_velocity, self.max_velocity)
        self.error += self.error_velocity * dt
        return self.error.copy(), self.error_velocity.copy(), acceleration.copy()


def soft_radial_deadzone(force, threshold):
    """Continuous N-dimensional deadzone preserving force direction."""
    force = np.asarray(force, dtype=float)
    magnitude = float(np.linalg.norm(force))
    if magnitude <= threshold or magnitude == 0.0:
        return np.zeros_like(force)
    return force * ((magnitude - threshold) / magnitude)
