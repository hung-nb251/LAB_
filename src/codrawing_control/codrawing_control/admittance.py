"""Pure numerical core for the planar admittance law."""

from __future__ import annotations

import numpy as np


class PlanarAdmittance:
    """Solve M*e_ddot + D*e_dot + K*e = F in two dimensions."""

    def __init__(self, mass, damping, stiffness, max_velocity, max_acceleration):
        self.mass = np.asarray(mass, dtype=float)
        self.damping = np.asarray(damping, dtype=float)
        self.stiffness = np.asarray(stiffness, dtype=float)
        if np.any(self.mass <= 0.0):
            raise ValueError('Virtual mass must be positive')
        if np.any(self.damping < 0.0) or np.any(self.stiffness < 0.0):
            raise ValueError('Damping and stiffness must be non-negative')
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.error = np.zeros(2, dtype=float)
        self.error_velocity = np.zeros(2, dtype=float)

    @staticmethod
    def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if maximum > 0.0 and norm > maximum:
            return vector * (maximum / norm)
        return vector

    def reset(self, error=None):
        self.error = (
            np.zeros(2, dtype=float)
            if error is None else np.asarray(error, dtype=float).copy()
        )
        self.error_velocity.fill(0.0)

    def step(self, force, dt):
        force = np.asarray(force, dtype=float)
        # 15 Hz tương ứng dt ~= 0.0667 s. Giới hạn 0.1 s vẫn chặn bước
        # tích phân quá lớn khi ROS bị trễ, nhưng không làm chậm thời gian mô phỏng.
        dt = float(np.clip(dt, 1e-4, 0.1))
        acceleration = (
            force
            - self.damping * self.error_velocity
            - self.stiffness * self.error
        ) / self.mass
        acceleration = self._limit_norm(acceleration, self.max_acceleration)
        self.error_velocity += acceleration * dt
        self.error_velocity = self._limit_norm(
            self.error_velocity, self.max_velocity)
        self.error += self.error_velocity * dt
        return self.error.copy(), self.error_velocity.copy(), acceleration.copy()


def soft_radial_deadzone(force, threshold):
    """Continuous 2D deadzone which preserves the direction of the force."""
    force = np.asarray(force, dtype=float)
    magnitude = float(np.linalg.norm(force))
    if magnitude <= threshold or magnitude == 0.0:
        return np.zeros(2, dtype=float)
    return force * ((magnitude - threshold) / magnitude)
