#!/usr/bin/env python3
"""Pure numerical helpers for joint-effort based force estimation.

The sign convention is deliberately explicit:

    tau_robot ~= J.T @ W_robot

No minus sign is introduced.  In an ideal Cartesian dynamics model this is
consistent with ``M x_ddot = F_robot + F_ext``.  It is only in the special
quasi-static case, with all other terms neglected, that the two forces happen
to be approximately opposite.
"""

from dataclasses import dataclass

import numpy as np


EFFORT_MODES = (
    'raw_only',
    'torque_nm',
    'normalized_rated_torque',
    'custom_scale',
)

# Provisional HC10DTP joint effort limits from the installed URDF.  These are
# available for diagnostic comparison only; they are not a calibration of the
# YRC1000 electrical signal.
HC10DTP_RATED_TORQUE_NM = np.array(
    [368.48, 414.54, 158.76, 41.16, 33.32, 31.36], dtype=np.float64)


@dataclass(frozen=True)
class WrenchEstimate:
    wrench: np.ndarray
    sigma_min: float
    condition_number: float
    damping: float
    relative_residual: float


def effort_to_joint_torque(
    raw_effort,
    mode: str,
    rated_torque_nm=HC10DTP_RATED_TORQUE_NM,
    custom_scale_nm=None,
    bias_nm=None,
):
    """Convert six raw joint efforts to N.m, or return ``None`` in raw mode.

    ``bias_nm`` is a known/modelled non-task joint torque to subtract.  A
    zero default does not claim gravity, friction or payload compensation.
    """
    raw = np.asarray(raw_effort, dtype=np.float64)
    if raw.shape != (6,) or not np.all(np.isfinite(raw)):
        raise ValueError('raw_effort must contain six finite values')
    if mode not in EFFORT_MODES:
        raise ValueError(f'unsupported effort mode: {mode}')
    if mode == 'raw_only':
        return None
    if mode == 'torque_nm':
        scale = np.ones(6, dtype=np.float64)
    elif mode == 'normalized_rated_torque':
        scale = np.asarray(rated_torque_nm, dtype=np.float64)
    else:
        if custom_scale_nm is None:
            raise ValueError('custom_scale mode requires custom_scale_nm')
        scale = np.asarray(custom_scale_nm, dtype=np.float64)
    if scale.shape != (6,) or not np.all(np.isfinite(scale)):
        raise ValueError('joint torque scale must contain six finite values')
    bias = np.zeros(6, dtype=np.float64) if bias_nm is None else np.asarray(
        bias_nm, dtype=np.float64)
    if bias.shape != (6,) or not np.all(np.isfinite(bias)):
        raise ValueError('bias_nm must contain six finite values')
    return raw * scale - bias


def estimate_robot_wrench(
    jacobian,
    joint_torque_nm,
    damping_min: float = 0.002,
    damping_max: float = 0.08,
    singularity_threshold: float = 0.05,
) -> WrenchEstimate:
    """Solve ``J.T W_robot ~= tau_robot`` with adaptive DLS.

    All six wrench components are solved internally so a real moment at tool0
    is not incorrectly folded into the translational force.  Callers may
    intentionally publish only ``wrench[:3]``.
    """
    jac = np.asarray(jacobian, dtype=np.float64)
    tau = np.asarray(joint_torque_nm, dtype=np.float64)
    if jac.shape != (6, 6) or not np.all(np.isfinite(jac)):
        raise ValueError('jacobian must be a finite 6x6 matrix')
    if tau.shape != (6,) or not np.all(np.isfinite(tau)):
        raise ValueError('joint_torque_nm must contain six finite values')
    if not (0.0 <= damping_min <= damping_max):
        raise ValueError('damping must satisfy 0 <= min <= max')
    if singularity_threshold <= 0.0:
        raise ValueError('singularity_threshold must be positive')

    singular_values = np.linalg.svd(jac, compute_uv=False)
    sigma_min = float(singular_values[-1])
    sigma_max = float(singular_values[0])
    condition = float('inf') if sigma_min <= np.finfo(float).eps else sigma_max / sigma_min

    proximity = float(np.clip(
        (singularity_threshold - sigma_min) / singularity_threshold, 0.0, 1.0))
    damping = float(damping_min + (damping_max - damping_min) * proximity ** 2)

    # A = J.T and W = argmin ||A W - tau||^2 + lambda^2 ||W||^2.
    normal = jac @ jac.T + (damping * damping) * np.eye(6, dtype=np.float64)
    rhs = jac @ tau
    try:
        wrench = np.linalg.solve(normal, rhs)
    except np.linalg.LinAlgError:
        wrench = np.linalg.lstsq(normal, rhs, rcond=None)[0]
    residual = jac.T @ wrench - tau
    relative_residual = float(
        np.linalg.norm(residual) / max(np.linalg.norm(tau), np.finfo(float).eps))
    return WrenchEstimate(
        wrench=wrench,
        sigma_min=sigma_min,
        condition_number=condition,
        damping=damping,
        relative_residual=relative_residual,
    )
