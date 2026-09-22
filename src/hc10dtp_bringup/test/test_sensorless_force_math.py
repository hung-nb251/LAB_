"""Tests for the read-only robot force estimator math."""

import importlib.util
from pathlib import Path

import numpy as np


_MODULE_PATH = Path(__file__).parents[1] / 'scripts' / 'sensorless_force_math.py'
_SPEC = importlib.util.spec_from_file_location('sensorless_force_math', _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

effort_to_joint_torque = _MODULE.effort_to_joint_torque
estimate_robot_wrench = _MODULE.estimate_robot_wrench
recover_calibrated_wrench = _MODULE.recover_calibrated_wrench
apply_mregister_calibration = _MODULE.apply_mregister_calibration


def test_raw_only_never_claims_a_torque_conversion():
    assert effort_to_joint_torque(np.ones(6), 'raw_only') is None


def test_normalized_effort_uses_explicit_per_joint_scale_and_bias():
    raw = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    rated = np.arange(1.0, 7.0) * 10.0
    bias = np.arange(6.0)

    torque = effort_to_joint_torque(
        raw, 'normalized_rated_torque', rated_torque_nm=rated, bias_nm=bias)

    assert np.allclose(torque, raw * rated - bias)


def test_wrench_mapping_does_not_reverse_the_sign():
    wanted_wrench = np.array([5.0, -4.0, 3.0, 2.0, -1.0, 0.5])
    jacobian = np.eye(6)
    joint_torque = jacobian.T @ wanted_wrench

    estimate = estimate_robot_wrench(
        jacobian, joint_torque, damping_min=0.0, damping_max=0.0)

    assert np.allclose(estimate.wrench, wanted_wrench)
    assert estimate.relative_residual < 1e-12


def test_all_six_wrench_components_are_solved_internally():
    rng = np.random.default_rng(42)
    q_matrix, _ = np.linalg.qr(rng.normal(size=(6, 6)))
    wanted_wrench = np.array([7.0, 1.0, -3.0, 4.0, -2.0, 6.0])
    joint_torque = q_matrix.T @ wanted_wrench

    estimate = estimate_robot_wrench(
        q_matrix, joint_torque, damping_min=0.0, damping_max=0.0)

    assert np.allclose(estimate.wrench, wanted_wrench)


def test_damping_increases_near_a_singularity():
    regular = estimate_robot_wrench(
        np.eye(6), np.ones(6), damping_min=0.002, damping_max=0.08)
    near_singular = estimate_robot_wrench(
        np.diag([1.0] * 5 + [0.001]), np.ones(6),
        damping_min=0.002, damping_max=0.08)

    assert regular.damping == 0.002
    assert near_singular.damping > regular.damping
    assert near_singular.sigma_min < regular.sigma_min


def test_calibrated_recovery_matches_forward_wrench_mapping_without_damping():
    rng = np.random.default_rng(7)
    jacobian = rng.normal(size=(6, 6))
    while abs(np.linalg.det(jacobian)) < .1:
        jacobian = rng.normal(size=(6, 6))
    wanted = np.array([4., -3., 2., .5, -.4, .3])
    torque = jacobian.T @ wanted
    estimate = recover_calibrated_wrench(
        jacobian, torque, damping=0., characteristic_length_m=.3)
    assert np.allclose(estimate.wrench, wanted)
    assert estimate.relative_residual < 1e-10


def test_mregister_pose_calibration_uses_baseline_and_q0():
    register = np.arange(6.) + 10.
    baseline = np.arange(6.)
    q = np.arange(6.) * .1
    q0 = np.zeros(6)
    scale = np.ones(12)
    coef = np.eye(12, 6)
    result = apply_mregister_calibration(
        register, baseline, q, q0, scale, coef)
    assert np.allclose(result, np.full(6, 10.))
