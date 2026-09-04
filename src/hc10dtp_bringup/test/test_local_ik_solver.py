"""Unit tests for the HC10DTP local weighted/adaptive DLS solver."""

import importlib.util
from pathlib import Path

import numpy as np


_SOLVER_PATH = Path(__file__).parents[1] / 'scripts' / 'local_ik_solver.py'
_SPEC = importlib.util.spec_from_file_location('local_ik_solver', _SOLVER_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
LocalIKSolver = _MODULE.LocalIKSolver


def test_weighting_is_applied_to_error_and_jacobian():
    error = np.arange(1.0, 7.0)
    jacobian = np.arange(36.0).reshape(6, 6)

    weighted_error, weighted_jacobian = LocalIKSolver._weighted_dls_system(
        error, jacobian)

    assert np.allclose(weighted_error[:3], error[:3])
    assert np.allclose(weighted_error[3:], 0.5 * error[3:])
    assert np.allclose(weighted_jacobian[:3], jacobian[:3])
    assert np.allclose(weighted_jacobian[3:], 0.5 * jacobian[3:])


def test_adaptive_damping_uses_floor_away_from_singularity():
    damping, sigma_min = LocalIKSolver._adaptive_damping(np.eye(6))

    assert np.isclose(sigma_min, 1.0)
    assert np.isclose(damping, LocalIKSolver.IK_DAMPING_MIN)


def test_adaptive_damping_increases_smoothly_toward_singularity():
    threshold = LocalIKSolver.IK_SINGULAR_VALUE_THRESHOLD
    jacobians = [
        np.diag([1.0] * 5 + [threshold]),
        np.diag([1.0] * 5 + [threshold / 2.0]),
        np.diag([1.0] * 5 + [0.0]),
    ]
    damping = [LocalIKSolver._adaptive_damping(j)[0] for j in jacobians]

    assert np.isclose(damping[0], LocalIKSolver.IK_DAMPING_MIN)
    assert damping[0] < damping[1] < damping[2]
    assert np.isclose(damping[2], LocalIKSolver.IK_DAMPING_MAX)


def test_solver_retains_accuracy_at_normal_operating_pose():
    solver = LocalIKSolver()
    target_joints = np.array([
        1.570742, 0.027735, -0.710960,
        0.000032, -0.847848, -0.000658,
    ])
    target_position, target_quaternion = solver.fk_pose(target_joints)
    seed = target_joints + np.array([0.01, -0.01, 0.01, 0.01, -0.01, 0.01])

    solution = solver.solve_ik(
        target_position, target_quaternion, seed)

    assert solution is not None
    solved_position, solved_quaternion = solver.fk_pose(solution)
    assert np.linalg.norm(solved_position - target_position) < 5e-4
    assert abs(float(np.dot(solved_quaternion, target_quaternion))) > 0.9999
