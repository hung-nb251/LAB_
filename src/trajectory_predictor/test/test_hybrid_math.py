import numpy as np

from trajectory_predictor.hybrid_math import (
    leader_start_position,
    minimum_jerk_positions,
)


def test_leader_starts_from_last_limited_reference():
    limited_reference = np.array((0.12, -0.04, 0.03))
    measured = np.array((0.10, -0.03, 0.02))
    start = leader_start_position(limited_reference)
    assert np.allclose(start, limited_reference)
    assert not np.allclose(start, measured)


def test_leader_start_is_a_finite_copy():
    limited_reference = np.array((0.12, -0.04, 0.03))
    start = leader_start_position(limited_reference)
    start[0] = 99.0
    assert np.isclose(limited_reference[0], 0.12)

    with np.testing.assert_raises(ValueError):
        leader_start_position((np.nan, 0.0, 0.0))


def test_minimum_jerk_includes_limited_start_and_goal():
    start = np.array((0.12, -0.04, 0.03))
    goal = np.array((0.40, 0.30, -0.02))
    trajectory = minimum_jerk_positions(start, goal, t_total=2.0, dt=1.0 / 15.0)
    assert np.allclose(trajectory[0], start)
    assert np.allclose(trajectory[-1], goal)
