import pytest

from predictor_ui.target_math import (
    manual_leader_rejection,
    relative_goal,
    requires_robot_ee_target,
)


def test_relative_goal_uses_robot_ee_pose_at_start():
    assert relative_goal(
        target_absolute=(0.50, 0.70, 0.40),
        start_absolute=(0.20, 0.30, 0.35),
    ) == pytest.approx((0.30, 0.40, 0.05))


def test_relative_goal_requires_xyz():
    with pytest.raises(ValueError):
        relative_goal((0.1, 0.2), (0.0, 0.0, 0.0))


def test_capture_target_requirement_is_isolated_from_camera_pipeline():
    assert requires_robot_ee_target(False, 'svgp_mjm')
    assert not requires_robot_ee_target(True, 'svgp_mjm')
    assert not requires_robot_ee_target(False, 'svgp')


def test_manual_leader_explains_missing_targets_and_selection():
    assert 'Target 1 và 2' in manual_leader_rejection({'targets': {}})
    assert 'Target 2' in manual_leader_rejection(
        {'targets': {'1': [0.1, 0.2, 0.3]}})
    assert 'chọn Target 1 hoặc Target 2' in manual_leader_rejection({
        'targets': {'1': [0.1, 0.2, 0.3], '2': [0.4, 0.5, 0.6]},
        'selected': None,
    })
    assert manual_leader_rejection({
        'targets': {'1': [0.1, 0.2, 0.3], '2': [0.4, 0.5, 0.6]},
        'selected': 2,
    }) is None
