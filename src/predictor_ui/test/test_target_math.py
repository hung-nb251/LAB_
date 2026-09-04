import pytest

from predictor_ui.target_math import relative_goal, requires_robot_ee_target


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
