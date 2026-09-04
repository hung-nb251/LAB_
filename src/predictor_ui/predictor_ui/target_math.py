"""Pure helpers for robot-EE goal capture in the co-carry dashboard."""

from __future__ import annotations


def relative_goal(target_absolute, start_absolute):
    """Return target displacement in base axes relative to the Start EE pose."""
    target = tuple(float(value) for value in target_absolute)
    start = tuple(float(value) for value in start_absolute)
    if len(target) != 3 or len(start) != 3:
        raise ValueError('Target and start pose must contain XYZ values')
    return tuple(target_value - start_value
                 for target_value, start_value in zip(target, start))


def requires_robot_ee_target(show_camera_plots, trajectory_profile):
    """Require Capture Target only in the robot-EE predictor+MJM pipeline."""
    return not bool(show_camera_plots) and trajectory_profile == 'svgp_mjm'
