import numpy as np
import pytest

from cocarry_admittance_control.admittance import (
    CartesianAdmittance,
    commanded_position,
    critical_damping,
    fresh_mjm_sample,
    force_watchdog_action,
    minimum_safe_ee_z,
    limit_position_lead,
    nominal_reference,
    phase_aligned_delay,
    soft_axis_deadzone,
    soft_radial_deadzone,
)


def make_controller(stiffness=(0.0, 0.0, 0.0)):
    mass = np.ones(3)
    stiffness = np.asarray(stiffness)
    damping = np.where(
        stiffness > 0.0,
        critical_damping(mass, stiffness),
        20.0,
    )
    return CartesianAdmittance(
        mass=mass,
        damping=damping,
        stiffness=stiffness,
        max_velocity=0.15,
        max_acceleration=0.50,
    )


def test_critical_damping_for_selected_gains():
    assert np.allclose(
        critical_damping((1.0, 1.0, 1.0), (10.0, 10.0, 10.0)),
        (6.32455532, 6.32455532, 6.32455532))


def test_ground_truth_keeps_captured_nominal_and_ignores_prediction():
    capture = np.array((0.1, 0.2, 0.3))
    prediction = np.array((0.4, -0.2, 0.1))
    assert np.allclose(
        nominal_reference(capture, 'ground_truth', prediction),
        capture,
    )


def test_prediction_adds_svgp_relative_displacement_to_capture():
    capture = np.array((0.1, 0.2, 0.3))
    prediction = np.array((0.4, -0.2, 0.1))
    assert np.allclose(
        nominal_reference(capture, 'prediction', prediction),
        (0.5, 0.0, 0.4),
    )
    assert nominal_reference(capture, 'prediction', None) is None


def test_downward_tool_clearance_is_applied_to_minimum_ee_z():
    assert np.isclose(minimum_safe_ee_z(0.05, 0.1814), 0.2314)


def test_leader_bypasses_admittance_error_while_follower_applies_it():
    nominal = np.array((0.2, 0.4, 0.6))
    error = np.array((0.01, -0.02, 0.03))
    assert np.allclose(commanded_position(nominal, error, leader=False), nominal + error)
    assert np.allclose(commanded_position(nominal, error, leader=True), nominal)


def test_leader_handoff_requires_a_fresh_explicit_mjm_sample():
    assert fresh_mjm_sample('mjm', 9.9, now=10.0, timeout=0.5)
    assert fresh_mjm_sample(' MJM ', 9.9, now=10.0, timeout=0.5)
    assert not fresh_mjm_sample('svgp', 9.9, now=10.0, timeout=0.5)
    assert not fresh_mjm_sample('mjm', 9.0, now=10.0, timeout=0.5)
    assert not fresh_mjm_sample('mjm', 10.1, now=10.0, timeout=0.5)


def test_prediction_nominal_lead_is_bounded_radially():
    limited, was_limited, requested = limit_position_lead(
        (0.06, 0.08, 0.0), (0.0, 0.0, 0.0), 0.05)
    assert was_limited
    assert np.isclose(requested, 0.10)
    assert np.allclose(limited, (0.03, 0.04, 0.0))


def test_prediction_nominal_inside_lead_limit_is_unchanged():
    desired = np.array((0.01, -0.02, 0.03))
    limited, was_limited, _ = limit_position_lead(
        desired, np.zeros(3), 0.05)
    assert not was_limited
    assert np.allclose(limited, desired)


def test_force_watchdog_has_soft_hold_then_hard_fault():
    assert force_watchdog_action(0.10, 0.20, 0.50) == 'fresh'
    assert force_watchdog_action(0.20, 0.20, 0.50) == 'fresh'
    assert force_watchdog_action(0.21, 0.20, 0.50) == 'hold'
    assert force_watchdog_action(0.50, 0.20, 0.50) == 'hold'
    assert force_watchdog_action(0.51, 0.20, 0.50) == 'fault'


def test_zero_force_keeps_zero_state():
    controller = make_controller()
    for _ in range(100):
        error, velocity, acceleration = controller.step((0.0, 0.0, 0.0), 0.01)
    assert np.allclose(error, 0.0)
    assert np.allclose(velocity, 0.0)
    assert np.allclose(acceleration, 0.0)


def test_each_force_axis_moves_only_its_matching_axis():
    for axis in range(3):
        controller = make_controller()
        force = np.zeros(3)
        force[axis] = 2.0
        for _ in range(100):
            error, _, _ = controller.step(force, 0.01)
        assert error[axis] > 0.0
        assert np.allclose(np.delete(error, axis), 0.0)


def test_velocity_and_acceleration_norms_are_limited():
    controller = make_controller()
    for _ in range(100):
        _, velocity, acceleration = controller.step((1000.0, 1000.0, 1000.0), 0.01)
        assert np.linalg.norm(velocity) <= 0.15 + 1e-12
        assert np.linalg.norm(acceleration) <= 0.50 + 1e-12


def test_stiffness_returns_xyz_error_toward_zero_after_release():
    controller = make_controller(stiffness=(20.0, 20.0, 20.0))
    for _ in range(100):
        controller.step((2.0, -2.0, 2.0), 0.01)
    displaced = np.linalg.norm(controller.error)
    for _ in range(500):
        controller.step((0.0, 0.0, 0.0), 0.01)
    assert np.linalg.norm(controller.error) < displaced


def test_zero_stiffness_damping_stops_without_returning_to_origin():
    controller = make_controller(stiffness=(0.0, 0.0, 0.0))
    for _ in range(100):
        controller.step((2.0, 0.0, 0.0), 0.01)
    displaced = controller.error[0]
    for _ in range(300):
        controller.step((0.0, 0.0, 0.0), 0.01)
    assert abs(controller.error_velocity[0]) < 1e-8
    assert controller.error[0] >= displaced
    assert controller.error[0] > 0.0


def test_soft_deadzone_is_3d_continuous_and_preserves_direction():
    assert np.allclose(soft_radial_deadzone((0.0, 0.0, 0.5), 0.5), 0.0)
    result = soft_radial_deadzone((0.6, 0.0, 0.8), 0.5)
    assert np.allclose(result, (0.3, 0.0, 0.4))


def test_soft_axis_deadzone_can_target_z_only():
    result = soft_axis_deadzone((1.5, -2.5, 3.73), (0.0, 0.0, 2.0))
    assert np.allclose(result, (1.5, -2.5, 1.73))
    assert np.allclose(
        soft_axis_deadzone((1.5, -2.5, -1.9), (0.0, 0.0, 2.0)),
        (1.5, -2.5, 0.0))


PERIOD = 1 / 15
OFFSET = 0.0333
# prediction_age_ms medians observed across launches on 2026-09-21.
OBSERVED_PHASES = (0.0054, 0.0182, 0.0212, 0.0332, 0.053, 0.0637, 0.0645)


def test_phase_alignment_lands_on_the_target_offset_from_any_launch_phase():
    """Whatever the launch dealt, one wait puts the tick at the same offset."""
    now = 1000.0
    for phase in OBSERVED_PHASES:
        sample_time = now - phase          # the sample arrived `phase` ago
        delay = phase_aligned_delay(sample_time, now, PERIOD, OFFSET)
        assert 0.0 < delay <= PERIOD + 1e-12
        # Age of that same sample when the re-phased tick finally runs.
        settled = (now + delay) - sample_time
        residue = (settled - OFFSET) % PERIOD
        assert min(residue, PERIOD - residue) < 1e-9


def test_phase_alignment_waits_less_than_one_period_and_never_returns_zero():
    for phase in np.linspace(0.0, PERIOD, 97):
        delay = phase_aligned_delay(1000.0 - phase, 1000.0, PERIOD, OFFSET)
        assert 0.0 < delay <= PERIOD + 1e-12


def test_phase_alignment_handles_a_sample_newer_than_the_offset():
    # Sample arrived 1 ms ago, target offset 33.3 ms: wait the remaining 32.3.
    assert phase_aligned_delay(999.999, 1000.0, PERIOD, OFFSET) == pytest.approx(
        OFFSET - 0.001, abs=1e-12)
    # Sample exactly at the target instant already: wait a full period.
    assert phase_aligned_delay(1000.0 - OFFSET, 1000.0, PERIOD, OFFSET) == pytest.approx(
        PERIOD, abs=1e-12)


def test_phase_alignment_tolerates_a_very_old_sample():
    delay = phase_aligned_delay(1000.0 - 7.4, 1000.0, PERIOD, OFFSET)
    assert 0.0 < delay <= PERIOD + 1e-12


@pytest.mark.parametrize('args', [
    (1000.0, 1000.0, 0.0, 0.0),            # period must be positive
    (1000.0, 1000.0, -PERIOD, 0.0),
    (1000.0, 1000.0, PERIOD, PERIOD),      # offset must be below one period
    (1000.0, 1000.0, PERIOD, -1e-9),
    (float('nan'), 1000.0, PERIOD, OFFSET),
    (1000.0, float('inf'), PERIOD, OFFSET),
])
def test_phase_alignment_rejects_invalid_arguments(args):
    with pytest.raises(ValueError):
        phase_aligned_delay(*args)
