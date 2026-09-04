import numpy as np

from cocarry_admittance_control.admittance import (
    CartesianAdmittance,
    commanded_position,
    critical_damping,
    fresh_mjm_sample,
    force_watchdog_action,
    minimum_safe_ee_z,
    limit_position_lead,
    nominal_reference,
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
