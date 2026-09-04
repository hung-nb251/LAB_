import numpy as np

from codrawing_control.admittance import PlanarAdmittance, soft_radial_deadzone


def make_controller(stiffness=(0.0, 0.0)):
    return PlanarAdmittance(
        mass=(4.0, 4.0),
        damping=(20.0, 20.0),
        stiffness=stiffness,
        max_velocity=0.05,
        max_acceleration=0.2,
    )


def test_zero_force_keeps_zero_state():
    controller = make_controller()
    for _ in range(100):
        error, velocity, acceleration = controller.step((0.0, 0.0), 0.01)
    assert np.allclose(error, 0.0)
    assert np.allclose(velocity, 0.0)
    assert np.allclose(acceleration, 0.0)


def test_positive_force_moves_positive_x_only():
    controller = make_controller()
    for _ in range(100):
        error, _, _ = controller.step((2.0, 0.0), 0.01)
    assert error[0] > 0.0
    assert abs(error[1]) < 1e-12


def test_velocity_and_acceleration_are_limited():
    controller = make_controller()
    for _ in range(100):
        _, velocity, acceleration = controller.step((1000.0, 1000.0), 0.01)
        assert np.linalg.norm(velocity) <= 0.05 + 1e-12
        assert np.linalg.norm(acceleration) <= 0.2 + 1e-12


def test_stiffness_returns_error_toward_zero_after_release():
    controller = make_controller(stiffness=(20.0, 20.0))
    for _ in range(100):
        controller.step((2.0, 0.0), 0.01)
    displaced = controller.error[0]
    for _ in range(500):
        controller.step((0.0, 0.0), 0.01)
    assert abs(controller.error[0]) < abs(displaced)


def test_soft_deadzone_is_continuous_and_preserves_direction():
    assert np.allclose(soft_radial_deadzone((0.3, 0.4), 0.5), (0.0, 0.0))
    result = soft_radial_deadzone((0.6, 0.8), 0.5)
    assert np.allclose(result, (0.3, 0.4))

