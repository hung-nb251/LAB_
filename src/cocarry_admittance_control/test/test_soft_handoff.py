import numpy as np
import pytest
from cocarry_admittance_control.soft_handoff import Bridge, mjm_state


@pytest.mark.parametrize('velocity', [[.1, 0, 0], [-.08, 0, 0], [.03, .05, 0], [0, 0, 0]])
def test_bridge_matches_states_and_advances_main_mjm(velocity):
    p, goal = np.array([0., .4, .5]), np.array([.3, .4, .5])
    v, a = np.array(velocity), np.zeros(3)
    bridge = Bridge(p, v, a, goal, 4.2, .18, .65, [-1, -1, 0], [1, 1, 1])
    for value, expected in zip(bridge.state(0), (p, v, a)):
        assert np.allclose(value, expected)
    left = bridge.state(bridge.time-1e-8)
    right = mjm_state(p, goal, 4.2, bridge.join)
    for value, expected in zip(left, right):
        assert np.allclose(value, expected, atol=1e-7)
    states = [bridge.state(t) for t in np.linspace(0, bridge.time+4.2-bridge.join, 4001)]
    assert max(np.linalg.norm(s[1]) for s in states) <= .18001
    assert max(np.linalg.norm(s[2]) for s in states) <= .65001
    assert np.allclose(states[-1][0], goal)
    assert np.linalg.norm(states[-1][1]) < 1e-8


def test_infeasible_bridge_rejected_not_silently_clipped():
    with pytest.raises(ValueError, match='No bounded soft bridge'):
        Bridge(np.array([.999, 0., .5]), np.array([.15, 0., 0.]), np.zeros(3),
               np.array([0., 0., .5]), 4., .18, .65, [-1, -1, 0], [1, 1, 1])


def test_noisy_acceleration_falls_back_without_resetting_velocity():
    p, v = np.array([0., .4, .5]), np.array([.1, 0., 0.])
    bridge = Bridge(p, v, np.array([20., 0., 0.]), np.array([.3, .4, .5]),
                    4.2, .18, .65, [-1, -1, 0], [1, 1, 1])
    assert bridge.continuity == 'C1'
    assert np.allclose(bridge.state(0)[0], p)
    assert np.allclose(bridge.state(0)[1], v)
    states = [bridge.state(t) for t in np.linspace(0, bridge.time, 4001)]
    assert max(np.linalg.norm(s[1]) for s in states) <= .18001
    assert max(np.linalg.norm(s[2]) for s in states) <= .65001


def test_bridge_selection_avoids_mid_bridge_slowdown():
    # Representative FOLLOWER -> LEADER geometry from the real GRU+MJM logs:
    # the old first-feasible selection fell to about 0.017 m/s here.
    p = np.array([-.097, .683, .601])
    v = np.array([.013, .050, .006])
    goal = np.array([.1393, .9997, .4404])
    bridge = Bridge(p, v, np.zeros(3), goal, 3.95, .22, .8,
                    [-1.4, -.5, .0314], [1.4, 1.3, 1.5])
    states = [bridge.state(t) for t in np.linspace(0, bridge.time, 1001)]
    minimum_speed = min(np.linalg.norm(state[1]) for state in states)
    assert minimum_speed > .045
    assert bridge.minimum_speed > .045
    assert bridge.speed_drop < .01
    assert bridge.join_velocity_error < .07


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_bridge_rejects_nonfinite_boundary(bad):
    with pytest.raises(ValueError, match='Invalid soft bridge'):
        Bridge([0, 0, .5], [bad, 0, 0], [0, 0, 0], [.3, 0, .5],
               4.2, .18, .65, [-1, -1, 0], [1, 1, 1])
