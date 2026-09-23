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


def _reference_fit(p, v, a, goal, duration, vmax, amax, lower, upper):
    """The original per-candidate implementation, kept verbatim as an oracle.

    The production _fit was rewritten 2026-09-23 purely for speed (it runs in
    the control tick on the FOLLOWER -> LEADER switch and was overrunning the
    66.7 ms budget). Behaviour must stay identical, including tie-breaking.
    """
    from cocarry_admittance_control.soft_handoff import mjm_state as _mjm
    if np.linalg.norm(v) > vmax+1e-9 or np.linalg.norm(a) > amax+1e-9:
        return None
    delta = goal-p
    forward = np.dot(v, delta) > 0
    initial_speed = float(np.linalg.norm(v))
    candidates = []
    for tb in (.4, .6, .8, 1., 1.2, 1.6, 2.):
        for join in np.linspace(tb, max(tb, duration*.85), 32):
            if join >= duration:
                continue
            q, w, b = _mjm(p, goal, duration, join)
            c = np.zeros((6, 3))
            c[:3] = p, v*tb, a*tb**2/2
            c[3:] = np.linalg.solve(
                [[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]],
                [q-c[0]-c[1]-c[2], w*tb-c[1]-2*c[2], b*tb**2-2*c[2]])
            u = np.linspace(0, 1, 401)[:, None]
            positions = sum(c[i]*u**i for i in range(6))
            velocities = sum(i*c[i]*u**(i-1)/tb for i in range(1, 6))
            accelerations = sum(i*(i-1)*c[i]*u**(i-2)/tb**2 for i in range(2, 6))
            if (np.max(np.linalg.norm(velocities, axis=1)) > vmax+1e-9
                    or np.max(np.linalg.norm(accelerations, axis=1)) > amax+1e-9
                    or np.any(positions < lower) or np.any(positions > upper)
                    or (forward and np.min(velocities@delta) < -1e-9)):
                continue
            speeds = np.linalg.norm(velocities, axis=1)
            minimum_speed = float(np.min(speeds))
            speed_drop = max(0., initial_speed-minimum_speed)
            jve = float(np.linalg.norm(w-v))
            candidates.append((jve + 2.*speed_drop, jve, speed_drop, tb, join, c,
                               minimum_speed))
    return min(candidates, key=lambda item: item[:5]) if candidates else None


def _reference_bridge(p, v, a, goal, duration, vmax, amax, lower, upper):
    r = _reference_fit(p, v, a, goal, duration, vmax, amax, lower, upper)
    if r is not None:
        return 'C2', r
    if np.any(a):
        r = _reference_fit(p, v, np.zeros(3), goal, duration, vmax, amax, lower, upper)
        if r is not None:
            return 'C1', r
    return 'FAIL', None


def test_fast_fit_is_identical_to_the_original_on_realistic_inputs():
    lo, hi = np.array([-1.4, -.5, .0314]), np.array([1.4, 1.3, 1.5])
    rng = np.random.default_rng(20260923)
    seen = {'C2': 0, 'C1': 0, 'FAIL': 0}
    for _ in range(200):
        p = np.array([-.1, .7, .5]) + rng.normal(0, .05, 3)
        v = rng.normal(0, 1, 3); v *= rng.uniform(.02, .2) / np.linalg.norm(v)
        a = rng.normal(0, 1, 3); a *= rng.uniform(0., .95) / np.linalg.norm(a)
        goal = p + rng.normal(0, 1, 3) / np.sqrt(3) * rng.uniform(.1, .6)
        duration = rng.uniform(1.2, 3.2)
        expected, ref = _reference_bridge(p, v, a, goal, duration, .25, 1., lo, hi)
        seen[expected] += 1
        if expected == 'FAIL':
            with pytest.raises(ValueError):
                Bridge(p, v, a, goal, duration, .25, 1., lo, hi)
            continue
        got = Bridge(p, v, a, goal, duration, .25, 1., lo, hi)
        assert got.continuity == expected
        assert got.time == ref[3] and got.join == ref[4]
        np.testing.assert_allclose(got.coefficients, ref[5], rtol=0, atol=1e-12)
        assert got.minimum_speed == pytest.approx(ref[6], abs=1e-12)
    # The sample must exercise every outcome, or equivalence is not shown.
    assert all(n > 0 for n in seen.values()), seen


def test_fast_fit_stays_well_inside_the_control_tick_budget():
    import time
    lo, hi = np.array([-1.4, -.5, .0314]), np.array([1.4, 1.3, 1.5])
    rng = np.random.default_rng(7)
    took = []
    for _ in range(40):
        p = np.array([-.1, .7, .5]) + rng.normal(0, .05, 3)
        v = rng.normal(0, 1, 3); v *= .12 / np.linalg.norm(v)
        a = rng.normal(0, 1, 3); a *= .5 / np.linalg.norm(a)
        goal = p + rng.normal(0, 1, 3) / np.sqrt(3) * .45
        t0 = time.perf_counter()
        try:
            Bridge(p, v, a, goal, 2.2, .25, 1., lo, hi)
        except ValueError:
            pass
        took.append(time.perf_counter() - t0)
    # One 15 Hz tick is 66.7 ms; leave most of it for the rest of the tick.
    assert np.median(took) < .030
