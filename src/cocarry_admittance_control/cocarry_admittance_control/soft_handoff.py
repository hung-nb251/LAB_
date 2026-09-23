"""Boundary-matched bridge into an unchanged rest-to-rest minimum-jerk leg."""
import numpy as np

# Every candidate is checked at the same 401 normalised instants, so the
# polynomial bases are fixed and each check becomes one matrix product. The
# fit runs inside the control tick on the FOLLOWER -> LEADER switch; building
# these per candidate cost ~36 ms median against a 66.7 ms tick budget.
_U = np.linspace(0., 1., 401)
_POSITION_BASIS = np.stack([_U**i for i in range(6)], axis=1)
_VELOCITY_BASIS = np.stack(
    [np.zeros_like(_U)] + [i * _U**(i - 1) for i in range(1, 6)], axis=1)
_ACCELERATION_BASIS = np.stack(
    [np.zeros_like(_U)] * 2 + [i * (i - 1) * _U**(i - 2) for i in range(2, 6)],
    axis=1)
_JOIN_SYSTEM = np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]])


def mjm_state(start, goal, duration, time):
    s = np.clip(time / duration, 0., 1.)
    d = goal - start
    return (start + d*(10*s**3-15*s**4+6*s**5),
            d*(30*s**2-60*s**3+30*s**4)/duration,
            d*(60*s-180*s**2+120*s**3)/duration**2)


class Bridge:
    def __init__(self, p, v, a, goal, duration, vmax, amax, lower, upper):
        p, v, a, goal, lower, upper = [np.asarray(x, dtype=float) for x in
                                       (p, v, a, goal, lower, upper)]
        if (any(x.shape != (3,) or not np.all(np.isfinite(x))
                for x in (p, v, a, goal, lower, upper))
                or not np.all(np.isfinite([duration, vmax, amax]))
                or min(duration, vmax, amax) <= 0 or np.any(lower > upper)):
            raise ValueError('Invalid soft bridge boundaries or limits')
        self.start, self.goal, self.duration = p.copy(), goal.copy(), duration
        self.continuity = 'C2'
        if self._fit(p, v, a, goal, duration, vmax, amax, lower, upper):
            return
        # Finite-difference acceleration can spike after a limiter or jitter.
        # Preserve position AND velocity; relax only acceleration continuity.
        self.continuity = 'C1'
        if np.any(a) and self._fit(p, v, np.zeros(3), goal, duration,
                                  vmax, amax, lower, upper):
            return
        raise ValueError('No bounded soft bridge; slow down before requesting LEADER')

    def _fit(self, p, v, a, goal, duration, vmax, amax, lower, upper):
        if np.linalg.norm(v) > vmax+1e-9 or np.linalg.norm(a) > amax+1e-9:
            return False
        delta = goal-p
        forward = np.dot(v, delta) > 0
        initial_speed = float(np.linalg.norm(v))
        candidates = []
        # Search the join phase as well as bridge time: the first .4 s of a
        # slow Fitts MJM may lie BEHIND an already moving reference.
        for tb in (.4, .6, .8, 1., 1.2, 1.6, 2.):
            for join in np.linspace(tb, max(tb, duration*.85), 32):
                if join >= duration:
                    continue
                q, w, b = mjm_state(p, goal, duration, join)
                c = np.zeros((6, 3))
                c[:3] = p, v*tb, a*tb**2/2
                c[3:] = np.linalg.solve(
                    _JOIN_SYSTEM,
                    [q-c[0]-c[1]-c[2], w*tb-c[1]-2*c[2], b*tb**2-2*c[2]])
                positions = _POSITION_BASIS @ c
                velocities = (_VELOCITY_BASIS @ c) / tb
                accelerations = (_ACCELERATION_BASIS @ c) / tb**2
                speeds = np.linalg.norm(velocities, axis=1)
                if (np.max(speeds) > vmax+1e-9
                        or np.max(np.linalg.norm(accelerations, axis=1)) > amax+1e-9
                        or np.any(positions < lower) or np.any(positions > upper)
                        or (forward and np.min(velocities@delta) < -1e-9)):
                    continue
                minimum_speed = float(np.min(speeds))
                speed_drop = max(0., initial_speed-minimum_speed)
                join_velocity_error = float(np.linalg.norm(w-v))
                # Matching only the endpoint velocity can still choose a
                # polynomial which almost stops in the middle.  Penalize that
                # dip more strongly while retaining endpoint matching as part
                # of the objective.  All candidates have already passed the
                # same hard velocity/acceleration/workspace checks above.
                score = join_velocity_error + 2.*speed_drop
                candidates.append((score, join_velocity_error, speed_drop,
                                   tb, join, c, minimum_speed))
        if not candidates:
            return False
        (_, self.join_velocity_error, self.speed_drop, self.time, self.join,
         self.coefficients, self.minimum_speed) = min(
             candidates, key=lambda item: item[:5])
        return True

    def state(self, elapsed):
        if elapsed >= self.time:
            return mjm_state(self.start, self.goal, self.duration,
                             self.join + elapsed-self.time)
        u, c, tb = max(0., elapsed)/self.time, self.coefficients, self.time
        return (sum(c[i]*u**i for i in range(6)),
                sum(i*c[i]*u**(i-1)/tb for i in range(1, 6)),
                sum(i*(i-1)*c[i]*u**(i-2)/tb**2 for i in range(2, 6)))
