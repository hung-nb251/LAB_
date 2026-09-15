"""ROS-independent manual intent/role state and durable absolute EE targets."""
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np
from .soft_handoff import Bridge


class ManualHybrid:
    def __init__(self, path, lower, upper, tolerance=.01, speed=.02, dwell=.5,
                 grace=5., vmax=.15, amax=.5, fitts_a=3.0, fitts_b=.7492, fitts_w=.3):
        self.path = Path(path).expanduser()
        self.lower, self.upper = np.array(lower), np.array(upper)
        self.tolerance, self.speed, self.dwell = tolerance, speed, dwell
        self.grace, self.vmax, self.amax = grace, vmax, amax
        self.fitts_a, self.fitts_b, self.fitts_w = fitts_a, fitts_b, fitts_w
        if not all(math.isfinite(x) and x > 0 for x in
                   (tolerance, speed, dwell, grace, vmax, amax, fitts_a, fitts_b, fitts_w)):
            raise ValueError('Hybrid thresholds must be finite and positive')
        self.targets = {}
        self.enabled = False
        self.test_mode = False
        self.test_elapsed = 0.
        self.test_fired = False
        self.role_source = 'manual'
        self.bridge = None
        self.force_follower = False
        self.role = 'FOLLOWER'
        self.selected = None
        self.active = None
        self.leg = 0
        self.reason = 'initialized'
        self.elapsed = 0.
        self.duration = 0.
        self.start = None
        self.goal = None
        self.distance = None
        self._inside_since = None
        self._reached = False
        if self.path.exists():
            data = json.loads(self.path.read_text())
            if data.get('frame') != 'base_link' or data.get('schema') != 1:
                raise ValueError('Unsupported target file frame/schema')
            for key, point in data['targets'].items():
                self._id(int(key))
                self.targets[int(key)] = self.point(point).tolist()
            self._check_separation(self.targets)

    @staticmethod
    def _id(target):
        if type(target) is not int or target not in (1, 2):
            raise ValueError('Target must be 1 or 2')

    def point(self, point):
        p = np.asarray(point, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError('Target must contain three finite coordinates')
        if np.any(p < self.lower) or np.any(p > self.upper):
            raise ValueError('Target outside current Cartesian workspace')
        return p

    def _check_separation(self, targets):
        if len(targets) == 2 and np.linalg.norm(
                np.array(targets[1]) - targets[2]) <= 2 * self.tolerance:
            raise ValueError('Targets must have non-overlapping arrival regions')

    def _persist(self, targets):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                             delete=False) as stream:
                name = stream.name
                json.dump(dict(schema=1, frame='base_link', targets=targets), stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.path)
        finally:
            if name and os.path.exists(name):
                os.unlink(name)
        self.targets = targets

    def save(self, target, point, running):
        self._id(target)
        if running or target in self.targets:
            raise ValueError('Stop Run / Reset Target before saving')
        targets = dict(self.targets)
        targets[target] = self.point(point).tolist()
        self._check_separation(targets)
        self._persist(targets)
        self.reason = 'target_saved'

    def reset_targets(self, running):
        if running:
            raise ValueError('Stop Run before Reset Target')
        self._persist({})
        self.reset_run('targets_reset')

    def reset_run(self, reason='start_run'):
        selected = self.selected if self.test_mode and reason == 'start_run' else None
        self.role, self.selected, self.active = 'FOLLOWER', None, None
        self.selected = selected
        self.test_elapsed, self.test_fired = 0., False
        self.role_source = 'manual'
        self.bridge, self.force_follower = None, False
        self.leg = 0
        self.elapsed = self.duration = 0.
        self.start = self.goal = self.distance = None
        self._inside_since, self._reached = None, False
        self.reason = reason

    def select(self, target):
        self._id(target)
        if not self.enabled or self.role == 'LEADER' or len(self.targets) != 2:
            raise ValueError('Select a saved target in Hybrid FOLLOWER')
        if target != self.selected:
            self.selected = target
            self._inside_since, self._reached = None, False
            self.reason = 'target_selected'

    def lead(self, actual, reference=None, velocity=None, acceleration=None):
        if self.role == 'LEADER':
            return False
        if not self.enabled or len(self.targets) != 2 or self.selected is None:
            raise ValueError('Hybrid requires two saved targets and an explicit selection')
        actual = self.point(actual)
        goal = self.point(self.targets[self.selected])
        distance = float(np.linalg.norm(goal - actual))
        if distance <= self.tolerance:
            self.reason = 'already_at_target'
            return False
        start = actual.copy() if reference is None else self.point(reference).copy()
        # Historical Fitts duration plus analytic minimum-jerk peak v/a bounds.
        length = float(np.linalg.norm(goal-start))
        duration = max(self.fitts_a + self.fitts_b * math.log2(1 + distance / self.fitts_w),
                       1.875 * length / self.vmax, math.sqrt(5.774 * length / self.amax))
        bridge = None if velocity is None else Bridge(
            start, np.asarray(velocity), np.zeros(3) if acceleration is None else acceleration,
            goal, duration, self.vmax, self.amax, self.lower, self.upper)
        self.start, self.goal, self.bridge = start, goal.copy(), bridge
        self.duration = duration if bridge is None else duration-bridge.join+bridge.time
        self.force_follower = False
        self.elapsed = 0.
        self.role, self.active = 'LEADER', self.selected
        self.leg += 1
        self._inside_since, self._reached = None, False
        self.reason = 'manual_leader'
        self.role_source = 'manual'
        return True

    def follow(self, reason='skipped_by_user'):
        if self.role != 'LEADER':
            return False
        self.role, self.active = 'FOLLOWER', None
        self.force_follower = True
        self.test_fired = True
        self._inside_since = None
        self.reason = reason
        self.role_source = ('arrival' if reason == 'reached' else
                            'safety' if reason.startswith('fault:') else 'manual')
        return True

    def resume_after_hold(self, actual):
        """A safety HOLD stopped the robot: rebase, never jump to the old curve."""
        start = self.point(actual).copy()
        length = float(np.linalg.norm(self.goal-start))
        duration = max(.8, self.duration-self.elapsed, 1.875*length/self.vmax,
                       math.sqrt(5.774*length/self.amax))
        bridge = Bridge(start, np.zeros(3), np.zeros(3), self.goal, duration,
                        self.vmax, self.amax, self.lower, self.upper)
        self.start, self.bridge = start, bridge
        self.duration, self.elapsed = duration-bridge.join+bridge.time, 0.
        self.reason = 'resumed_after_force_hold'

    def observe(self, actual, speed, now):
        if self.selected is None:
            return False
        self.distance = float(np.linalg.norm(np.asarray(actual) - self.targets[self.selected]))
        if self.distance > self.tolerance or speed > self.speed:
            self._inside_since = None
            return False
        if self._inside_since is None:
            self._inside_since = now
        if self._reached or now - self._inside_since < self.dwell:
            return False
        self._reached = True
        self.reason = 'reached'
        return True

    def sample(self, dt):
        if self.role != 'LEADER':
            raise ValueError('No active MJM leg')
        if self.bridge is not None:
            self.elapsed += max(0., min(dt, .1))
            return self.bridge.state(self.elapsed)[0]
        s = min(self.elapsed / self.duration, 1.)
        out = self.start + (10*s**3 - 15*s**4 + 6*s**5) * (self.goal - self.start)
        self.elapsed += max(0., min(dt, .1))
        return out

    def snapshot(self):
        return dict(enabled=self.enabled, role=self.role, selected=self.selected,
                    test_mode=self.test_mode, test_elapsed=self.test_elapsed,
                    test_fired=self.test_fired, force_follower=self.force_follower,
                    control_source=('force_admittance' if self.force_follower else
                                    'mjm' if self.role == 'LEADER' else 'prediction_admittance'),
                    transition=('FOLLOWER_TO_LEADER' if self.role == 'LEADER' and
                                self.bridge is not None and self.elapsed < self.bridge.time else ''),
                    bridge_time=None if self.bridge is None else self.bridge.time,
                    bridge_continuity=None if self.bridge is None else self.bridge.continuity,
                    bridge_join=None if self.bridge is None else self.bridge.join,
                    bridge_join_velocity_error=None if self.bridge is None else
                    self.bridge.join_velocity_error,
                    bridge_minimum_speed=None if self.bridge is None else
                    self.bridge.minimum_speed,
                    bridge_speed_drop=None if self.bridge is None else
                    self.bridge.speed_drop,
                    active=self.active, targets=self.targets, frame='base_link',
                    leg=self.leg, reason=self.reason, elapsed=self.elapsed,
                    duration=self.duration, distance=self.distance,
                    start=None if self.start is None else self.start.tolist(),
                    goal=None if self.goal is None else self.goal.tolist(),
                    target_source='manual', role_source=self.role_source)
