"""Run the production smoother with an inert clock and Cartesian messages."""
import ast
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np


class Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position or NS(x=0., y=0., z=0.)
        self.orientation = orientation or NS(x=0., y=0., z=0., w=1.)


def timed_pose_selector():
    path = Path(__file__).parents[1] / 'scripts/cartesian_streamer_hc10dtp.py'
    tree = ast.parse(path.read_text())
    function = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef)
                    and n.name == 'advance_timed_pose')
    env = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), env)
    return env['advance_timed_pose']


def smoother(velocity=(0., 0., 0.), continuous=True):
    path = Path(__file__).parents[1] / 'scripts/cartesian_streamer_hc10dtp.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == '_smooth_pose')
    env = dict(Pose=Pose, Point=NS, math=math, MAX_CARTESIAN_VELOCITY=.15,
               MAX_CARTESIAN_ACCELERATION=.5, MAX_CARTESIAN_JERK=10.)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    clock = NS(nanoseconds=1_000_000_000)
    state = NS(_current_ee_pose=Pose(), _prev_ee_velocity=list(velocity),
               _prev_ee_acceleration=[0., 0., 0.],
               _last_smooth_time_ns=clock.nanoseconds, _stream_period_sec=1 / 15,
               _continuous_cartesian_smoothing=continuous,
               get_clock=lambda: NS(now=lambda: clock),
               _slerp_quat=lambda a, b, t: b)
    def step(target, dt=1 / 15):
        clock.nanoseconds += round(dt * 1e9)
        result = env['_smooth_pose'](state, Pose(position=NS(
            x=target[0], y=target[1], z=target[2])))
        state._current_ee_pose = result
        return np.array([result.position.x, result.position.y, result.position.z])
    return state, step


def test_reversal_brakes_without_19mm_snap():
    state, step = smoother(velocity=(-.1, 0., 0.))
    position = step((.019, 0., 0.))
    # Finite braking initially travels in the old direction; it cannot teleport.
    assert -.007 < position[0] < 0
    assert np.isclose(position[0], state._prev_ee_velocity[0] / 15)
    for _ in range(100):
        position = step((.019, 0., 0.))
    assert np.linalg.norm(position - [.019, 0., 0.]) < 1e-5
    assert np.linalg.norm(state._prev_ee_velocity) < 1e-5


def test_small_final_gap_does_not_invent_velocity_or_reset_it_early():
    state, step = smoother()
    position = step((.0015, 0., 0.))
    assert np.isclose(position[0], .0015)
    assert np.isclose(state._prev_ee_velocity[0], .0015 / (round(1e9/15)/1e9))
    step((.0015, 0., 0.))
    assert np.linalg.norm(state._prev_ee_velocity) < 1e-8


def test_positions_obey_velocity_and_acceleration_bounds_during_xyz_reversals():
    state, step = smoother()
    rng = np.random.default_rng(43)
    position = np.zeros(3)
    velocity = np.zeros(3)
    target = np.array([.12, -.09, .08])
    for i in range(600):
        if i % 25 == 0:
            target = position + rng.uniform(-.019, .019, 3)
        dt = round(float(rng.uniform(.05, .085)) * 1e9) / 1e9
        new = step(target, dt)
        actual_velocity = (new - position) / dt
        assert np.linalg.norm(actual_velocity) <= .15 + 1e-9
        assert np.linalg.norm(actual_velocity - velocity) / dt <= .5 + 1e-8
        assert np.allclose(actual_velocity, state._prev_ee_velocity, atol=1e-9)
        position, velocity = new, actual_velocity
    for _ in range(120):
        position = step(target)
    assert np.linalg.norm(position - target) < 1e-5


def test_continuous_mode_keeps_jerk_bound_inside_twenty_mm():
    state, step = smoother()
    previous_acceleration = np.zeros(3)
    for target in ((.019, 0., 0.), (-.019, 0., 0.)) * 12:
        step(target)
        acceleration = np.array(state._prev_ee_acceleration)
        assert np.max(np.abs(acceleration - previous_acceleration)) \
            <= 10.0 / 15 + 1e-7
        previous_acceleration = acceleration


def test_camera_profile_keeps_historical_behavior_until_separate_review():
    _, step = smoother(velocity=(-.1, 0., 0.), continuous=False)
    assert np.allclose(step((.019, 0., 0.)), [.019, 0., 0.])


def test_tracking_uses_latest_pose_that_is_due_not_newest_queued_pose():
    select = timed_pose_selector()
    history = deque([(100, 'p1'), (200, 'p2'), (300, 'p3')])
    assert select(history, 'seed', 250) == 'p2'
    assert list(history) == [(300, 'p3')]
    assert select(history, 'p2', 299) == 'p2'
    assert select(history, 'p2', 300) == 'p3'
