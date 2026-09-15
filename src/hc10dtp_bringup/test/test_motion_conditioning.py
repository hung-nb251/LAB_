import ast
from collections import deque
from pathlib import Path
import sys
import threading
import math
from types import MethodType
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from motion_conditioning import synchronized_joint_step, sample_timed_position
from local_ik_solver import LocalIKSolver


def test_limited_wrist_preserves_joint_direction_and_does_not_speed_up_other_joints():
    target = np.array([.02, -.01, .025, .02, -.01, .025])
    caps = np.array([.5, .5, .5, .08, .5, .4])
    q, v, scale = synchronized_joint_step([0.] * 6, target, 1 / 15, caps)
    assert scale == pytest.approx(.08 / (.02 * 15))
    assert np.allclose(q, target * scale)
    assert np.all(np.abs(v) <= caps + 1e-12)
    assert np.allclose(q, np.array(v) / 15)
    assert np.linalg.norm(np.cross(np.array(q)[:3], target[:3])) < 1e-12


def test_unlimited_step_is_exact_and_stationary_target_is_zero():
    q = [.1] * 6
    target = [.1001] * 6
    out, vel, scale = synchronized_joint_step(q, target, .06, [.5] * 6)
    assert out == target
    assert scale == 1
    assert synchronized_joint_step(q, q, .06, [.5] * 6) == (q, [0.] * 6, 1.)


def test_random_reversals_stay_on_segment_and_within_velocity_limits():
    rng = np.random.default_rng(20260915)
    caps = np.array([.5, .5, .5, .08, .5, .4])
    q = rng.normal(size=6)
    for _ in range(500):
        target = q + rng.normal(size=6) * .08
        dt = rng.uniform(.03, .10)
        new, v, scale = synchronized_joint_step(q, target, dt, caps)
        assert np.allclose(new, q + scale * (target - q))
        assert np.all(np.abs(v) <= caps + 1e-12)
        assert 0 < scale <= 1
        q = np.array(new)


@pytest.mark.parametrize('dt,caps', [(0, [.5]*6), (.1, [float('nan')]*6),
                                    (.1, [-1.]*6)])
def test_invalid_motion_input_rejected(dt, caps):
    with pytest.raises(ValueError):
        synchronized_joint_step([0.]*6, [.1]*6, dt, caps)


def test_tracking_interpolates_from_endpoint_not_previous_sample():
    history = deque([(0, [0., 0., 0.]), (100, [1., 2., 3.])])
    p, anchor = sample_timed_position(history, None, 25)
    assert p == [.25, .5, .75]
    p, anchor = sample_timed_position(history, anchor, 50)
    assert p == [.5, 1., 1.5]
    p, anchor = sample_timed_position(history, anchor, 200)
    assert p == [1., 2., 3.]  # no extrapolation or lag-following epoch shift
    assert not history


def test_tracking_never_uses_future_point_without_an_anchor():
    history = deque([(100, [1., 2., 3.])])
    assert sample_timed_position(history, None, 99) == (None, None)
    assert len(history) == 1


def test_busy_retry_keeps_reserved_timestamp_and_releases_gate_after_state_update():
    # Execute the production callback without ROS. BUSY must not change the
    # cumulative clock or advance accepted joint/FK state.
    path = Path(__file__).parents[1] / 'scripts/cartesian_streamer_hc10dtp.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == '_on_queue_result')
    env = {'JointTrajectoryPoint': object}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    state = NS(_last_call_time_ns=10, _send_lock=threading.Lock(),
               _queue_call_inflight=True, _cumulative_time_ns=200_000_000,
               _pending_point_to_resend=None, _window_busy_count=0,
               _window_retry_count=0, _retry_backoff_sec=.066,
               get_clock=lambda: NS(now=lambda: NS(nanoseconds=300_000_000)))
    def warn(*args, **kwargs):
        assert state._queue_call_inflight  # not released midway through ACK
    state.get_logger = lambda: NS(warn=warn, error=lambda s: pytest.fail(s))
    point = object()
    for _ in range(3):
        state._queue_call_inflight = True
        env['_on_queue_result'](state, NS(result=lambda: NS(
            result_code=NS(value=4), message='busy')), point)
        assert state._cumulative_time_ns == 200_000_000
        assert state._pending_point_to_resend is point
        assert not state._queue_call_inflight


def test_production_send_ack_chain_keeps_schedule_and_feeds_back_accepted_fk_velocity():
    """Exercise production send/ACK/reset with fake RPC and real local FK."""
    path = Path(__file__).parents[1] / 'scripts/cartesian_streamer_hc10dtp.py'
    tree = ast.parse(path.read_text())
    names = ('_send_joint_point', '_on_queue_result', '_record_accepted_tracking_pose',
             '_reset_tracking_schedule')
    methods = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name in names]
    class Time:
        def __init__(self, ns):
            self.nanoseconds = ns
        def __sub__(self, other):
            return Time(self.nanoseconds - other.nanoseconds)
    clock = NS(ns=1_000_000_000)
    calls, callbacks = [], []
    def call(request):
        calls.append(request.point)
        return NS(add_done_callback=callbacks.append)
    client = NS(call_async=call)
    caps = [.5, .5, .5, .08, .5, .4]
    env = dict(JointTrajectoryPoint=NS, QueueTrajPoint=NS(Request=NS), Duration=NS,
               JOINT_NAMES=list(range(6)), MAX_JOINT_VELOCITIES=caps,
               synchronized_joint_step=synchronized_joint_step,
               _time=NS(monotonic_ns=lambda: clock.ns),
               STREAM_STATE_SEEDING='seeding', STREAM_STATE_PREBUFFERING='prebuffering',
               STREAM_STATE_STREAMING='streaming')
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    solver = LocalIKSolver()
    def fk(q):
        p = solver.fk_position(np.array(q))
        return NS(position=NS(x=p[0], y=p[1], z=p[2]))
    q = [1.57, .7, .4, 0., -1.3, 0.]
    s = NS(_send_lock=threading.Lock(), _queue_call_inflight=False,
           _last_call_time_ns=0, _stream_state='streaming',
           _joint_coordination='synchronized', _select_queue_client=lambda: client,
           _active_queue_service_name='fake', _queue_dt_sec=1/15,
           _pending_point_to_resend=None, _last_send_time_ns=clock.ns,
           _cumulative_time_ns=0, _last_queued_joints=q,
           _current_joints=q, _motion_diagnostics={}, _hold_point_count=0,
           _queue_sent_count=0, _min_send_interval_ns=0,
           _last_accepted_point_time_ns=0, _accepted_ee_velocity=[0.]*3,
           _prev_ee_velocity=[1.]*3, _prev_ee_acceleration=[0.]*3,
           _queued_ee_pose=fk(q), _solve_fk_local_as_pose=fk,
           _tracking_history_lock=threading.Lock(), _tracking_pose_history=deque(),
           _tracking_xyz_history=deque(), _tracking_xyz_anchor=None,
           _queue_epoch_monotonic_ns=clock.ns, _accepted_points=0,
           _window_ack_count=0, _last_ack_time=None, _window_ack_interval_sum=0.,
           _window_ack_interval_count=0, _queue_debug_log_count=5,
           _window_busy_count=0, _window_retry_count=0, _retry_backoff_sec=.066,
           get_clock=lambda: NS(now=lambda: Time(clock.ns)),
           get_logger=lambda: NS(info=lambda *a, **kw: None,
                                 warn=lambda *a, **kw: None,
                                 error=lambda text: pytest.fail(text)))
    for name in names:
        setattr(s, name, MethodType(env[name], s))
    def ack(code):
        callbacks.pop(0)(NS(result=lambda: NS(result_code=NS(value=code), message='')))
    clock.ns += 66_666_667
    target = np.array(q) + [.03, .01, .04, .025, -.02, -.03]
    s._send_joint_point(target)
    first = calls[-1]
    assert s._last_queued_joints == q  # no update before ACK
    old_pose = solver.fk_position(np.array(q))
    ack(4)
    reserved = s._cumulative_time_ns
    clock.ns += 66_666_667
    s._send_joint_point(target)
    assert calls[-1] is first
    ack(0)
    assert s._cumulative_time_ns == reserved
    dt = reserved / 1e9
    assert np.allclose(s._prev_ee_velocity,
                       (solver.fk_position(np.array(first.positions)) - old_pose) / dt)
    assert np.all(np.abs(first.velocities) <= np.array(caps) + 1e-12)
    assert not s._queue_call_inflight
    clock.ns += 66_666_667
    s._send_joint_point(target)
    second = calls[-1]
    assert second.time_from_start.nanosec > first.time_from_start.nanosec
    ack(0)
    s._reset_tracking_schedule()
    assert not s._tracking_xyz_history
    assert s._tracking_xyz_anchor is None
    assert s._last_accepted_point_time_ns is None
    assert s._accepted_ee_velocity == [0.]*3
