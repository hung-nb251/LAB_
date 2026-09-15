"""Check the actual input callback, including delta features at release."""
import ast
import json
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
import numpy as np
from test_inference_schedule import ingest_method
from trajectory_predictor.inference_schedule import InferenceSchedule


def test_release_clears_history_not_coordinates_and_counts_unique_robot_samples():
    clock = NS(value=100.)
    clock.time = clock.monotonic = lambda: clock.value
    requests = []
    state = NS(_predicting=True, _hybrid_enabled=False, num_features=6,
               _velocity_feature_mode='delta_position', _last_data_time=99.,
               _last_meas=[99., 99., 99.], _buffer=deque([[99.]*6]*20, maxlen=20),
               _hold_enabled=False, _time_based_hold=True, _worker_ready=True,
               _inference_schedule=InferenceSchedule(15.), window_size=20,
               _predict_epoch=4, _send_to_worker=requests.append,
               _reentry_token=0, _reentry_samples=0, _reentry_last_stamp=0,
               get_parameter=lambda name: NS(value=True))
    path = Path(__file__).parents[1]/'trajectory_predictor/predictor_node.py'
    tree = ast.parse(path.read_text())
    env = dict(json=json, np=np, HandState=object)
    for name in ('_on_reentry_request', '_on_hand'):
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
        setattr(state, name, MethodType(env[name], state))
    state._ingest_point = MethodType(ingest_method(clock), state)
    token = int(clock.value*1e9)
    request = NS(data=json.dumps(dict(enabled=True, controller_state='RUNNING', reentry_token=token)))
    state._on_reentry_request(request)
    assert not state._buffer and state._predict_epoch == 5
    for i in range(1, 21):
        clock.value += 1/15
        stamp = int(clock.value*1e9)
        point = NS(is_tracked=True, source='robot_ee', x=.1+i*.001, y=.2, z=.3,
                   header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)))
        state._on_hand(point)
        state._on_hand(point)  # Duplicate DDS input must not count twice.
        state._on_reentry_request(request)  # Repeated status must not reset.
    assert state._reentry_samples == 20 and state._predict_epoch == 5
    assert requests[0]['reentry']['samples'] == 1
    assert requests[-1]['reentry']['samples'] == 20
    window = np.asarray(requests[-1]['data'])
    assert window.shape == (20, 6)
    assert np.allclose(window[0], [.101, .2, .3, 0, 0, 0])
    assert np.allclose(window[1:, 3], .001)
    assert np.max(window[:, :3]) < 1  # No LEADER history in the qualified window.
    point.header.stamp.sec = 99
    state._on_hand(point)
    assert state._reentry_samples == 20
    assert requests[-1]['reentry']['required_samples'] == 10

    # Manual LEADER invalidates pending inference and does not enqueue work.
    leader = dict(enabled=True, controller_state='RUNNING', role='LEADER',
                  stamp_ns=token+100, reentry_token=0)
    state._on_reentry_request(NS(data=json.dumps(leader)))
    assert state._manual_leader and state._predict_epoch == 6
    count = len(requests)
    clock.value += .1
    state._ingest_point(.13, .2, .3)
    assert len(requests) == count
    # Out-of-order FOLLOWER status cannot undo the current LEADER phase.
    state._on_reentry_request(NS(data=json.dumps(dict(
        leader, role='FOLLOWER', stamp_ns=token+99))))
    assert state._manual_leader
    new_token = int(clock.value*1e9)
    state._on_reentry_request(NS(data=json.dumps(dict(
        leader, role='FOLLOWER', stamp_ns=token+101, reentry_token=new_token))))
    assert not state._manual_leader and not state._buffer
    assert state._reentry_token == new_token and state._reentry_samples == 0
