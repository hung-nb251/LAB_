import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from trajectory_predictor.feature_math import per_sample_displacement
from trajectory_predictor.inference_schedule import InferenceSchedule


def test_all_uniform_samples_survive_small_timer_jitter():
    schedule = InferenceSchedule(15.0)
    timestamps = 1000 + np.arange(3000) / 15
    timestamps += np.random.default_rng(42).normal(0.0, 0.0001, len(timestamps))
    assert all(schedule.ready(t) for t in timestamps)


def test_higher_rate_input_is_limited_without_cumulative_drift():
    schedule = InferenceSchedule(15.0)
    accepted = [t for t in np.arange(0.0, 10.0, 0.01) if schedule.ready(t)]
    assert abs(len(accepted) - 150) <= 1
    assert min(np.diff(accepted)) >= 0.05


def test_long_gap_sends_once_without_catch_up_burst():
    schedule = InferenceSchedule(15.0)
    assert schedule.ready(0.0)
    assert schedule.ready(8.0)
    assert not schedule.ready(8.001)
    assert not schedule.ready(8.002)
    assert schedule.ready(8.0 + 1 / 15)


def test_reset_allows_immediate_first_sample():
    schedule = InferenceSchedule(15.0)
    assert schedule.ready(100.0)
    assert not schedule.ready(100.001)
    schedule.reset()
    assert schedule.ready(100.002)


def test_unlimited_camera_profile_accepts_every_sample():
    schedule = InferenceSchedule(0.0)
    assert all(schedule.ready(t) for t in np.arange(100) / 1000)


@pytest.mark.parametrize('rate', [-1, float('nan'), float('inf')])
def test_invalid_rates_rejected(rate):
    with pytest.raises(ValueError):
        InferenceSchedule(rate)


def ingest_method(clock):
    """Exercise the production callback without ROS or a model subprocess."""
    path = Path(__file__).parents[1] / 'trajectory_predictor/predictor_node.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == '_ingest_point')
    env = dict(time=clock, np=np, per_sample_displacement=per_sample_displacement)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    return env['_ingest_point']


def test_cocarry_stationary_input_keeps_predicting_and_retains_full_history():
    clock = SimpleNamespace(value=1000.0)
    clock.time = clock.monotonic = lambda: clock.value
    requests = []
    def unexpected_hold(*args):
        pytest.fail('Disabled stationary HOLD was invoked')
    state = SimpleNamespace(
        _predicting=True, _hybrid_enabled=False, num_features=6,
        _velocity_feature_mode='delta_position', _last_data_time=0.0,
        _last_meas=[0., 0., 0.], _buffer=deque(maxlen=20),
        _hold_enabled=False, _time_based_hold=True,
        _hold_detector=SimpleNamespace(update=unexpected_hold),
        _legacy_prediction_hold=unexpected_hold,
        _worker_ready=True, _inference_schedule=InferenceSchedule(15.0),
        window_size=20, _predict_epoch=7, _send_to_worker=requests.append, _reentry_token=0,
    )
    ingest = ingest_method(clock)
    for i in range(100):
        clock.value = 1000 + i / 15 + (0.0001 if i % 2 else 0.0)
        ingest(state, 0., 0., 0.)
    assert len(requests) == 100
    clock.value = 1000 + 100 / 15
    ingest(state, .002, 0., 0.)
    assert len(requests) == 101
    history = np.asarray(requests[-1]['data'])
    assert history.shape == (20, 6)
    assert np.allclose(history[:-1], 0.0)
    assert np.allclose(history[-1], [.002, 0., 0., .002, 0., 0.])
    assert requests[-1]['epoch'] == 7


def test_camera_legacy_hold_is_still_called_when_enabled():
    clock = SimpleNamespace(time=lambda: 1000., monotonic=lambda: 1000.)
    calls = []
    state = SimpleNamespace(
        _predicting=True, _hybrid_enabled=False, num_features=3,
        _buffer=deque(maxlen=20), _hold_enabled=True, _time_based_hold=False,
        _legacy_prediction_hold=lambda *p: calls.append(p), _hold_active=True,
    )
    ingest_method(clock)(state, .1, .2, .3)
    assert calls == [(.1, .2, .3)]
