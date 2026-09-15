"""Production controller: no DDS and no physical robot commands."""
import json
import numpy as np
import pytest
from types import SimpleNamespace as NS
from test_manual_hybrid_controller import controller, cmd, tick


def release(c, clock):
    cmd(c, 'select', target=1)
    cmd(c, 'leader')
    tick(c, clock)
    cmd(c, 'follower')
    assert c._reentry_phase == 'WAIT'


def qualified(c, clock, samples, prediction=(1., 0., 0.), **overrides):
    data = dict(token=c._reentry_token, samples=samples,
                input_stamp_ns=int(clock[0]*1e9), prediction=prediction, model='gru')
    data.update(overrides)
    c._on_reentry_prediction(NS(data=json.dumps(data)))


@pytest.mark.parametrize('prediction', [(1., 0., 0.), (-1., .5, 0.)])
def test_warmup_fresh_samples_then_bounded_blend(controller, prediction):
    c, clock = controller
    release(c, clock)
    n = c._reentry_warmup_samples
    for count in range(1, n):
        tick(c, clock)
        qualified(c, clock, count, prediction)
        assert c._reentry_phase == 'WAIT' and c._prediction is None
    tick(c, clock)
    qualified(c, clock, n, prediction)
    previous_velocity = c._emitted_velocity.copy()
    for i in range(1, 31):
        tick(c, clock)
        qualified(c, clock, n+i, prediction)
        assert not c._fault
        assert np.linalg.norm(c._emitted_velocity) <= c._admittance.max_velocity+1e-8
        assert np.linalg.norm(c._emitted_velocity-previous_velocity)*15 <= c._admittance.max_acceleration+1e-7
        previous_velocity = c._emitted_velocity.copy()
        if i == 1:
            assert c._reentry_phase == 'BLEND'
            assert np.linalg.norm(c._emitted_velocity) < 1e-8
    assert c._reentry_phase == 'ACTIVE' and not c._manual.force_follower
    assert np.allclose(c._admittance.stiffness, [5, 5, 5])


def test_wrong_epoch_late_and_duplicate_results_never_reenable(controller):
    c, clock = controller
    release(c, clock)
    n = c._reentry_warmup_samples
    tick(c, clock)
    qualified(c, clock, n, token=c._reentry_token-1)
    qualified(c, clock, n, input_stamp_ns=c._reentry_token)
    qualified(c, clock, n, prediction=[float('nan'), 0, 0])
    assert c._prediction is None and c._reentry_samples == 0
    qualified(c, clock, n - 1)
    qualified(c, clock, n)  # Same input timestamp: not a new sample/window.
    assert c._prediction is None and c._reentry_samples == n - 1
    for _ in range(15):
        tick(c, clock)
    qualified(c, clock, n, input_stamp_ns=int((clock[0]-1)*1e9))
    assert c._reentry_phase == 'WAIT' and not c._fault


def test_stop_and_force_hold_during_blend_invalidate_reentry(controller):
    c, clock = controller
    release(c, clock)
    n = c._reentry_warmup_samples
    tick(c, clock)
    qualified(c, clock, n)
    tick(c, clock)
    assert c._reentry_phase == 'BLEND'
    token = c._reentry_token
    clock[0] += .3
    c._current_pose_time = clock[0]
    c._control_tick()
    assert c._force_stale_active and c._reentry_phase == 'WAIT'
    assert c._reentry_token > token
    tick(c, clock)
    qualified(c, clock, n + 1, token=token)
    assert c._prediction is None
    c._stop_run()
    count = len(c._target_pub.messages)
    tick(c, clock)
    qualified(c, clock, n)
    assert c._reentry_phase == 'OFF' and len(c._target_pub.messages) == count


def test_prediction_timeout_is_restored_after_reentry(controller):
    c, clock = controller
    release(c, clock)
    n = c._reentry_warmup_samples
    tick(c, clock)
    qualified(c, clock, n)
    for i in range(15):
        tick(c, clock)
        qualified(c, clock, n + 1 + i)
    assert c._reentry_phase == 'ACTIVE'
    for _ in range(12):
        tick(c, clock)
    assert 'position timeout' in c._fault
