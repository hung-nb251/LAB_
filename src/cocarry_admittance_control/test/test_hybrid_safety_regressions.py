import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_manual_hybrid_controller import controller, cmd, tick
from test_prediction_reentry import release, qualified


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_bad_force_faults_without_publishing(controller, bad):
    c, clock = controller
    count = len(c._target_pub.messages)
    c._on_force(NS(vector=NS(x=bad, y=0., z=0.)))
    assert c._state == 'FAULT'
    assert c._manual.reason.startswith('fault:')
    assert len(c._target_pub.messages) == count
    assert np.all(np.isfinite(c._force))


def test_final_reference_guard(controller):
    c, clock = controller
    count = len(c._target_pub.messages)
    c._publish_reference(np.zeros(3), np.zeros(3), np.array([np.nan, 0, 0]))
    assert c._state == 'FAULT'
    assert len(c._target_pub.messages) == count


def test_stop_clears_follower_warmup_and_reason(controller):
    c, clock = controller
    release(c, clock)
    c._stop_run()
    assert c._manual.reason == 'stopped'
    assert not c._manual.force_follower
    assert c._reentry_phase == 'OFF' and c._reentry_token == 0
    c._manual_status()
    assert json.loads(c._manual_pub.messages[-1].data)['control_phase'] == 'STOPPED'


def test_invalid_mode_request_is_atomic(controller):
    c, clock = controller
    c._state = 'STOPPED'
    before = c._manual.enabled
    cmd(c, 'mode', enabled=not before, test_mode='invalid')
    assert c._manual.enabled == before


def test_blend_limiter_reconciles_both_branch_positions(controller):
    c, clock = controller
    release(c, clock)
    tick(c, clock)
    qualified(c, clock, c._reentry_warmup_samples)
    tick(c, clock)
    # Stress hidden states while the two branches have nonzero weights.
    c._free_admittance.error[0] += .2
    c._admittance.error[0] += .2
    elapsed = c._reentry_elapsed
    s = min(elapsed/c._reentry_blend_sec, 1.)
    weight = 10*s**3-15*s**4+6*s**5
    tick(c, clock)
    nominal_msg = c._nominal_pub.messages[-1].point
    nominal = np.array([nominal_msg.x, nominal_msg.y, nominal_msg.z])
    reconstructed = ((1-weight)*(c._free_origin+c._free_admittance.error)
                     + weight*(nominal+c._admittance.error))
    assert not c._fault
    assert np.allclose(reconstructed, c._emitted_position)
