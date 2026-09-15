import json
import numpy as np
import pytest

from cocarry_admittance_control.manual_hybrid import ManualHybrid


@pytest.fixture
def hybrid(tmp_path):
    h = ManualHybrid(tmp_path / 'targets.json', [-1, -1, 0], [1, 1, 1])
    h.save(1, [.2, .4, .5], False)
    h.save(2, [-.2, .4, .5], False)
    h.enabled = True
    return h


def test_persistence_and_reset_start(hybrid):
    h = hybrid
    h.select(2)
    h.lead([0, .4, .5])
    h.reset_run()
    assert h.selected is None and h.role == 'FOLLOWER'
    loaded = ManualHybrid(h.path, [-1, -1, 0], [1, 1, 1])
    assert loaded.targets == h.targets
    assert loaded.selected is None
    with pytest.raises(ValueError):
        h.save(1, [0, 0, .5], False)
    with pytest.raises(ValueError):
        h.reset_targets(True)
    h.reset_targets(False)
    assert json.loads(h.path.read_text())['targets'] == {}


def test_explicit_selection_no_autoswitch(hybrid):
    h = hybrid
    with pytest.raises(ValueError):
        h.lead([0, .4, .5])
    h.select(2)
    assert h.role == 'FOLLOWER'
    h.follow()
    assert h.selected == 2
    assert h.lead([0, .4, .5])
    assert not h.lead([0, .4, .5])
    assert h.leg == 1
    with pytest.raises(ValueError):
        h.select(1)
    h.follow()
    assert h.selected == 2 and h.reason == 'skipped_by_user'
    assert not h.follow()
    h.select(1)
    assert h.lead([0, .4, .5])
    assert h.active == 1 and h.leg == 2


def test_actual_arrival_dwell_speed_and_once(hybrid):
    h = hybrid
    h.select(1)
    p = h.targets[1]
    assert not h.observe(p, .2, 0.)
    assert not h.observe(p, 0., 1.)
    assert not h.observe(p, 0., 1.4)
    assert h.observe(p, 0., 1.6)
    assert not h.observe(p, 0., 2.)
    assert h.selected == 1 and h.role == 'FOLLOWER'
    assert not h.lead(p)
    assert h.reason == 'already_at_target'


def test_mjm_endpoint_and_bounds(hybrid):
    h = hybrid
    h.select(1)
    p = np.array([-.2, .4, .5])
    h.lead(p)
    samples = np.array([h.sample(.001) for _ in range(int(h.duration/.001)+3)])
    assert np.allclose(samples[0], p)
    assert np.allclose(samples[-1], h.targets[1])
    velocity = np.diff(samples, axis=0)/.001
    acceleration = np.diff(velocity, axis=0)/.001
    assert np.linalg.norm(velocity, axis=1).max() <= h.vmax * 1.001
    assert np.linalg.norm(acceleration, axis=1).max() <= h.amax * 1.001
    assert h.role == 'LEADER'  # elapsed duration is NOT actual arrival


@pytest.mark.parametrize('point', [[float('nan'), 0, .5], [2, 0, .5], [0, 0]])
def test_invalid_target(hybrid, point):
    hybrid.reset_targets(False)
    with pytest.raises(ValueError):
        hybrid.save(1, point, False)
    assert not hybrid.targets


def test_target_regions_cannot_overlap(hybrid):
    hybrid.reset_targets(False)
    hybrid.save(1, [0, 0, .5], False)
    with pytest.raises(ValueError):
        hybrid.save(2, [.01, 0, .5], False)
