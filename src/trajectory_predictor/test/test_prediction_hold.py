import numpy as np
import pytest

from trajectory_predictor.prediction_hold import TimeBasedPredictionHold


def test_stationary_detection_depends_on_elapsed_time_not_sample_count():
    hold = TimeBasedPredictionHold(0.4, 0.003, 0.010)
    for timestamp in (0.0, 0.1, 0.2, 0.3):
        state, _ = hold.update((0.0, 0.0, 0.0), timestamp)
        assert state == 'tracking'
    state, target = hold.update((0.0, 0.0, 0.0), 0.4)
    assert state == 'entered'
    assert np.allclose(target, 0.0)


def test_smooth_motion_is_not_misclassified_as_stationary():
    hold = TimeBasedPredictionHold(0.4, 0.003, 0.010)
    states = []
    for i in range(9):
        state, _ = hold.update((0.002 * i, 0.0, 0.0), 0.05 * i)
        states.append(state)
    assert 'entered' not in states


def test_hold_target_is_measurement_and_releases_after_real_motion():
    hold = TimeBasedPredictionHold(0.4, 0.003, 0.010)
    position = np.array((0.10, -0.20, 0.30))
    for i in range(5):
        state, target = hold.update(position, 0.1 * i)
    assert state == 'entered'
    assert np.allclose(target, position)
    state, target = hold.update(position + (0.011, 0.0, 0.0), 0.5)
    assert state == 'released'
    assert target is None


def test_invalid_thresholds_are_rejected():
    with pytest.raises(ValueError):
        TimeBasedPredictionHold(0.4, 0.003, 0.002)
