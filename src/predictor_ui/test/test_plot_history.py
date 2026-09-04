from collections import deque

import pytest

from predictor_ui.plot_history import append_time_window_xyz


def _buffers():
    return {axis: deque() for axis in ('x', 'y', 'z')}, deque()


def test_high_rate_actual_ee_keeps_the_full_time_window():
    buffers, timestamps = _buffers()

    for index in range(2001):
        timestamp = index / 100.0
        append_time_window_xyz(
            buffers, timestamps, timestamp,
            (timestamp, 2.0 * timestamp, 3.0 * timestamp),
            window_sec=20.0,
        )

    assert len(timestamps) == 2001
    assert timestamps[0] == pytest.approx(0.0)
    assert timestamps[-1] == pytest.approx(20.0)
    assert len(buffers['x']) == len(buffers['y']) == len(buffers['z']) == 2001


def test_samples_older_than_the_window_are_removed_in_lockstep():
    buffers, timestamps = _buffers()
    for timestamp in (0.0, 5.0, 10.0, 20.01):
        append_time_window_xyz(
            buffers, timestamps, timestamp,
            (timestamp, timestamp + 1.0, timestamp + 2.0),
            window_sec=20.0,
        )

    assert list(timestamps) == pytest.approx([5.0, 10.0, 20.01])
    assert list(buffers['x']) == pytest.approx([5.0, 10.0, 20.01])
    assert list(buffers['y']) == pytest.approx([6.0, 11.0, 21.01])
    assert list(buffers['z']) == pytest.approx([7.0, 12.0, 22.01])


def test_invalid_window_is_rejected():
    buffers, timestamps = _buffers()

    with pytest.raises(ValueError):
        append_time_window_xyz(
            buffers, timestamps, 0.0, (0.0, 0.0, 0.0), window_sec=0.0)
