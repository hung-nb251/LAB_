"""Timing and input-integrity checks for offline force replay."""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from axia_force_replay_ui import ReplayClock, load_recording


def test_epoch_nanoseconds_preserve_precision_duplicates_and_gaps(tmp_path):
    path = tmp_path / 'force.csv'
    path.write_text(
        'ros_timestamp_ns,f_human_x,f_human_y,f_human_z\n'
        '1790074592471522345,1,2,3\n'
        '1790074592471522346,-4,5,6\n'
        '1790074592471522346,7,8,9\n'
        '1790074594471522345,0,0,0\n', encoding='utf-8')
    data = load_recording(path)
    np.testing.assert_array_equal(data.seconds, [0, 1e-9, 1e-9, 2])
    np.testing.assert_array_equal(data.forces[1], [-4, 5, 6])
    assert data.duration == 2
    assert data.rate_hz == 1.5


@pytest.mark.parametrize('last_row', [
    '99,1,2,3', '101,nan,2,3', 'NaN,1,2,3', '101,1,,3', '101,1,2,inf',
])
def test_invalid_data_is_reported_with_line_number(tmp_path, last_row):
    path = tmp_path / 'bad.csv'
    path.write_text('time_s,f_human_x,f_human_y,f_human_z\n100,1,2,3\n'
                    + last_row + '\n', encoding='utf-8')
    with pytest.raises(ValueError, match='Dòng CSV 3'):
        load_recording(path)


def test_custom_columns_and_units(tmp_path):
    path = tmp_path / 'custom.csv'
    path.write_text('t,Fx,Fy,Fz\n1234,1,2,3\n1254,4,5,6\n', encoding='utf-8-sig')
    data = load_recording(path, 't', 'ms', ('Fx', 'Fy', 'Fz'))
    np.testing.assert_array_equal(data.seconds, [0, .02])
    with pytest.raises(ValueError, match='đơn vị'):
        load_recording(path, 't', force_columns=('Fx', 'Fy', 'Fz'))


def test_missing_columns_and_empty_recording(tmp_path):
    path = tmp_path / 'empty.csv'
    path.write_text('time_s,f_human_x,f_human_y\n', encoding='utf-8')
    with pytest.raises(ValueError, match='f_human_z'):
        load_recording(path)
    path.write_text('time_s,f_human_x,f_human_y,f_human_z\n', encoding='utf-8')
    with pytest.raises(ValueError, match='không có mẫu'):
        load_recording(path)


def test_monotonic_playback_pause_seek_speed_and_late_tick():
    wall = [100.0]
    clock = ReplayClock(10, now=lambda: wall[0])
    clock.play()
    wall[0] += .07
    assert clock.position() == pytest.approx(.07)
    wall[0] += 2.0  # A delayed UI tick must catch up, without accumulating drift.
    assert clock.position() == pytest.approx(2.07)
    clock.pause()
    wall[0] += 100
    assert clock.position() == pytest.approx(2.07)
    clock.seek(4)
    clock.play()
    wall[0] += 1
    clock.set_speed(2)
    assert clock.position() == pytest.approx(5)
    wall[0] += 1
    assert clock.position() == pytest.approx(7)
    clock.seek(1)
    wall[0] += 1
    assert clock.position() == pytest.approx(3)
    wall[0] += 20
    assert clock.position() == 10
    clock.pause()
    clock.play()
    assert clock.position() == 0


def test_offscreen_ui_includes_all_due_samples_and_stops_at_end(tmp_path, monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5 import QtWidgets
    from axia_force_replay_ui import create_window

    path = tmp_path / 'ui.csv'
    path.write_text('time_s,f_human_x,f_human_y,f_human_z\n'
                    '0,1,2,3\n0.01,4,5,6\n0.02,7,8,9\n2,10,11,12\n', encoding='utf-8')
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = create_window(load_recording(path), paused=True)
    try:
        window.show()
        app.processEvents()
        assert len(window.plots) == 3
        assert [plot.getAxis('left').labelText for plot in window.plots] == [
            'F<sub>x</sub> (N)', 'F<sub>y</sub> (N)', 'F<sub>z</sub> (N)']
        assert window.plots[-1].getAxis('bottom').labelText == 'Time(s)'
        assert window.plots[0].getAxis('bottom')._tickSpacing == [(2., 0.)]
        assert window.font().family() == 'Times New Roman'
        assert window.width() > 1200 or app.primaryScreen().availableGeometry().width() <= 1200
        window.clock.seek(.5)
        window.refresh()
        x, y = window.curves[0].getData()
        np.testing.assert_array_equal(x, [0, .01, .02])
        np.testing.assert_array_equal(y, [1, 4, 7])
        window.clock.play()
        window.clock.seek(2)
        window.refresh()
        assert not window.clock.playing
        assert window.curves[0].getData()[1][-1] == 10
        window.slider.setValue(2500)
        assert window.clock.position() == .5
        assert len(window.curves[0].getData()[1]) == 3
    finally:
        window.close()
        app.processEvents()
