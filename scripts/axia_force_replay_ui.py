#!/usr/bin/env python3
"""Replay recorded XYZ forces at CSV timestamps, without ROS or hardware access.

    python3 scripts/axia_force_replay_ui.py cocarry_logs/<trial>.csv
"""

import argparse
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
import sys
import time

import numpy as np


@dataclass
class ForceRecording:
    path: Path
    seconds: np.ndarray
    forces: np.ndarray
    time_column: str
    force_columns: tuple

    @property
    def duration(self):
        return float(self.seconds[-1])

    @property
    def rate_hz(self):
        return (len(self.seconds) - 1) / self.duration if self.duration else 0.0


def load_recording(path, time_column=None, time_unit=None, force_columns=None):
    """Preserve row order, duplicate timestamps and gaps; reject corrupt input."""
    path = Path(path).expanduser().resolve()
    columns = tuple(force_columns or ('f_human_x', 'f_human_y', 'f_human_z'))
    if len(columns) != 3:
        raise ValueError('Cần đúng 3 cột lực theo thứ tự X Y Z.')
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if time_column is None:
            time_column = next((name for name in (
                'ros_timestamp_ns', 'timestamp_ns', 'fh_timestamp_ns',
                'elapsed_sec', 'time_sec', 'timestamp_sec', 'time_s',
            ) if name in fields), None)
        if time_column is None:
            raise ValueError('Không tìm thấy timestamp. Dùng --time-column và --time-unit.')
        missing = [name for name in (time_column, *columns) if name not in fields]
        if missing:
            raise ValueError('CSV thiếu cột: ' + ', '.join(missing))
        if time_unit is None:
            if time_column.endswith('_ns'):
                time_unit = 'ns'
            elif time_column.endswith(('_sec', '_s')):
                time_unit = 's'
            else:
                raise ValueError('Hãy chỉ rõ đơn vị timestamp bằng --time-unit s/ms/us/ns.')
        scale = {'s': Decimal('1'), 'ms': Decimal('.001'),
                 'us': Decimal('.000001'), 'ns': Decimal('.000000001')}[time_unit]
        seconds, forces = [], []
        origin = previous = None
        for row in reader:
            try:
                stamp = Decimal(row[time_column])
                force = [float(row[name]) for name in columns]
                if not stamp.is_finite() or not all(np.isfinite(force)):
                    raise ValueError('timestamp/lực chứa NaN hoặc Inf')
                if previous is not None and stamp < previous:
                    raise ValueError('timestamp đi lùi')
                if origin is None:
                    origin = stamp
                # Subtract epoch timestamps BEFORE converting to float.
                elapsed = float((stamp - origin) * scale)
                if not np.isfinite(elapsed):
                    raise ValueError('timestamp vượt miền biểu diễn')
            except (InvalidOperation, TypeError, ValueError, OverflowError) as exc:
                raise ValueError(f'Dòng CSV {reader.line_num}: {exc}') from exc
            seconds.append(elapsed)
            forces.append(force)
            previous = stamp
    if not seconds:
        raise ValueError('CSV không có mẫu dữ liệu.')
    return ForceRecording(path, np.asarray(seconds), np.asarray(forces), time_column, columns)


class ReplayClock:
    """Absolute monotonic anchors prevent timer jitter from accumulating drift."""

    def __init__(self, duration, speed=1.0, now=time.monotonic):
        self.duration = duration
        self.speed = speed
        self.now = now
        self.playing = False
        self.anchor_position = 0.0
        self.anchor_time = now()

    def position(self):
        elapsed = (self.now() - self.anchor_time) * self.speed if self.playing else 0.0
        return min(self.duration, self.anchor_position + elapsed)

    def seek(self, position):
        self.anchor_position = max(0.0, min(self.duration, position))
        self.anchor_time = self.now()

    def pause(self):
        self.seek(self.position())
        self.playing = False

    def play(self):
        if self.position() >= self.duration:
            self.seek(0.0)
        self.anchor_time = self.now()
        self.playing = True

    def set_speed(self, speed):
        self.seek(self.position())
        self.speed = speed


def create_window(recording, speed=1.0, window_sec=10.0, paused=False):
    # Keep CSV validation and --help usable without GUI imports or ROS setup.
    from PyQt5 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg

    class ForceReplayWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.clock = ReplayClock(recording.duration, speed)
            self.resume_after_seek = False
            self.setFont(QtGui.QFont('Times New Roman', 12))
            self.setWindowTitle(f'Axia Force Replay — {recording.path.name}')
            screen = QtWidgets.QApplication.primaryScreen()
            available = screen.availableGeometry() if screen else None
            self.resize(min(1600, int(available.width() * .95)) if available else 1600,
                        min(1000, int(available.height() * .92)) if available else 1000)
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)

            controls = QtWidgets.QHBoxLayout()
            self.play_button = QtWidgets.QPushButton('Phát')
            self.play_button.clicked.connect(self.toggle_play)
            controls.addWidget(self.play_button)
            restart = QtWidgets.QPushButton('Phát lại từ đầu')
            restart.clicked.connect(self.restart)
            controls.addWidget(restart)
            controls.addWidget(QtWidgets.QLabel('Tốc độ:'))
            speed_box = QtWidgets.QDoubleSpinBox()
            speed_box.setRange(0.1, 10.0)
            speed_box.setSingleStep(0.1)
            speed_box.setSuffix(' ×')
            speed_box.setValue(speed)
            speed_box.valueChanged.connect(self.clock.set_speed)
            controls.addWidget(speed_box)
            controls.addStretch()
            self.state_label = QtWidgets.QLabel()
            controls.addWidget(self.state_label)
            layout.addLayout(controls)

            self.values = QtWidgets.QLabel()
            self.values.setStyleSheet(
                'font-family:"Times New Roman"; font-size:18pt; '
                'font-weight:bold; padding:8px;')
            layout.addWidget(self.values)
            graph = pg.GraphicsLayoutWidget()
            graph.setBackground('w')
            graph.ci.setSpacing(12)
            graph.setStyleSheet('border:1px solid #303030;')
            layout.addWidget(graph)
            self.plots, self.curves = [], []
            colors = ('#e64524', '#1655d8', '#20945a')
            axis_labels = ('F<sub>x</sub> (N)', 'F<sub>y</sub> (N)', 'F<sub>z</sub> (N)')
            axis_style = {'font-family': 'Times New Roman', 'font-size': '16pt'}
            for axis, color in enumerate(colors):
                plot = graph.addPlot(row=axis, col=0)
                plot.showGrid(x=True, y=True, alpha=.18)
                plot.setLabel('left', axis_labels[axis], color='#202020', **axis_style)
                if axis == 2:
                    plot.setLabel('bottom', 'Time(s)', color='#202020', **axis_style)
                for side in ('left', 'bottom'):
                    plot.getAxis(side).setPen('#303030')
                    plot.getAxis(side).setTextPen('#202020')
                    plot.getAxis(side).setStyle(tickFont=QtGui.QFont('Times New Roman', 12))
                plot.getAxis('bottom').setTickSpacing(levels=[(2., 0.)])
                plot.getViewBox().setBorder(pg.mkPen('#303030', width=1))
                plot.setMouseEnabled(x=False, y=True)
                low, high = np.min(recording.forces[:, axis]), np.max(recording.forces[:, axis])
                margin = max(0.5, float(high - low) * 0.1)
                plot.setYRange(min(0.0, low) - margin, max(0.0, high) + margin, padding=0)
                self.plots.append(plot)
                self.curves.append(plot.plot(pen=pg.mkPen(color, width=2)))

            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, 10000)
            self.slider.sliderPressed.connect(self.begin_seek)
            self.slider.sliderReleased.connect(self.end_seek)
            self.slider.valueChanged.connect(self.seek_slider)
            layout.addWidget(self.slider)
            self.progress = QtWidgets.QLabel()
            layout.addWidget(self.progress)
            self.timer = QtCore.QTimer(self)
            self.timer.setTimerType(QtCore.Qt.PreciseTimer)
            self.timer.timeout.connect(self.refresh)
            # Paint at 50 Hz; include ALL recorded samples due at each paint.
            self.timer.start(20)
            if not paused:
                self.clock.play()
            self.refresh()

        def toggle_play(self):
            if self.clock.playing:
                self.clock.pause()
            else:
                self.clock.play()
            self.refresh()

        def restart(self):
            self.clock.seek(0.0)
            self.clock.play()
            self.refresh()

        def begin_seek(self):
            self.resume_after_seek = self.clock.playing
            self.clock.pause()

        def end_seek(self):
            if self.resume_after_seek and self.clock.position() < recording.duration:
                self.clock.play()
            self.refresh()

        def seek_slider(self, value):
            self.clock.seek(recording.duration * value / 10000)
            self.refresh()

        def refresh(self):
            position = self.clock.position()
            if position >= recording.duration:
                self.clock.pause()
            stop = int(np.searchsorted(recording.seconds, position, side='right'))
            start = max(0, int(np.searchsorted(recording.seconds, position - window_sec)) - 1)
            for axis, (plot, curve) in enumerate(zip(self.plots, self.curves)):
                curve.setData(recording.seconds[start:stop], recording.forces[start:stop, axis])
                right = max(window_sec, position)
                plot.setXRange(right - window_sec, right, padding=0)
            fx, fy, fz = recording.forces[max(0, stop - 1)]
            norm = float(np.linalg.norm([fx, fy, fz]))
            self.values.setText(f'Fx  {fx:+.3f} N     Fy  {fy:+.3f} N     '
                                f'Fz  {fz:+.3f} N     |F|  {norm:.3f} N')
            self.play_button.setText('Tạm dừng' if self.clock.playing else 'Phát')
            self.state_label.setText('Đang phát' if self.clock.playing else (
                'Đã hết file' if position >= recording.duration else 'Tạm dừng'))
            self.progress.setText(f'{position:.3f} / {recording.duration:.3f} s'
                                  f'     Mẫu {stop:,} / {len(recording.seconds):,}')
            if not self.slider.isSliderDown():
                self.slider.blockSignals(True)
                fraction = position / recording.duration if recording.duration else 1.0
                self.slider.setValue(round(fraction * 10000))
                self.slider.blockSignals(False)

        def closeEvent(self, event):
            self.timer.stop()
            event.accept()

    return ForceReplayWindow()


def positive_float(value):
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Giá trị phải hữu hạn và lớn hơn 0.')
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', help='Đường dẫn file CSV cần replay')
    parser.add_argument('--speed', type=positive_float, default=1.0, help='Tốc độ 0.1–10 (mặc định 1×)')
    parser.add_argument('--window-sec', type=positive_float, default=10.0, help='Cửa sổ đồ thị, giây (mặc định 10)')
    parser.add_argument('--paused', action='store_true', help='Mở file ở trạng thái tạm dừng')
    parser.add_argument('--time-column', help='Cột timestamp; tự nhận ros_timestamp_ns cho log co-carry')
    parser.add_argument('--time-unit', choices=('s', 'ms', 'us', 'ns'))
    parser.add_argument('--force-columns', nargs=3, metavar=('X', 'Y', 'Z'),
                        help='Mặc định: f_human_x f_human_y f_human_z; đơn vị N')
    args = parser.parse_args(argv)
    if not 0.1 <= args.speed <= 10:
        parser.error('--speed phải trong khoảng 0.1–10.')
    try:
        recording = load_recording(args.csv_file, args.time_column, args.time_unit, args.force_columns)
    except (OSError, ValueError, csv.Error) as exc:
        parser.error(str(exc))
    try:
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication([sys.argv[0]])
        window = create_window(recording, args.speed, args.window_sec, args.paused)
    except ImportError as exc:
        parser.error(f'Thiếu thư viện UI: {exc}. Cần numpy, PyQt5 và pyqtgraph.')
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
