import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import queue
import time
import unittest

spec = importlib.util.spec_from_file_location('recorder', Path(__file__).resolve().parents[1] / 'scripts/hc_force_trial_logger.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Tests(unittest.TestCase):
    def test_register_groups(self):
        self.assertEqual(m.REGISTER_GROUPS['torque'], list(range(310, 316)))
        self.assertEqual(m.REGISTER_GROUPS['wrench'], list(range(320, 326)))
        self.assertEqual(
            m.REGISTER_GROUPS['torque_wrench'],
            [*range(310, 316), *range(320, 326)])
        self.assertEqual(len(m.REGISTER_GROUPS['all']), 24)

    def test_decode(self):
        self.assertEqual(m.decode(310, 9500, True), (-50.0, 'Nm'))
        self.assertEqual(m.decode(321, 11055, True), (105.5, 'N'))
        for a in [316, 330, 345]:
            self.assertEqual(m.decode(a, 17000, True), (None, None))
        self.assertEqual(m.decode(321, 0, False), (None, None))

    def test_timeout_retains_request_no_overlap(self):
        n = object.__new__(m.Recorder)
        n.commands = queue.Queue()
        f = Mock()
        f.done.return_value = False
        n.pending = (f, 310, time.monotonic_ns()-4_000_000_000, 'baseline', False)
        n.args = SimpleNamespace(timeout=3)
        n.paused = False
        n.scan = 1
        n.emit = Mock()
        n.client = Mock()
        n.tick()
        self.assertTrue(n.paused)
        self.assertTrue(n.pending[-1])
        n.commands.put('resume')
        n.tick()
        n.client.call_async.assert_not_called()
        self.assertTrue(n.paused)
        self.assertEqual(n.emit.call_count, 1)

    def test_session_output_layout(self):
        args = SimpleNamespace(
            output='logs', session='20260916_tool0', category='static_cog',
            pose='pose_01_home', trial='cog_pose_01')
        self.assertEqual(
            m.build_output_directory(args, '20260916_090000_000000'),
            Path('logs/20260916_tool0/01_static_cog/pose_01_home/'
                 '20260916_090000_000000_cog_pose_01'))

    def test_legacy_output_layout(self):
        args = SimpleNamespace(
            output='logs', session='', category='static_cog',
            pose='home', trial='S1_home')
        self.assertEqual(
            m.build_output_directory(args, '20260916_090000_000000'),
            Path('logs/20260916_090000_000000'))

    def test_session_labels_reject_paths(self):
        args = SimpleNamespace(
            output='logs', session='../outside', category='static_cog',
            pose='home', trial='trial')
        with self.assertRaises(ValueError):
            m.build_output_directory(args, '20260916_090000_000000')

    def test_static_and_three_axis_markers_are_known(self):
        for phase in ['settling', 'stable', 'yplus_1', 'yminus_2',
                      'zplus_3', 'zminus_3', 'release_yplus_1',
                      'release_zminus_3', 'gru_running',
                      'mjm_running', 'arrived_target1', 'arrived_target2']:
            self.assertIsNotNone(m.Recorder.KNOWN_PHASE.fullmatch(phase))

    def test_marker_topic_queues_command(self):
        recorder = object.__new__(m.Recorder)
        recorder.commands = queue.Queue()
        recorder.receive_marker_command(SimpleNamespace(data=' yplus_1 '))
        self.assertEqual(recorder.commands.get_nowait(), 'yplus_1')

if __name__ == '__main__':
    unittest.main()
