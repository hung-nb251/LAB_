import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import queue
import time
import unittest

spec = importlib.util.spec_from_file_location('recorder', Path(__file__).resolve().parents[1] / 'hc_force_trial_logger.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Tests(unittest.TestCase):
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

if __name__ == '__main__':
    unittest.main()
