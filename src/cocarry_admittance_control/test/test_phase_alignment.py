"""Controller wiring for the one-shot control-tick phase alignment.

The maths lives in `admittance.phase_aligned_delay` and is tested in
test_admittance.py. Here we check the guards that decide when it may run and
the timer-swap invariant that must never leave the node without a control tick.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cocarry_admittance_control.admittance import phase_aligned_delay

SOURCE = (Path(__file__).parents[1]
          / 'cocarry_admittance_control/admittance_controller.py')


def method(name, **env_extra):
    tree = ast.parse(SOURCE.read_text())
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == name)
    env = dict(np=np, phase_aligned_delay=phase_aligned_delay, **env_extra)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), env)
    return env[name]


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    """Minimal stand-in that tracks which timers are alive."""

    def __init__(self):
        self.alive = []
        self.destroyed = []
        self.created = []

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.alive.append(timer)
        self.created.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)
        self.alive.remove(timer)

    def live_control_timers(self):
        return [t for t in self.alive if not t.cancelled]


def align_state(**over):
    base = dict(_phase_align_enabled=True, _phase_aligned=False,
                _mode='prediction', _prediction=np.zeros(3),
                _prediction_time=100.)
    base.update(over)
    return NS(**base)


@pytest.mark.parametrize('over, expected', [
    ({}, True),
    ({'_phase_align_enabled': False}, False),
    ({'_phase_aligned': True}, False),                 # once per run only
    ({'_mode': 'ground_truth'}, False),                # no predictor in the loop
    ({'_mode': 'mjm'}, False),
    ({'_prediction': None}, False),                    # nothing to align to
    ({'_prediction_time': 0.}, False),
])
def test_alignment_runs_only_with_a_live_prediction_stream(over, expected):
    should = method('_should_align_phase')
    assert should(align_state(**over)) is expected


def test_swap_always_leaves_exactly_one_live_control_timer():
    swap = method('_swap_control_timer')
    node = FakeNode()
    state = node
    state._control_timer = node.create_timer(1 / 15, 'steady')
    state._retired_timer = None

    swap(state, 0.02, 'phase')
    assert len(node.live_control_timers()) == 1
    assert state._control_timer.callback == 'phase'
    # The timer we just left must survive this swap: its callback may be running.
    assert node.destroyed == []

    swap(state, 1 / 15, 'steady-again')
    assert len(node.live_control_timers()) == 1
    assert state._control_timer.callback == 'steady-again'
    # Only now is the first timer safe to destroy.
    assert [t.callback for t in node.destroyed] == ['steady']


def test_swap_creates_the_replacement_before_cancelling_the_old_one():
    """Order matters: a gap with no timer would stall every watchdog."""
    swap = method('_swap_control_timer')
    node = FakeNode()
    node._control_timer = node.create_timer(1 / 15, 'steady')
    node._retired_timer = None
    original = node._control_timer

    swap(node, 0.02, 'phase')
    assert node.created.index(original) < node.created.index(node._control_timer)
    assert original.cancelled is True
    assert node._control_timer.cancelled is False


def test_alignment_marks_itself_done_even_if_the_delay_is_rejected():
    """A bad offset must not retry forever and stall PREPARING."""
    warnings = []
    align = method('_align_control_phase')
    node = FakeNode()
    node._control_timer = node.create_timer(1 / 15, 'steady')
    node._retired_timer = None
    node._phase_aligned = False
    node._prediction_time = 100.
    node._rate = 15.0
    node._phase_offset = 1.0            # >= one period: invalid
    node.get_logger = lambda: NS(warn=warnings.append, info=lambda *a: None)

    align(node, 100.01)
    assert node._phase_aligned is True
    assert warnings and 'offset' in warnings[0]
    # The steady timer must be untouched when alignment bails out.
    assert node._control_timer.callback == 'steady'


def test_aligned_tick_reaches_the_designed_offset():
    node_rate, offset = 15.0, .0333
    for launch_phase in (.0054, .0332, .0645):
        sample_time = 100.0 - launch_phase
        delay = phase_aligned_delay(sample_time, 100.0, 1 / node_rate, offset)
        assert (100.0 + delay) - sample_time == pytest.approx(
            offset + (1 / node_rate if launch_phase > offset else 0.), abs=1e-9)


def test_full_alignment_sequence_settles_the_prediction_age():
    """align -> phase timer fires -> steady timer -> tick, at the target age.

    Composes the two real methods against a fake clock, so a mistake in how
    they hand over shows up here rather than only on the robot.
    """
    align = method('_align_control_phase')
    finish = method('_finish_phase_alignment')
    swap = method('_swap_control_timer')

    for launch_phase in (.0054, .0182, .0332, .053, .0645):
        node = FakeNode()
        node._rate = 15.0
        node._phase_offset = .0333
        node._phase_aligned = False
        node._retired_timer = None
        node._control_timer = node.create_timer(1 / node._rate, 'steady')
        node.get_logger = lambda: NS(info=lambda *a, **k: None,
                                     warn=lambda *a, **k: None)
        node._swap_control_timer = lambda p, c: swap(node, p, c)
        node._finish_phase_alignment = lambda: finish(node)
        ticks = []
        node._control_tick = lambda: ticks.append(now[0])

        now = [100.0]
        sample_time = now[0] - launch_phase
        node._prediction_time = sample_time

        align(node, now[0])
        phase_timer = node._control_timer
        assert phase_timer.callback != 'steady'

        # The phase timer fires once, after exactly its period.
        now[0] += phase_timer.period
        finish(node)

        assert len(ticks) == 1
        age = ticks[0] - sample_time
        residue = (age - node._phase_offset) % (1 / node._rate)
        assert min(residue, 1 / node._rate - residue) < 1e-9
        assert node._control_timer.period == pytest.approx(1 / node._rate)
        assert len(node.live_control_timers()) == 1


def _launch_arg_default(profile, name):
    path = (Path(__file__).parents[1] / 'launch'
            / f'cocarry_admittance_{profile}_gui.launch.py')
    tree = ast.parse(path.read_text())
    call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name)
                and n.func.id == 'DeclareLaunchArgument'
                and n.args and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == name)
    return float(next(k.value.value for k in call.keywords
                      if k.arg == 'default_value'))


def test_tracking_threshold_keeps_headroom_over_the_command_lead():
    """The two are one safety pair and must never drift apart.

    Measured on the real robot 2026-09-21/22: normal tracking error is about
    0.70*lead, worst case about 0.92*lead. A threshold below that is a false
    fault waiting to happen; the gap above it is what still catches a robot
    that is genuinely off its commanded path.
    """
    lead = _launch_arg_default('real', 'command_lead_m')
    threshold = _launch_arg_default('real', 'max_tracking_error_m')
    worst_case_tracking = 0.92 * lead        # trial 121935: 50.6 mm at 55 mm
    assert threshold > worst_case_tracking, 'threshold would fault on normal motion'
    assert threshold >= 1.15 * worst_case_tracking, 'less headroom than 2026-09-22'
    # And it must not be opened so far that a real deviation goes unnoticed.
    assert threshold <= 1.6 * worst_case_tracking


def test_real_launch_passes_both_safety_values_to_the_streamer():
    path = (Path(__file__).parents[1] / 'launch'
            / 'cocarry_admittance_real_gui.launch.py')
    src = path.read_text()
    assert "'--max-tracking-error'" in src
    assert "LaunchConfiguration('max_tracking_error_m')" in src
    assert "LaunchConfiguration('command_lead_m')" in src
