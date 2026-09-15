import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cocarry_admittance_control.admittance import nominal_reference, limit_position_lead
from cocarry_admittance_control.prediction_reference import PredictionReference


def test_constant_target_converges_without_overshoot_or_modifying_raw():
    ref = PredictionReference(.4)
    ref.reset([0., 0., 0.], 0.)
    raw = np.array([.04, -.03, .02])
    previous = np.zeros(3)
    for tick in range(1, 101):
        out = ref.step(raw, tick / 15)
        assert np.linalg.norm(raw - out) <= np.linalg.norm(raw - previous)
        assert np.all(np.abs(out) <= np.abs(raw))
        previous = out
    assert np.allclose(raw, [.04, -.03, .02])
    assert np.linalg.norm(out - raw) < 1e-7


def test_disabled_reference_preserves_exact_legacy_prediction():
    ref = PredictionReference(0.)
    ref.reset([1., 2., 3.], 100.)
    assert np.array_equal(ref.step([4., 5., 6.], 100.), [4., 5., 6.])


def test_jitter_uses_elapsed_time_and_repeated_calls_do_not_advance():
    ref = PredictionReference(.4)
    ref.reset([0., 0., 0.], 1.)
    for t in [1.05, 1.12, 1.20]:
        out = ref.step([1., 0., 0.], t)
        assert np.array_equal(ref.step([2., 0., 0.], t), out)
    assert np.isclose(out[0], 1 - np.exp(-.2/.4))
    assert np.array_equal(ref.step([2., 0., 0.], 1.1), out)


def test_delayed_tick_is_capped_and_reset_discards_old_trial():
    ref = PredictionReference(.4)
    ref.reset([0., 0., 0.], 0.)
    assert np.isclose(ref.step([1., 0., 0.], 10.)[0], 1-np.exp(-.1/.4))
    ref.reset([3., 2., 1.], 20.)
    assert np.array_equal(ref.step([9., 9., 9.], 20.), [3., 2., 1.])


@pytest.mark.parametrize('tau', [-1., float('nan'), float('inf')])
def test_invalid_time_constant_rejected(tau):
    with pytest.raises(ValueError):
        PredictionReference(tau)


def nominal_method():
    path = Path(__file__).parents[1] / 'cocarry_admittance_control/admittance_controller.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef) and n.name == '_nominal_position')
    env = dict(np=np, nominal_reference=nominal_reference,
               limit_position_lead=limit_position_lead)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    return env['_nominal_position']


def controller_state():
    ref = PredictionReference(.4)
    ref.reset([.1, .4, .5], 100.)
    return NS(_mode='prediction', _capture_ee=np.array([.1, .4, .5]),
              _prediction=np.array([.04, 0., 0.]), _prediction_time=100.,
              _prediction_timeout=.5, _prediction_reference=ref,
              _current_pose=NS(position=NS(x=.1, y=.4, z=.5)),
              _prediction_max_nominal_lead=.05,
              get_logger=lambda: NS(warn=lambda *a, **k: None))


def test_controller_conditions_nominal_not_raw_and_keeps_geometric_safety():
    state = controller_state()
    out = nominal_method()(state, 100. + 1/15)
    assert .1 < out[0] < .14
    assert np.array_equal(state._prediction, [.04, 0., 0.])
    state._prediction = np.array([10., 0., 0.])
    out = nominal_method()(state, 100. + 2/15)
    assert np.isclose(np.linalg.norm(out - state._capture_ee), .05)
    assert np.array_equal(state._prediction_reference.position, out)
    assert nominal_method()(state, 101.) is None  # No hiding stale prediction.


def test_ground_truth_and_direct_mjm_bypass_conditioner():
    state = controller_state()
    state._mode = 'ground_truth'
    assert np.array_equal(nominal_method()(state, 100.1), state._capture_ee)
    state._mode = 'prediction'
    out = nominal_method()(state, 100.1, apply_prediction_limit=False)
    assert np.array_equal(out, state._capture_ee + state._prediction)
    assert state._prediction_reference.time == 100.


def test_force_hold_resets_reference_clock_before_early_return():
    # Guard wiring as well as numerical reset: no hidden nominal advancement.
    path = Path(__file__).parents[1] / 'cocarry_admittance_control/admittance_controller.py'
    tree = ast.parse(path.read_text())
    tick = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == '_control_tick')
    hold = next(n for n in tick.body if isinstance(n, ast.If)
                and ast.unparse(n.test) == "force_action == 'hold'")
    assert 'self._prediction_reference.reset(nominal, now)' in ast.unparse(hold)
    assert isinstance(hold.body[-1], ast.Return)


@pytest.mark.parametrize('profile, expected_tau', [('real', '0.4'), ('sim', '0.4')])
def test_launch_nominal_default_and_controller_wiring(profile, expected_tau):
    path = Path(__file__).parents[1] / 'launch' / f'cocarry_admittance_{profile}_gui.launch.py'
    tree = ast.parse(path.read_text())
    declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Name)
                       and n.func.id == 'DeclareLaunchArgument'
                       and n.args and isinstance(n.args[0], ast.Constant)
                       and n.args[0].value == 'prediction_reference_tau_sec')
    assert next(
        k.value.value for k in declaration.keywords
        if k.arg == 'default_value') == expected_tau
    controller = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'admittance' for t in n.targets))
    params = next(k.value for k in controller.value.keywords if k.arg == 'parameters')
    assert "LaunchConfiguration('prediction_reference_tau_sec')" in ast.unparse(params)
    assert 'value_type=float' in ast.unparse(params)


def test_lead_reduces_constant_velocity_lag_without_changing_raw_prediction():
    old, new = PredictionReference(.4), PredictionReference(.4, lead_sec=.15)
    for ref in (old, new):
        ref.reset([0., 0., 0.], 0.)
    for i in range(1, 151):
        raw = np.array([.1 * i / 15, 0., 0.])
        a, b = old.step(raw, i / 15), new.step(raw, i / 15)
    assert np.isclose(raw[0], 1.)
    assert raw[0] - b[0] < .65 * (raw[0] - a[0])
    assert b[0] - a[0] == pytest.approx(.015, abs=1e-6)


def test_lead_cap_reset_stall_and_repeated_timestamps():
    ref = PredictionReference(.4, lead_sec=.15)
    ref.reset([0., 0., 0.], 0.)
    for i in range(1, 61):
        out = ref.step([i, -i, i], i / 15)
        assert np.linalg.norm(out - ref.position) <= .02 + 1e-12
        assert np.array_equal(ref.step([99., 99., 99.], i / 15), out)
    out = ref.step([100., 100., 100.], 10.)
    assert np.array_equal(ref.velocity, np.zeros(3))
    assert np.array_equal(out, ref.position)
    ref.reset([1., 2., 3.], 20.)
    assert np.array_equal(ref.step([9., 9., 9.], 20.), [1., 2., 3.])


def test_compensated_constant_target_converges_and_attenuates_stationary_noise():
    ref = PredictionReference(.4, lead_sec=.15)
    ref.reset([0., 0., 0.], 0.)
    for i in range(1, 151):
        out = ref.step([.04, -.02, .01], i / 15)
        assert np.all(out <= np.maximum(ref.position, [.04, -.02, .01]) + 1e-12)
        assert np.all(out >= np.minimum(ref.position, [.04, -.02, .01]) - 1e-12)
    assert np.allclose(out, [.04, -.02, .01], atol=1e-8)
    rng = np.random.default_rng(15)
    raw, result = [], []
    for i in range(151, 751):
        sample = np.array([.04, -.02, .01]) + rng.normal(0., .003, 3)
        raw.append(sample)
        result.append(ref.step(sample, i / 15))
    assert np.std(result, axis=0).max() < .6 * np.std(raw, axis=0).min()


@pytest.mark.parametrize('kwargs', [{'lead_sec': -1}, {'lead_sec': float('nan')},
                                   {'velocity_tau_sec': 0}, {'max_lead_m': -1}])
def test_invalid_lead_parameters_fail(kwargs):
    with pytest.raises(ValueError):
        PredictionReference(.4, **kwargs)
