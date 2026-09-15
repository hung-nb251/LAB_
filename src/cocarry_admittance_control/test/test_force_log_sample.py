import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from cocarry_admittance_control.force_log_sample import validate_force_sample, force_log_record


def sample():
    return dict(schema=1, stamp_ns=1_000_000_000, frame='base_link',
                mode='normalized_rated_torque', calibration_confirmed=False,
                status='UNCALIBRATED:normalized_rated_torque', role_valid=False,
                force=[1., 2., 3.], force_unfiltered=[1., 2., 3.],
                torque_nm=[1.] * 6, position=[0.] * 6, effort_raw=[.01] * 6,
                scale_nm_per_effort=[100.] * 6, bias_nm=[0.] * 6,
                diagnostics=[.1, 10., .002, .01])


def test_uncalibrated_estimate_is_recorded_but_not_role_valid():
    s = validate_force_sample(sample())
    r = force_log_record(s, 1_020_000_000, .02, .25)
    assert r['force'] == [1., 2., 3.]
    assert not r['role_valid']
    assert r['age_ms'] == 20.


@pytest.mark.parametrize('now,age', [(1_300_000_000, .01), (1_010_000_000, .3),
                                    (900_000_000, .01)])
def test_stale_or_future_sample_cannot_repeat_force(now, age):
    r = force_log_record(sample(), now, age, .25)
    assert r['force'] is None and r['force_unfiltered'] is None
    assert r['status'].startswith('STALE:') and not r['role_valid']


def test_rejected_estimate_keeps_raw_diagnostic_without_valid_force():
    s = sample()
    s.update(status='INVALID:force_norm=600_N', force_unfiltered=[600., 0., 0.])
    r = force_log_record(s, 1_020_000_000, .02, .25)
    assert r['force'] is None and r['force_unfiltered'][0] == 600.


def test_missing_and_malformed_are_explicit():
    assert force_log_record(None, 0, 0, .25)['status'] == 'NO_SAMPLE'
    s = sample();s['force'][0] = float('nan')
    with pytest.raises(ValueError): validate_force_sample(s)


@pytest.mark.parametrize('override', ['', 'raw_only', 'torque_nm', 'normalized_rated_torque'])
def test_launch_only_overrides_yaml_if_explicit(override):
    path = Path(__file__).parents[1] / 'launch/cocarry_admittance_real_gui.launch.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == '_launch_sensorless_force')
    env = dict(Node=lambda **kwargs: kwargs,
               ParameterValue=lambda *a, **k: False,
               LaunchConfiguration=lambda name: NS(perform=lambda context: override))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    params = env['_launch_sensorless_force'](None, 'params.yaml')[0]['parameters']
    assert params[0] == 'params.yaml'
    if override: assert params[1]['effort_unit_mode'] == override
    else: assert 'effort_unit_mode' not in params[1]
