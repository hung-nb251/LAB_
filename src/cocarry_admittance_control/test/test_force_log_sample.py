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


def test_stale_clears_mregister_source_values():
    s = sample()
    s.update(mregister_values_nm=[1.] * 6, mregister_delta_nm=[.5] * 6)
    r = force_log_record(s, 1_300_000_000, .01, .25)
    assert r['mregister_values_nm'] is None
    assert r['mregister_delta_nm'] is None


def test_missing_and_malformed_are_explicit():
    assert force_log_record(None, 0, 0, .25)['status'] == 'NO_SAMPLE'
    s = sample();s['force'][0] = float('nan')
    with pytest.raises(ValueError): validate_force_sample(s)


def test_real_launch_uses_mregister_calibration_file():
    path = Path(__file__).parents[1] / 'launch/cocarry_admittance_real_gui.launch.py'
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == '_launch_sensorless_force')
    env = dict(Node=lambda **kwargs: kwargs,
               ParameterValue=lambda value, **kwargs: value,
               LaunchConfiguration=lambda name: f'<{name}>')
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    params = env['_launch_sensorless_force'](None, 'params.yaml')[0]['parameters']
    assert params[0] == 'params.yaml'
    assert params[1]['calibration_file'] == '<robot_force_calibration_file>'
    assert env['_launch_sensorless_force'](None, 'params.yaml')[0]['executable'] == 'mregister_force_node.py'
