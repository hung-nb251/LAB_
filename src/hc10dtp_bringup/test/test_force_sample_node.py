import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from test_sensorless_force_math import effort_to_joint_torque, estimate_robot_wrench


def callback():
    path = Path(__file__).parents[1] / 'scripts/sensorless_force_node.py'
    method = next(n for n in ast.walk(ast.parse(path.read_text()))
                  if isinstance(n, ast.FunctionDef) and n.name == '_joint_state')
    def wrench():
        return NS(header=NS(), wrench=NS(force=NS(), torque=NS()))
    env = dict(np=np, math=math, effort_to_joint_torque=effort_to_joint_torque,
               estimate_robot_wrench=estimate_robot_wrench, WrenchStamped=wrench,
               JointState=NS, Float64MultiArray=NS, JOINT_NAMES=list('abcdef'))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    return env['_joint_state']


def state(mode):
    published = []
    pub = NS(publish=published.append)
    s = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
           _minimum_period_ns=1, _last_process_ns=0, _base_link='base_link',
           _mode=mode, _calibration_confirmed=False, _rated_torque=np.ones(6)*100,
           _custom_scale=np.ones(6), _torque_bias=np.zeros(6), _deadband=1.,
           _max_force_norm=500., _damping_min=.002, _damping_max=.08,
           _singularity_threshold=.05, _indices_for=lambda m: list(range(6)),
           _ik=NS(compute_jacobian=lambda q: np.eye(6)),
           _wrench_pub=pub, _torque_pub=pub, _diagnostics_pub=pub)
    s._publish_health = lambda valid, status: s._sample.update(role_valid=valid, status=status)
    return s, published


def msg(effort):
    return NS(header=NS(stamp=NS(sec=1, nanosec=0)), position=[0.]*6, effort=effort)


def test_selected_conversion_publishes_force_and_preserves_sub_deadband():
    s, published = state('normalized_rated_torque')
    callback()(s, msg([.002, .02, -.03, 0., 0., 0.]))
    assert len(published) == 3
    assert s._sample['force'][0] == 0.
    assert .19 < s._sample['force_unfiltered'][0] < .21
    assert not s._sample['role_valid']


def test_raw_only_and_missing_effort_have_no_invented_force():
    for mode, effort in [('raw_only', [1.]*6), ('torque_nm', [])]:
        s, published = state(mode);callback()(s, msg(effort))
        assert not published and s._sample['force'] is None


def test_over_limit_keeps_diagnostic_but_does_not_publish_standard_force():
    s, published = state('normalized_rated_torque')
    callback()(s, msg([6., 0., 0., 0., 0., 0.]))
    assert not published
    assert s._sample['force'] is None
    assert s._sample['force_unfiltered'][0] > 500.
    assert s._sample['status'].startswith('INVALID:')
