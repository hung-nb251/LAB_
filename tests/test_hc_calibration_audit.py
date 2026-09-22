"""Offline checks independent of a running ROS graph."""
import ast
from collections import deque
import csv
import json
from pathlib import Path
from types import SimpleNamespace as NS
import time
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/cocarry_admittance_control'))
from cocarry_admittance_control.force_log_sample import force_log_record


def methods(path, names, env):
    tree = ast.parse(path.read_text())
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name in names:
            exec(compile(ast.Module(body=[n], type_ignores=[]), str(path), 'exec'), env)
    return type('Subject', (), {name: env[name] for name in names})


def logger_headers(path):
    tree = ast.parse(path.read_text())
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in tree.body
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in ('COMPACT_HEADER', 'DIAGNOSTIC_HEADER'))
    }


def test_calibration_profile_preserves_raw_sidecar_and_diagnostic_csv(tmp_path):
    logger_path = ROOT/'src/cocarry_admittance_control/cocarry_admittance_control/cocarry_logger.py'
    env = dict(time=time, json=json, csv=csv, force_log_record=force_log_record)
    env.update(logger_headers(logger_path))
    cls = methods(logger_path,
                  ['_calibration_event', '_reference', '_values', '_write'],
                  env)
    s = cls(); s._values = lambda value,size: list(value) if value is not None else ['']*size
    s.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_010_000_000))
    s.get_logger = lambda: NS(info=lambda *a: None, warn=lambda *a: None)
    s._calibration_preroll = deque(); s._path = str(tmp_path/'trial.csv')
    s._calibration_stream = open(s._path+'.calibration.jsonl', 'x')
    s._logging_profile = 'calibration'
    s._logging=True; s._rows=[]; s._latest={}; s._hybrid_events=[]
    s._force_sample_timeout=.4; s._force_log_unfiltered=True; s._force_sample_received=time.monotonic()
    s._force_sample=dict(stamp_ns=1_000_000_000, status='SHADOW_VALID:test', frame='base_link',
                         force=[0.,0.,0.], force_unfiltered=[.2,.3,.4], torque_nm=list(range(6)))
    raw=dict(mregister_values_nm=[1,2,3,4,5,6], baseline_q0_rad=[0]*6)
    s._calibration_event('m310_sample',raw)
    s._reference(NS(point=NS(x=0,y=0,z=0)))
    s._write()
    with open(s._path) as f: header,row=list(csv.reader(f))
    assert len(header)==len(row)==61
    record=dict(zip(header,row))
    assert float(record['f_robot_x'])==.2
    assert float(record['torque_J6_Nm'])==5
    assert [k for k in header if k.startswith('f_robot_')]==['f_robot_x','f_robot_y','f_robot_z']
    events=[json.loads(l) for l in Path(s._path+'.calibration.jsonl').read_text().splitlines()]
    assert events[0]['data']==raw
    assert events[-1]['kind']=='logger_stop'


def test_compact_profile_has_one_small_csv_schema(tmp_path):
    logger_path = ROOT/'src/cocarry_admittance_control/cocarry_admittance_control/cocarry_logger.py'
    env = dict(time=time, json=json, csv=csv, force_log_record=force_log_record)
    env.update(logger_headers(logger_path))
    cls = methods(logger_path, ['_reference', '_values', '_write'], env)
    s = cls(); s._values = lambda value,size: list(value) if value is not None else ['']*size
    s.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_010_000_000))
    s.get_logger = lambda: NS(info=lambda *a: None, warn=lambda *a: None)
    s._logging_profile = 'compact'; s._logging = True
    s._rows = []; s._hybrid_events = []; s._calibration_stream = None
    s._path = str(tmp_path/'trial.csv')
    s._latest = dict(
        ee=(1., 2., 3.), pred=(4., 5., 6.), nominal=(7., 8., 9.),
        fh=(10., 11., 12.), effective_force=(13., 14., 15.),
        z_deadzone_weight=.5, model='gru', manual_role='FOLLOWER',
        controller_state='RUNNING', control_phase='FOLLOWER', status_reason='ok',
        fh_ts=999, force_age_ms=1., pose_age_ms=2., prediction_age_ms=3.,
        udp_gap_ms=4.)
    s._reference(NS(point=NS(x=16., y=17., z=18.)))
    s._write()
    with open(s._path) as stream:
        header, row = list(csv.reader(stream))
    assert len(header) == len(row) == 30
    assert 'f_robot_x' not in header
    assert 'hybrid_status_json' not in header
    assert not Path(s._path + '.calibration.jsonl').exists()
    assert not Path(s._path + '.hybrid_events.json').exists()


def test_axia_diagnostic_records_before_deadband_without_changing_force():
    cls=methods(ROOT/'scripts/axia_sensor_ui.py',['compensate'],dict(
        np=np,json=json,BASE_FRAME='base_link',PAYLOAD_MASS_KG=1.126,String=lambda **k:NS(**k)))
    s=cls(); s._bias_F=np.array([1.,0.,0.]); s._deadband=4.; s._is_calibrated=True
    s._filter_name='test'
    s.get_rotation_world_to_sensor=lambda:np.eye(3)
    s.compute_gravity_force=lambda r:np.array([0.,0.,-10.])
    s.get_clock=lambda:NS(now=lambda:NS(nanoseconds=123))
    messages=[]; s._pub_calibration_sample=NS(publish=messages.append)
    out=s.compensate(np.array([7.,0.,-10.]))
    np.testing.assert_allclose(out,[2,0,0])
    data=json.loads(messages[0].data)
    assert data['force_pre_deadband']==[6,0,0]
    assert data['force_post_deadband']==[2,0,0]
    assert data['tf_valid']
    s.get_rotation_world_to_sensor=lambda:None
    s.compensate(np.array([7.,0.,0.]))
    assert not json.loads(messages[-1].data)['tf_valid']
