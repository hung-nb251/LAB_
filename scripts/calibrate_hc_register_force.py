#!/usr/bin/env python3
"""Offline, fixed-pose external-force calibration. Never sends robot commands.

Uses raw Axia baseline differences and recorded TF, with the existing -90 deg
mount correction. Output is a candidate contact-force estimate, not drive torque.
"""
import argparse
import json
import re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

TRIALS = {
 'target1': ['target1/20260917_125927_479349_target1_static_force'],
 'home': ['home/20260917_132544_477259_home_static_force_validation',
          'target1/20260917_133451_892918_target1_z',
          'home/20260917_142459_050508_home_static_force_validation'],
 'near_target2': ['new_pose/20260917_143656_102660_new_pose_static_force_validation',
                  'new_pose/20260917_145157_643170_new_pose_static_force_validation'],
}
ACTIVE = re.compile(r'^([xyz](?:plus|minus))_([12])$')


def rotation_to_base(edges):
    child = 'axia_sensor_link'
    result = np.eye(3)
    visited = set()
    while child != 'base_link':
        if child in visited or child not in edges:
            raise ValueError('Missing or cyclic TF chain to Axia')
        visited.add(child)
        parent, rotation = edges[child]
        result = rotation @ result
        child = parent
    return result @ Rotation.from_euler('z', -90, degrees=True).as_matrix().T


def parse(path):
    raw, joints, markers, scans, complete = [], [], [], {}, set()
    edges = {}; transform = None; timeout = float('inf'); failures = 0
    for line in (path / 'events.jsonl').open():
        e = json.loads(line); t = e['monotonic_ns'] / 1e9
        kind = e['kind']; phase = e.get('phase', '')
        if kind == 'marker': markers.append((t, phase))
        if 'timeout' in kind: timeout = min(timeout, t); failures += 1
        if kind == 'scan_end': complete.add(e['scan_id'])
        if kind == 'register' and e['address'] in (320,321,322):
            if e.get('response', {}).get('success') and not e.get('late',False):
                scans.setdefault(e['scan_id'], {})[e['address']] = (t, e['scaled'], phase, e.get('sent_phase',phase))
        if kind != 'topic': continue
        topic = e['topic']; m = e['message']
        if topic in ('/tf', '/tf_static'):
            for tf in m['transforms']:
                q = tf['transform']['rotation']
                edges[tf['child_frame_id']] = (tf['header']['frame_id'], Rotation.from_quat([q[a] for a in ('x','y','z','w')]).as_matrix())
            if transform is None:
                try: transform = rotation_to_base(edges)
                except ValueError: pass
        elif topic == '/axia/raw_wrench':
            raw.append([t, *[m['wrench']['force'][a] for a in 'xyz']])
        elif topic == '/joint_states': joints.append(m['position'][:6])
    if transform is None: raise ValueError(f'Missing TF: {path}')
    raw = np.asarray(raw); joints = np.asarray(joints)
    if np.max(np.ptp(joints, axis=0)) > .01:
        raise ValueError(f'Not a fixed pose: {path}')
    # Exclude boundary transients, and everything after the first timeout.
    rows = []
    for sid in sorted(complete):
        s = scans.get(sid,{})
        if set(s) != {320,321,322}: continue
        a = [s[k] for k in (320,321,322)]
        if len({v[2] for v in a}) != 1 or any(v[2]!=v[3] for v in a): continue
        t = np.mean([v[0] for v in a]); phase = a[0][2]
        if t >= timeout or max(v[0] for v in a)-min(v[0] for v in a) > .2: continue
        before = [mt for mt, _ in markers if mt <= t]
        after = [mt for mt, _ in markers if mt > t]
        if not before or t-max(before)<2 or (after and min(after)-t<1): continue
        window = raw[(raw[:,0]>=t-.1)&(raw[:,0]<=t+.1),1:]
        if len(window)<5: continue
        f = transform @ np.median(window,axis=0)
        rows.append((t,phase,[v[1] for v in a],f))
    # Initial baseline only: no validation active-force labels used to estimate bias.
    baseline = [r for r in rows if r[1]=='baseline']
    if len(baseline)<5: raise ValueError(f'Insufficient initial baseline: {path}')
    mb = np.median([r[2] for r in baseline],axis=0)
    fb = np.median([r[3] for r in baseline],axis=0)
    active = []
    for t,phase,m,f in rows:
        match = ACTIVE.match(phase)
        if match:
            active.append(dict(x=(np.array(m)-mb).tolist(),y=(f-fb).tolist(),
                               repeat=int(match[2]),phase=phase,trial=path.name))
    release = [(np.asarray(m)-mb,f-fb) for _,ph,m,f in rows if ph.startswith('release_') or ph=='baseline_end']
    return active, dict(trial=path.name, complete_scans=len(complete),timeout_events=failures,
        register_baseline_n=mb.tolist(),raw_baseline_base_n=fb.tolist(),
        joint_pose_rad=np.median(joints,axis=0).tolist(),joint_range_max_rad=float(np.max(np.ptp(joints,axis=0))),
        active_rows=len(active),
        release_register_norm_p95_n=float(np.percentile([np.linalg.norm(m) for m,f in release],95)) if release else None,
        release_axia_norm_p95_n=float(np.percentile([np.linalg.norm(f) for m,f in release],95)) if release else None)


def arrays(rows):
    return np.array([r['x'] for r in rows]), np.array([r['y'] for r in rows])


def fit(rows):
    x,y = arrays(rows)
    # Balance action phases; dense/long holds must not dominate.
    keys=[(r['trial'],r['phase']) for r in rows]
    weights=np.array([1/keys.count(k) for k in keys])**.5
    coef,_,rank,_=np.linalg.lstsq(x*weights[:,None],y*weights[:,None],rcond=None)
    if rank != 3: raise ValueError('Force directions do not span 3D')
    return coef


def metrics(rows, coef):
    x,y=arrays(rows); pred=x@coef; err=pred-y
    valid=(np.linalg.norm(y,axis=1)>=4)&(np.linalg.norm(pred,axis=1)>=4)
    angle=np.degrees(np.arccos(np.clip(np.sum(pred[valid]*y[valid],axis=1)/
        (np.linalg.norm(pred[valid],axis=1)*np.linalg.norm(y[valid],axis=1)),-1,1)))
    return dict(n=len(rows), vector_rmse_n=float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),
        axis_rmse_n=np.sqrt(np.mean(err**2,axis=0)).tolist(),angle_n=int(valid.sum()),
        angle_median_deg=float(np.median(angle)) if len(angle) else None,
        angle_p95_deg=float(np.percentile(angle,95)) if len(angle) else None,
        angle_over_90_fraction=float(np.mean(angle>90)) if len(angle) else None)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--session',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); args=p.parse_args()
    groups={}; quality=[]
    for pose, paths in TRIALS.items():
        groups[pose]=[]
        for relative in paths:
            rows,q=parse(args.session/'02_dynamic_force'/relative)
            groups[pose].extend(rows); quality.append(q)
    result=dict(status='offline_candidate_not_deployed',source='M320-M322 estimated external force',
        formula='F_base = matrix @ (0.1*(register_raw-10000) - initial_no_contact_baseline_N)',
        reference='Delta Axia raw, initial fixed-pose baseline removed, recorded TF and existing yaw -90 deg',
        limitations=['Mount offsets are source defaults, not a logged runtime UI setting.',
          'Fixed-pose incremental forces only; baseline required at the same pose.',
          'Not validated during motion or as independent robot drive force for conflict.',
          'Angle metrics compare sensors, not labeled agreement/conflict decisions.'],quality=quality,local={},leave_one_pose_out={})
    for pose,rows in groups.items():
        train=[r for r in rows if r['repeat']==1]; test=[r for r in rows if r['repeat']==2]
        coef=fit(train)
        result['local'][pose]=dict(matrix_fit_repeat1=coef.T.tolist(),validation_repeat2=metrics(test,coef),
            validation_by_direction={d:metrics([r for r in test if r['phase'].startswith(d+'_')],coef)
              for d in sorted({r['phase'].rsplit('_',1)[0] for r in test})},
            candidate_matrix_all_repeats=fit(rows).T.tolist())
        others=[r for key,rr in groups.items() if key!=pose for r in rr]
        result['leave_one_pose_out'][pose]=metrics(rows,fit(others))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'register_force_vector_calibration.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('local','leave_one_pose_out')},indent=2))

if __name__=='__main__': main()
