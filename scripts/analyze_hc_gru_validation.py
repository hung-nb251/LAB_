#!/usr/bin/env python3
"""Offline GRU/register validation; no ROS or hardware commands."""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from calibrate_hc_register_force import rotation_to_base


def summary(v):
    v = np.asarray(v)
    return dict(median=float(np.median(v)), p95=float(np.percentile(v, 95)),
                maximum=float(np.max(v))) if len(v) else None


def errors(pred, ref):
    pn, rn = np.linalg.norm(pred, axis=1), np.linalg.norm(ref, axis=1)
    mask = (pn >= 4) & (rn >= 4)
    angle = np.degrees(np.arccos(np.clip(
        np.sum(pred[mask]*ref[mask], axis=1)/(pn[mask]*rn[mask]), -1, 1)))
    return dict(n=len(ref), angle_n=int(mask.sum()), reference_above_4n=int((rn >= 4).sum()),
                rmse_vector_n=float(np.sqrt(np.mean(np.sum((pred-ref)**2, axis=1)))),
                angle_deg=summary(angle),
                obtuse_fraction=float(np.mean(angle > 90)) if len(angle) else None)


def analyze(path, candidate, output):
    counts = Counter(); scans = {}; complete = set(); raw = []; human = []
    edges = {}; markers = []; states = []; latency = []; joints = []
    state = ''; t0 = None; transitions = []; previous = {}; calibrated = None
    end_y = []; end_joints = []; end_raw = []
    rotation = None; rotation_time = -np.inf
    g = np.array([0., 0., -9.80665]); mass = 1.126
    for line in (path/'events.jsonl').open():
        e = json.loads(line)
        if t0 is None: t0 = e['monotonic_ns']/1e9
        t = e['monotonic_ns']/1e9-t0; kind = e['kind']; phase = e.get('phase', '')
        counts[kind] += 1
        if kind == 'marker': markers.append([t, e['command']])
        if kind == 'scan_end': complete.add(e['scan_id'])
        if kind == 'register':
            latency.append(e['latency_sec'])
            if e.get('response', {}).get('success') and not e.get('late'):
                scans.setdefault(e['scan_id'], {})[e['address']] = (t, e['scaled'], phase, e.get('sent_phase'))
                if phase == 'baseline_end' and e['address'] == 321: end_y.append(e['scaled'])
        if kind != 'topic': continue
        topic = e['topic']; m = e['message']; counts[topic] += 1
        if topic in ('/cocarry/status', '/axia/calibrated', '/run_status'):
            val = m['data']
            if previous.get(topic) != val: transitions.append([t, topic, val]); previous[topic] = val
            if topic == '/cocarry/status': state = val; states.append((t, state))
            if topic == '/axia/calibrated': calibrated = val
        if topic in ('/tf', '/tf_static'):
            for tf in m['transforms']:
                q = tf['transform']['rotation']
                edges[tf['child_frame_id']] = (tf['header']['frame_id'], Rotation.from_quat([q[a] for a in ('x','y','z','w')]).as_matrix())
            try: rotation = rotation_to_base(edges); rotation_time = t
            except ValueError: pass
        elif topic == '/axia/raw_wrench' and rotation is not None and t-rotation_time < .2:
            f = np.array([m['wrench']['force'][a] for a in 'xyz'])
            if phase == 'baseline_end': end_raw.append(f)
            raw.append((t, f-mass*(rotation.T@g), rotation.copy(), phase))
        elif topic == '/axia/human_force': human.append([t, *[m['vector'][a] for a in 'xyz']])
        elif topic == '/joint_states':
            joints.append([t, *m['position'][:6]])
            if phase == 'baseline_end': end_joints.append(m['position'][:6])
    # Restrict phase windows away from marker boundaries.
    bounds = {}
    for i, (mt, ph) in enumerate(markers):
        bounds[ph] = (mt+2, (markers[i+1][0] if i+1 < len(markers) else t)-1)
    lo, hi = bounds['baseline']
    braw = np.median([f for rt,f,r,ph in raw if lo <= rt <= hi and ph=='baseline'], axis=0)
    rt = np.array([r[0] for r in raw]); rf = np.array([r@(f-braw) for _,f,r,_ in raw])
    rows = []
    for sid in sorted(complete):
        s = scans.get(sid, {})
        if not all(k in s for k in range(320,326)): continue
        a = [s[k] for k in (320,321,322)]
        if len({x[2] for x in a}) != 1 or any(x[2] != x[3] for x in a): continue
        rows.append((np.mean([x[0] for x in a]), [x[1] for x in a], a[0][2], a[-1][0]-a[0][0]))
    ts = np.array([x[0] for x in rows]); m = np.array([x[1] for x in rows])
    ph = np.array([x[2] for x in rows]); base = (ts >= lo)&(ts <= hi)&(ph=='baseline')
    mb = np.median(m[base], axis=0)
    endlo,endhi = bounds['baseline_end']; end = (ts >= endlo)&(ts <= endhi)&(ph=='baseline_end')
    endmb = np.median(m[end], axis=0)
    ref = np.column_stack([np.interp(ts,rt,rf[:,k]) for k in range(3)])
    si = np.searchsorted([v[0] for v in states],ts,side='right')-1
    running = np.array([i >= 0 and states[i][1].startswith('RUNNING') for i in si])
    # Do not count samples after the explicit stop request as moving-run validation.
    stop_times = [tt for tt,topic,val in transitions if topic=='/run_status' and val is False]
    if stop_times: running &= ts < stop_times[0]
    matrices = {key: np.asarray(v['candidate_matrix_all_repeats']) for key,v in candidate['local'].items()}
    evaluations = {key: errors((m[running]-mb)@a.T,ref[running]) for key,a in matrices.items()}
    h = np.asarray(human)
    human_ref = np.column_stack([np.interp(ts,h[:,0],h[:,k+1]) for k in range(3)])
    raw_ui_check = errors(ref[running], human_ref[running])
    # Endpoint correction is retrospective, not a deployable baseline model.
    start_run, end_run = ts[running][[0,-1]]
    w = np.clip((ts-start_run)/(end_run-start_run),0,1)
    endpoint_b = mb[None,:]+w[:,None]*(endmb-mb)[None,:]
    retrospective = {key: errors((m[running]-endpoint_b[running])@a.T,ref[running]) for key,a in matrices.items()}
    # Residual delay diagnostic: positive lag means register trails Axia.
    # Correlation on levels may contain pose trends; report axis-wise, no claim of sensor-only latency.
    lag_results = {}
    use = running & (ts > start_run+1) & (ts < end_run-1)
    for k,axis in enumerate('xyz'):
        curve=[]
        for lag in np.arange(-1,1.001,.02):
            x = m[use,k]-endpoint_b[use,k]
            y = np.interp(ts[use]-lag, rt, rf[:,k])
            corr = np.corrcoef(x,y)[0,1] if np.std(x)>1e-8 and np.std(y)>1e-8 else np.nan
            curve.append((float(lag), float(corr)))
        best=max(curve,key=lambda x: x[1] if np.isfinite(x[1]) else -np.inf)
        lag_results[axis] = dict(lag_s=best[0],correlation=best[1],zero_lag_correlation=curve[50][1],boundary=abs(best[0])>.99)
    joint_arr = np.asarray(joints)
    j0=np.median(joint_arr[(joint_arr[:,0]>=lo)&(joint_arr[:,0]<=hi),1:],axis=0)
    j1=np.median(joint_arr[(joint_arr[:,0]>=endlo)&(joint_arr[:,0]<=endhi),1:],axis=0)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    predicted=(m-mb)@matrices['home'].T
    retrospective_pred=(m-endpoint_b)@matrices['home'].T
    for k,ax in enumerate(axes):
        ax.plot(ts,ref[:,k],label='Axia raw: TF + initial bias/gravity',lw=1)
        ax.plot(ts,predicted[:,k],label='Home matrix + initial M baseline',lw=1)
        ax.plot(ts,retrospective_pred[:,k],label='Retrospective endpoint baseline (diagnostic)',lw=1,alpha=.75)
        ax.axvspan(start_run,end_run,color='green',alpha=.08)
        for ft,topic,val in transitions:
            if topic=='/cocarry/status' and isinstance(val,str) and val.startswith('FAULT'):
                ax.axvline(ft,color='red',ls='--',label='Fault' if k==0 else None)
        ax.set_ylabel('F'+ 'xyz'[k]+' [N]');ax.grid(alpha=.25)
    axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds since logger start')
    fig.suptitle(path.name);fig.tight_layout()
    fig.savefig(output/(path.name+'.png'),dpi=150);plt.close(fig)
    return dict(trial=path.name,duration_sec=t,counts=dict(counts),markers=markers,transitions=transitions,
        baseline_end_integrity=dict(m321_n=len(end_y),m321_sign_changes=int(np.sum(np.asarray(end_y)[1:]*np.asarray(end_y)[:-1]<0)),
            m321_range_n=[min(end_y),max(end_y)],m321_positive=int(np.sum(np.asarray(end_y)>0)),
            m321_negative=int(np.sum(np.asarray(end_y)<0)),joint_range_rad=np.ptp(end_joints,axis=0).tolist(),
            axia_raw_std_n=np.std(end_raw,axis=0).tolist()),
        scan_hz=1/float(np.median(np.diff(ts))),scan_interval_s=summary(np.diff(ts)),
        service_latency_s=summary(latency),force_triplet_span_s=summary([r[3] for r in rows]),
        baseline_register_n=mb.tolist(),baseline_end_register_n=endmb.tolist(),baseline_change_n=(endmb-mb).tolist(),
        baseline_raw_sensor_bias_n=braw.tolist(),initial_joint_rad=j0.tolist(),final_joint_rad=j1.tolist(),
        baseline_end_axia_estimate_n=np.median(ref[end],axis=0).tolist(),
        baseline_end_axia_norm_n=summary(np.linalg.norm(ref[end],axis=1)),
        running_window_sec=[start_run,end_run],running_samples=int(running.sum()),
        raw_vs_ui_direction_check=raw_ui_check,
        fixed_home_baseline_evaluation=evaluations,retrospective_endpoint_baseline_evaluation=retrospective,
        apparent_axis_lag=lag_results)


def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',action='append',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); candidate=json.loads(a.candidate.read_text())
    a.output.mkdir(parents=True,exist_ok=True)
    result=dict(status='diagnostic_validation_not_deployed',
        assumptions=['Axia gravity mass 1.126 kg and mount yaw -90 deg from existing source, not runtime UI readback.',
                     'Initial no-contact baseline estimates constant sensor bias. Raw includes dynamic/inertial loads.',
                     'Apparent correlation lag mixes pose bias, filtering and transport; not sensor acquisition latency.',
                     'Retrospective endpoint interpolation uses validation endpoint and is not a deployable calibration.'],
        trials=[analyze(path,candidate,a.output) for path in a.trial])
    (a.output/'gru_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in result['trials']:
        print(json.dumps({k:r[k] for k in ('trial','scan_hz','baseline_register_n','baseline_end_register_n','baseline_end_axia_estimate_n','running_samples','fixed_home_baseline_evaluation','retrospective_endpoint_baseline_evaluation','apparent_axis_lag')},indent=2))


if __name__=='__main__':main()
