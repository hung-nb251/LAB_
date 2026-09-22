#!/usr/bin/env python3
"""Reproducible offline P0 calibration. No ROS, hardware, or runtime writes.

Source logs are immutable. Output directories must be new. Trial/session splits
and quality gates are declared here, never optimized on the runtime holdouts.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import uniform_filter1d

import analyze_hc_dynamic_branches as base

ROOT = Path(__file__).resolve().parent.parent
JOINTS = base.JOINTS
SOLVER = base.LocalIKSolver()
OFFSET = np.array([0., .120, .0354])
GRAVITY = np.array([0., 0., -9.80665 * 1.126])
CORRECTION = Rotation.from_euler('z', -90, degrees=True).as_matrix().T


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def spectrum(x, floor=.05):
    x = np.asarray(x, float).reshape(-1, 6)
    if not len(x):
        return {'n': 0, 'rank': 0}
    scale = np.maximum(np.sqrt(np.mean(x*x, axis=0)), floor)
    _, s, vt = np.linalg.svd(x / scale, full_matrices=True)
    rank = int(np.linalg.matrix_rank(x))
    # Numerical rank and practical excitation are deliberately separate.
    padded = np.pad(s, (0, 6-len(s)))
    return dict(n=len(x), rank=rank, scale=scale.tolist(), singular_values=padded.tolist(),
                practical_rank_1pct=int(np.sum(padded > padded[0]*.01)) if padded[0] else 0,
                condition=float(padded[0]/padded[-1]) if padded[-1] > 1e-12 else None,
                weak_direction_normalized=vt[-1].tolist(),
                joint_range_rad=np.ptp(x, axis=0).tolist())


def interpolate(times, series, gap=.2):
    """No extrapolation, including one-point/empty inputs. Dedupe timestamps."""
    a = np.asarray(series, float)
    if a.ndim != 2 or len(a) < 2:
        width = a.shape[1]-1 if a.ndim == 2 else 6
        return np.full((len(times), width), np.nan), np.zeros(len(times), bool)
    a = a[np.argsort(a[:, 0], kind='stable')]
    _, idx = np.unique(a[:, 0], return_index=True)
    a = a[idx]
    if len(a) < 2:
        return np.full((len(times), a.shape[1]-1), np.nan), np.zeros(len(times), bool)
    out, good = base.interp(np.asarray(times), a, gap)
    return out, good & np.isfinite(out).all(axis=1)


def at(times, changes, default):
    return base.status_at(times, changes, default)


def load_log(path):
    """Inventory every log, including failed/partial trials, in one pass."""
    row = dict(path=str(path.relative_to(ROOT)), trial=path.parent.name,
               source_sha256=digest(path), bytes=path.stat().st_size)
    mp = path.parent/'metadata.json'
    if mp.exists():
        metadata = json.loads(mp.read_text())
        row['metadata_sha256'] = digest(mp)
        row['snapshot_hashes'] = {k: v.get('sha256') for k, v in metadata.get('source_snapshots', {}).items()}
    counts = Counter(); regs = {}; complete = set(); markers = []; raw = []; joints = []
    edges = {}; states = []; running = []; health = {'connected': [], 'calibrated': []}
    faults = []; latency = []; start_ns = None; t = 0.; args = {}; tf_samples = []
    for line in path.open():
        e = json.loads(line)
        if start_ns is None:
            start_ns = e['monotonic_ns']
        t = (e['monotonic_ns']-start_ns)/1e9
        kind = e['kind']; counts[kind] += 1
        if kind == 'start':
            args = e.get('arguments', {})
        elif kind == 'scan_end':
            complete.add(e['scan_id'])
        elif kind == 'marker' and e.get('command', '').strip():
            markers.append((t, e['command'].strip()))
        elif kind == 'register' and e.get('response', {}).get('success') and not e.get('late'):
            sent = (e.get('sent_monotonic_ns', e['monotonic_ns'])-start_ns)/1e9
            regs.setdefault(e['address'], []).append(((t+sent)/2, (e['response']['value']-10000)*.1, e['scan_id']))
            latency.append(t-sent)
        elif kind == 'topic':
            m = e['message']; topic = e['topic']
            if topic == '/joint_states' and all(j in m['name'] for j in JOINTS):
                joints.append([t, *[m['position'][m['name'].index(j)] for j in JOINTS]])
            elif topic == '/axia/raw_wrench':
                raw.append([t, *[m['wrench'][part][a] for part in ['force', 'torque'] for a in 'xyz']])
            elif topic == '/cocarry/status':
                states.append((t, m['data']))
                if m['data'].startswith('FAULT'):
                    faults.append((t, m['data']))
            elif topic == '/run_status':
                running.append((t, bool(m['data'])))
            elif topic in ['/axia/connected', '/axia/calibrated']:
                health[topic.split('/')[-1]].append((t, bool(m['data'])))
            elif topic in ['/tf', '/tf_static']:
                for tr in m['transforms']:
                    v = tr['transform']; mat = np.eye(4)
                    mat[:3, :3] = Rotation.from_quat([v['rotation'][a] for a in 'xyzw']).as_matrix()
                    mat[:3, 3] = [v['translation'][a] for a in 'xyz']
                    edges[tr['child_frame_id']] = (tr['header']['frame_id'], mat)
                if not tf_samples or t-tf_samples[-1][0] >= 1:
                    try:
                        tf_samples.append((t, base.transform_chain(edges, 'tool0').copy()))
                    except ValueError:
                        pass
    static = args.get('category') == 'static_cog' and 'no_contact' in args.get('notes', '')
    mode = args.get('mode', 'unknown').upper()
    session = args.get('session') or ('legacy_'+path.parent.name[:8])
    pose = args.get('pose', 'unknown')
    # Metadata pose is preferable to parent directory (one recorded path is wrong).
    route = 'T1' if 'target1' in pose else 'T2' if 'target2' in pose else pose
    labels = ['static_no_contact'] if static else ['force_interaction', mode]
    if any(p.startswith('no_contact_motion_begin') for _, p in markers):
        labels.append('dynamic_no_contact')
    row.update(arguments=args, session=session, date=path.parent.name[:8], mode=mode,
               route=route, labels=labels, duration_s=t, event_counts=dict(counts),
               register_counts={str(k):len(v) for k,v in regs.items()}, markers=markers,
               faults=sorted(set(s for _,s in faults)), service_latency_s=base.stats(latency),
               inclusion=[], exclusion=[], baseline_segments=[])
    return row, dict(regs=regs, complete=complete, raw=np.asarray(raw), joints=np.asarray(joints),
                     markers=markers, states=states, running=running, health=health, edges=edges,
                     static=static, tf_samples=tf_samples)


def prepare_trial(row, source):
    regs = source['regs']; complete = source['complete']; joints = source['joints']; raw = source['raw']
    if not all(a in regs for a in range(310,316)):
        row['exclusion'].append('missing_M310_M315'); return None, []
    if len(joints) < 2 or len(raw) < 2:
        row['exclusion'].append('missing_joint_states_or_raw_Axia'); return None, []
    scan = {}
    for a in range(310,316):
        for tm, val, sid in regs[a]:
            if sid in complete:
                scan.setdefault(sid, {})[a] = tm
    scans = [s for s in scan.values() if len(s) == 6]
    if len(scans) < 3:
        row['exclusion'].append('too_few_complete_tau_scans'); return None, []
    times = np.array([np.mean(list(s.values())) for s in scans])
    row['tau_scan_span_s'] = base.stats([max(s.values())-min(s.values()) for s in scans])
    row['tau_sample_interval_s'] = base.stats(np.diff(times))
    # Align scalar channels to common mean timestamp; original sequential span remains recorded.
    tau = []; good = np.ones(len(times), bool)
    for a in range(310,316):
        series = np.asarray([v[:2] for v in regs[a] if v[2] in complete])
        values, valid = interpolate(times, series, 1.5)
        tau.append(values[:, 0]); good &= valid
    tau = np.column_stack(tau)
    q, valid = interpolate(times, joints); good &= valid
    raw_q, raw_valid = interpolate(raw[:, 0], joints)
    measured, valid = interpolate(times, raw); good &= valid
    states = at(times, source['states'], '')
    isrunning = np.array([s.startswith('RUNNING') for s in states]) & at(times, source['running'], True)
    healthy = np.ones(len(times), bool)
    raw_healthy = np.ones(len(raw), bool)
    for changes in source['health'].values():
        healthy &= at(times, changes, False)
        raw_healthy &= at(raw[:, 0], changes, False)
    good &= healthy
    row['healthy_paired_samples'] = int(good.sum())
    phase = at(times, source['markers'], 'setup')
    baselines = []
    windows = []
    if source['static']:
        windows.append(('static_no_contact', max(0, row['duration_s']-60), row['duration_s']))
    else:
        markers = source['markers']
        for i, (lo, name) in enumerate(markers):
            if name in ['baseline', 'baseline_end'] or name.startswith('no_contact_begin:'):
                hi = markers[i+1][0] if i+1 < len(markers) else row['duration_s']
                windows.append((name, lo+1., hi-1.))
    for name, lo, hi in windows:
        # Deliberate static no-admittance trials may have no status topic at all.
        # GT protocol explicitly marks a no-contact initial baseline after Start.
        # Its RUNNING flag does not invalidate stationary, healthy baseline data.
        mask = good & (times >= lo) & (times <= hi)
        if name != 'baseline':
            mask &= ~isrunning
        rmask = raw_valid & raw_healthy & (raw[:, 0]>=lo) & (raw[:, 0]<=hi)
        jmask = (joints[:,0]>=lo) & (joints[:,0]<=hi)
        item = dict(name=name, window_s=[lo,hi], n=int(mask.sum()), accepted=False)
        if mask.sum() >= 3 and rmask.sum() >= 100 and jmask.sum() >= 20 and hi-lo >= 2:
            qr = np.ptp(joints[jmask,1:], axis=0)
            limit = .002 if source['static'] else .02
            item.update(joint_range_rad=qr.tolist(), raw_axia_std_n=np.std(raw[rmask,1:4],axis=0).tolist())
            if np.max(qr) < limit:
                item.update(accepted=True, q=np.median(q[mask],axis=0).tolist(),
                            tau=np.median(tau[mask],axis=0).tolist(),
                            tau_std_nm=np.std(tau[mask],axis=0).tolist(),
                            raw=np.median(raw[rmask,1:],axis=0).tolist(),
                            trial=row['trial'], session=row['session'], route=row['route'],
                            pose=row['arguments'].get('pose'), source=row['path'],
                            static=source['static'])
                baselines.append(item)
        if not item['accepted']:
            item['reason'] = 'insufficient_healthy_samples_or_not_stationary'
        row['baseline_segments'].append(item)
    if source['static']:
        if baselines:
            row['inclusion'].append('static_baseline')
        else:
            row['exclusion'].append('no_usable_static_window')
        return None, baselines
    initial = next((b for b in baselines if b['name']=='baseline'), None)
    if initial is None:
        row['exclusion'].append('missing_healthy_stationary_initial_baseline'); return None, baselines
    try:
        static_transform = base.transform_chain(source['edges'], 'axia_sensor_link', 'tool0')
    except ValueError as exc:
        row['exclusion'].append(str(exc)); return None, baselines
    q0 = np.array(initial['q']); tau0 = np.array(initial['tau'])
    # Reconstruct physical sensor->base exactly as the current Axia UI convention.
    poses = np.array([SOLVER.forward_kinematics(v) for v in q])
    rotations = poses[:,:3,:3] @ static_transform[:3,:3] @ CORRECTION
    r0 = SOLVER.forward_kinematics(q0)[:3,:3] @ static_transform[:3,:3] @ CORRECTION
    grav_sensor = np.einsum('nji,j->ni', rotations, GRAVITY)
    bias = np.array(initial['raw'])[:3] - r0.T @ GRAVITY
    # Moment gravity is not calibrated: baseline moment subtraction, documented sensitivity.
    def wrench(values):
        force = np.einsum('nij,nj->ni', rotations, values[:,:3]-grav_sensor-bias)
        moment = np.einsum('nij,nj->ni', rotations, values[:,3:]-np.array(initial['raw'])[3:])
        moment += np.cross(np.einsum('nij,j->ni',rotations,OFFSET),force)
        return np.column_stack((force,moment))
    w_raw = wrench(measured)
    win = max(1, int(round(.10/np.median(np.diff(raw[:,0])))))
    smoothed = np.column_stack((raw[:,0],uniform_filter1d(raw[:,1:],size=win,axis=0,mode='nearest')))
    sm, valid = interpolate(times, smoothed); good &= valid
    w = wrench(sm)
    # Filter support must be healthy, continuous and joint-paired; reject gaps within ±50 ms.
    health_series = np.column_stack((raw[:,0],raw_valid & raw_healthy))
    for shift in [-.05, 0., .05]:
        h, v = interpolate(times+shift, health_series)
        good &= v & (h[:,0] >= 1)
    j = np.array([SOLVER.compute_jacobian(v) for v in q])
    speed = np.linalg.norm(np.gradient(poses[:,:3,3],times,axis=0),axis=1)
    no_contact = np.zeros(len(times),bool)
    for b in baselines:
        if b['name'] != 'baseline':
            lo,hi = b['window_s']; bm = good & (times>=lo)&(times<=hi)
            drift = float(np.median(np.linalg.norm(w[bm,:3],axis=1))) if bm.any() else None
            b['reference_residual_n'] = drift
            # Marker is necessary, force threshold only quality-gates an already marked segment.
            if drift is not None and drift < 2:
                no_contact |= bm
            else:
                b['accepted'] = False; b['reason'] = 'marked_end_baseline_has_reference_residual_ge2N'
    baselines = [b for b in baselines if b['accepted']]
    static_force = row['mode']=='STATIC'
    excluded_phase = np.isin(phase,['baseline','baseline_end','setup','stopped','pre_start'])
    excluded_phase |= np.array([p.startswith('no_contact_') for p in phase])
    force_mask = good & ~excluded_phase & (isrunning if not static_force else True)
    # Discard fault states even for static force trials; WAIT_AXIA_CALIBRATION can be
    # controller-local while Axia calibrated/raw topics remain healthy.
    force_mask &= ~np.array([s.startswith('FAULT') for s in states])
    # Bound each trial before its first fault after RUNNING; never use recovery without a new baseline.
    starts = times[isrunning]
    if len(starts):
        ft = [t for t,s in source['states'] if t>starts[0] and s.startswith('FAULT')]
        if ft:
            force_mask &= times < min(ft)
    row['force_paired_samples'] = int(force_mask.sum())
    row['moving_force_samples'] = int((force_mask & (speed>.005)).sum())
    row['static_tool_sensor_transform'] = static_transform.tolist()
    row['raw_reference_filter'] = dict(kind='offline_centered_boxcar', nominal_window_s=.10, samples=win, lag_s=0.)
    row['orientation_excursion_deg'] = base.stats(np.degrees(Rotation.from_matrix(r0.T @ rotations).magnitude()))
    qt, v = interpolate(np.array([t for t,_ in source['tf_samples']]),joints)
    errs=[np.linalg.norm(SOLVER.fk_position(a)-tf[:3,3]) for a,(_,tf),g in zip(qt,source['tf_samples'],v) if g]
    row['fk_vs_tf_position_m'] = base.stats(errs)
    if force_mask.sum() < 6:
        row['exclusion'].append('fewer_than_6_healthy_force_samples'); return None, baselines
    row['inclusion'].append('force_fit_and_grouped_validation')
    use = force_mask | no_contact
    return dict(t=times[use], tau=tau[use]-tau0, q=q[use], dq=q[use]-q0, j=j[use],
                w=w[use], w_raw=w_raw[use], force_mask=force_mask[use], no_contact=no_contact[use],
                speed=speed[use], q0=q0, tau0=tau0, meta=row), baselines


def prepare(output):
    paths = sorted((ROOT/'cocarry_logs').glob('hc_force*/**/events.jsonl'))
    manifest=[]; baseline=[]; arrays={}; metas=[]
    for i,path in enumerate(paths):
        row, src = load_log(path)
        trial, bs = prepare_trial(row,src)
        manifest.append(row); baseline.extend(bs)
        if trial is not None:
            ix=len(metas); metas.append(row)
            for key,val in trial.items():
                if key!='meta': arrays[f'{ix}_{key}']=val
        print(f"{i+1}/{len(paths)} {row['trial']}: {row['inclusion']} {row['exclusion']}", flush=True)
    # Repeated attempts at same static pose: use latest valid trial, retain all inventory entries.
    latest={}
    for b in baseline:
        if b['static']:
            key=(b['session'],b['pose'])
            if key not in latest or b['trial'] > latest[key]['trial']: latest[key]=b
    baseline=[b for b in baseline if not b['static'] or latest[(b['session'],b['pose'])] is b]
    save(output/'manifest.json',manifest); save(output/'baselines.json',baseline)
    save(output/'trial_metadata.json',metas); np.savez_compressed(output/'paired_samples.npz',**arrays)
    save(output/'provenance.json',dict(script_sha256=digest(Path(__file__)),
         helper_sha256=digest(ROOT/'analyze_hc_dynamic_branches.py'),
         kinematics_sha256=digest(ROOT/'src/hc10dtp_bringup/scripts/local_ik_solver.py'),
         status='OFFLINE_INPUTS_NO_RUNTIME_CHANGE',event_files=len(paths),
         calibration_root_files=sum('hc_force_calibration' in p.parts for p in paths),
         sidecars=[str(p.relative_to(ROOT)) for p in (ROOT/'cocarry_logs').rglob('*.calibration.jsonl')],
         assumptions=['Recorded session labels are not proof that tool/tare remained unchanged.',
          'Raw Axia may contain hardware tare. Each force trial has its own marked initial baseline.',
          'Axia reference uses 1.126 kg, recorded static TF, yaw -90 deg, moment arm [0,.120,.0354] m.',
          'Moment gravity and inertia are not independently calibrated.',
          'Receive monotonic clock for topics; mean request/response midpoint for M310..315.',
          'Common tau timestamp is not an atomic hardware sample; no lag optimization.',
          'Reference smoothing 0.10 s offline centered boxcar; unsmoothed sensitivity retained.']))


def read_prepared(path):
    metas=json.loads((path/'trial_metadata.json').read_text())
    z=np.load(path/'paired_samples.npz',allow_pickle=False)
    trials=[]
    for i,m in enumerate(metas):
        d={k.split('_',1)[1]:z[k] for k in z.files if k.startswith(f'{i}_')}; d['meta']=m; trials.append(d)
    return trials,json.loads((path/'baselines.json').read_text())


def baseline_design(baselines):
    """One pose median, equal session weight, session centering removes arbitrary offsets."""
    xx=[]; yy=[]; ww=[]
    for session in sorted({b['session'] for b in baselines}):
        bs=[b for b in baselines if b['session']==session]
        q=np.array([b['q'] for b in bs]); tau=np.array([b['tau'] for b in bs])
        xx.extend(q-q.mean(axis=0)); yy.extend(tau-tau.mean(axis=0)); ww.extend([1/len(bs)]*len(bs))
    x=np.asarray(xx).reshape(-1,6); y=np.asarray(yy).reshape(-1,6); weight=np.asarray(ww)
    if len(weight): weight/=weight.sum()
    return x,y,weight


def ridge(x,y,weight,alpha):
    scale=np.maximum(np.sqrt(np.sum(weight[:,None]*x*x,axis=0)),.05)
    z=x/scale
    coef=np.linalg.solve(z.T@(weight[:,None]*z)+alpha*np.eye(x.shape[1]),z.T@(weight[:,None]*y))
    return coef/scale[:,None]


def fit(trials, baselines, family, alpha=.01):
    b=np.zeros((6,6))
    if family.startswith('baseline_pose'):
        x,y,weights=baseline_design(baselines)
        if len(x): b=ridge(x,y,weights,alpha)
    xx=[]; yy=[]; weights=[]
    for d in trials:
        mask=d['force_mask']; x=d['tau'][mask]-d['dq'][mask]@b
        if family=='joint_pose_gain': x=np.column_stack((x,d['dq'][mask]))
        xx.append(x); yy.append(np.einsum('nji,nj->ni',d['j'][mask],d['w'][mask]))
        weights.extend([1/mask.sum()]*int(mask.sum()))
    x=np.vstack(xx); y=np.vstack(yy); weight=np.asarray(weights); weight/=weight.sum()
    if family.endswith('diagonal'):
        scale=np.maximum(np.sqrt(np.sum(weight[:,None]*x*x,axis=0)),.05)
        z=x/scale
        a=np.diag(np.sum(weight[:,None]*z*y,axis=0)/(np.sum(weight[:,None]*z*z,axis=0)+alpha)/scale)
    else:
        a=ridge(x,y,weight,alpha)
    coef=a if family=='joint_pose_gain' else np.vstack((a,-b@a))
    return dict(branch='torque',variant='pose',scale=[1.]*12,coef=coef.tolist(),lag=0,
                family=family,alpha=alpha,baseline_coef_nm_per_rad=b.tolist(),damping=.01,
                characteristic_length_m=.3,status='OFFLINE_CANDIDATE_NOT_DEPLOYED',
                calibration_confirmed=False,role_valid=False,
                train_trials=[d['meta']['trial'] for d in trials],
                baseline_trials=sorted({b['trial'] for b in baselines}) if family.startswith('baseline_pose') else [],
                available_baseline_trials=sorted({b['trial'] for b in baselines}))


def prediction(model,d):
    tau=(np.column_stack((d['tau'],d['dq']))/model['scale'])@model['coef']
    return base.recover(tau,d['j'])[:,:3]


def evaluate(model,trials):
    results=[]
    for d in trials:
        p=prediction(model,d); mask=d['force_mask']; nc=d['no_contact']
        results.append(dict(trial=d['meta']['trial'],session=d['meta']['session'],route=d['meta']['route'],
                            mode=d['meta']['mode'],metrics=base.metrics(p[mask],d['w'][mask,:3]),
                            unsmoothed_reference=base.metrics(p[mask],d['w_raw'][mask,:3]),
                            moving=base.metrics(p[mask & (d['speed']>.005)],d['w'][mask & (d['speed']>.005),:3]),
                            no_contact=base.metrics(p[nc],np.zeros((nc.sum(),3)))))
    return results


def aggregate(rows):
    if not rows: return {'trials':0}
    n=sum(r['metrics']['n'] for r in rows)
    return dict(trials=len(rows),n=n,
                macro_rmse_n=float(np.mean([r['metrics']['rmse_vector_n'] for r in rows])),
                pooled_rmse_n=float(np.sqrt(sum(r['metrics']['n']*r['metrics']['rmse_vector_n']**2 for r in rows)/n)),
                worst_trial_rmse_n=float(max(r['metrics']['rmse_vector_n'] for r in rows)))


def audit_stability(baselines, trials):
    """Describe drift without silently removing inconvenient samples."""
    pairs=[]
    for i,b in enumerate(baselines):
        for c in baselines[i+1:]:
            distance=float(np.max(np.abs(np.array(b['q'])-c['q'])))
            if distance < .01:
                delta=np.array(c['tau'])-b['tau']
                pairs.append(dict(first=b['trial']+':'+b['name'],second=c['trial']+':'+c['name'],
                    same_session=b['session']==c['session'],joint_distance_max_rad=distance,
                    torque_delta_nm=delta.tolist(),torque_delta_norm_nm=float(np.linalg.norm(delta))))
    result=dict(same_pose_tolerance_rad=.01,pairs=pairs,
        summary={k:base.stats([p['torque_delta_norm_nm'] for p in pairs if p['same_session']==same])
                 for k,same in [('within_session',True),('between_sessions',False)]},
        high_scatter_threshold_nm=.5,
        high_scatter_baselines=[dict(trial=b['trial'],phase=b['name'],tau_std_nm=b['tau_std_nm'],
                                     raw_axia_std_n=b['raw_axia_std_n'])
                               for b in baselines if max(b['tau_std_nm'])>.5],
        caveat='Pair differences are correlated and are not independent confidence intervals. '
               'The 0.5 Nm scatter threshold is exploratory QC, not an exclusion in the primary model.')
    route_directions={}
    for route in sorted({d['meta']['route'] for d in trials}):
        ds=[d for d in trials if d['meta']['route']==route]
        w=np.vstack([d['w'][d['force_mask'],:3] for d in ds])
        norm=np.linalg.norm(w,axis=1); unit=w[norm>=4]/norm[norm>=4,None]
        route_directions[route]=dict(trials=len(ds),force_samples=len(w),
            cone_counts={a+label:int(np.sum(sign*unit[:,i]>=np.cos(np.pi/6)))
                         for i,a in enumerate('XYZ') for sign,label in [(1,'+'),(-1,'-')]})
    result['direction_coverage_by_route']=route_directions
    result['direction_definition']='Reference force >=4 N, cone half-angle 30 degrees; route is not fixed target pose.'
    static=[b for b in baselines if b['static']]
    rot=np.array([SOLVER.forward_kinematics(b['q'])[:3,:3] for b in static])
    angles=[float(np.degrees(Rotation.from_matrix(a.T@b).magnitude())) for i,a in enumerate(rot) for b in rot[i+1:]]
    result['static_tool_orientation_separation_deg']=base.stats(angles)
    return result


def verify_sources(prepared,manifest):
    """Abort evaluation if cached data no longer refer to the same immutable sources."""
    for row in manifest:
        if digest(ROOT/row['path'])!=row['source_sha256']:
            raise ValueError('Source log changed: '+row['path'])
        metadata=(ROOT/row['path']).parent/'metadata.json'
        if 'metadata_sha256' in row and digest(metadata)!=row['metadata_sha256']:
            raise ValueError('Source metadata changed: '+str(metadata))
    return dict(verified_event_files=len(manifest),manifest_sha256=digest(prepared/'manifest.json'))


def run_evaluation(prepared,output):
    trials,baselines=read_prepared(prepared)
    manifest=json.loads((prepared/'manifest.json').read_text())
    integrity=verify_sources(prepared,manifest)
    # September 12 is a legacy configuration; evaluate separately, do not mix into current fit.
    current=[d for d in trials if d['meta']['date'] >= '20260915']
    baselines=[b for b in baselines if b['trial'][:8]>='20260915']
    static=[b for b in baselines if b['static']]
    stability=audit_stability(baselines,current)
    save(output/'baseline_stability_and_direction_coverage.json',stability)
    x,_,_=baseline_design(baselines); sx,_,_=baseline_design(static)
    coverage=dict(all_baselines=spectrum(x),static_poses=spectrum(sx),
                  baseline_count=len(baselines),static_pose_count=len(static),
                  force_trials=len(current),force_samples=sum(int(d['force_mask'].sum()) for d in current),
                  by_session={s:spectrum(baseline_design([b for b in baselines if b['session']==s])[0])
                              for s in sorted({b['session'] for b in baselines})})
    directions=[]
    for d in current:
        w=d['w'][d['force_mask'],:3]; norm=np.linalg.norm(w,axis=1); active=norm>=4
        unit=w[active]/norm[active,None]; directions.extend(unit)
        for axis,a in enumerate('XYZ'):
            for sign,label in [(1,'+'),(-1,'-')]:
                coverage.setdefault('direction_cones_30deg',{}).setdefault(a+label,[]).append(dict(
                    trial=d['meta']['trial'],n=int(np.sum(unit[:,axis]*sign>=np.cos(np.pi/6)))))
    units=np.asarray(directions)
    coverage['force_direction_singular_values']=np.linalg.svd(units,compute_uv=False).tolist()
    save(output/'coverage.json',coverage)
    families=['constant_diagonal','constant_full','baseline_pose_diagonal','baseline_pose_full','joint_pose_gain']
    variants=[(f,a) for f in families for a in [.01,.1,1.]]
    deployed=json.loads((ROOT/'src/hc10dtp_bringup/config/f_robot_m310_candidate_20260918.json').read_text())
    models={}; results=[]; folds=[]
    # Chronological train/validation/history evaluation, no random sample splits.
    train=[d for d in current if d['meta']['date']<'20260918' or
           (d['meta']['date']=='20260918' and d['meta']['trial']<'20260918_163000')]
    validation=[d for d in current if '20260918_163006' in d['meta']['trial'] or '20260918_163243' in d['meta']['trial']]
    test=[d for d in current if '20260918_163518' in d['meta']['trial']]
    # Include healthy baseline-only/aborted trials as well, before the chronological
    # validation boundary. Their lack of force samples does not invalidate a tare.
    train_bs=[b for b in baselines if b['trial']<'20260918_163000']
    for family,alpha in variants:
        key=f'{family}_ridge_{alpha:g}'; model=fit(train,train_bs,family,alpha); models[key]=model
        for split,ds in [('validation_r3',validation),('historical_r4',test)]:
            rr=evaluate(model,ds)
            results.append(dict(model=key,protocol=split,aggregate=aggregate(rr),per_trial=rr))
    candidates=[r for r in results if r['protocol']=='validation_r3']
    selected=min(candidates,key=lambda r:r['aggregate']['macro_rmse_n'])['model']
    save(output/'selection.json',dict(selected=selected,criterion='lowest validation r3 trial-macro force RMSE',
         train=[d['meta']['trial'] for d in train],validation=[d['meta']['trial'] for d in validation],
         historical_test=[d['meta']['trial'] for d in test],
         caveat='r3/r4 already inspected historically; neither is a new unseen prospective test'))
    # Explicit complete session transfer and route transfer. Each fold fits its own baselines.
    for session in sorted({d['meta']['session'] for d in current}):
        tr=[d for d in current if d['meta']['session']!=session]
        te=[d for d in current if d['meta']['session']==session]
        bs=[b for b in baselines if b['session']!=session]
        folds.append(('leave_session_out:'+session,tr,te,bs))
    for day in sorted({d['meta']['date'] for d in current}):
        folds.append(('leave_day_out:'+day,
                      [d for d in current if d['meta']['date']!=day],
                      [d for d in current if d['meta']['date']==day],
                      [b for b in baselines if b['trial'][:8]!=day]))
    for route in ['T1','T2']:
        tr=[d for d in current if d['meta']['route']!=route]
        te=[d for d in current if d['meta']['route']==route]
        allowed={d['meta']['trial'] for d in tr}
        # Omit static baseline session for route transfer: its unlabeled XYZ could
        # overlap held-out route poses. This deliberately tests route extrapolation.
        bs=[b for b in baselines if b['trial'] in allowed]
        folds.append(('leave_route_out:'+route,tr,te,bs))
    selected_family=models[selected]['family']; selected_alpha=models[selected]['alpha']
    for protocol,tr,te,bs in folds:
        if not tr or not te: continue
        for family,alpha in [(f,.01) for f in families]+([(selected_family,selected_alpha)] if selected_alpha!=.01 else []):
            model=fit(tr,bs,family,alpha); key=f'{family}_ridge_{alpha:g}'
            test_ids={d['meta']['trial'] for d in te}
            assert not test_ids.intersection(model['train_trials'])
            assert not test_ids.intersection(model['baseline_trials'])
            rr=evaluate(model,te)
            results.append(dict(model=key,protocol=protocol,aggregate=aggregate(rr),per_trial=rr,
                                train_trials=model['train_trials'],baseline_trials=model['baseline_trials'],
                                interpretation=('Fixed predeclared ridge 0.01 per family; no held-out coefficient fitting.'
                                  if alpha==.01 else 'Exploratory selected hyperparameters reuse historical r3; not nested CV.')))
    for protocol,ds in [('all_current_historical',current),('validation_r3',validation),('historical_r4',test)]:
        rr=evaluate(deployed,ds)
        results.append(dict(model='deployed_20260918',protocol=protocol,aggregate=aggregate(rr),per_trial=rr,
                            caveat='deployed contains some evaluated trials in training; comparison only'))
    # Static leave-one-pose-out validates baseline shape without force samples or held-out tau.
    static_rows=[]
    for b in static:
        tr=[s for s in static if s['pose']!=b['pose']]
        x,y,weight=baseline_design(tr)
        qmean=np.mean([s['q'] for s in tr],axis=0); tmean=np.mean([s['tau'] for s in tr],axis=0)
        for family in ['constant','pose']:
            bc=np.zeros((6,6)) if family=='constant' else ridge(x,y,weight,.01)
            residual=np.array(b['tau'])-tmean-(np.array(b['q'])-qmean)@bc
            static_rows.append(dict(pose=b['pose'],trial=b['trial'],model=family,
                torque_residual_nm=residual.tolist(),torque_vector_error_nm=float(np.linalg.norm(residual))))
    save(output/'static_baseline_cross_validation.json',static_rows)
    # Quantify noisy-baseline sensitivity without tuning or replacing the selected model.
    quiet=[b for b in train_bs if max(b['tau_std_nm'])<=.5]
    sensitivity=[]
    for family in ['baseline_pose_diagonal','baseline_pose_full']:
        model=fit(train,quiet,family,.01)
        for protocol,ds in [('validation_r3',validation),('historical_r4',test)]:
            rr=evaluate(model,ds)
            sensitivity.append(dict(family=family,protocol=protocol,aggregate=aggregate(rr),
                baseline_trials=model['baseline_trials'],per_trial=rr))
    save(output/'baseline_scatter_sensitivity.json',dict(threshold_nm=.5,baseline_count=len(quiet),
         caveat='Exploratory audit after initial analysis; not used for candidate selection.',results=sensitivity))
    # Refit train+r3 only after choosing the family/ridge; r4 is excluded from
    # coefficients and end-baseline training. Keep validation-stage model separately.
    frozen_bs=[b for b in baselines if b['trial']<'20260918_163500']
    frozen=fit(train+validation,frozen_bs,selected_family,selected_alpha)
    frozen['selection_stage_model']=selected
    frozen['scope']='Train plus r3 refit; r4 and runtime CSVs excluded; local shadow research candidate.'
    save(output/'candidate_selection_stage.json',models[selected])
    save(output/'candidate_frozen.json',frozen)
    rr=evaluate(frozen,test)
    results.append(dict(model='p0_frozen_train_plus_r3',protocol='historical_r4',aggregate=aggregate(rr),per_trial=rr))
    static_residuals=[]
    anchor=min(static,key=lambda b:b['trial'])
    for b in static:
        d=dict(tau=np.array(b['tau'])[None]-anchor['tau'],dq=np.array(b['q'])[None]-anchor['q'],
               j=SOLVER.compute_jacobian(b['q'])[None])
        for name,model in [('p0_frozen',frozen),('deployed_20260918',deployed)]:
            value=prediction(model,d)[0]
            static_residuals.append(dict(model=name,pose=b['pose'],force_n=value.tolist(),norm_n=float(np.linalg.norm(value))))
    save(output/'static_no_contact_force_residuals.json',dict(anchor_trial=anchor['trial'],results=static_residuals,
        caveat='Descriptive replay, not held-out validation; some candidates may use these static baselines.'))
    save(output/'candidate_refit_all_exploratory.json',fit(current,baselines,selected_family,selected_alpha))
    save(output/'candidate_models.json',models); save(output/'results.json',results)
    # These runtime logs cannot supply calibration targets during training/model selection.
    from audit_hc_baseline_calibration import runtime_comparison
    runtime=[]
    for stamp in ['172207','174205']:
        path=ROOT/f'cocarry_logs/cocarry_admittance_3d_20260918_{stamp}.csv'
        item,_=runtime_comparison(path,{'selected':frozen,'deployed':deployed})
        runtime.append(item)
    save(output/'runtime_historical_checks.json',runtime)
    save(output/'provenance.json',dict(input_directory=str(prepared),script_sha256=digest(Path(__file__)),
        integrity=integrity,
        input_manifest_sha256=digest(prepared/'manifest.json'),input_samples_sha256=digest(prepared/'paired_samples.npz'),
        runtime_model_sha256=digest(ROOT/'src/hc10dtp_bringup/config/f_robot_m310_candidate_20260918.json'),
        runtime_csv_sha256={r['file']:digest(Path(r['file'])) for r in runtime},
        status='OFFLINE_COMPLETE_CALIBRATION_NOT_CONFIRMED'))
    save(output/'source_snapshot.json',dict(script=Path(__file__).read_text(),
         helper=(ROOT/'analyze_hc_dynamic_branches.py').read_text(),
         runtime_audit=(ROOT/'audit_hc_baseline_calibration.py').read_text()))
    save(output/'decision.json',dict(status='RETAIN_DEPLOYED_SHADOW_NOT_CALIBRATED',
        candidate='candidate_frozen.json',selection_stage=selected,
        reason='Candidate must improve both historical r4 and long runtime without loss of direction/zero-load quality. '
               'No independent prospective test or validated global calibration is available.',
        raw_runtime_candidate_rmse_n=runtime[0]['candidate_metrics']['selected']['rmse_vector_n'],
        raw_runtime_deployed_rmse_n=runtime[0]['candidate_metrics']['deployed']['rmse_vector_n'],
        interpretation='Runtime reference inferred before deadband, not truly raw Axia.',
        runtime_model_modified=False,robot_commands_sent=False))
    report(output,prepared,manifest,coverage,results,selected,runtime,static_rows)
    review_plots(output,prepared,current,baselines,frozen,deployed)
    with (output/'manifest.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['trial','session','mode','route','labels','force_paired_samples',
                              'inclusion','exclusion','source_sha256','path'])
        writer.writeheader()
        for r in manifest:
            writer.writerow({k:json.dumps(r[k],ensure_ascii=False) if isinstance(r.get(k),list) else r.get(k,'')
                             for k in writer.fieldnames})
    with (output/'metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['model','protocol','trials','n','macro_rmse_n','pooled_rmse_n','worst_trial_rmse_n'])
        writer.writeheader()
        for r in results: writer.writerow(dict(model=r['model'],protocol=r['protocol'],**r['aggregate']))


def review_plots(output,prepared,trials,baselines,selected,deployed):
    import matplotlib.pyplot as plt
    # r3 was used for frozen-model refitting; r4 remains coefficient-held-out.
    for d in trials:
        if not any(t in d['meta']['trial'] for t in ['163006','163243','163518']): continue
        mask=d['force_mask']; t=d['t'][mask]-d['t'][mask][0]
        p=prediction(selected,d)[mask]; old=prediction(deployed,d)[mask]
        fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
        for i,ax in enumerate(axes):
            ax.plot(t,d['w'][mask,i],'k.-',label='Axia reconstructed')
            ax.plot(t,p[:,i],'.-',label='P0 frozen candidate')
            ax.plot(t,old[:,i],'--',label='Deployed 18 Sep')
            ax.set_ylabel('XYZ'[i]+' (N)'); ax.grid(alpha=.2)
        role='historical holdout' if '163518' in d['meta']['trial'] else 'r3 included in frozen refit'
        axes[0].legend(); axes[0].set_title(d['meta']['trial']+'\n'+role)
        axes[-1].set_xlabel('Force segment elapsed time (s)')
        fig.tight_layout(); fig.savefig(output/(d['meta']['trial']+'_force.png'),dpi=140); plt.close(fig)
    static=[b for b in baselines if b['static']]
    xyz=np.array([SOLVER.fk_position(b['q']) for b in static])
    fig=plt.figure(figsize=(8,6)); ax=fig.add_subplot(111,projection='3d')
    ax.scatter(*xyz.T,label='12 static no-contact poses',s=30)
    for b,pos in zip(static,xyz): ax.text(*pos,b['pose'],fontsize=7)
    for route,color in [('T1','tab:orange'),('T2','tab:green')]:
        ds=[d for d in trials if d['meta']['route']==route and d['meta']['mode']=='GRU']
        if ds:
            coords=np.array([SOLVER.fk_position(q) for d in ds for q in d['q'][d['force_mask']]])
            ax.scatter(*coords.T,label=route+' GRU samples',s=5,alpha=.4,color=color)
    ax.set(xlabel='Base X (m)',ylabel='Base Y (m)',zlabel='Base Z (m)'); ax.legend()
    fig.tight_layout(); fig.savefig(output/'pose_coverage_3d.png',dpi=150); plt.close(fig)


def report(output,prepared,manifest,coverage,results,selected,runtime,static_rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected_rows=[r for r in results if r['model']==selected]
    frozen=next(r for r in results if r['model']=='p0_frozen_train_plus_r3')
    old=next(r for r in results if r['model']=='deployed_20260918' and r['protocol']=='historical_r4')
    lines=['# P0 — Calibration F_robot hợp nhất, 19/09/2026','',
        'Trạng thái: phân tích offline hoàn tất; candidate chưa deploy và calibration chưa được xác nhận.', '',
        f"Manifest: {len(manifest)} event logs; {sum('hc_force_calibration' in r['path'] for r in manifest)} trong kho calibration.",
        f"Dữ liệu fit/kiểm chứng hiện tại: {coverage['force_trials']} trial, {coverage['force_samples']} mẫu lực, "
        f"{coverage['baseline_count']} đoạn baseline, trong đó {coverage['static_pose_count']} pose tĩnh riêng biệt.", '',
        '## Quyết định', '',
        'Giữ candidate runtime 18/09 ở shadow. Candidate P0 chưa cải thiện nhất quán; không bật calibrated hoặc role selection.', '',
        '| Đối chứng cùng reference | Runtime 18/09 | P0 frozen (train+r3) |', '|---|---:|---:|',
        f"| T2-r4 RMSE vector | {old['aggregate']['pooled_rmse_n']:.3f} N | {frozen['aggregate']['pooled_rmse_n']:.3f} N |",
        f"| 17:22 RMSE vector, suy ngược deadband | {runtime[0]['candidate_metrics']['deployed']['rmse_vector_n']:.3f} N | {runtime[0]['candidate_metrics']['selected']['rmse_vector_n']:.3f} N |", '',
        'RMSE T2-r4 khác audit 18/09 do P0 dùng timestamp trung bình M310–M315 thay vì timestamp M321, '
        'và áp quality mask/reference tái dựng thống nhất. Hai model trong bảng dùng cùng mẫu.', '',
        '## Observability', '',
        '| Tập baseline | Rank số học | Rank thực dụng (1%) | Condition chuẩn hóa |','|---|---:|---:|---:|']
    for name,key in [('12 pose tĩnh','static_poses'),('Hợp nhất, đã trừ trung bình từng session','all_baselines')]:
        s=coverage[key]
        lines.append(f"| {name} | {s['rank']} | {s.get('practical_rank_1pct')} | {s.get('condition')} |")
    stability=json.loads((output/'baseline_stability_and_direction_coverage.json').read_text())
    lines+=['','Rank số học đầy đủ không đảm bảo nhận dạng ổn định; xem singular values, hướng yếu và độ phủ theo session trong `coverage.json`.',
            '',f"Có {len(stability['high_scatter_baselines'])} baseline có độ lệch chuẩn một kênh torque >0.5 Nm. "
            'Giữ trong phân tích chính và báo thêm sensitivity riêng; không loại theo kết quả test.',
            'Độ phủ lực theo tuyến, lực ≥4 N và góc với trục ≤30°:', '',
            '| Tuyến/pose khai báo | X+ | X− | Y+ | Y− | Z+ | Z− |', '|---|---:|---:|---:|---:|---:|---:|']
    for route,r in stability['direction_coverage_by_route'].items():
        lines.append('| '+route+' | '+' | '.join(str(r['cone_counts'][a]) for a in ['X+','X-','Y+','Y-','Z+','Z-'])+' |')
    lines+=[ '', 'Đây là mẫu trên cả tuyến, không chứng minh lực đó đã được đo tại đúng pose đích. '
            'T2 thiếu các cone X±/Y− trong bộ M310 hiện tại; dữ liệu M320 tại T2 không thay thế được.',
            '', '## Candidate và kiểm chứng', '',f'Candidate chọn bằng r3: `{selected}`.',
            '`candidate_selection_stage.json` là model trước refit, có metric validation r3. '
            '`candidate_frozen.json` refit train+r3 sau chọn family/ridge, vẫn loại r4 và runtime khỏi fit. '
            'Mọi fold loại cả trial/session/ngày/tuyến khỏi fit; baseline đầu trial test chỉ dùng làm tare đầu vào. '
            'Baseline cuối trial test không được đưa vào fit. r3/r4 là kiểm chứng lịch sử đã xem trước đây.', '',
            '| Model | Protocol | Trial | RMSE trung bình theo trial (N) | RMSE gộp (N) |',
            '|---|---|---:|---:|---:|']
    for r in results:
        a=r['aggregate']
        lines.append(f"| {r['model']} | {r['protocol']} | {a['trials']} | {a.get('macro_rmse_n',0):.3f} | {a.get('pooled_rmse_n',0):.3f} |")
    lines+=['','RMSE trung bình theo trial tránh để một log dài lấn át các log ngắn. '
            '`results.json` lưu thêm sai số XYZ, góc trung vị/P95, độ phủ góc, tỷ lệ sai hướng, '
            'mẫu chuyển động, residual không tiếp xúc và đối chứng Axia chưa làm trơn. '
            'Các fold ridge=0.01 là cấu hình định trước. Fold dùng ridge chọn bằng r3 là kiểm tra thăm dò, '
            'không phải nested CV vì r3 đã ảnh hưởng lựa chọn hyperparameter.', '',
            '## Đối chứng runtime lịch sử', '']
    for r in runtime:
        lines.append(f"- `{Path(r['file']).name}`: deployed RMSE {r['source_time_active_metrics'].get('rmse_vector_n',0):.3f} N; "
                     f"candidate có thể replay: {r['raw_registers_available']}; "
                     f"candidate RMSE {r['candidate_metrics'].get('selected',{}).get('rmse_vector_n','không đủ raw')} N.")
    lines+=['','Các CSV runtime thiếu raw Axia: chỉ suy ngược radial deadband 4 N khi cả hai đầu nội suy khác 0, '
            'không giả lập raw cho 17:42. Đây là đánh giá có điều kiện, không phải lực raw đã xác minh.', '',
            '## Giới hạn và dữ liệu còn thiếu', '',
            '- Không có sidecar calibration mới trong kho ở thời điểm kiểm kê. Cần dùng logger đã bổ sung sidecar khi thu phần còn thiếu.',
            '- Session lấy từ metadata; tare/tool/config thực tế không được chứng minh chỉ bằng snapshot source. '
            'Log legacy 12/09 được kiểm kê nhưng tách khỏi fit hiện tại vì cấu hình cũ chưa xác nhận tương thích.',
            '- Không suy nhãn không tiếp xúc từ Axia sau deadband bằng 0 hoặc từ trạng thái không RUNNING.',
            '- Baseline tĩnh dùng raw M310 và nhãn no_contact; offset từng session được loại trước khi fit pose.',
            '- Reference Axia tái dựng với payload 1.126 kg, yaw -90°, TF ghi trong log và moment arm [0,0.120,0.0354] m. '
            'Không thay frame, payload hoặc Tool Data. Moment gravity/CoG và lực quán tính còn là giới hạn.',
            '- Sáu register đọc tuần tự. Nội suy về một timestamp phân tích không biến chúng thành phép đo đồng thời.',
            '- Baseline chuyển động chỉ được nhận nếu có marker no_contact_motion; các đoạn GRU/MJM chưa đủ bằng chứng không tiếp xúc.',
            '- Chưa có test mới độc lập sau khi đóng băng model. Không lấy RMSE training làm bằng chứng calibration hoàn chỉnh.', '',
            '## Thu bổ sung tối thiểu được dữ liệu chỉ ra', '',
            '1. Ưu tiên lực X± và Y− ở vùng T2: hiện không có mẫu M310 ≥4 N trong cone 30° tương ứng. '
            'Ghi raw M310, raw Axia và q đồng thời; giữ nguyên mức lực/giới hạn đã được vận hành.',
            '2. Nếu muốn nhận dạng baseline động: cần đoạn không tiếp xúc có marker trên tuyến đã được phép. '
            'GRU/MJM đang có không đủ nhãn để suy ra người đã nhả tay.',
            '3. Nếu muốn nhận dạng baseline đủ sáu chiều ngoài manifold hiện tại: chọn thêm pose kích thích '
            'hướng yếu trong coverage.json (tổ hợp J2/J3/J5), chấm lại singular values sau từng pose. '
            'Không đặt số pose cố định và không thu lại 12 pose cũ. Pose cụ thể cần kiểm tra reachability/clearance theo quy trình robot.',
            '4. Chỉ sau khi model được chọn và frozen mới thu một lượt test mới để đánh giá khả năng chuyển session; '
            'không refit bằng lượt test đó.', '',
            '## Tái lập', '', '```bash',
            f'python3 scripts/calibrate_hc_p0_unified.py --prepare-only --output <thu_muc_input_moi>',
            f'python3 scripts/calibrate_hc_p0_unified.py --prepared <thu_muc_input_moi> --output <thu_muc_ket_qua_moi>',
            '```', '',f'Input đã đóng băng: `{prepared}`.',
            '`candidate_frozen.json` giữ hệ số refit train+r3 sau chọn bằng validation; `candidate_refit_all_exploratory.json` là refit toàn dữ liệu, '
            'không được gán metric test của model frozen. Không thay model đang chạy.']
    (output/'report_vi.md').write_text('\n'.join(lines)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for key,label in [('static_poses','Static poses'),('all_baselines','All session-centered baselines')]:
        axes[0].semilogy(range(1,7),np.maximum(coverage[key]['singular_values'],1e-10),'o-',label=label)
    axes[0].set(xlabel='Singular component',ylabel='Singular value'); axes[0].legend()
    rows=[r for r in selected_rows if r['protocol'].startswith('leave_') or r['protocol'] in ['validation_r3','historical_r4']]
    axes[1].barh([r['protocol'].replace('leave_session_out:','session:') for r in rows],[r['aggregate']['macro_rmse_n'] for r in rows])
    axes[1].set_xlabel('Trial-macro force RMSE (N)')
    fig.tight_layout(); fig.savefig(output/'coverage_and_transfer.png',dpi=150); plt.close(fig)
    print(json.dumps(dict(selected=selected,coverage=coverage['all_baselines'],
                         selected_results=[dict(protocol=r['protocol'],**r['aggregate']) for r in selected_rows]),indent=2),flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--prepared',type=Path)
    ap.add_argument('--prepare-only',action='store_true')
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    if args.prepared:
        run_evaluation(args.prepared.resolve(),args.output.resolve())
    else:
        prepare(args.output.resolve())
        if not args.prepare_only:
            evaluation=args.output/'evaluation'; evaluation.mkdir()
            run_evaluation(args.output.resolve(),evaluation.resolve())


if __name__=='__main__':
    main()
