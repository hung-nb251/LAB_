#!/usr/bin/env python3
"""Offline sidecar audit and conservative local torque-gain development.

Never imports ROS, reads registers, or writes runtime configuration. Outputs
must be new. This is within-session development, not an independent test.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
from local_ik_solver import LocalIKSolver

PROTOCOL = dict(
    purpose='T1 local development only; independent new trial required',
    reference='raw Axia force with recorded bias, mass and effective rotation',
    time='M310 scan midpoint; recorded runtime q; no lag optimization',
    reference_max_gap_s=.05, reference_state_max_age_s=.05,
    joint_max_gap_s=.10, maximum_scan_span_ms=250.,
    fault_guard_s=1., split_fractions=[.6, .8], split_guard_s=1.,
    candidate_families=['diagonal', 'full'], ridge_alphas=[.01, .1, 1., 10.],
    prior='identity correction of existing calibrated torque; no new baseline fit',
    selection='minimum validation vector RMSE, including unchanged runtime',
    angle_threshold_n=4., cone_degrees=30.,
    post_baseline='diagnostic only; never used to fit or tare earlier samples',
    inferred_no_contact=('operator-confirmed file-head/file-tail rule; additionally require '
                         'small pre-deadband Axia force and stationary joints'),
)


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(n=0)
    return dict(n=int(len(a)), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def metrics(pred, ref):
    if not len(ref):
        return dict(n=0)
    pn = np.linalg.norm(pred, axis=1)
    rn = np.linalg.norm(ref, axis=1)
    eligible = rn >= 4
    angle_good = eligible & (pn >= 4)
    dot = np.sum(pred*ref, axis=1)
    angles = np.degrees(np.arccos(np.clip(
        dot[angle_good]/(pn[angle_good]*rn[angle_good]), -1, 1)))
    return dict(n=len(ref), rmse_vector_n=float(np.sqrt(np.mean(np.sum((pred-ref)**2, axis=1)))),
                rmse_axes_n=np.sqrt(np.mean((pred-ref)**2, axis=0)).tolist(),
                bias_axes_n=np.mean(pred-ref, axis=0).tolist(),
                reference_norm_n=stats(rn), reference_ge4=int(eligible.sum()),
                angle_deg=stats(angles), angle_n=int(angle_good.sum()),
                angle_coverage=float(angle_good.sum()/eligible.sum()) if eligible.any() else None,
                opposite_fraction=float(np.mean(dot[eligible] <= 0)) if eligible.any() else None,
                weak_or_opposite_fraction=float(np.mean((pn[eligible]<4)|(dot[eligible]<=0)))
                if eligible.any() else None)


def interpolate(t, source_t, values, max_gap):
    """Exact hits allowed; otherwise require brackets, finite data and bounded gap."""
    t = np.asarray(t, float)
    source_t = np.asarray(source_t, float)
    values = np.asarray(values, float)
    width = values.shape[1] if values.ndim == 2 else 1
    out = np.full((len(t), width), np.nan)
    good = np.zeros(len(t), bool)
    if not len(source_t):
        return out, good
    order = np.argsort(source_t, kind='stable')
    st = source_t[order]
    sv = values.reshape(len(source_t), width)[order]
    st, first = np.unique(st, return_index=True)
    sv = sv[first]
    right = np.searchsorted(st, t)
    exact = (right < len(st)) & (st[np.minimum(right, len(st)-1)] == t)
    out[exact] = sv[right[exact]]
    good[exact] = True
    bracket = ~exact & (right > 0) & (right < len(st))
    ids = np.flatnonzero(bracket)
    r = right[ids]
    ids = ids[(st[r]-st[r-1]) <= max_gap]
    r = right[ids]
    weight = ((t[ids]-st[r-1])/(st[r]-st[r-1]))[:, None]
    out[ids] = sv[r-1]*(1-weight)+sv[r]*weight
    good[ids] = True
    return out, good & np.isfinite(out).all(axis=1)


def read_events(paths):
    events = []
    seen = set()
    inventory = []
    for path in paths:
        counts = Counter()
        duplicates = 0
        first = last = None
        for lineno, line in enumerate(path.open(), 1):
            try:
                e = json.loads(line)
            except ValueError as exc:
                raise ValueError(f'{path}:{lineno}: invalid JSON') from exc
            first = e['receipt_ns'] if first is None else first
            last = e['receipt_ns']
            counts[e['kind']] += 1
            key = (e['kind'], e['receipt_ns'], json.dumps(e['data'], sort_keys=True))
            if key in seen:
                duplicates += 1
                continue
            seen.add(key)
            events.append(e)
        inventory.append(dict(path=str(path), sha256=digest(path), bytes=path.stat().st_size,
                              counts=dict(counts), duplicate_preroll_events=duplicates,
                              first_receipt_ns=first, last_receipt_ns=last))
    events.sort(key=lambda e: e['receipt_ns'])
    return events, inventory


def raw_force(raw, rotations, bias, mass):
    gravity = np.column_stack((np.zeros(len(mass)), np.zeros(len(mass)), -9.80665*mass))
    return np.einsum('nji,nj->ni', rotations, raw-bias)-gravity


def recovery(j, damping):
    return np.linalg.solve(j@np.swapaxes(j, 1, 2)+damping**2*np.eye(6), j)[:, :3, :]


def predict(gain, tau, op):
    return np.einsum('nij,nj->ni', op, tau@gain)


def design(tau, op, family):
    # tau_out[k] = sum_j tau_in[j] G[j,k].
    if family == 'diagonal':
        return op*tau[:, None, :]
    if family == 'full':
        return np.einsum('nj,nik->nijk', tau, op).reshape(len(tau), 3, 36)
    raise ValueError(family)


def fit_gain(tau, op, target, family, alpha):
    a = design(tau, op, family).reshape(-1, 6 if family=='diagonal' else 36)
    residual = (target-predict(np.eye(6), tau, op)).reshape(-1)
    scale = np.maximum(np.sqrt(np.mean(a*a, axis=0)), .05)
    z = a/scale
    delta = np.linalg.solve(z.T@z/len(z)+alpha*np.eye(z.shape[1]), z.T@residual/len(z))/scale
    gain = np.eye(6)+(np.diag(delta) if family=='diagonal' else delta.reshape(6, 6))
    return gain


def splits(t, start, stop, guard=1.):
    b1 = start+.6*(stop-start)
    b2 = start+.8*(stop-start)
    masks = dict(train=(t>=start)&(t<b1-guard),
                 validation=(t>=b1+guard)&(t<b2-guard),
                 internal_holdout=(t>=b2+guard)&(t<=stop))
    assert not np.any(masks['train'] & masks['validation'])
    assert not np.any(masks['internal_holdout'] & (masks['train']|masks['validation']))
    return masks, [start, b1, b2, stop]


def coverage(force):
    norm = np.linalg.norm(force, axis=1)
    return {axis+sign: int(np.sum((norm>=4)&(sgn*force[:,k]>=np.cos(np.pi/6)*norm)))
            for k,axis in enumerate('XYZ') for sign,sgn in [('+',1),('-',-1)]}


def rank_info(values):
    centered = values-values.mean(axis=0)
    scale = np.maximum(np.sqrt(np.mean(centered**2, axis=0)), .05)
    s = np.linalg.svd(centered/scale, compute_uv=False)
    return dict(n=len(values), numerical_rank=int(np.linalg.matrix_rank(centered)),
                practical_rank_1pct=int(np.sum(s>s[0]*.01)), singular_values=s.tolist(),
                condition=float(s[0]/s[-1]) if s[-1]>1e-12 else None,
                range=np.ptp(values, axis=0).tolist())


def analyze(paths, output, csv_path=None):
    output.mkdir(parents=True, exist_ok=False)
    save(output/'protocol.json', PROTOCOL)
    events, inventory = read_events(paths)
    save(output/'input_manifest.json', inventory)
    origin = min(e['receipt_ns'] for e in events)
    time_of = lambda ns: (int(ns)-origin)/1e9
    groups = {k:[e for e in events if e['kind']==k] for k in {e['kind'] for e in events}}
    proc = sorted(groups['axia_processed'], key=lambda e:e['data']['stamp_ns'])
    pt = np.array([time_of(e['data']['stamp_ns']) for e in proc])
    raw = sorted(groups['axia_raw'], key=lambda e:e['data']['stamp_ns'])
    rt = np.array([time_of(e['data']['stamp_ns']) for e in raw])
    rv = np.array([e['data']['wrench'] for e in raw])
    r = np.array([e['data']['rotation_base_to_sensor'] for e in proc])
    bias = np.array([e['data']['bias_sensor_n'] for e in proc])
    mass = np.array([e['data']['mass_kg'] for e in proc])
    healthy = np.array([e['data']['calibrated'] and e['data']['tf_valid'] for e in proc])
    raw_at_proc, raw_good = interpolate(pt, rt, rv[:, :3], .05)
    fraw = raw_force(raw_at_proc, r, bias, mass)
    filtered = np.array([e['data']['filtered_sensor_force'] for e in proc])
    fprocessed = np.array([e['data']['force_pre_deadband'] for e in proc])
    reproduction = raw_force(filtered, r, bias, mass)
    health_ref = healthy & raw_good
    # Invalid reference samples must remain in interpolation and invalidate both brackets.
    fraw[~health_ref] = np.nan
    fprocessed[~healthy] = np.nan
    jrows = groups['joints']
    jt = np.array([time_of(e['data']['stamp_ns']) for e in jrows])
    jq = np.array([e['data']['position'] for e in jrows])
    rows = groups['m310_sample']
    stamp_ids = set()
    unique = []
    for e in rows:
        if e['data']['stamp_ns'] not in stamp_ids:
            stamp_ids.add(e['data']['stamp_ns']); unique.append(e)
    rows = sorted(unique, key=lambda e:e['data']['stamp_ns'])
    mt = np.array([time_of(e['data']['stamp_ns']) for e in rows])
    md = [e['data'] for e in rows]
    # Baseline-only sidecars can contain M310 rows without acquisition timing.
    # They remain useful for offset diagnostics but are never valid force pairs.
    spans = np.array([float(d['register_scan_span_ms'])
                      if d.get('register_scan_span_ms') is not None else np.inf for d in md])
    states = []
    for e in groups['controller_status']:
        d = json.loads(e['data'])
        key = [d.get('controller_state'), d.get('reason'), d.get('control_phase')]
        if not states or states[-1]['state'] != key:
            states.append(dict(t=time_of(e['receipt_ns']), stamp_ns=d['stamp_ns'], state=key))
    run_start = next(s['t'] for s in states if s['state'][0]=='RUNNING')
    terminal_states = [s for s in states if s['t']>run_start and
                       s['state'][0] in ('STOPPED','FAULT')]
    logger_stops = [time_of(e['receipt_ns']) for e in groups.get('logger_stop', [])
                    if time_of(e['receipt_ns']) > run_start]
    stop_candidates = ([(s['t'], s['state']) for s in terminal_states] +
                       [(t, ['LOGGER_STOP_BOUNDARY',
                             'no terminal controller event in main sidecar', None])
                        for t in logger_stops])
    if stop_candidates:
        # Supplemental post-baseline files may begin in STOPPED after the main
        # logger has already closed. The earliest boundary belongs to the run.
        run_stop, stop_reason = min(stop_candidates, key=lambda item: item[0])
    else:
        raise ValueError('No closed RUNNING boundary; stop the logger before analysis')
    cutoff = run_stop-PROTOCOL['fault_guard_s']
    active = np.array([d['status'].startswith('SHADOW_VALID:') and d['bias_nm'] is not None
                       and d['position'] is not None and d['force_unfiltered'] is not None for d in md])
    ref, ref_good = interpolate(mt, pt, fraw, .05)
    processed_ref, processed_good = interpolate(mt, pt, fprocessed, .05)
    qmid, qgood = interpolate(mt, jt, jq, .10)
    within_run = (mt-spans/2000 >= run_start) & (mt+spans/2000 <= cutoff)
    valid = active & within_run & (spans<=250) & ref_good & processed_good & qgood
    ids = np.flatnonzero(valid)
    if len(ids)<50:
        raise ValueError('Too few valid source scans for development')
    q = np.array([md[i]['position'] for i in ids])
    q0 = np.array([md[i]['baseline_q0_rad'] for i in ids])
    tau_raw = np.array([md[i]['mregister_values_nm'] for i in ids])
    tau0 = np.array([md[i]['bias_nm'] for i in ids])
    dq = q-q0
    models = [e['data'] for e in groups['model_snapshot']]
    if any(m != models[0] for m in models):
        raise ValueError('Model changed between files; split the session')
    model = models[0]
    tau = (np.column_stack((tau_raw-tau0, dq))/model['scale'])@model['coef']
    solver = LocalIKSolver()
    jac = np.array([solver.compute_jacobian(v) for v in q])
    op = recovery(jac, model['damping'])
    runtime = predict(np.eye(6), tau, op)
    recorded = np.array([md[i]['force_unfiltered'] for i in ids])
    time = mt[ids]
    target = ref[ids]
    masks, boundaries = splits(time, run_start, cutoff)
    if any(mask.sum()<20 for mask in masks.values()):
        raise ValueError('Temporal development blocks are too short')
    candidates = [dict(name='runtime', family=None, alpha=None, gain=np.eye(6),
                       validation=metrics(runtime[masks['validation']], target[masks['validation']]))]
    for family in PROTOCOL['candidate_families']:
        for alpha in PROTOCOL['ridge_alphas']:
            train = masks['train']
            gain = fit_gain(tau[train], op[train], target[train], family, alpha)
            pred = predict(gain, tau, op)
            candidates.append(dict(name=f'{family}_{alpha}', family=family, alpha=alpha,
                                   gain=gain, validation=metrics(pred[masks['validation']], target[masks['validation']])))
    selected = min(candidates, key=lambda c:c['validation']['rmse_vector_n'])
    selection_gain = selected['gain']
    development = masks['train']|masks['validation']
    frozen_gain = (fit_gain(tau[development],op[development],target[development],
                           selected['family'],selected['alpha']) if selected['family'] else np.eye(6))
    frozen = dict(model)
    frozen.update(coef=(np.asarray(model['coef'])@frozen_gain).tolist(),
                  status='OFFLINE_T1_DEVELOPMENT_CANDIDATE_PENDING_INDEPENDENT_TEST',
                  calibration_confirmed=False, role_valid=False,
                  source='audit_hc_t1_sidecar.py', family=selected['family'], alpha=selected['alpha'],
                  correction_gain=frozen_gain.tolist(), independent_test=None,
                  training_source=[str(p) for p in paths],
                  training_window_s=[run_start, boundaries[2]-1],
                  excluded_from_fit='final 20% temporal block, split guards, fault tail, post baseline',
                  target='force only; moments and active robot intention NOT calibrated')
    save(output/'candidate_frozen.json', frozen)
    save(output/'runtime_snapshot.json', model)
    frozen_pred = predict(frozen_gain, tau, op)
    selections = [{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in c.items()}
                  for c in candidates]
    save(output/'selection_stage.json', selections)
    comparisons = {}
    for name, pred in [('runtime',runtime),('selected_train_only',predict(selection_gain,tau,op)),
                       ('frozen_train_validation',frozen_pred)]:
        comparisons[name] = {key:metrics(pred[mask],target[mask]) for key,mask in masks.items()}
        comparisons[name]['all_development_descriptive'] = metrics(pred,target)
        comparisons[name]['processed_reference_sensitivity'] = metrics(pred,processed_ref[ids])
    markers = [dict(t=time_of(e['receipt_ns']), text=e['data']) for e in groups.get('marker',[])]
    final_event_t = max(time_of(e['receipt_ns']) for e in events)
    baselines = []
    for marker in markers:
        is_post_baseline = marker['text'].startswith('post_baseline_for:')
        if not (marker['text'].startswith('no_contact_begin:') or is_post_baseline):
            continue
        end_label = marker['text'].replace('no_contact_begin:', 'no_contact_end:', 1)
        end = (None if is_post_baseline else
               next((m['t'] for m in markers if m['text']==end_label and m['t']>marker['t']), None))
        boundary_source = 'explicit_no_contact_end_marker'
        if end is None and marker['text'].endswith(':start'):
            end = run_start
            boundary_source = 'operator_rule_file_head_to_run_start'
        elif end is None and marker['text'].endswith(':end'):
            later_stops=[time_of(e['receipt_ns']) for e in groups.get('logger_stop',[])
                         if time_of(e['receipt_ns'])>marker['t']]
            end = later_stops[0] if later_stops else None
            boundary_source = 'operator_rule_marker_to_logger_stop'
        elif end is None and is_post_baseline:
            end = final_event_t
            boundary_source = 'operator_rule_marker_to_file_tail'
        if end is None:
            baselines.append(dict(marker=marker['text'], accepted=False, reason='missing_end_marker')); continue
        lo,hi = marker['t']+1,end-1
        jm = (jt>=lo)&(jt<=hi)
        pm = (pt>=lo)&(pt<=hi)&health_ref
        mm = (mt-spans/2000>=lo)&(mt+spans/2000<=hi)
        item = dict(marker=marker['text'], boundary_source=boundary_source,
                    inferred_no_contact=boundary_source != 'explicit_no_contact_end_marker',
                    window_s=[lo,hi], joints=int(jm.sum()),
                    axia=int(pm.sum()), m310=int(mm.sum()),
                    axia_residual_n=stats(np.linalg.norm(fraw[pm],axis=1)),
                    q_median=np.median(jq[jm],axis=0).tolist(),
                    q_range=np.ptp(jq[jm],axis=0).tolist())
        item['accepted_axia_static'] = bool(pm.sum()>=100 and jm.sum()>=20 and
                                            np.max(np.ptp(jq[jm],axis=0))<.02)
        item['accepted_torque_baseline'] = bool(item['accepted_axia_static'] and mm.sum()>=12)
        if mm.any():
            vals = np.array([md[i]['mregister_values_nm'] for i in np.flatnonzero(mm)])
            item.update(tau_median_nm=np.median(vals,axis=0).tolist(),
                        tau_std_nm=np.std(vals,axis=0).tolist(),
                        frozen_baseline_minus_marker_median_nm=(tau0[0]-np.median(vals,axis=0)).tolist())
        baselines.append(item)
    # Baseline windows constrain offset only at their measured pose; no b(q) fit.
    initial = next(b for b in baselines if b['marker'].endswith(':start'))
    bq = np.array(initial['q_median'])
    bdt = np.array(initial['tau_median_nm'])-tau0[0]
    bx = np.r_[bdt,bq-q0[0]]/model['scale']
    bop = recovery(np.array([solver.compute_jacobian(bq)]),model['damping'])
    initial['runtime_force_median_n'] = predict(np.eye(6),(bx@model['coef'])[None],bop)[0].tolist()
    initial['candidate_force_median_n'] = predict(frozen_gain,(bx@model['coef'])[None],bop)[0].tolist()
    qmid_op = recovery(np.array([solver.compute_jacobian(v) for v in qmid[ids]]),model['damping'])
    qmid_tau = (np.column_stack((tau_raw-tau0,qmid[ids]-q0))/model['scale'])@model['coef']
    xyz = np.array([solver.forward_kinematics(v)[:3,3] for v in q])
    sv = np.linalg.svd(jac,compute_uv=False)
    report = dict(origin_ns=origin, protocol=PROTOCOL, controller_transitions=states, markers=markers,
                  runtime_replay_max_abs_n=float(np.max(np.abs(runtime-recorded))),
                  axia_processed_replay_max_abs_n=float(np.nanmax(np.abs(reproduction-fprocessed))),
                  source_scans=len(rows), runtime_force_scans=int(active.sum()), paired_scans=len(ids),
                  quality_exclusions=dict(outside_run_or_fault_guard=int((active&~within_run).sum()),
                    excessive_span=int((active&(spans>250)).sum()),
                    reference_gap_or_invalid=int((active&~ref_good).sum()),
                    joint_gap=int((active&~qgood).sum())),
                  run_duration_s=run_stop-run_start, analysis_cutoff_s=cutoff,
                  run_stop_reason=stop_reason,
                  run_stop_local=datetime.fromtimestamp((origin+int(run_stop*1e9))/1e9).isoformat(),
                  m310_interval_s=stats(np.diff(mt)), m310_scan_span_ms=stats(spans),
                  complete_vector_rate_hz=float((len(mt)-1)/(mt[-1]-mt[0])),
                  last_m310_stamp_ns=rows[-1]['data']['stamp_ns'],
                  baseline=baselines, split_boundaries_s=boundaries,
                  split_counts={k:int(v.sum()) for k,v in masks.items()},
                  coverage={k:coverage(target[v]) for k,v in masks.items()},
                  coverage_all=coverage(target), selected=selected['name'], comparisons=comparisons,
                  runtime_qmid_sensitivity=metrics(predict(np.eye(6),qmid_tau,qmid_op),target),
                  joint_timestamp_pose_delta_rad=stats(np.linalg.norm(q-qmid[ids],axis=1)),
                  xyz_min_m=xyz.min(axis=0).tolist(), xyz_max_m=xyz.max(axis=0).tolist(),
                  pose_excitation=rank_info(q), force_excitation=rank_info(target),
                  jacobian_condition=stats(sv[:,0]/sv[:,-1]),
                  axia_settings=dict(filter_names=sorted({e['data']['filter_name'] for e in proc}),
                    masses=sorted(set(mass)), deadbands=sorted({e['data']['deadband_n'] for e in proc}),
                    bias_range=np.ptp(bias[healthy],axis=0).tolist(),
                    invalid_health=int((~healthy).sum())),
                  decision='NOT_DEPLOYED; development evidence only; independent test and reader recovery required')
    if csv_path:
        with csv_path.open() as stream:
            csv_rows=list(csv.DictReader(stream))
        diag=[json.loads(d['motion_diagnostics_json']) for d in csv_rows if d['motion_diagnostics_json']]
        report['csv_audit']=dict(rows=len(csv_rows),sha256=digest(csv_path),
            tracking_error_m=stats([d.get('tracking_error_m',np.nan) for d in diag]),
            last_counters={k:diag[-1].get(k) for k in ['queue_busy_total','queue_retry_total','queue_reject_total','ik_consecutive_failures']},
            last_diagnostics=diag[-1],
            force_age_ms=stats([float(d['force_age_ms']) for d in csv_rows if d['force_age_ms']]),
            udp_gap_ms=stats([float(d['udp_gap_ms']) for d in csv_rows if d['udp_gap_ms']]))
    save(output/'audit.json',report)
    np.savez_compressed(output/'paired_samples.npz',t=time,ref=target,processed_ref=processed_ref[ids],
                        q=q,q0=q0,tau_raw=tau_raw,tau0=tau0,tau=tau,op=op,j=jac,
                        runtime=runtime,candidate=frozen_pred,**masks)
    with (output/'paired_samples.csv').open('x') as stream:
        writer=csv.writer(stream)
        writer.writerow(['source_t_s','split',*[f'{name}_{a}' for name in ['axia_raw','runtime','candidate'] for a in 'xyz']])
        for i,tm in enumerate(time):
            split=next((k for k,v in masks.items() if v[i]),'guard_excluded')
            writer.writerow([tm,split,*target[i],*runtime[i],*frozen_pred[i]])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True)
    for k,ax in enumerate(axes):
        ax.plot(time-run_start,target[:,k],label='Axia raw compensated',color='black',lw=.8)
        ax.plot(time-run_start,runtime[:,k],label='Runtime 18/09',alpha=.8,lw=.8)
        ax.plot(time-run_start,frozen_pred[:,k],label='Candidate (development fit)',alpha=.8,lw=.8)
        for boundary in boundaries[1:3]: ax.axvline(boundary-run_start,color='gray',ls='--')
        ax.set_ylabel('F'+ 'xyz'[k]+' [N]');ax.grid(alpha=.2)
    axes[0].legend(loc='upper left',fontsize=8)
    axes[-1].set_xlabel('Seconds after RUNNING; dashed boundaries: 60% / 80% (internal only)')
    fig.tight_layout();fig.savefig(output/'force_comparison.png',dpi=160);plt.close(fig)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main',type=Path,required=True)
    parser.add_argument('--post',type=Path,
                        help='Optional closed post-baseline sidecar from the same trial')
    parser.add_argument('--csv',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    paths=[args.main]+([args.post] if args.post else [])
    report=analyze(paths,args.output,args.csv)
    summary={k:report[k] for k in ['paired_scans','selected','coverage_all','split_counts']}
    summary['internal_holdout_rmse_n']={
        k:v['internal_holdout']['rmse_vector_n'] for k,v in report['comparisons'].items()}
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
