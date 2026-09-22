#!/usr/bin/env python3
"""Score a frozen F_robot candidate against the runtime model on a test session.

Evaluation only. Nothing here fits, selects or refits: the gain is read from the
frozen candidate file and both models are scored on the identical masked sample
set. Preprocessing follows audit_hc_t1_sidecar.PROTOCOL.

    python3 scripts/score_hc_t1_independent_test.py --candidate <candidate_frozen.json> \
        --plan <plan.json> --output <new_dir> <main.jsonl> [<post.jsonl> ...]
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
import audit_hc_t1_sidecar as audit
from local_ik_solver import LocalIKSolver

# Fixed in the development report before this session existed.
DEVELOPED_EE_BOX = {'x': (-0.228, 0.407), 'y': (0.707, 1.110), 'z': (0.432, 0.717)}


def pair_samples(paths):
    """Same pairing as audit_hc_t1_sidecar.analyze, without any fitting."""
    events, inventory = audit.read_events(paths)
    origin = min(e['receipt_ns'] for e in events)
    time_of = lambda ns: (int(ns) - origin) / 1e9
    groups = {k: [e for e in events if e['kind'] == k] for k in {e['kind'] for e in events}}

    proc = sorted(groups['axia_processed'], key=lambda e: e['data']['stamp_ns'])
    pt = np.array([time_of(e['data']['stamp_ns']) for e in proc])
    raw = sorted(groups['axia_raw'], key=lambda e: e['data']['stamp_ns'])
    rt = np.array([time_of(e['data']['stamp_ns']) for e in raw])
    rv = np.array([e['data']['wrench'] for e in raw])
    r = np.array([e['data']['rotation_base_to_sensor'] for e in proc])
    bias = np.array([e['data']['bias_sensor_n'] for e in proc])
    mass = np.array([e['data']['mass_kg'] for e in proc])
    healthy = np.array([e['data']['calibrated'] and e['data']['tf_valid'] for e in proc])
    raw_at_proc, raw_good = audit.interpolate(pt, rt, rv[:, :3], .05)
    fraw = audit.raw_force(raw_at_proc, r, bias, mass)
    fprocessed = np.array([e['data']['force_pre_deadband'] for e in proc])
    fraw[~(healthy & raw_good)] = np.nan
    fprocessed[~healthy] = np.nan

    jrows = groups['joints']
    jt = np.array([time_of(e['data']['stamp_ns']) for e in jrows])
    jq = np.array([e['data']['position'] for e in jrows])

    stamp_ids, unique = set(), []
    for e in groups['m310_sample']:
        if e['data']['stamp_ns'] not in stamp_ids:
            stamp_ids.add(e['data']['stamp_ns'])
            unique.append(e)
    rows = sorted(unique, key=lambda e: e['data']['stamp_ns'])
    mt = np.array([time_of(e['data']['stamp_ns']) for e in rows])
    md = [e['data'] for e in rows]
    spans = np.array([float(d['register_scan_span_ms'])
                      if d.get('register_scan_span_ms') is not None else np.inf
                      for d in md])

    states = []
    for e in groups['controller_status']:
        d = json.loads(e['data'])
        key = [d.get('controller_state'), d.get('reason'), d.get('control_phase')]
        if not states or states[-1]['state'] != key:
            states.append(dict(t=time_of(e['receipt_ns']), state=key))
    run_start = next(s['t'] for s in states if s['state'][0] == 'RUNNING')
    terminal = [(s['t'], s['state']) for s in states
                if s['t'] > run_start and s['state'][0] in ('STOPPED', 'FAULT')]
    stops = [(time_of(e['receipt_ns']), ['LOGGER_STOP_BOUNDARY', None, None])
             for e in groups.get('logger_stop', [])
             if time_of(e['receipt_ns']) > run_start]
    run_stop, stop_reason = min(terminal + stops, key=lambda item: item[0])
    cutoff = run_stop - audit.PROTOCOL['fault_guard_s']

    active = np.array([d['status'].startswith('SHADOW_VALID:') and d['bias_nm'] is not None
                       and d['position'] is not None and d['force_unfiltered'] is not None
                       for d in md])
    ref, ref_good = audit.interpolate(mt, pt, fraw, .05)
    processed_ref, processed_good = audit.interpolate(mt, pt, fprocessed, .05)
    _, qgood = audit.interpolate(mt, jt, jq, .10)
    within = (mt - spans / 2000 >= run_start) & (mt + spans / 2000 <= cutoff)
    valid = active & within & (spans <= 250) & ref_good & processed_good & qgood
    ids = np.flatnonzero(valid)
    if len(ids) < 20:
        raise ValueError(f'only {len(ids)} usable scans')

    q = np.array([md[i]['position'] for i in ids])
    q0 = np.array([md[i]['baseline_q0_rad'] for i in ids])
    tau_raw = np.array([md[i]['mregister_values_nm'] for i in ids])
    tau0 = np.array([md[i]['bias_nm'] for i in ids])
    models = [e['data'] for e in groups['model_snapshot']]
    if any(m != models[0] for m in models):
        raise ValueError('model snapshot changed between files; split the session')
    model = models[0]
    tau = (np.column_stack((tau_raw - tau0, q - q0)) / model['scale']) @ model['coef']
    solver = LocalIKSolver()
    op = audit.recovery(np.array([solver.compute_jacobian(v) for v in q]), model['damping'])
    ee = np.array([solver.fk_position(v) for v in q])
    return dict(inventory=inventory, ids=ids, tau=tau, op=op, ee=ee, q=q,
                target=ref[ids], time=mt[ids], span_ms=spans[ids],
                recorded=np.array([md[i]['force_unfiltered'] for i in ids]),
                md=md, run_start=run_start, run_stop=run_stop,
                stop_reason=stop_reason, cutoff=cutoff, model=model,
                total_m310=len(md))


def in_box(ee):
    b = DEVELOPED_EE_BOX
    return ((ee[:, 0] >= b['x'][0]) & (ee[:, 0] <= b['x'][1]) &
            (ee[:, 1] >= b['y'][0]) & (ee[:, 1] <= b['y'][1]) &
            (ee[:, 2] >= b['z'][0]) & (ee[:, 2] <= b['z'][1]))


def acquisition_health(md):
    errs = {'service_error': 0, 'not_success': 0,
            'request_timeout': 0, 'abandoned_request': 0}
    seen = False
    for d in md:
        a = d.get('acquisition')
        if a and a.get('errors_cumulative'):
            seen = True
            for k, v in a['errors_cumulative'].items():
                errs[k] = max(errs.get(k, 0), int(v))
    return {'instrumented': seen, 'errors_cumulative': errs}


def score(pack, gain, mask):
    return audit.metrics(audit.predict(gain, pack['tau'][mask], pack['op'][mask]),
                         pack['target'][mask])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+', type=Path)
    ap.add_argument('--candidate', type=Path, required=True)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')

    plan = json.loads(args.plan.read_text())
    cand = json.loads(args.candidate.read_text())
    gain = np.asarray(cand['correction_gain'], dtype=float)
    frozen_sha = audit.digest(args.candidate)
    expected = plan['frozen_under_test']['candidate_sha256']
    if frozen_sha != expected:
        raise SystemExit(f'candidate hash {frozen_sha} != pre-registered {expected}')

    pack = pair_samples(args.files)

    # Self-check: applying the runtime model here must reproduce the force the
    # node actually published, otherwise the pairing or model use is wrong.
    runtime_pred = audit.predict(np.eye(6), pack['tau'], pack['op'])
    replay_error = float(np.nanmax(np.abs(runtime_pred - pack['recorded'])))

    ee = pack['ee']
    inside = in_box(ee)
    segments = {'all': np.ones(len(inside), bool),
                'in_developed_box': inside,
                'outside_developed_box': ~inside}

    results = {}
    for name, mask in segments.items():
        if mask.sum() < 20:
            results[name] = {'n': int(mask.sum()), 'skipped': 'fewer than 20 samples'}
            continue
        rt = score(pack, np.eye(6), mask)
        cd = score(pack, gain, mask)
        rel = (rt['rmse_vector_n'] - cd['rmse_vector_n']) / rt['rmse_vector_n']
        results[name] = {'n': int(mask.sum()), 'runtime': rt, 'candidate': cd,
                         'relative_improvement': float(rel)}

    cov = audit.coverage(pack['target'])
    speed = np.linalg.norm(np.diff(ee, axis=0), axis=1) / np.maximum(np.diff(pack['time']), 1e-9)

    health = acquisition_health(pack['md'])
    guard = {
        'reader_no_timeout_or_abandon': (
            health['errors_cumulative']['request_timeout'] == 0 and
            health['errors_cumulative']['abandoned_request'] == 0),
        'paired_scans_at_least_300': int(len(pack['ids'])) >= 300,
        'axis_cones_with_ge4N_at_least_4': sum(1 for v in cov.values() if v > 0) >= 4,
        'stop_boundary': pack['stop_reason'],
        'replay_matches_published_force': replay_error < 1e-6,
    }

    a = results.get('in_developed_box', {})
    if 'runtime' in a:
        primary = {
            'candidate_beats_runtime': a['candidate']['rmse_vector_n'] < a['runtime']['rmse_vector_n'],
            'relative_improvement_ge_10pct': a['relative_improvement'] >= 0.10,
            'candidate_rmse_le_2_60_N': a['candidate']['rmse_vector_n'] <= 2.60,
        }
        primary['PASS'] = all(primary.values())
    else:
        primary = {'PASS': None, 'reason': 'no in-box samples'}

    no_harm = {name: (r['relative_improvement'] >= -0.10)
               for name, r in results.items() if 'runtime' in r}
    no_harm['PASS'] = all(no_harm.values())

    report = dict(
        plan=str(args.plan), plan_sha256=audit.digest(args.plan),
        candidate=str(args.candidate), candidate_sha256=frozen_sha,
        inputs=pack['inventory'], paired_scans=int(len(pack['ids'])),
        total_m310_rows=pack['total_m310'],
        runtime_replay_max_abs_error_n=replay_error,
        run_window_s=[pack['run_start'], pack['cutoff']],
        coverage_ge4N_cone30=cov,
        ee_speed_m_s={'median': float(np.median(speed)), 'p95': float(np.percentile(speed, 95))},
        scan_span_ms={'median': float(np.median(pack['span_ms'])),
                      'p95': float(np.percentile(pack['span_ms'], 95))},
        acquisition=health,
        developed_ee_box=DEVELOPED_EE_BOX,
        segments=results,
        validity_guardrails=guard,
        primary_in_scope=primary,
        do_no_harm=no_harm,
        note=('Evaluation only. No fitting, selection or threshold choice was '
              'performed on this session.'))
    args.output.mkdir(parents=True)
    audit.save(args.output / 'test_report.json', report)
    for p in (Path(__file__), args.candidate, args.plan):
        (args.output / p.name).write_text(p.read_text())

    print(json.dumps(dict(
        paired_scans=report['paired_scans'],
        replay_check_n=replay_error,
        coverage=cov,
        ee_speed=report['ee_speed_m_s'],
        segments={k: (v if 'runtime' not in v else dict(
            n=v['n'],
            runtime_rmse=round(v['runtime']['rmse_vector_n'], 3),
            candidate_rmse=round(v['candidate']['rmse_vector_n'], 3),
            improvement=round(v['relative_improvement'] * 100, 1),
            runtime_angle_p95=round(v['runtime']['angle_deg']['p95'], 1)
            if v['runtime'].get('angle_deg') else None,
            candidate_angle_p95=round(v['candidate']['angle_deg']['p95'], 1)
            if v['candidate'].get('angle_deg') else None))
            for k, v in results.items()},
        validity=guard, primary_in_scope=primary, do_no_harm=no_harm), indent=2))


if __name__ == '__main__':
    main()
