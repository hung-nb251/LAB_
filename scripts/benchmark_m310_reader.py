#!/usr/bin/env python3
"""Benchmark the M310-M315 register reader from calibration sidecar logs.

Read-only offline audit. It never opens a ROS client, so it can run while the
pipeline is live without adding a second reader on /read_mregister.

    python3 scripts/benchmark_m310_reader.py --output <new_dir> <file.calibration.jsonl> ...

Reports vector rate, jitter, per-register round trip, six-channel scan span,
the drift that span costs during motion, error/timeout counts and the ceilings
reachable without a controller-side batch API.
"""
import argparse
import json
from pathlib import Path

import numpy as np

TIMER_PERIOD_S = 0.005
N_REGISTERS = 6


def _stats(a, scale=1.0):
    a = np.asarray(a, dtype=float) * scale
    if not a.size:
        return None
    return {'n': int(a.size), 'median': float(np.median(a)),
            'p05': float(np.percentile(a, 5)), 'p95': float(np.percentile(a, 95)),
            'min': float(a.min()), 'max': float(a.max()),
            'mean': float(a.mean())}


def load(path):
    out = {'m310': [], 'run_status': [], 'marker': [], 'joints': [], 'other': 0}
    unparsable = 0
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            unparsable += 1
            continue
        kind = rec.get('kind')
        if kind == 'm310_sample':
            out['m310'].append(rec)
        elif kind in ('run_status', 'marker', 'joints'):
            out[kind].append(rec)
        else:
            out['other'] += 1
    out['unparsable'] = unparsable
    return out


def running_mask(recv, run_status):
    edges = [(r['receipt_ns'], bool(r['data'])) for r in run_status]
    mask = np.zeros(len(recv), dtype=bool)
    state = False
    j = 0
    for i, t in enumerate(recv):
        while j < len(edges) and edges[j][0] <= t:
            state = edges[j][1]
            j += 1
        mask[i] = state
    return mask


def audit(path, scan_gap_sec):
    recs = load(path)
    m = recs['m310']
    result = {'file': str(path), 'unparsable_lines': recs['unparsable'],
              'm310_samples': len(m)}
    if not m:
        result['verdict'] = ('NO m310_sample records: the reader published '
                             'nothing for the whole file')
        return result

    recv = np.array([r['receipt_ns'] for r in m], dtype=np.int64)
    stamp = np.array([r['data']['stamp_ns'] for r in m], dtype=np.int64)
    span = np.array([r['data'].get('register_scan_span_ms') or np.nan for r in m])
    status = [r['data']['status'] for r in m]
    vals = np.array([r['data']['mregister_values_nm'] for r in m], dtype=float)
    period = np.diff(recv) / 1e6
    duration_s = float((recv[-1] - recv[0]) / 1e9)

    result['duration_s'] = duration_s
    result['mean_vector_rate_hz'] = len(m) / duration_s if duration_s else None
    result['status_counts'] = {
        k: status.count(k) for k in sorted(set(s.split(':')[0] for s in status))
        for k in [k]}
    result['status_counts'] = dict(
        sorted(((s.split(':')[0], sum(1 for x in status if x.split(':')[0] == s.split(':')[0]))
                for s in set(status)), key=lambda kv: -kv[1]))
    result['invalid_detail'] = dict(
        sorted(((s, status.count(s)) for s in set(status) if s.startswith('INVALID')),
               key=lambda kv: -kv[1]))
    result['period_ms'] = _stats(period)
    result['scan_span_ms'] = _stats(span[np.isfinite(span)])
    result['per_register_round_trip_ms'] = _stats(span[np.isfinite(span)] / N_REGISTERS)
    result['stamp_to_receipt_ms'] = _stats((recv - stamp) / 1e6)
    result['gaps_over_1s'] = int((period > 1000).sum())

    # Does the acquisition block from the instrumented reader exist yet?
    acq = [r['data'].get('acquisition') for r in m if r['data'].get('acquisition')]
    result['has_acquisition_block'] = bool(acq)
    if acq:
        rtt = [reg['rtt_ms'] for a in acq for reg in a.get('registers', [])]
        result['measured_rtt_ms'] = _stats(rtt)
        result['reader_backend'] = acq[-1].get('reader_backend')
        result['reader_version'] = acq[-1].get('reader_version')
        result['errors_cumulative'] = acq[-1].get('errors_cumulative')

    med_span = float(np.median(span[np.isfinite(span)]))
    med_period = float(np.median(period))
    saved = N_REGISTERS * TIMER_PERIOD_S * 1e3 / 2.0
    result['budget'] = {
        'scan_span_share': med_span / med_period,
        'scan_gap_share': scan_gap_sec * 1e3 / med_period,
        'ceiling_hz_as_configured': 1e3 / med_period,
        'ceiling_hz_zero_gap': 1e3 / med_span,
        'ceiling_hz_zero_gap_no_tick_waste': 1e3 / max(med_span - saved, 1e-6),
        'per_register_ms_required_for_15hz': (1e3 / 15) / N_REGISTERS,
        'per_register_ms_observed_floor': float(np.nanmin(span) / N_REGISTERS),
    }
    result['budget']['fifteen_hz_reachable_with_scalar_reads'] = bool(
        result['budget']['per_register_ms_observed_floor']
        <= result['budget']['per_register_ms_required_for_15hz'])

    run = running_mask(recv, recs['run_status'])
    if run.sum() > 10:
        rv, rt = vals[run], recv[run] / 1e9
        dt = np.diff(rt)
        rate = np.abs(np.diff(rv, axis=0)) / dt[:, None]
        span_s = float(np.median(span[run])) / 1e3
        result['non_simultaneity'] = {
            'running_samples': int(run.sum()),
            'scan_span_s': span_s,
            'per_channel_nm_per_s': {
                f'M{310 + j}': {'median': float(np.median(rate[:, j])),
                                'p95': float(np.percentile(rate[:, j], 95))}
                for j in range(N_REGISTERS)},
            'vector_skew_nm': {
                'median': float(np.median(np.linalg.norm(np.diff(rv, axis=0), axis=1) / dt)
                                * span_s),
                'p95': float(np.percentile(np.linalg.norm(np.diff(rv, axis=0), axis=1) / dt, 95)
                             * span_s)},
        }
        same = np.all(np.isclose(rv[1:], rv[:-1]), axis=1)
        result['non_simultaneity']['identical_consecutive_fraction'] = float(same.mean())
        result['non_simultaneity']['undersampling'] = bool(same.mean() < 0.05)

    if recs['joints']:
        jt = np.array([r['receipt_ns'] for r in recs['joints']], dtype=np.int64)
        idx = np.clip(np.searchsorted(jt, recv) - 1, 0, len(jt) - 1)
        result['joint_state_rate_hz'] = float(1e3 / np.median(np.diff(jt) / 1e6))
        result['joint_age_at_sample_ms'] = _stats((recv - jt[idx]) / 1e6)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--output', required=True,
                    help='new directory for results.json (must not exist)')
    ap.add_argument('--scan-gap-sec', type=float, default=0.05)
    args = ap.parse_args()

    out = Path(args.output)
    if out.exists():
        raise SystemExit(f'refusing to overwrite existing output directory: {out}')
    results = [audit(f, args.scan_gap_sec) for f in args.files]
    out.mkdir(parents=True)
    payload = {'scan_gap_sec_assumed': args.scan_gap_sec,
               'timer_period_sec': TIMER_PERIOD_S, 'files': results}
    (out / 'results.json').write_text(json.dumps(payload, indent=2))

    for r in results:
        print('=' * 72)
        print(Path(r['file']).name)
        if not r['m310_samples']:
            print(f"  {r['verdict']}")
            continue
        print(f"  samples {r['m310_samples']}  over {r['duration_s']:.1f} s  "
              f"-> {r['mean_vector_rate_hz']:.2f} Hz")
        print(f"  period      med {r['period_ms']['median']:7.2f}  "
              f"p95 {r['period_ms']['p95']:7.2f} ms")
        print(f"  scan span   med {r['scan_span_ms']['median']:7.2f}  "
              f"p95 {r['scan_span_ms']['p95']:7.2f} ms  "
              f"({100 * r['budget']['scan_span_share']:.0f}% of period)")
        print(f"  per reg     med {r['per_register_round_trip_ms']['median']:7.2f} ms")
        print(f"  status {r['status_counts']}  gaps>1s {r['gaps_over_1s']}")
        b = r['budget']
        print(f"  ceilings: configured {b['ceiling_hz_as_configured']:.2f} Hz | "
              f"gap=0 {b['ceiling_hz_zero_gap']:.2f} Hz | "
              f"gap=0+no tick waste {b['ceiling_hz_zero_gap_no_tick_waste']:.2f} Hz")
        print(f"  15 Hz with scalar reads: "
              f"{'YES' if b['fifteen_hz_reachable_with_scalar_reads'] else 'NO'} "
              f"(needs {b['per_register_ms_required_for_15hz']:.2f} ms/register, "
              f"floor is {b['per_register_ms_observed_floor']:.2f} ms)")
        ns = r.get('non_simultaneity')
        if ns:
            print(f"  six-channel skew while moving: med "
                  f"{ns['vector_skew_nm']['median']:.2f} Nm  "
                  f"p95 {ns['vector_skew_nm']['p95']:.2f} Nm")
    print('=' * 72)
    print(f'wrote {out / "results.json"}')


if __name__ == '__main__':
    main()
