#!/usr/bin/env python3
"""Test whether M-register F_robot is only a slower external-force estimate.

The analysis is descriptive and does not fit, calibrate, or deploy a model.
It pairs raw compensated Axia force with unique M310 scans, then compares
cos(F_robot, F_ext) against cos(v_EE, F_ext) during LEADER.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
import audit_hc_t1_sidecar as audit  # noqa: E402
import score_hc_t1_independent_test as scorer  # noqa: E402

FORCE_MIN_N = 4.0
SPEED_MIN_MPS = 0.01
VELOCITY_HALF_WINDOW_S = 0.20


def local_slope(t, values, half_window=VELOCITY_HALF_WINDOW_S):
    """Centered local-linear derivative on an irregular time grid."""
    out = np.full_like(values, np.nan, dtype=float)
    for i, center in enumerate(t):
        mask = np.abs(t - center) <= half_window
        if mask.sum() < 3:
            continue
        dt = t[mask] - np.mean(t[mask])
        denom = float(dt @ dt)
        if denom <= 0:
            continue
        out[i] = np.sum(dt[:, None] * (values[mask] - np.mean(values[mask], axis=0)),
                        axis=0) / denom
    return out


def interpolate_vectors(t, source_t, values):
    return np.column_stack([np.interp(t, source_t, values[:, k]) for k in range(3)])


def cosine(a, b):
    denom = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
    out = np.full(len(a), np.nan)
    good = denom > np.finfo(float).eps
    out[good] = np.sum(a[good] * b[good], axis=1) / denom[good]
    return np.clip(out, -1.0, 1.0)


def summary(values, mask):
    selected = values[mask & np.isfinite(values)]
    if not len(selected):
        return {'n': 0}
    return {
        'n': int(len(selected)),
        'negative_n': int(np.sum(selected < 0)),
        'negative_fraction': float(np.mean(selected < 0)),
        'median': float(np.median(selected)),
        'p5': float(np.percentile(selected, 5)),
        'p95': float(np.percentile(selected, 95)),
        'minimum': float(np.min(selected)),
        'maximum': float(np.max(selected)),
    }


def leader_spans(t, roles):
    spans, start = [], None
    for i, role in enumerate(roles):
        if role == 'LEADER' and start is None:
            start = t[i]
        elif role != 'LEADER' and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def inspect_run(csv_path):
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 5:
        raise ValueError(f'too few CSV rows: {csv_path}')
    ct = np.array([int(row['ros_timestamp_ns']) / 1e9 for row in rows])
    if np.any(np.diff(ct) <= 0):
        raise ValueError(f'non-increasing CSV timestamps: {csv_path}')
    actual = np.array([[float(row[f'actual_ee_{axis}']) for axis in 'xyz']
                       for row in rows])
    reference = np.array([[float(row[f'reference_{axis}r']) for axis in 'xyz']
                          for row in rows])
    roles = np.array([row['role'] for row in rows])
    v_ee_csv = local_slope(ct, actual)
    v_ref_csv = local_slope(ct, reference)

    sidecar = Path(str(csv_path) + '.calibration.jsonl')
    events, inventory = audit.read_events([sidecar])
    origin_s = min(event['receipt_ns'] for event in events) / 1e9
    pack = scorer.pair_samples([sidecar])
    pt = pack['time'] + origin_s
    row_ids = np.clip(np.searchsorted(ct, pt, side='right') - 1, 0, len(ct) - 1)
    role = roles[row_ids]
    v_ee = interpolate_vectors(pt, ct, v_ee_csv)
    v_ref = interpolate_vectors(pt, ct, v_ref_csv)
    f_ext = pack['target']
    f_robot = audit.predict(np.eye(6), pack['tau'], pack['op'])
    phi = cosine(f_robot, f_ext)
    cos_v_ee = cosine(v_ee, f_ext)
    cos_v_ref = cosine(v_ref, f_ext)
    n_ext = np.linalg.norm(f_ext, axis=1)
    n_robot = np.linalg.norm(f_robot, axis=1)
    speed_ee = np.linalg.norm(v_ee, axis=1)
    speed_ref = np.linalg.norm(v_ref, axis=1)
    leader = role == 'LEADER'

    spans_abs = leader_spans(ct, roles)
    later = np.zeros(len(pt), dtype=bool)
    for start, stop in spans_abs:
        later |= (pt >= (start + stop) / 2) & (pt < stop)

    phi_mask = leader & (n_ext >= FORCE_MIN_N) & (n_robot >= FORCE_MIN_N)
    vee_mask = leader & (n_ext >= FORCE_MIN_N) & (speed_ee >= SPEED_MIN_MPS)
    vref_mask = leader & (n_ext >= FORCE_MIN_N) & (speed_ref >= SPEED_MIN_MPS)

    raw = sorted((event for event in events if event['kind'] == 'axia_raw'),
                 key=lambda event: event['data']['stamp_ns'])
    raw_t = np.array([event['data']['stamp_ns'] / 1e9 for event in raw])
    axia_interval = np.diff(raw_t)
    m310_interval = np.diff(pack['time'])
    t_run = pack['time'] - pack['run_start']
    spans_run = [(a - origin_s - pack['run_start'], b - origin_s - pack['run_start'])
                 for a, b in spans_abs]

    report = {
        'csv': str(csv_path),
        'csv_sha256': audit.digest(csv_path),
        'sidecar': str(sidecar),
        'sidecar_sha256': audit.digest(sidecar),
        'input_inventory': inventory,
        'paired_unique_m310_scans': int(len(pt)),
        'leader_spans_since_run_start_s': spans_run,
        'thresholds': {
            'force_norm_min_n': FORCE_MIN_N,
            'speed_norm_min_mps': SPEED_MIN_MPS,
            'velocity_local_linear_half_window_s': VELOCITY_HALF_WINDOW_S,
        },
        'rates': {
            'axia_hz_from_median_interval': float(1 / np.median(axia_interval)),
            'axia_interval_p95_ms': float(1000 * np.percentile(axia_interval, 95)),
            'm310_hz_from_median_paired_interval': float(1 / np.median(m310_interval)),
            'm310_interval_p95_ms': float(1000 * np.percentile(m310_interval, 95)),
            'm310_scan_span_median_ms': float(np.median(pack['span_ms'])),
            'm310_scan_span_p95_ms': float(np.percentile(pack['span_ms'], 95)),
            'axia_to_m310_rate_ratio': float(
                (1 / np.median(axia_interval)) / (1 / np.median(m310_interval))),
        },
        'runtime_replay_max_abs_error_n': float(np.max(np.abs(f_robot - pack['recorded']))),
        'leader': {
            'cos_f_robot_f_ext': summary(phi, phi_mask),
            'cos_v_ee_f_ext': summary(cos_v_ee, vee_mask),
            'cos_v_reference_f_ext': summary(cos_v_ref, vref_mask),
        },
        'leader_second_half': {
            'cos_f_robot_f_ext': summary(phi, phi_mask & later),
            'cos_v_ee_f_ext': summary(cos_v_ee, vee_mask & later),
            'cos_v_reference_f_ext': summary(cos_v_ref, vref_mask & later),
        },
        'notes': [
            'Each row is one unique M310 scan; repeated CSV force values are not independent samples.',
            'F_ext is reconstructed from raw Axia with recorded bias, mass and rotation before deadband.',
            'No lag optimization, model fitting, sign selection or threshold selection was performed.',
            'Cosine with velocity is excluded below 0.01 m/s; velocity uses a fixed 0.4 s local-linear window.',
        ],
    }
    samples = {
        't': t_run, 'role': role, 'later': later, 'f_ext': f_ext, 'f_robot': f_robot,
        'v_ee': v_ee, 'v_ref': v_ref, 'n_ext': n_ext, 'n_robot': n_robot,
        'speed_ee': speed_ee, 'speed_ref': speed_ref, 'phi': phi,
        'cos_v_ee': cos_v_ee, 'cos_v_ref': cos_v_ref, 'phi_mask': phi_mask,
        'vee_mask': vee_mask, 'vref_mask': vref_mask, 'leader': leader,
        'spans': spans_run,
    }
    return report, samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')
    args.output.mkdir(parents=True)

    reports, samples = [], []
    for run in args.runs:
        report, sample = inspect_run(run)
        reports.append(report)
        samples.append(sample)

    pooled = {}
    for section in ('leader', 'leader_second_half'):
        pooled[section] = {}
        for key in ('cos_f_robot_f_ext', 'cos_v_ee_f_ext', 'cos_v_reference_f_ext'):
            rows = [report[section][key] for report in reports]
            pooled[section][key] = {
                'n': sum(row['n'] for row in rows),
                'negative_n': sum(row.get('negative_n', 0) for row in rows),
            }
            n = pooled[section][key]['n']
            pooled[section][key]['negative_fraction'] = (
                pooled[section][key]['negative_n'] / n if n else None)

    combined = {
        'status': 'DESCRIPTIVE_EVIDENCE_MREGISTER_IS_EXTERNAL_FORCE_NOT_ROBOT_INTENT',
        'runs': reports,
        'pooled_counts_descriptive_only': pooled,
        'interpretation': [
            'M-register F_robot remains aligned with Axia F_ext during LEADER opposition.',
            'EE/reference velocity becomes opposed to Axia force, especially in the second half of LEADER.',
            'Therefore the M-register path estimates external interaction force and cannot represent robot intent.',
        ],
        'limitation': ('Samples inside a run are autocorrelated; pooled counts are descriptive and are not '
                       'treated as independent Bernoulli trials.'),
    }
    audit.save(args.output / 'report.json', combined)

    with (args.output / 'paired_cosines.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['run', 't_since_run_start_s', 'role', 'leader_second_half',
                         'f_ext_norm_n', 'f_robot_norm_n', 'v_ee_norm_mps',
                         'v_reference_norm_mps', 'cos_f_robot_f_ext',
                         'cos_v_ee_f_ext', 'cos_v_reference_f_ext',
                         'eligible_force_cosine', 'eligible_ee_velocity_cosine'])
        for path, item in zip(args.runs, samples):
            for i in range(len(item['t'])):
                writer.writerow([path.stem, item['t'][i], item['role'][i],
                                 int(item['later'][i]), item['n_ext'][i],
                                 item['n_robot'][i], item['speed_ee'][i],
                                 item['speed_ref'][i], item['phi'][i],
                                 item['cos_v_ee'][i], item['cos_v_ref'][i],
                                 int(item['phi_mask'][i]), int(item['vee_mask'][i])])

    fig, axes = plt.subplots(len(samples), 1, figsize=(12, 4.2 * len(samples)),
                             sharex=False, squeeze=False)
    for ax, path, item in zip(axes[:, 0], args.runs, samples):
        for start, stop in item['spans']:
            ax.axvspan(start, stop, color='#dddddd', alpha=0.7, label='LEADER')
            ax.axvline((start + stop) / 2, color='#777777', linestyle=':', linewidth=1)
        ax.axhline(0, color='black', linewidth=1)
        ax.scatter(item['t'][item['phi_mask']], item['phi'][item['phi_mask']],
                   s=30, color='#b02a7c', label='cos(F_robot register, F_ext)')
        ax.scatter(item['t'][item['vee_mask']], item['cos_v_ee'][item['vee_mask']],
                   s=30, color='#1976d2', marker='x', label='cos(v_EE, F_ext)')
        ax.set_ylim(-1.08, 1.08)
        ax.set_xlim(item['t'].min(), item['t'].max())
        ax.set_ylabel('cosine')
        ax.set_title(path.stem)
        ax.grid(alpha=0.25)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), loc='lower left', fontsize=9)
    axes[-1, 0].set_xlabel('time since RUNNING [s]; dotted line = midpoint of LEADER')
    fig.tight_layout()
    fig.savefig(args.output / 'leader_force_direction_evidence.png', dpi=180)
    plt.close(fig)

    print(json.dumps(combined, indent=2))


if __name__ == '__main__':
    main()
