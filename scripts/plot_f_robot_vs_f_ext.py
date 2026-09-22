#!/usr/bin/env python3
"""Plot |F_ext| and |F_robot| and the disagreement index Phi for one run.

Evaluation only: the runtime model is replayed from the sidecar; nothing is fit.
Plot style follows simulation_hri/shared_control_lib.py.

    python3 scripts/plot_f_robot_vs_f_ext.py <run>.csv --output <dir>
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
import audit_hc_t1_sidecar as audit          # noqa: E402
import score_hc_t1_independent_test as S     # noqa: E402

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12
plt.rcParams['legend.fontsize'] = 10

PHI_ANGLE = 90
MASK_N = 4.0


def leader_spans(csv_path, t_abs, t_rel):
    """Time spans where the controller reported role LEADER, in plot time."""
    try:
        rows = list(csv.DictReader(open(csv_path)))
    except OSError:
        return []
    if not rows or 'role' not in rows[0]:
        return []
    ct = np.array([float(r['ros_timestamp_ns']) for r in rows])
    role = np.array([r['role'] for r in rows])
    idx = np.clip(np.searchsorted(ct, t_abs) - 1, 0, len(ct) - 1)
    is_leader = role[idx] == 'LEADER'
    spans, start = [], None
    for i, flag in enumerate(is_leader):
        if flag and start is None:
            start = t_rel[i]
        elif not flag and start is not None:
            spans.append((start, t_rel[i])); start = None
    if start is not None:
        spans.append((start, t_rel[-1]))
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', type=Path)
    ap.add_argument('--candidate', type=Path,
                    help='optional frozen candidate; default is the runtime model')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')

    csv_path = args.run
    sidecar = (csv_path if str(csv_path).endswith('.calibration.jsonl')
               else Path(str(csv_path) + '.calibration.jsonl'))
    events, _ = audit.read_events([sidecar])
    origin = min(e['receipt_ns'] for e in events)
    pack = S.pair_samples([sidecar])
    gain = np.eye(6)
    which = 'runtime'
    if args.candidate:
        gain = np.asarray(json.loads(args.candidate.read_text())['correction_gain'], float)
        which = 'candidate'

    f_ext = pack['target']
    f_robot = audit.predict(gain, pack['tau'], pack['op'])
    t = pack['time'] - pack['run_start']
    n_ext = np.linalg.norm(f_ext, axis=1)
    n_rob = np.linalg.norm(f_robot, axis=1)

    phi = np.sum(f_robot * f_ext, axis=1) / (n_rob * n_ext)
    phi_threshold = np.cos(np.radians(PHI_ANGLE))
    # The 4 N rule filters the statistics only; the curve shows every sample.
    weak = (n_ext < MASK_N) | (n_rob < MASK_N)

    leader = leader_spans(csv_path, pack['time'] * 1e9 + origin, t)

    args.output.mkdir(parents=True)
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    def shade(ax):
        for a, b in leader:
            ax.axvspan(a, b, facecolor='0.85', alpha=0.6, zorder=0, linewidth=0)

    for idx, label in enumerate(['Force X (N)', 'Force Y (N)', 'Force Z (N)']):
        ax = axes[idx]
        shade(ax)
        ax.plot(t, f_ext[:, idx], 'g-', linewidth=1.5, label='F_ext', zorder=2)
        ax.plot(t, f_robot[:, idx], 'm--', linewidth=1.5, alpha=0.8,
                label='F_robot', zorder=2)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3, zorder=1)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='--', zorder=1)
        ax.legend(loc='upper left')
        if idx == 0:
            ax.set_title(f'Interaction Forces: F_ext vs F_robot ({csv_path.stem})')

    ax = axes[3]
    shade(ax)
    ax.plot(t, phi, 'k-', linewidth=1.0, label='Φ')
    ax.axhline(phi_threshold, color='r', linestyle='--', linewidth=1.2,
               label=f'Φ={phi_threshold:.2f} (conflict, {PHI_ANGLE}°)')
    ax.fill_between(t, phi, phi_threshold, where=phi < phi_threshold,
                    alpha=0.3, color='red', label='Conflict region')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Φ')
    ax.set_ylim(-1.1, 1.1)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    ax.set_title('Disagreement Index')

    plt.tight_layout()
    fig.savefig(args.output / 'f_robot_vs_f_ext.png', dpi=150)
    plt.close('all')

    valid = ~weak
    report = dict(
        run=str(csv_path), model=which, paired_scans=int(len(pack['ids'])),
        evaluated_samples=int(valid.sum()),
        replay_self_check_n=float(np.nanmax(np.abs(
            audit.predict(np.eye(6), pack['tau'], pack['op']) - pack['recorded']))),
        magnitude_n=dict(
            f_ext=dict(median=float(np.median(n_ext)), p95=float(np.percentile(n_ext, 95))),
            f_robot=dict(median=float(np.median(n_rob)), p95=float(np.percentile(n_rob, 95)))),
        per_axis_n={
            axis: dict(
                f_ext_range=[float(f_ext[:, i].min()), float(f_ext[:, i].max())],
                f_robot_range=[float(f_robot[:, i].min()), float(f_robot[:, i].max())],
                rmse=float(np.sqrt(np.mean((f_robot[:, i] - f_ext[:, i]) ** 2))))
            for i, axis in enumerate('XYZ')},
        rmse_vector_n=float(audit.metrics(f_robot, f_ext)['rmse_vector_n']),
        phi_all_samples=dict(
            n=int(len(phi)), median=float(np.median(phi)),
            minimum=float(phi.min()),
            fraction_below_threshold=float((phi < phi_threshold).mean())),
        phi_above_4N=dict(
            n=int(valid.sum()), median=float(np.median(phi[valid])),
            p5=float(np.percentile(phi[valid], 5)),
            minimum=float(phi[valid].min()),
            fraction_below_threshold=float((phi[valid] < phi_threshold).mean())),
        note='Evaluation only; no fitting or threshold selection on this run.')
    audit.save(args.output / 'phi_report.json', report)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
