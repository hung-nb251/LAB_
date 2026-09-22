#!/usr/bin/env python3
"""Plot F_ext vs F_robot and cos(F_robot, F_ext) for one cooperative run.

Evaluation only: the candidate gain is read from the frozen file, nothing is fit.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hc10dtp_bringup/scripts'))
import audit_hc_t1_sidecar as audit
import score_hc_t1_independent_test as S

# Categorical slots 1-3 of the reference palette (validated all-pairs, light).
EXT, RUNTIME, CAND = '#2a78d6', '#eb6834', '#1baf7a'
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#d9d8d4'
MASK_N = 4.0


def style(ax):
    ax.set_facecolor('#fcfcfb')
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.9)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sidecar', type=Path)
    ap.add_argument('--candidate', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')

    pack = S.pair_samples([args.sidecar])
    gain = np.asarray(json.loads(args.candidate.read_text())['correction_gain'], float)
    ref = pack['target']
    rt = audit.predict(np.eye(6), pack['tau'], pack['op'])
    cd = audit.predict(gain, pack['tau'], pack['op'])
    t = pack['time'] - pack['run_start']

    n_ref, n_rt, n_cd = (np.linalg.norm(a, axis=1) for a in (ref, rt, cd))
    cos = lambda F, n: np.sum(F * ref, axis=1) / (n * n_ref)
    cos_rt, cos_cd = cos(rt, n_rt), cos(cd, n_cd)
    mask_rt = (n_ref >= MASK_N) & (n_rt >= MASK_N)
    mask_cd = (n_ref >= MASK_N) & (n_cd >= MASK_N)

    args.output.mkdir(parents=True)

    # --- Figure 1: per-axis force, small multiples on one shared scale ---
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True,
                             gridspec_kw={'hspace': 0.28})
    fig.patch.set_facecolor('#fcfcfb')
    lim = np.nanmax(np.abs(np.concatenate([ref, rt, cd]))) * 1.12
    for k, (ax, name) in enumerate(zip(axes[:3], ('Fx', 'Fy', 'Fz'))):
        style(ax)
        ax.axhline(0, color=MUTED, linewidth=1, alpha=0.5)
        ax.plot(t, ref[:, k], color=EXT, linewidth=2, label='F_ext (Axia, raw ref)')
        ax.plot(t, rt[:, k], color=RUNTIME, linewidth=2, label='F_robot runtime')
        ax.plot(t, cd[:, k], color=CAND, linewidth=2, label='F_robot candidate')
        ax.set_ylim(-lim, lim)
        ax.set_ylabel(f'{name}  [N]', color=INK, fontsize=10)
    style(axes[3])
    axes[3].plot(t, n_ref, color=EXT, linewidth=2, label='|F_ext|')
    axes[3].plot(t, n_rt, color=RUNTIME, linewidth=2, label='|F_robot| runtime')
    axes[3].plot(t, n_cd, color=CAND, linewidth=2, label='|F_robot| candidate')
    axes[3].axhline(MASK_N, color=MUTED, linewidth=1, linestyle=':',
                    label=f'{MASK_N:g} N analysis mask')
    axes[3].set_ylabel('magnitude  [N]', color=INK, fontsize=10)
    axes[3].set_xlabel('time since Start Run  [s]', color=INK, fontsize=10)
    axes[0].legend(loc='upper left', frameon=False, fontsize=9, ncol=3,
                   labelcolor=INK)
    axes[3].legend(loc='upper left', frameon=False, fontsize=9, ncol=4,
                   labelcolor=INK)
    axes[0].set_title('F_ext versus F_robot, cooperative Home to Target 1 run',
                      color=INK, fontsize=13, loc='left', pad=12)
    fig.savefig(args.output / 'force_ext_vs_robot.png', dpi=150,
                bbox_inches='tight', facecolor='#fcfcfb')
    plt.close(fig)

    # --- Figure 2: cosine over time and its distribution ---
    fig = plt.figure(figsize=(11, 7))
    fig.patch.set_facecolor('#fcfcfb')
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.35)
    ax = fig.add_subplot(gs[0]); style(ax)
    ax.axhspan(-1, 0, color='#e34948', alpha=0.07)
    ax.axhline(0, color=MUTED, linewidth=1.2)
    ax.axhline(1, color=GRID, linewidth=1)
    ax.plot(t, cos_rt, color=RUNTIME, linewidth=1.2, alpha=0.35, zorder=1)
    ax.plot(t, cos_cd, color=CAND, linewidth=1.2, alpha=0.35, zorder=1)
    ax.scatter(t[mask_rt], cos_rt[mask_rt], s=44, color=RUNTIME, zorder=3,
               edgecolor='#fcfcfb', linewidth=2, label='runtime, both norms >= 4 N')
    ax.scatter(t[mask_cd], cos_cd[mask_cd], s=44, color=CAND, zorder=3,
               edgecolor='#fcfcfb', linewidth=2, label='candidate, both norms >= 4 N')
    ax.set_ylim(-1.08, 1.08)
    ax.set_ylabel('cos(F_robot, F_ext)', color=INK, fontsize=10)
    ax.set_xlabel('time since Start Run  [s]', color=INK, fontsize=10)
    ax.legend(loc='lower left', frameon=False, fontsize=9, labelcolor=INK)
    ax.set_title('Cosine under KNOWN agreement - faint line is below the mask',
                 color=INK, fontsize=13, loc='left', pad=12)
    ax.text(0.995, 0.06, 'conflict region', transform=ax.transAxes, ha='right',
            fontsize=9, color='#e34948')

    ax2 = fig.add_subplot(gs[1]); style(ax2)
    bins = np.linspace(-1, 1, 41)
    ax2.hist(cos_rt[mask_rt], bins=bins, color=RUNTIME, alpha=0.75,
             label=f'runtime  (n={int(mask_rt.sum())})')
    ax2.hist(cos_cd[mask_cd], bins=bins, color=CAND, alpha=0.75,
             label=f'candidate  (n={int(mask_cd.sum())})')
    ax2.axvline(0, color=MUTED, linewidth=1.2)
    ax2.set_xlim(-1.02, 1.02)
    ax2.set_xlabel('cos(F_robot, F_ext)', color=INK, fontsize=10)
    ax2.set_ylabel('samples', color=INK, fontsize=10)
    ax2.legend(loc='upper left', frameon=False, fontsize=9, labelcolor=INK)
    fig.savefig(args.output / 'cosine_agreement.png', dpi=150,
                bbox_inches='tight', facecolor='#fcfcfb')
    plt.close(fig)

    def summary(c, m):
        s = c[m]
        return dict(n=int(m.sum()), median=float(np.median(s)),
                    p5=float(np.percentile(s, 5)), p95=float(np.percentile(s, 95)),
                    minimum=float(s.min()), fraction_negative=float((s < 0).mean()))

    report = dict(
        sidecar=str(args.sidecar), paired_scans=int(len(pack['ids'])),
        running_window_s=[pack['run_start'], pack['cutoff']],
        replay_self_check_n=float(np.nanmax(np.abs(rt - pack['recorded']))),
        rmse_vector_n=dict(runtime=audit.metrics(rt, ref)['rmse_vector_n'],
                           candidate=audit.metrics(cd, ref)['rmse_vector_n']),
        force_norm_n=dict(ext_median=float(np.median(n_ref)),
                          runtime_median=float(np.median(n_rt)),
                          candidate_median=float(np.median(n_cd))),
        cosine=dict(runtime=summary(cos_rt, mask_rt),
                    candidate=summary(cos_cd, mask_cd)),
        note='Evaluation only; the model was not fitted or reselected here.')
    audit.save(args.output / 'coop_report.json', report)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
