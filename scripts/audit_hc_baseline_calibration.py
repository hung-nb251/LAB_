#!/usr/bin/env python3
"""Offline audit; immutable source logs and no ROS/hardware access."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import analyze_hc_gru_tau_calibration as historical

base = historical.base
ROOT = Path(__file__).resolve().parent.parent


def baseline_pairs(trials):
    """One median per explicitly marked end baseline, never inferred from deadband."""
    xx, yy, inventory = [], [], []
    for trial in trials:
        a = trial['audit_samples']
        start = next((t for t, p in trial['meta']['markers'] if p == 'baseline_end'), np.inf)
        mask = a['valid'] & ~a['running'] & (a['phase'] == 'baseline_end') & (a['t'] > start + 1)
        n = int(mask.sum())
        item = dict(trial=trial['meta']['label'], samples=n)
        if n >= 3:
            q = a['q'][mask]
            item['joint_range_rad'] = np.ptp(q, axis=0).tolist()
            item['axia_norm_median_n'] = float(np.median(np.linalg.norm(a['w'][mask, :3], axis=1)))
            item['accepted'] = bool(np.max(np.ptp(q, axis=0)) < .02 and item['axia_norm_median_n'] < 2)
            if item['accepted']:
                xx.append(np.median(q, axis=0) - trial['q0'])
                yy.append(np.median(a['tau'][mask], axis=0))
        inventory.append(item)
    return np.asarray(xx).reshape(-1, 6), np.asarray(yy).reshape(-1, 6), inventory


def fit_separate(trials, variant):
    x, y, inv = baseline_pairs(trials)
    if not len(x):
        raise ValueError('No explicit stationary end baselines available')
    scale = np.maximum(np.sqrt(np.mean(x*x, axis=0)), .05)
    z = x / scale
    b = np.linalg.solve(z.T @ z / len(z) + .01*np.eye(6), z.T @ y / len(z))
    corrected = []
    for t in trials:
        c = dict(t)
        c['tau'] = t['tau'] - (t['dq']/scale) @ b
        corrected.append(c)
    gain = base.fit_model(corrected, 'torque', variant)
    # Store as the exact existing 12-feature runtime format; offline only.
    a = np.diag(1/np.asarray(gain['scale'])) @ np.asarray(gain['coef'])
    coef = np.vstack((a, -np.diag(1/scale) @ b @ a))
    model = dict(branch='torque', variant='pose', scale=[1.]*12, coef=coef.tolist(), lag=0)
    return model, dict(baseline_inventory=inv, baseline_rank=int(np.linalg.matrix_rank(x)),
                       baseline_singular_values=np.linalg.svd(x, compute_uv=False).tolist())


def runtime_comparison(path, models=None):
    d = pd.read_csv(path)
    status = 'robot_force_status' if 'robot_force_status' in d else 'f_robot_status'
    valid = d[status].str.startswith('SHADOW_VALID') & d[['f_robot_x','f_robot_y','f_robot_z']].notna().all(axis=1)
    u = d[valid].drop_duplicates('fr_timestamp_ns', keep='first')
    t0 = int(d.ros_timestamp_ns.iloc[0])
    # Subtract integer epoch before conversion to floating-point seconds.
    th = (d.fh_timestamp_ns.to_numpy(np.int64)-t0)/1e9
    tr = (u.fr_timestamp_ns.to_numpy(np.int64)-t0)/1e9
    h = d[['f_human_x','f_human_y','f_human_z']].to_numpy(float)
    norm = np.linalg.norm(h, axis=1)
    hp = h.copy()
    active = norm > 1e-9
    hp[active] *= ((norm[active]+4)/norm[active])[:,None]
    times, ix = np.unique(th, return_index=True)
    ref, good = base.interp(tr, np.column_stack((times, hp[ix])), .2)
    # Do not interpolate across censored zero outputs to reconstruct hidden forces.
    right = np.clip(np.searchsorted(times, tr, side='right'), 1, len(times)-1)
    good &= active[ix[right-1]] & active[ix[right]]
    f = u[['f_robot_x','f_robot_y','f_robot_z']].to_numpy(float)
    candidate_metrics = {}
    if 'f_robot_mregister_m310_nm' in u:
        raw = u[[f'f_robot_mregister_m{i}_nm' for i in range(310,316)]].to_numpy(float)
        baseline = u[[f'f_robot_bias_nm_j{i}' for i in range(1,7)]].to_numpy(float)
        q = u[[f'f_robot_source_joint_position_j{i}' for i in range(1,7)]].to_numpy(float)
        q0 = u[[f'f_robot_baseline_q0_j{i}_rad' for i in range(1,7)]].to_numpy(float)
        features = np.column_stack((raw-baseline, q-q0))
        j = np.array([base.LocalIKSolver().compute_jacobian(v) for v in q])
        for name, model in (models or {}).items():
            tau = (features / np.asarray(model['scale'])) @ np.asarray(model['coef'])
            prediction = base.recover(tau, j)[:,:3]
            candidate_metrics[name] = base.metrics(prediction[good], ref[good])
            if name == 'deployed':
                candidate_metrics[name]['replay_max_abs_error_n'] = float(np.max(np.abs(prediction-f)))
    return dict(file=str(path), rows=len(d), unique_estimates=len(u),
                duration_s=float((int(d.ros_timestamp_ns.iloc[-1])-t0)/1e9),
                assumption='Axia radial deadband remained 4 N; raw wrench/filter settings absent in CSV',
                source_time_active_metrics=base.metrics(f[good], ref[good]),
                raw_registers_available='f_robot_mregister_m310_nm' in d,
                candidate_metrics=candidate_metrics), (tr, f, ref, good)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    gt, gru = historical.parse_inputs()
    results = []
    models = {}
    # r1/r2 training, r3 validation. r4 remains a historical test, already inspected.
    for variant in ['diagonal', 'linear']:
        model, audit = fit_separate(gru[:4], variant)
        models[variant] = dict(model=model, audit=audit)
        for split, trials in [('validation_r3', gru[4:6]), ('historical_test_r4', gru[6:])]:
            results.append(dict(model='separate_baseline_'+variant, split=split,
                                metrics=historical.pooled(model, trials)))
    deployed = json.loads((ROOT/'src/hc10dtp_bringup/config/f_robot_m310_candidate_20260918.json').read_text())
    deployed.update(branch='torque', variant='pose', lag=0)
    results.append(dict(model='deployed', split='historical_test_r4', metrics=historical.pooled(deployed, gru[6:])))
    runtime = []
    for stamp in ['172207', '174205']:
        path = ROOT/f'cocarry_logs/cocarry_admittance_3d_20260918_{stamp}.csv'
        replay_models = {name: c['model'] for name,c in models.items()}
        replay_models['deployed'] = deployed
        item, (t, f, h, good) = runtime_comparison(path, replay_models)
        runtime.append(item)
        fig, axes = plt.subplots(3, 1, figsize=(12,8), sharex=True)
        for k, ax in enumerate(axes):
            ax.plot(t, f[:,k], label='M310 deployed', lw=1)
            masked = h[:,k].copy(); masked[~good] = np.nan
            ax.plot(t, masked, label='Axia inferred before deadband, source aligned', lw=1)
            ax.set_ylabel('XYZ'[k]+' (N)'); ax.grid(alpha=.2)
        axes[0].legend(); axes[-1].set_xlabel('Time (s)')
        fig.tight_layout(); fig.savefig(args.output/f'runtime_{stamp}.png'); plt.close(fig)
    _, _, inventory = baseline_pairs(gru)
    payload = dict(status='OFFLINE_AUDIT_NOT_DEPLOYED', candidates=models, results=results,
                   runtime=runtime, baseline_inventory=inventory,
                   limitations=['No truly unseen test after this audit',
                                'New compact CSV has no raw M310 and no raw Axia',
                                'End baselines cover route endpoints, not dynamic no-contact motion'])
    (args.output/'audit.json').write_text(json.dumps(payload, indent=2)+'\n')
    for row in results:
        print(row['model'], row['split'], row['metrics'], flush=True)
    print(json.dumps(dict(runtime=runtime, baseline_inventory=inventory), indent=2))


if __name__ == '__main__':
    main()
