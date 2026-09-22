#!/usr/bin/env python3
"""Combine two T1 sidecar developments with frozen historical T1 data.

This creates an offline candidate and regression review. It never changes a
runtime model, ROS configuration, or source log. A new session remains needed.
"""
import argparse
import json
from pathlib import Path

import numpy as np

import audit_hc_t1_sidecar as audit
import review_hc_t1_candidates as review


def unit(z, mask):
    return z['tau'][mask], z['op'][mask], z['ref'][mask]


def unit_metric(gain, item):
    tau, op, ref = item
    return audit.metrics(audit.predict(gain, tau, op), ref)


def macro(gain, units):
    rows = [unit_metric(gain, item) for item in units]
    return float(np.mean([row['rmse_vector_n'] for row in rows])), rows


def directional(gain, datasets, mask_name):
    tau = np.vstack([z['tau'][z[mask_name]] for z in datasets])
    op = np.vstack([z['op'][z[mask_name]] for z in datasets])
    ref = np.vstack([z['ref'][z[mask_name]] for z in datasets])
    pred = audit.predict(gain, tau, op)
    rn = np.linalg.norm(ref, axis=1)
    rows = {}
    for k, axis in enumerate('XYZ'):
        for sign, sgn in [('+', 1), ('-', -1)]:
            mask = (rn >= 4) & (sgn*ref[:, k] >= np.cos(np.pi/6)*rn)
            rows[axis+sign] = audit.metrics(pred[mask], ref[mask])
    return rows


def verify_analysis(path):
    for entry in json.loads((path/'input_manifest.json').read_text()):
        if audit.digest(Path(entry['path'])) != entry['sha256']:
            raise ValueError('Changed sidecar source: '+entry['path'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xy-analysis', type=Path, required=True)
    parser.add_argument('--z-analysis', type=Path, required=True)
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for path in [args.xy_analysis, args.z_analysis]:
        verify_analysis(path)
    snapshots = [json.loads((p/'runtime_snapshot.json').read_text())
                 for p in [args.xy_analysis, args.z_analysis]]
    if snapshots[0] != snapshots[1]:
        raise ValueError('Runtime model changed between T1 sidecar trials')
    runtime = snapshots[0]
    datasets = [np.load(p/'paired_samples.npz', allow_pickle=False)
                for p in [args.xy_analysis, args.z_analysis]]
    historical, historical_sources = review.load_history(args.history, runtime)

    # Candidate selection: older 17/09 T1 is development, 18/09 T1 and the
    # validation blocks of both new logs are validation units. Each unit gets
    # equal total weight. All data have been viewed, so this is retrospective.
    train = [(d['tau'], d['op'], d['raw_reference']) for d in historical
             if d['date'] == '20260917']
    train += [unit(z, z['train']) for z in datasets]
    validation = [(d['tau'], d['op'], d['raw_reference']) for d in historical
                  if d['date'] == '20260918']
    validation += [unit(z, z['validation']) for z in datasets]
    selection = []
    baseline_score, baseline_rows = macro(np.eye(6), validation)
    selection.append(dict(name='runtime', family=None, alpha=None,
                          macro_validation_rmse_n=baseline_score,
                          validation=baseline_rows))
    for family in ['diagonal', 'full']:
        for alpha in [.01, .1, 1., 10., 100.]:
            gain = review.fit_uniform_prior(train, family, alpha)
            score, rows = macro(gain, validation)
            selection.append(dict(name=f'{family}_{alpha}', family=family, alpha=alpha,
                                  macro_validation_rmse_n=score,
                                  validation=rows, gain=gain.tolist()))
    chosen = min(selection, key=lambda row: row['macro_validation_rmse_n'])
    refit = train+validation
    gain = (np.eye(6) if chosen['family'] is None else
            review.fit_uniform_prior(refit, chosen['family'], chosen['alpha']))
    candidate = dict(runtime)
    candidate.update(
        coef=(np.asarray(runtime['coef'])@gain).tolist(),
        status='OFFLINE_T1_COMBINED_CANDIDATE_PENDING_NEW_SESSION_TEST',
        calibration_confirmed=False, role_valid=False,
        source='calibrate_hc_t1_combined.py', family=chosen['family'],
        alpha=chosen['alpha'], correction_gain=gain.tolist(),
        selection_macro_validation_rmse_n=chosen['macro_validation_rmse_n'],
        runtime_macro_validation_rmse_n=baseline_score,
        independent_test=None,
        training_sources=[str(args.xy_analysis), str(args.z_analysis), str(args.history)],
        target='force XYZ only; moments and active robot intention are not calibrated')
    audit.save(args.output/'candidate_frozen.json', candidate)
    audit.save(args.output/'runtime_snapshot.json', runtime)
    audit.save(args.output/'selection.json', selection)

    holdouts = []
    for label, z in zip(['xy_180049', 'z_183108'], datasets):
        mask = z['internal_holdout']
        holdouts.append(dict(label=label, n=int(mask.sum()),
            runtime=unit_metric(np.eye(6), unit(z, mask)),
            candidate=unit_metric(gain, unit(z, mask))))
    history_runtime = review.transfer(np.eye(6), historical)
    history_candidate = review.transfer(gain, historical)
    previous = json.loads((args.xy_analysis/'candidate_frozen.json').read_text())
    previous_gain = np.asarray(previous['correction_gain'])
    z = datasets[1]
    previous_prospective = dict(
        note='XY candidate was frozen before the Z trial; Z trial was not used in that fit',
        runtime=unit_metric(np.eye(6), unit(z, np.ones(len(z['t']), bool))),
        previous_xy_candidate=unit_metric(previous_gain, unit(z, np.ones(len(z['t']), bool))),
        runtime_per_direction=directional(np.eye(6), [z], 'internal_holdout'),
        previous_candidate_per_direction=directional(previous_gain, [z], 'internal_holdout'))
    report = dict(
        status='CANDIDATE_FROZEN_OFFLINE_NOT_DEPLOYED',
        selected=chosen['name'], selection_macro_validation_rmse_n=chosen['macro_validation_rmse_n'],
        runtime_macro_validation_rmse_n=baseline_score,
        correction_gain_singular_values=np.linalg.svd(gain,compute_uv=False).tolist(),
        holdouts=holdouts,
        historical=dict(runtime_macro_rmse_n=history_runtime['macro_rmse_n'],
                        candidate_macro_rmse_n=history_candidate['macro_rmse_n'],
                        runtime_macro_raw_rmse_n=history_runtime['macro_raw_rmse_n'],
                        candidate_macro_raw_rmse_n=history_candidate['macro_raw_rmse_n'],
                        candidate_trials=history_candidate['trials']),
        combined_internal_holdout_directional=dict(
            runtime=directional(np.eye(6), datasets, 'internal_holdout'),
            candidate=directional(gain, datasets, 'internal_holdout')),
        prospective_previous_candidate=previous_prospective,
        decision_reasons=[
            'The new Z trial supplies both Z+ and Z- but is now part of development',
            'The previous XY-only candidate failed its prospective check on the new Z trial',
            'The combined candidate improves both internal sidecar holdouts and historical T1 macro RMSE',
            'All historical splits and both same-day internal blocks have been inspected',
            'A new session with a frozen model is required before any runtime replacement'],
        decision='KEEP_CURRENT_RUNTIME_SHADOW; SAVE_COMBINED_CANDIDATE_OFFLINE; REQUIRE_NEW_SESSION_TEST')
    audit.save(args.output/'report.json', report)
    provenance_files = [Path(__file__), audit.ROOT/'audit_hc_t1_sidecar.py',
                        audit.ROOT/'review_hc_t1_candidates.py']
    for p in [args.xy_analysis,args.z_analysis]:
        provenance_files += [p/'paired_samples.npz',p/'audit.json',p/'input_manifest.json',p/'runtime_snapshot.json']
    provenance_files += [args.history/'paired_samples.npz',args.history/'trial_metadata.json',
                         args.history/'manifest.json',args.history/'provenance.json']
    audit.save(args.output/'provenance.json', dict(
        files=[dict(path=str(p),sha256=audit.digest(p)) for p in provenance_files],
        verified_historical_sources=historical_sources,
        protocol='equal total weight per historical trial/new temporal unit; raw reference for old data'))
    for p in [Path(__file__), audit.ROOT/'audit_hc_t1_sidecar.py',
              audit.ROOT/'review_hc_t1_candidates.py']:
        (args.output/p.name).write_text(p.read_text())
    print(json.dumps(dict(status=report['status'],selected=report['selected'],
        validation=[baseline_score,chosen['macro_validation_rmse_n']],holdouts=holdouts,
        historical=report['historical'],decision=report['decision']),indent=2))


if __name__ == '__main__':
    main()
