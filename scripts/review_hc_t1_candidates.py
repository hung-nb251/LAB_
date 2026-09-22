#!/usr/bin/env python3
"""Retrospective transfer/sensitivity review of T1 development candidates.

All data here have been inspected. No result is an independent test. This
script writes a new review directory, never runtime files or source logs.
"""
import argparse
import json
from pathlib import Path

import numpy as np

import audit_hc_t1_sidecar as audit


def fit_uniform_prior(trials, family, alpha):
    """Equal trial weights and isotropic parameter prior (one common scale).

    A common scale avoids relaxing the gain prior along weakly observed
    torque directions, as happens with separate feature standardization.
    """
    arrays = []
    labels = []
    weights = []
    for tau, op, reference in trials:
        a = audit.design(tau, op, family).reshape(-1, 6 if family=='diagonal' else 36)
        arrays.append(a)
        labels.append((reference-audit.predict(np.eye(6), tau, op)).ravel())
        weights.extend([1/len(a)]*len(a))
    x = np.vstack(arrays)
    y = np.concatenate(labels)
    w = np.asarray(weights); w /= w.sum()
    scale = max(float(np.sqrt(np.sum(w[:,None]*x*x)/x.shape[1])), .05)
    z = x/scale
    delta = np.linalg.solve(z.T@(w[:,None]*z)+alpha*np.eye(z.shape[1]), z.T@(w*y))/scale
    return np.eye(6)+(np.diag(delta) if family=='diagonal' else delta.reshape(6,6))


def load_history(folder, runtime):
    """Verify every used source and metadata against the frozen P0 inventory."""
    metadata = json.loads((folder/'trial_metadata.json').read_text())
    cache = np.load(folder/'paired_samples.npz', allow_pickle=False)
    trials = []
    sources = []
    for i, meta in enumerate(metadata):
        if meta['route'] != 'T1':
            continue
        source = audit.ROOT/meta['path']
        if audit.digest(source) != meta['source_sha256']:
            raise ValueError(f'Changed source: {source}')
        if 'metadata_sha256' in meta:
            if audit.digest(source.parent/'metadata.json') != meta['metadata_sha256']:
                raise ValueError(f'Changed metadata: {source}')
        sources.append(dict(path=str(source), sha256=meta['source_sha256']))
        d = {k.split('_',1)[1]:cache[k] for k in cache.files if k.startswith(f'{i}_')}
        mask = d['force_mask']
        tau = (np.column_stack((d['tau'][mask],d['dq'][mask]))/runtime['scale'])@runtime['coef']
        trials.append(dict(trial=meta['trial'], date=meta['date'], mode=meta['mode'],
                           tau=tau, op=audit.recovery(d['j'][mask],runtime['damping']),
                           reference=d['w'][mask,:3], raw_reference=d['w_raw'][mask,:3]))
    return trials, sources


def transfer(gain, trials):
    rows = []
    for d in trials:
        pred = audit.predict(gain,d['tau'],d['op'])
        rows.append(dict(trial=d['trial'], date=d['date'], mode=d['mode'],
                         metrics=audit.metrics(pred,d['reference']),
                         raw_reference=audit.metrics(pred,d['raw_reference'])))
    return dict(trials=rows,
                macro_rmse_n=float(np.mean([r['metrics']['rmse_vector_n'] for r in rows])),
                macro_raw_rmse_n=float(np.mean([r['raw_reference']['rmse_vector_n'] for r in rows])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    # Verify current data files are still the ones used in the preceding fit.
    original_manifest=json.loads((args.analysis/'input_manifest.json').read_text())
    for entry in original_manifest:
        if audit.digest(Path(entry['path']))!=entry['sha256']:
            raise ValueError('Development source changed: '+entry['path'])
    runtime=json.loads((args.analysis/'runtime_snapshot.json').read_text())
    winner=json.loads((args.analysis/'candidate_frozen.json').read_text())
    selection=json.loads((args.analysis/'selection_stage.json').read_text())
    z=np.load(args.analysis/'paired_samples.npz',allow_pickle=False)
    historical,used_sources=load_history(args.history,runtime)
    audit.save(args.output/'protocol.json',dict(
        status='RETROSPECTIVE_DEVELOPMENT_REVIEW_NOT_NEW_TEST',
        historical='T1-labeled trials from frozen P0 cache; same samples/reference for comparisons',
        caveat='Runtime prior already trained on some historical trials; this is a regression check',
        local_uniform='Same 60/20/20 masks, isotropic identity prior; holdout already viewed in v1',
        mixed='Fit 17/09 T1 plus new train; validate 18/09 T1 plus new validation, equal group score',
        mixed_reference='raw compensated force; equal total training weight per trial',
        no_new_threshold_optimized_on_holdout=True))
    diag_selected=min([c for c in selection if c['family']=='diagonal'],
                      key=lambda c:c['validation']['rmse_vector_n'])
    dev=z['train']|z['validation']
    gains=dict(runtime=np.eye(6), local_full_scaled_prior=np.array(winner['correction_gain']),
               local_diagonal_scaled_prior=audit.fit_gain(z['tau'][dev],z['op'][dev],z['ref'][dev],
                                                          'diagonal',diag_selected['alpha']))
    uniform_selection=[]
    for family in ['diagonal','full']:
        rows=[]
        for alpha in [.01,.1,1.,10.]:
            mask=z['train']
            gain=fit_uniform_prior([(z['tau'][mask],z['op'][mask],z['ref'][mask])],family,alpha)
            mask=z['validation']
            row=dict(family=family,alpha=alpha,validation=audit.metrics(
                audit.predict(gain,z['tau'][mask],z['op'][mask]),z['ref'][mask]))
            rows.append(row);uniform_selection.append(row)
        chosen=min(rows,key=lambda r:r['validation']['rmse_vector_n'])
        gains['local_'+family+'_uniform_prior']=fit_uniform_prior(
            [(z['tau'][dev],z['op'][dev],z['ref'][dev])],family,chosen['alpha'])
    results={}
    for name,gain in gains.items():
        pred=audit.predict(gain,z['tau'],z['op'])
        common=(np.linalg.norm(z['ref'],axis=1)>=4)&(np.linalg.norm(z['runtime'],axis=1)>=4)&(np.linalg.norm(pred,axis=1)>=4)&z['internal_holdout']
        rn=np.linalg.norm(z['ref'],axis=1)
        directions={}
        for k,axis in enumerate('XYZ'):
            for sign,sgn in [('+',1),('-',-1)]:
                mask=(rn>=4)&(sgn*z['ref'][:,k]>=np.cos(np.pi/6)*rn)
                directions[axis+sign]=audit.metrics(pred[mask],z['ref'][mask])
        results[name]=dict(gain=gain.tolist(),gain_singular_values=np.linalg.svd(gain,compute_uv=False).tolist(),
            internal_holdout=audit.metrics(pred[z['internal_holdout']],z['ref'][z['internal_holdout']]),
            per_direction_all_development_descriptive=directions,
            common_angle_runtime=audit.metrics(z['runtime'][common],z['ref'][common]),
            common_angle_candidate=audit.metrics(pred[common],z['ref'][common]),
            historical=transfer(gain,historical))
    audit.save(args.output/'local_candidates.json',results)
    audit.save(args.output/'uniform_prior_selection.json',uniform_selection)
    # Retrospective attempt to retain older T1 behavior while using the new trial.
    train=[(d['tau'],d['op'],d['raw_reference']) for d in historical if d['date']=='20260917']
    train.append((z['tau'][z['train']],z['op'][z['train']],z['ref'][z['train']]))
    validation=[d for d in historical if d['date']=='20260918']
    def mixed_score(gain):
        mask=z['validation']
        new=audit.metrics(audit.predict(gain,z['tau'][mask],z['op'][mask]),z['ref'][mask])['rmse_vector_n']
        old=float(np.mean([audit.metrics(audit.predict(gain,d['tau'],d['op']),d['raw_reference'])['rmse_vector_n'] for d in validation]))
        return dict(score=.5*(new+old),new_validation_rmse_n=new,old_validation_macro_rmse_n=old)
    mixed=[dict(name='runtime',**mixed_score(np.eye(6)))]
    for family in ['diagonal','full']:
        for alpha in [.01,.1,1.,10.]:
            gain=fit_uniform_prior(train,family,alpha)
            mixed.append(dict(name=f'{family}_{alpha}',gain=gain.tolist(),**mixed_score(gain)))
    audit.save(args.output/'mixed_development.json',mixed)
    files=[Path(__file__),audit.ROOT/'audit_hc_t1_sidecar.py',
           audit.ROOT/'tests/test_hc_t1_sidecar.py',audit.ROOT/'src/hc10dtp_bringup/scripts/local_ik_solver.py',
           *[args.analysis/name for name in ['audit.json','protocol.json','candidate_frozen.json','paired_samples.npz','runtime_snapshot.json']],
           *[args.history/name for name in ['paired_samples.npz','trial_metadata.json','manifest.json','provenance.json']]]
    audit.save(args.output/'provenance.json',dict(
        files=[dict(path=str(p),sha256=audit.digest(p)) for p in files], verified_historical_sources=used_sources,
        historical_cache_limit='Cache hashed now, source hashes checked; cache not regenerated in this review'))
    for source in [Path(__file__),audit.ROOT/'audit_hc_t1_sidecar.py']:
        (args.output/source.name).write_text(source.read_text())
    decision=dict(status='KEEP_RUNTIME_SHADOW_NO_CANDIDATE_APPROVED',
        local_winner='local_full_scaled_prior',
        local_winner_internal_rmse_n=results['local_full_scaled_prior']['internal_holdout']['rmse_vector_n'],
        runtime_internal_rmse_n=results['runtime']['internal_holdout']['rmse_vector_n'],
        reasons=[
            'Local winner improves within-session error but worsens historical T1 macro error',
            'Scaled prior permits a large weakly-observed torque gain; uniform prior does not restore transfer',
            'No Z-minus force samples in the declared >=4 N / 30 degree cone',
            'Internal validation and holdout contain no Z-plus cone samples either',
            'Post baseline has no M310: end-to-end torque drift cannot be assessed',
            'Mixed fits trade off validation groups or give only ~0.2 percent combined improvement',
            'Z-plus descriptive error increases from 4.881 N to 5.455 N for the local winner',
            'No independent new-session test or accepted absolute accuracy requirement yet'],
        candidate_file_warning='analysis_v1/candidate_frozen.json is a saved fit, NOT a deployment recommendation',
        next_collection='Short targeted Z+/Z- with release and marked M310 baseline before/after once reader stable; development only',
        no_robot_commands=True,no_runtime_changes=True)
    audit.save(args.output/'decision.json',decision)
    print(json.dumps(dict(local={k:dict(internal_rmse=v['internal_holdout']['rmse_vector_n'],
                                       historical_macro_rmse=v['historical']['macro_rmse_n']) for k,v in results.items()},
                          mixed=[{k:v for k,v in row.items() if k!='gain'} for row in mixed],
                          decision=decision['status']),indent=2))


if __name__=='__main__':
    main()
