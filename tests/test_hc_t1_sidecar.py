"""Data isolation, time alignment, and physical replay for T1 sidecar analysis."""
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1] / 'scripts'))
import audit_hc_t1_sidecar as audit


def test_interpolation_never_bridges_loss_or_invalid_reference():
    out,valid=audit.interpolate([-.1,0,.05,.2,.5,1.1],[0,.1,.3,1],[[0],[1],[np.nan],[10]],.15)
    assert valid.tolist()==[False,True,True,False,False,False]
    np.testing.assert_allclose(out[2],[.5])


def test_deduplicate_preroll_without_collapsing_distinct_events(tmp_path):
    event=dict(kind='marker',receipt_ns=3,data='x')
    a=tmp_path/'a';b=tmp_path/'b'
    a.write_text(json.dumps(event)+'\n')
    b.write_text(json.dumps(event)+'\n'+json.dumps(dict(event,data='y'))+'\n')
    events,meta=audit.read_events([a,b])
    assert len(events)==2 and meta[1]['duplicate_preroll_events']==1


def test_force_reconstruction_uses_recorded_rotation_once():
    r=np.array([[[0.,-1,0],[1,0,0],[0,0,1]]])
    f=np.array([[2.,3,4]])
    bias=np.array([[1.,2,3]])
    mass=np.array([1.126])
    gravity=np.array([[0,0,-9.80665*mass[0]]])
    raw=np.einsum('nij,nj->ni',r,f+gravity)+bias
    np.testing.assert_allclose(audit.raw_force(raw,r,bias,mass),f)


def test_gain_design_matches_physical_forward_map():
    rng=np.random.default_rng(9)
    tau=rng.normal(size=(20,6));op=rng.normal(size=(20,3,6));g=rng.normal(size=(6,6))
    np.testing.assert_allclose(np.einsum('nip,p->ni',audit.design(tau,op,'full'),g.ravel()),
                               audit.predict(g,tau,op),atol=1e-14)
    diag=np.diag(np.diag(g))
    np.testing.assert_allclose(np.einsum('nip,p->ni',audit.design(tau,op,'diagonal'),np.diag(g)),
                               audit.predict(diag,tau,op),atol=1e-14)


def test_fit_recovers_known_gain_without_using_holdout_labels():
    rng=np.random.default_rng(10)
    tau=rng.normal(size=(200,6));op=rng.normal(size=(200,3,6))
    truth=np.diag([1.1,.9,1.2,.8,1.05,.95])
    labels=audit.predict(truth,tau,op)
    masks,_=audit.splits(np.arange(200),0,199)
    train=masks['train'];hold=masks['internal_holdout']
    gain=audit.fit_gain(tau[train],op[train],labels[train],'diagonal',1e-12)
    np.testing.assert_allclose(gain,truth,atol=1e-10)
    labels[hold]+=1000
    np.testing.assert_allclose(audit.fit_gain(tau[train],op[train],labels[train],'diagonal',1e-12),gain)
    assert not np.any(train & hold)


def test_recovery_matches_runtime_physics():
    from sensorless_force_math import recover_calibrated_wrench
    solver=audit.LocalIKSolver();q=np.array([1.2,.5,.1,.01,-1.1,.3]);j=solver.compute_jacobian(q)
    tau=np.array([1.,2,3,4,5,6])
    result=audit.predict(np.eye(6),tau[None],audit.recovery(j[None],.01))[0]
    np.testing.assert_allclose(result,recover_calibrated_wrench(j,tau).wrench[:3],atol=1e-12)


def test_uniform_prior_keeps_completely_unobservable_gain_at_identity():
    import review_hc_t1_candidates as review
    rng=np.random.default_rng(12)
    tau=rng.normal(size=(100,6))
    op=rng.normal(size=(100,3,6));op[:,:,3]=0
    target=rng.normal(size=(100,3))
    gain=review.fit_uniform_prior([(tau,op,target)],'full',.1)
    np.testing.assert_allclose(gain[:,3],np.eye(6)[:,3],atol=1e-12)


def test_trial_weight_does_not_change_when_a_trial_is_duplicated_in_time():
    import review_hc_t1_candidates as review
    rng=np.random.default_rng(13)
    trials=[(rng.normal(size=(n,6)),rng.normal(size=(n,3,6)),rng.normal(size=(n,3))) for n in [30,80]]
    original=review.fit_uniform_prior(trials,'diagonal',.1)
    repeated=[tuple(np.repeat(x,5,axis=0) for x in trials[0]),trials[1]]
    np.testing.assert_allclose(review.fit_uniform_prior(repeated,'diagonal',.1),original,atol=1e-12)


def test_macro_validation_gives_each_unit_equal_weight():
    import calibrate_hc_t1_combined as combined
    one=(np.ones((10,6)),np.ones((10,3,6)),np.zeros((10,3)))
    two=(np.ones((100,6)),np.ones((100,3,6)),np.ones((100,3))*2)
    score,rows=combined.macro(np.eye(6),[one,two])
    assert score==(rows[0]['rmse_vector_n']+rows[1]['rmse_vector_n'])/2


def test_zero_force_does_not_count_as_six_axis_coverage():
    assert audit.coverage(np.zeros((20,3)))==dict.fromkeys(['X+','X-','Y+','Y-','Z+','Z-'],0)


def test_split_guard_excludes_boundaries_from_every_partition():
    masks,bounds=audit.splits(np.array([0,5,5.5,6,7.,7.5,8,8.5,10]),0,10,guard=.5)
    assert masks['train'].tolist()==[True,True,False,False,False,False,False,False,False]
    assert masks['validation'].tolist()==[False,False,False,False,True,False,False,False,False]
    assert masks['internal_holdout'].tolist()==[False,False,False,False,False,False,False,True,True]
    assert bounds==[0,6,8,10]
