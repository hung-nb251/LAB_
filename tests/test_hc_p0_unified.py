"""Numerical and data-isolation checks for the offline P0 analysis."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import calibrate_hc_p0_unified as p0


def test_interpolation_handles_duplicates_gaps_and_no_extrapolation():
    t=np.array([-.1,.05,.5,1.05,1.2])
    values,valid=p0.interpolate(t,[[0,0],[0,99],[.1,1],[1,10],[1.1,11]])
    assert valid.tolist()==[False,True,False,True,False]
    np.testing.assert_allclose(values[[1,3],0],[.5,10.5])
    _,valid=p0.interpolate(t,[[0,0]])
    assert not valid.any()


def test_baseline_fit_is_invariant_to_arbitrary_session_offsets():
    rng=np.random.default_rng(91)
    coef=rng.normal(size=(6,6))
    bs=[]
    for i in range(3):
        for q in rng.normal(size=(20,6)):
            bs.append(dict(session=str(i),q=q.tolist(),tau=(q@coef+100*i).tolist()))
    x,y,w=p0.baseline_design(bs)
    np.testing.assert_allclose(p0.ridge(x,y,w,1e-10),coef,atol=1e-8)
    assert abs(w.sum()-1)<1e-12


def test_numerical_rank_does_not_hide_unexcited_joint_direction():
    q=np.eye(6); q[-1,-1]=1e-7
    s=p0.spectrum(q)
    assert s['rank']==6
    assert s['practical_rank_1pct']==5


def test_gain_fit_recovers_independent_force_and_ignores_test_baseline():
    rng=np.random.default_rng(48)
    q=np.array([1.2,.3,-.4,.2,-.8,.1])
    j=p0.SOLVER.compute_jacobian(q)
    w=rng.normal(size=(250,6))*np.array([10,10,10,1,1,1])
    tau=w@j
    gain=np.eye(6)+rng.normal(scale=.02,size=(6,6))
    d=dict(meta=dict(trial='train'),tau=tau@np.linalg.inv(gain),dq=np.zeros((len(w),6)),
           j=np.repeat(j[None],len(w),axis=0),w=w,force_mask=np.ones(len(w),bool))
    model=p0.fit([d],[],'constant_full',alpha=1e-10)
    np.testing.assert_allclose(np.asarray(model['coef'])[:6],gain,atol=1e-6)
    assert model['train_trials']==['train'] and model['baseline_trials']==[]
    new_w=rng.normal(size=(10,6))
    test=dict(tau=(new_w@j)@np.linalg.inv(gain),dq=np.zeros((10,6)),
              j=np.repeat(j[None],10,axis=0))
    # Compare the common regularized inverse, not an undamped ideal wrench.
    np.testing.assert_allclose(p0.prediction(model,test),p0.base.recover(new_w@j,test['j'])[:,:3],atol=1e-6)


def test_trial_macro_score_does_not_allow_long_log_to_dominate():
    rows=[dict(metrics=dict(n=10000,rmse_vector_n=1)),dict(metrics=dict(n=10,rmse_vector_n=9))]
    out=p0.aggregate(rows)
    assert out['macro_rmse_n']==5
    assert out['pooled_rmse_n']<1.1


def baseline_fixture(markers, connected=True):
    t=np.arange(0,10,.01)
    q=np.zeros((len(t),6))
    rt=np.arange(.1,9.9,.1)
    src=dict(regs={a:[(float(t),0.,i) for i,t in enumerate(rt)] for a in range(310,316)},
             complete=set(range(len(rt))),joints=np.column_stack((t,q)),raw=np.column_stack((t,q)),
             markers=markers,states=[(0,'RUNNING')],running=[(0,True)],
             health={'connected':[(0,connected)],'calibrated':[(0,True)]},static=False,edges={})
    row=dict(trial='synthetic',session='s',route='home',path='synthetic',mode='GROUND_TRUTH',
             duration_s=10.,arguments={'pose':'home'},inclusion=[],exclusion=[],baseline_segments=[])
    return row,src


def test_zero_force_and_stationarity_do_not_invent_no_contact_label():
    row,src=baseline_fixture([])
    trial,bs=p0.prepare_trial(row,src)
    assert trial is None and bs==[]
    assert 'missing_healthy_stationary_initial_baseline' in row['exclusion']


def test_explicit_stationary_gt_baseline_allowed_while_running():
    row,src=baseline_fixture([(0,'baseline'),(5,'xplus_1')])
    _,bs=p0.prepare_trial(row,src)
    assert len(bs)==1 and bs[0]['name']=='baseline'


def test_disconnected_reference_cannot_supply_a_baseline():
    row,src=baseline_fixture([(0,'baseline'),(5,'xplus_1')],connected=False)
    _,bs=p0.prepare_trial(row,src)
    assert bs==[]
