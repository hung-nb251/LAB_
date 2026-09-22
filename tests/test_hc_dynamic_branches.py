"""Numerical checks for offline calibration, with independent synthetic ground truth."""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from analyze_hc_dynamic_branches import fit_model, interp, metrics, recover, LocalIKSolver


def test_wrench_reconstruction_and_shift():
    solver=LocalIKSolver()
    q=np.array([1.2,.3,-.4,.2,-.8,.1])
    j=solver.compute_jacobian(q)
    force=np.array([12.,-3.,5.])
    sensor_moment=np.array([.4,-.2,.1])
    r=np.array([0.,.12,0.])
    wrench=np.r_[force,sensor_moment+np.cross(r,force)]
    # Independent power check using velocity of the sensor point.
    qdot=np.array([.1,-.2,.05,.03,-.06,.02])
    twist=j@qdot
    sensor_v=twist[:3]+np.cross(twist[3:],r)
    sensor_power=force@sensor_v+sensor_moment@twist[3:]
    tau=j.T@wrench
    np.testing.assert_allclose(tau@qdot,sensor_power,atol=1e-12)
    np.testing.assert_allclose(recover(tau[None],j[None],1e-8)[0],wrench,atol=1e-9)


def test_rotation_and_common_gain_identified_without_reflection():
    rng=np.random.default_rng(42)
    x=rng.normal(size=(200,3))*10
    r=Rotation.from_euler('xyz',[.2,-.4,.7]).as_matrix()
    y=1.7*x@r.T
    d={'f':x,'w':np.column_stack((y,np.zeros_like(y)))}
    model=fit_model([d],'force','rotation_gain')
    np.testing.assert_allclose(np.asarray(model['coef']),1.7*r.T,atol=1e-12)
    assert np.linalg.det(np.asarray(model['coef']))>0


def test_interpolation_does_not_hide_gaps_or_extrapolate():
    values,valid=interp(np.array([-.1,.05,.5,1.05,1.2]),np.array([[0.,0.],[.1,1.],[1.,10.],[1.1,11.]]),.2)
    assert valid.tolist()==[False,True,False,True,False]
    np.testing.assert_allclose(values[[1,3],0],[.5,10.5])


def test_angle_coverage_counts_low_predictions_as_failures():
    ref=np.array([[10.,0.,0.],[10.,0.,0.],[1.,0.,0.]])
    pred=np.array([[10.,0.,0.],[0.,0.,0.],[1.,0.,0.]])
    m=metrics(pred,ref)
    assert m['reference_ge4']==2 and m['angle_n']==1
    assert m['angle_coverage']==.5 and m['direction_failure_fraction']==.5
    assert m['rmse_vector_n']>0
