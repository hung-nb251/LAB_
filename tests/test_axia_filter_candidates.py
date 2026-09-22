"""P1 causal filtering checks; independent of ROS and measured log artifacts."""
import ast
from collections import deque
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT / 'scripts'))
from axia_filter_candidates import CausalForceFilter, PRESETS, radial_deadband, replay


@pytest.mark.parametrize('preset',list(PRESETS))
def test_future_samples_cannot_change_prefix(preset):
    rng=np.random.default_rng(100)
    t=np.arange(100)*.01; x=rng.normal(size=(100,3))
    full=replay(preset,t,x)
    x[60:]+=100
    changed=replay(preset,t,x)
    np.testing.assert_array_equal(full[:60],changed[:60])


@pytest.mark.parametrize('alpha',[1.,.1])
def test_existing_presets_match_actual_ui_class(alpha):
    tree=ast.parse((ROOT/'scripts/axia_sensor_ui.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ForceFilter')
    env={'np':np,'deque':deque}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'ForceFilter','exec'),env)
    original=env['ForceFilter'](median_size=5,ema_alpha=alpha)
    x=np.random.default_rng(11).normal(size=(120,3)); t=np.arange(len(x))*.01
    expected=np.array([original.apply(v.copy()).copy() for v in x])
    actual=replay('median5_raw' if alpha==1 else 'median5_ema01',t,x)
    np.testing.assert_allclose(actual,expected,atol=1e-14)


@pytest.mark.parametrize('preset',list(PRESETS))
def test_constant_input_and_invalid_packet_reset(preset):
    f=CausalForceFilter(preset)
    for t in np.arange(0,1,.01):
        np.testing.assert_allclose(f.update([2,3,4],t),[2,3,4])
    assert f.update([float('nan'),0,0],1.) is None
    np.testing.assert_allclose(f.update([8,9,10],1.01),[8,9,10])
    np.testing.assert_allclose(f.update([1,2,3],2.),[1,2,3])
    assert f.update([1,2,3],2.) is None


def test_hampel_rejects_single_spike_but_accepts_sustained_step():
    t=np.arange(200)*.01; x=np.zeros((200,3)); x[50]=[20,0,0]; x[100:,0]=8
    y=replay('hampel_confirm_adaptive',t,x)
    assert np.max(np.abs(y[50:90]))<1e-9
    assert y[110,0]>7.5


def test_radial_deadband_preserves_direction_and_is_separate():
    x=np.array([[3.,4.,0.],[1.,1.,1.]])
    np.testing.assert_allclose(radial_deadband(x),[[.6,.8,0.],[0,0,0]])
    np.testing.assert_array_equal(replay('raw',[0,.01],x),x)


def test_second_order_step_stable_with_variable_dt():
    t=np.cumsum(np.tile([.005,.015,.01],100))
    x=np.zeros((len(t),3)); x[20:]=[8,-3,1]
    y=replay('median3_lowpass2_6hz',t,x)
    assert np.isfinite(y).all()
    assert np.min(y[:,0])>=-1e-10 and np.max(y[:,0])<=8+1e-10
    np.testing.assert_allclose(y[-1],[8,-3,1],atol=1e-8)


def test_offline_impulse_annotation_detects_spike_not_persistent_step():
    from benchmark_axia_p1 import impulse_like_indices
    x=np.zeros((200,3)); x[40,0]=10.; x[100:,0]=8.
    ix,_,_=impulse_like_indices(x,np.ones(len(x),bool))
    assert ix.tolist()==[40]


def test_onset_metric_reports_first_sustained_crossing():
    from benchmark_axia_p1 import first_sustained
    t=np.arange(20)*.01; x=np.zeros(20); x[3]=10.; x[10:]=8.
    assert first_sustained(t,x,4,0,.2)==.1
    assert first_sustained(t,x,12,0,.2) is None
