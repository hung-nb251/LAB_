"""Pure causal filter candidates for offline Axia P1. Not imported by ROS/UI.

All operations use the current and earlier samples only. Input is packet-native
sensor force, in N. Compensation/deadband/safety are deliberately external.
"""
from collections import deque
import math

import numpy as np


PRESETS = {
    'raw': dict(median=1, smoother='none'),
    'median5_raw': dict(median=5, smoother='ema', alpha=1.),
    'median5_ema01': dict(median=5, smoother='ema', alpha=.1),
    'median3_ema035': dict(median=3, smoother='ema', alpha=.35),
    'median3_adaptive': dict(median=3, smoother='adaptive', min_cutoff_hz=1.5,
                             max_cutoff_hz=15., innovation_span_n=.5, noise_multiplier=4.),
    'hampel_confirm_adaptive': dict(median=1, despiker='hampel_confirm', smoother='adaptive',
                                    min_cutoff_hz=1.5, max_cutoff_hz=15., innovation_span_n=.5,
                                    noise_multiplier=4., hampel_window=7, hampel_sigma=4.,
                                    hampel_floor_n=.15, confirm_samples=2),
    'median3_one_euro': dict(median=3, smoother='one_euro', min_cutoff_hz=1.,
                             beta=.15, derivative_cutoff_hz=1., max_cutoff_hz=20.),
    'median3_lowpass2_6hz': dict(median=3, smoother='critical2', cutoff_hz=6.),
}


def lowpass_alpha(cutoff_hz, dt):
    return 1. / (1. + 1. / (2. * math.pi * cutoff_hz * dt))


def radial_deadband(value, threshold=4.):
    value=np.asarray(value,float)
    n=np.linalg.norm(value,axis=-1,keepdims=True)
    return value*np.maximum(0.,1.-threshold/np.maximum(n,1e-12))


class CausalForceFilter:
    """Timestamp-aware candidates; dt-independent EMA presets match existing UI.

    Invalid packets produce None and reset state (never emit an old force as
    fresh). A gap >200 ms also resets. This is a replay state policy, not a
    replacement for the controller watchdog. Startup cannot reject a spike
    when there is no history; benchmark startup separately from steady state.
    """
    def __init__(self, preset, gap_reset_sec=.2):
        self.config=dict(PRESETS[preset] if isinstance(preset,str) else preset)
        self.gap_reset_sec=gap_reset_sec
        self.reset()

    def reset(self):
        self.buffer=deque(maxlen=self.config.get('median',1))
        self.history=deque(maxlen=self.config.get('hampel_window',7))
        self.differences=deque(maxlen=21)
        self.y=self.previous=self.pending=self.last_t=None
        self.derivative=np.zeros(3); self.velocity=np.zeros(3)
        self.pending_count=0; self.diagnostic={}

    def update(self, value, timestamp):
        x=np.asarray(value,float)
        if x.shape!=(3,) or not np.isfinite(x).all() or not math.isfinite(timestamp):
            self.reset(); return None
        if self.last_t is not None and timestamp<=self.last_t:
            self.reset(); return None
        if self.last_t is not None and timestamp-self.last_t>self.gap_reset_sec:
            self.reset()
        dt=timestamp-self.last_t if self.last_t is not None else .01
        self.last_t=float(timestamp); c=self.config; rejected=False
        self.buffer.append(x.copy())
        pre=np.median(self.buffer,axis=0) if len(self.buffer)>=3 else x.copy()
        if c.get('despiker')=='hampel_confirm' and len(self.history)>=3:
            h=np.asarray(self.history); center=np.median(h,axis=0)
            sigma=1.4826*np.median(np.abs(h-center),axis=0)
            limit=np.maximum(c['hampel_floor_n'],c['hampel_sigma']*sigma)
            outlier=bool(np.any(np.abs(x-center)>limit))
            if outlier:
                consistent=(self.pending is not None and
                            np.linalg.norm(x-self.pending)<=max(.3,.25*np.linalg.norm(x-center)))
                self.pending_count=self.pending_count+1 if consistent else 1
                self.pending=x.copy()
                if self.pending_count<c['confirm_samples']:
                    pre=self.previous.copy(); rejected=True
                else:
                    # A persistent change is accepted, not permanently trapped as an outlier.
                    self.history.clear(); self.pending=None; self.pending_count=0
            else:
                self.pending=None; self.pending_count=0
        if not rejected:
            self.history.append(pre.copy())
        if self.y is None:
            self.y=pre.copy(); self.previous=pre.copy()
        delta=pre-self.previous
        self.differences.append(delta)
        cutoff=None; alpha=1.
        smoother=c['smoother']
        if smoother=='ema':
            alpha=c['alpha']
        elif smoother=='adaptive':
            dif=np.asarray(self.differences)
            mad=np.median(np.abs(dif-np.median(dif,axis=0)),axis=0)
            # Robust difference scatter rejects constant slope, unlike raw signal variance.
            noise=max(.01,float(np.linalg.norm(1.4826*mad/math.sqrt(2))))
            innovation=float(np.linalg.norm(pre-self.y))
            blend=float(np.clip((innovation-c['noise_multiplier']*noise)/c['innovation_span_n'],0,1))
            cutoff=c['min_cutoff_hz']+blend*(c['max_cutoff_hz']-c['min_cutoff_hz'])
            alpha=lowpass_alpha(cutoff,dt)
        elif smoother=='one_euro':
            da=lowpass_alpha(c['derivative_cutoff_hz'],dt)
            self.derivative+=da*(delta/dt-self.derivative)
            # Vector extension: one common alpha avoids independent axis gains.
            cutoff=min(c['max_cutoff_hz'],c['min_cutoff_hz']+c['beta']*np.linalg.norm(self.derivative))
            alpha=lowpass_alpha(cutoff,dt)
        elif smoother=='critical2':
            # Critically damped second-order lowpass, specified by -3 dB cutoff.
            # Exact zero-order-hold state transition, stable for variable dt.
            omega=2*math.pi*c['cutoff_hz']/math.sqrt(math.sqrt(2)-1)
            error=self.y-pre; temp=self.velocity+omega*error; decay=math.exp(-omega*dt)
            self.y=pre+(error+temp*dt)*decay
            self.velocity=(self.velocity-omega*temp*dt)*decay
        if smoother!='critical2':
            self.y+=alpha*(pre-self.y)
        self.previous=pre.copy()
        self.diagnostic=dict(despiked_sensor_n=pre.copy(),alpha=float(alpha),
                             cutoff_hz=None if cutoff is None else float(cutoff),rejected=rejected)
        return self.y.copy()


def replay(preset,times,values,stages=False):
    f=CausalForceFilter(preset)
    y=np.full_like(values,np.nan,dtype=float)
    pre=np.full_like(values,np.nan,dtype=float); rejected=np.zeros(len(values),bool)
    for i,(t,x) in enumerate(zip(times,values)):
        out=f.update(x,float(t))
        if out is not None:
            y[i]=out; pre[i]=f.diagnostic['despiked_sensor_n']; rejected[i]=f.diagnostic['rejected']
    return (y,pre,rejected) if stages else y
