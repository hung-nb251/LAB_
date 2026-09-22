#!/usr/bin/env python3
"""P2 — offline A/B of reference-governor candidates for the co-carry pipeline.

Pure simulation.  Never imports ROS, never launches anything, never sends a
robot command.  Every candidate is evaluated through the *same* downstream
model so that a difference can be attributed to the governor:

    governor -> admittance -> command-lead clamp -> streamer profile limiter

The streamer stage reimplements the position math of ``_smooth_pose`` in
``src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py`` (trapezoidal
desired speed, jerk-limited acceleration, acceleration cap, velocity cap).
Joint-space IK, soft joint limits and MotoROS2 queue admission are NOT
modelled; the measured queue ACK latency is applied as a feedback transport
delay so the simulation is not optimistically fast.

    python3 scripts/simulate_hc_p2_reference_candidates.py --output <new_dir>
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

DEFAULT_WORKSPACE = Path('/home/hungnb/cocarry_ws')

CONTROL_DT = 1.0 / 15.0
# Values the REAL launch passes, not the YAML/module defaults:
#   --max-vel 0.25 --max-accel 1.00 --max-jerk 10.0
#   admittance max_virtual_velocity_mps 0.25 / max_virtual_acceleration_mps2 1.00
STREAM_MAX_VEL = 0.25
STREAM_MAX_ACCEL = 1.00
STREAM_MAX_JERK = 10.0
# Median queue execution lag measured in the audit: the profile limiter
# restarts from the accepted queue pose, which already runs this far ahead of
# the measured EE, so this much of the command-lead budget is unavailable.
QUEUE_EXECUTION_LAG_M = 0.029
COMMAND_LEAD_CAP_M = 0.04          # launch override, see the audit script
NOMINAL_LEAD_CAP_M = 0.05
# Measured in 20260919_p2_pipeline_audit_v2: queue ACK latency median ~14.6 ms,
# p95 ~29 ms.  One control tick of transport plus the ACK median is used as the
# default feedback delay of the simulated plant.
FEEDBACK_DELAY_SEC = CONTROL_DT + 0.0146


def load_prediction_reference(workspace: Path):
    pkg = workspace / 'src/cocarry_admittance_control/cocarry_admittance_control'
    sys.path.insert(0, str(pkg))
    import prediction_reference as pr_mod
    import admittance as adm_mod
    return pr_mod, adm_mod


# ── Streamer profile limiter (position math of _smooth_pose) ──────────────

def smooth_step(current, target, prev_vel, prev_accel, dt,
                max_vel=STREAM_MAX_VEL, max_accel=STREAM_MAX_ACCEL,
                max_jerk=STREAM_MAX_JERK, continuous=True):
    """One tick of the streamer's Cartesian profile limiter.

    Mirrors the runtime order: trapezoidal desired speed toward the target,
    jerk limit on the acceleration change (disabled within 20 mm exactly as in
    the runtime), acceleration norm cap, then velocity norm cap.
    """
    current = np.asarray(current, float)
    target = np.asarray(target, float)
    prev_vel = np.asarray(prev_vel, float)
    prev_accel = np.asarray(prev_accel, float)
    error = target - current
    dist = float(np.linalg.norm(error))
    if dist < 1e-5:
        desired_speed = 0.0
        direction = np.zeros(3)
    else:
        decel_dist = max_vel ** 2 / (2.0 * max_accel)
        desired_speed = (math.sqrt(2.0 * max_accel * dist) if dist < decel_dist
                         else max_vel)
        desired_speed = min(desired_speed, max_vel)
        if continuous:
            desired_speed = min(desired_speed, dist / dt)
        direction = error / dist
    desired_vel = desired_speed * direction
    target_accel = (desired_vel - prev_vel) / dt
    max_da = float('inf') if dist < 0.020 else max_jerk * dt
    accel = np.array(target_accel, dtype=float)
    for i in range(3):
        da = accel[i] - prev_accel[i]
        if abs(da) > max_da:
            accel[i] = prev_accel[i] + math.copysign(max_da, da)
    accel_mag = float(np.linalg.norm(accel))
    if accel_mag > max_accel:
        accel = accel * (max_accel / accel_mag)
    vel = prev_vel + accel * dt
    speed = float(np.linalg.norm(vel))
    if speed > max_vel:
        vel = vel * (max_vel / speed)
    return current + vel * dt, vel, accel


# ── Reference governors ──────────────────────────────────────────────────

class BaselineGovernor:
    """Current runtime: first-order lag tau with a capped velocity lead."""

    name = 'baseline_tau0.4_lead0.15'
    description = ('Runtime configuration: PredictionReference(tau=0.4 s, '
                   'lead=0.15 s, lead cap 0.02 m) from the live working tree.')

    def __init__(self, pr_mod, tau=0.4, lead=0.15, max_lead_m=0.02):
        self._ref = pr_mod.PredictionReference(tau, lead_sec=lead, max_lead_m=max_lead_m)
        self._started = False

    def reset(self, position, now):
        self._ref.reset(np.asarray(position, float), now)
        self._started = True

    def step(self, desired, now, dt):
        return self._ref.step(np.asarray(desired, float), now)


class FastBoundedGovernor:
    """Faster first-order lag with explicit rate and acceleration bounds.

    A shorter tau alone would let a GRU jump through, so the output is bounded
    by an explicit velocity and acceleration limit instead of relying on the
    downstream streamer to absorb it.
    """

    name = 'fast_tau0.15_rate_accel_bounded'
    description = ('First-order lag tau=0.15 s, no velocity lead, with an '
                   'explicit 0.25 m/s and 1.00 m/s^2 bound on the conditioned '
                   'nominal itself.')

    def __init__(self, pr_mod, tau=0.15, max_vel=0.25, max_accel=1.00):
        self._ref = pr_mod.PredictionReference(tau, lead_sec=0.0)
        self._max_vel = max_vel
        self._max_accel = max_accel
        self._pos = None
        self._vel = np.zeros(3)

    def reset(self, position, now):
        position = np.asarray(position, float)
        self._ref.reset(position, now)
        self._pos = position.copy()
        self._vel = np.zeros(3)

    def step(self, desired, now, dt):
        raw = self._ref.step(np.asarray(desired, float), now)
        want_vel = (raw - self._pos) / dt
        dv = want_vel - self._vel
        dv_norm = float(np.linalg.norm(dv))
        if dv_norm > self._max_accel * dt:
            dv = dv * (self._max_accel * dt / dv_norm)
        vel = self._vel + dv
        speed = float(np.linalg.norm(vel))
        if speed > self._max_vel:
            vel = vel * (self._max_vel / speed)
        self._vel = vel
        self._pos = self._pos + vel * dt
        return self._pos.copy()


class SecondOrderGovernor:
    """Critically damped second-order reference model, optionally jerk limited.

    A critically damped second order system has a continuous velocity and a
    bounded acceleration by construction, so the profile shaping happens once,
    here, instead of being split between a first-order lag and the streamer.
    """

    def __init__(self, settling_sec=0.45, jerk_limit=None, feed_forward=0.0,
                 max_vel=0.25, max_accel=1.00):
        # For a critically damped system the 5% settling time is ~4.75/wn.
        self._wn = 4.75 / float(settling_sec)
        self._jerk_limit = jerk_limit
        self._feed_forward = float(feed_forward)
        self._max_vel = max_vel
        self._max_accel = max_accel
        self._pos = None
        self._vel = np.zeros(3)
        self._accel = np.zeros(3)
        self.name = (f'second_order_ts{settling_sec:g}'
                     + (f'_jerk{jerk_limit:g}' if jerk_limit else '')
                     + (f'_ff{feed_forward:g}' if feed_forward else ''))
        self.description = (
            f'Critically damped second-order tracker, 5% settling {settling_sec} s '
            f'(wn={self._wn:.2f} rad/s)'
            + (f', jerk limited to {jerk_limit} m/s^3' if jerk_limit else '')
            + (f', velocity feed-forward {feed_forward} s' if feed_forward else '')
            + '. Single shaping stage; the streamer limiter stays as a bound only.')

    def reset(self, position, now):
        self._pos = np.asarray(position, float).copy()
        self._vel = np.zeros(3)
        self._accel = np.zeros(3)

    def step(self, desired, now, dt):
        desired = np.asarray(desired, float)
        accel = self._wn ** 2 * (desired - self._pos) - 2.0 * self._wn * self._vel
        if self._jerk_limit is not None:
            d_accel = accel - self._accel
            limit = self._jerk_limit * dt
            norm = float(np.linalg.norm(d_accel))
            if norm > limit:
                d_accel = d_accel * (limit / norm)
            accel = self._accel + d_accel
        accel_norm = float(np.linalg.norm(accel))
        if accel_norm > self._max_accel:
            accel = accel * (self._max_accel / accel_norm)
        self._accel = accel
        vel = self._vel + accel * dt
        speed = float(np.linalg.norm(vel))
        if speed > self._max_vel:
            vel = vel * (self._max_vel / speed)
        self._vel = vel
        self._pos = self._pos + vel * dt
        out = self._pos + self._feed_forward * self._vel
        # The feed-forward term must not overshoot the commanded nominal.
        residual = desired - self._pos
        correction = np.clip(out - self._pos, np.minimum(0.0, residual),
                             np.maximum(0.0, residual))
        return self._pos + correction


def build_candidates(pr_mod):
    return [
        BaselineGovernor(pr_mod),
        FastBoundedGovernor(pr_mod),
        SecondOrderGovernor(settling_sec=0.45),
        SecondOrderGovernor(settling_sec=0.45, jerk_limit=STREAM_MAX_JERK,
                            feed_forward=0.08),
    ]


# ── Plant simulation shared by every candidate ───────────────────────────

def simulate(governor, desired_seq, t_seq, force_seq=None, adm_mod=None,
             feedback_delay_sec=FEEDBACK_DELAY_SEC, start=None):
    """Run governor -> admittance -> lead clamp -> streamer limiter.

    ``force_seq`` may be None (pure position tracking, isolating the
    governor+streamer bandwidth) or the logged effective force (adds the real
    admittance response).  Feedback used by the command-lead clamp is delayed
    by ``feedback_delay_sec`` to represent queue/ACK/FK transport.
    """
    n = len(t_seq)
    start = np.asarray(desired_seq[0] if start is None else start, float)
    governor.reset(start, t_seq[0])
    admittance = None
    if force_seq is not None and adm_mod is not None:
        admittance = adm_mod.CartesianAdmittance(
            np.ones(3), np.full(3, 4.47213595), np.full(3, 5.0), 0.25, 1.00)
        admittance.reset(np.zeros(3))
    plant = start.copy()
    plant_vel = np.zeros(3)
    plant_accel = np.zeros(3)
    history = [(t_seq[0], plant.copy())]
    out = dict(nominal=np.zeros((n, 3)), reference=np.zeros((n, 3)),
               plant=np.zeros((n, 3)), plant_vel=np.zeros((n, 3)),
               plant_accel=np.zeros((n, 3)), lead_hits=0, nominal_lead_hits=0)
    for i in range(n):
        dt = CONTROL_DT if i == 0 else max(t_seq[i] - t_seq[i - 1], 1e-4)
        # Delayed feedback the controller would have seen at this tick.
        cutoff = t_seq[i] - feedback_delay_sec
        fb = history[0][1]
        for stamp, pos in history:
            if stamp <= cutoff:
                fb = pos
            else:
                break
        nominal = np.asarray(governor.step(desired_seq[i], t_seq[i], dt), float)
        delta = nominal - fb
        dist = float(np.linalg.norm(delta))
        if dist > NOMINAL_LEAD_CAP_M:
            nominal = fb + delta * (NOMINAL_LEAD_CAP_M / dist)
            out['nominal_lead_hits'] += 1
        if admittance is not None:
            error, _, _ = admittance.step(np.asarray(force_seq[i], float), dt)
            reference = nominal + error
        else:
            reference = nominal
        lead = reference - fb
        lead_norm = float(np.linalg.norm(lead))
        if lead_norm > COMMAND_LEAD_CAP_M:
            reference = fb + lead * (COMMAND_LEAD_CAP_M / lead_norm)
            out['lead_hits'] += 1
            if admittance is not None:
                admittance.error = reference - nominal
        plant, plant_vel, plant_accel = smooth_step(
            plant, reference, plant_vel, plant_accel, dt)
        history.append((t_seq[i], plant.copy()))
        history = history[-256:]
        out['nominal'][i] = nominal
        out['reference'][i] = reference
        out['plant'][i] = plant
        out['plant_vel'][i] = plant_vel
        out['plant_accel'][i] = plant_accel
    return out


# ── Metrics ──────────────────────────────────────────────────────────────

def step_metrics(result, t_seq, step_target, start):
    """t90 / peak velocity / jerk / overshoot for a controlled step input."""
    plant = result['plant']
    travel = float(np.linalg.norm(np.asarray(step_target) - np.asarray(start)))
    direction = (np.asarray(step_target) - np.asarray(start)) / travel
    progress = (plant - np.asarray(start)) @ direction
    reached90 = np.flatnonzero(progress >= 0.9 * travel)
    reached95 = np.flatnonzero(progress >= 0.95 * travel)
    speed = np.linalg.norm(result['plant_vel'], axis=1)
    accel = np.linalg.norm(result['plant_accel'], axis=1)
    jerk = np.linalg.norm(np.diff(result['plant_accel'], axis=0), axis=1) / CONTROL_DT
    return dict(
        t90_ms=float((t_seq[reached90[0]] - t_seq[0]) * 1000.0) if reached90.size else None,
        t95_ms=float((t_seq[reached95[0]] - t_seq[0]) * 1000.0) if reached95.size else None,
        peak_speed_mps=float(np.max(speed)),
        peak_accel_mps2=float(np.max(accel)),
        jerk_p95_mps3=float(np.percentile(jerk, 95)) if jerk.size else None,
        jerk_integral=float(np.trapz(jerk, dx=CONTROL_DT)) if jerk.size else None,
        overshoot_m=float(max(0.0, np.max(progress) - travel)),
        command_lead_hits=result['lead_hits'],
        nominal_lead_hits=result['nominal_lead_hits'],
    )


def tracking_metrics(result, desired_seq, t_seq, vel_cap=STREAM_MAX_VEL):
    """How far the plant trails the un-conditioned desired nominal.

    ``unsaturated_*`` restricts the lag estimate to samples where the desired
    nominal itself moves slower than the Cartesian velocity cap, i.e. where the
    plant could in principle keep up.  Without that split, the lag is dominated
    by the human simply moving faster than the robot is allowed to move, and
    every governor looks identical.
    """
    gap = np.linalg.norm(np.asarray(desired_seq) - result['plant'], axis=1)
    speed = np.zeros(len(t_seq))
    dt = np.diff(t_seq)
    speed[1:] = np.linalg.norm(np.diff(np.asarray(desired_seq), axis=0), axis=1) / np.maximum(dt, 1e-6)
    usable = speed >= 0.02
    lag_ms = (gap[usable] / speed[usable] * 1000.0) if usable.any() else np.array([])
    unsat = usable & (speed < vel_cap)
    unsat_lag = (gap[unsat] / speed[unsat] * 1000.0) if unsat.any() else np.array([])
    saturated_fraction = float(np.mean(speed >= vel_cap))
    jerk = np.linalg.norm(np.diff(result['plant_accel'], axis=0), axis=1) / CONTROL_DT
    return dict(
        gap_median_m=float(np.median(gap)),
        gap_p95_m=float(np.percentile(gap, 95)),
        effective_lag_ms_median=float(np.median(lag_ms)) if lag_ms.size else None,
        effective_lag_ms_p95=float(np.percentile(lag_ms, 95)) if lag_ms.size else None,
        unsaturated_lag_ms_median=float(np.median(unsat_lag)) if unsat_lag.size else None,
        unsaturated_lag_ms_p95=float(np.percentile(unsat_lag, 95)) if unsat_lag.size else None,
        unsaturated_samples=int(unsat.sum()),
        desired_speed_over_cap_fraction=saturated_fraction,
        peak_speed_mps=float(np.max(np.linalg.norm(result['plant_vel'], axis=1))),
        jerk_p95_mps3=float(np.percentile(jerk, 95)) if jerk.size else None,
        jerk_integral=float(np.trapz(jerk, dx=CONTROL_DT)) if jerk.size else None,
        command_lead_hits=result['lead_hits'],
        command_lead_hit_fraction=result['lead_hits'] / len(t_seq),
        nominal_lead_hits=result['nominal_lead_hits'],
    )


def load_replay_trial(workspace: Path, trial: str):
    path = workspace / 'cocarry_logs' / f'{trial}.csv'
    with path.open(newline='') as fh:
        rows = list(csv.DictReader(fh))

    def col(name):
        vals = []
        for r in rows:
            try:
                vals.append(float(r.get(name, '')))
            except (TypeError, ValueError):
                vals.append(np.nan)
        return np.array(vals, float)

    t = np.array([int(r['ros_timestamp_ns']) for r in rows], np.int64) / 1e9
    actual = np.stack([col('actual_ee_x'), col('actual_ee_y'), col('actual_ee_z')], 1)
    raw_pred = np.stack([col('raw_predicted_xd_relative'),
                          col('raw_predicted_yd_relative'),
                          col('raw_predicted_zd_relative')], 1)
    force = np.stack([col('f_effective_x'), col('f_effective_y'), col('f_effective_z')], 1)
    keep = (np.isfinite(actual).all(1) & np.isfinite(raw_pred).all(1)
            & np.isfinite(force).all(1))
    t, actual, raw_pred, force = t[keep], actual[keep], raw_pred[keep], force[keep]
    desired = actual[0] + raw_pred          # un-conditioned GRU nominal
    return dict(trial=trial, t=t - t[0], desired=desired, force=force,
                actual=actual, n=len(t))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--workspace', type=Path, default=DEFAULT_WORKSPACE)
    ap.add_argument('--step-m', type=float, default=0.10)
    ap.add_argument('--replay-trials', nargs='*', default=[
        'cocarry_admittance_3d_20260918_172207',
        'cocarry_admittance_3d_20260918_174205',
        'cocarry_admittance_3d_20260919_103009',
    ])
    args = ap.parse_args()
    if args.output.exists():
        raise SystemExit(f'Output dir exists, refusing to overwrite: {args.output}')
    pr_mod, adm_mod = load_prediction_reference(args.workspace)
    args.output.mkdir(parents=True)

    # ── Controlled synthetic step: the metric the real logs cannot provide ──
    n_step = int(6.0 / CONTROL_DT)
    t_step = np.arange(n_step) * CONTROL_DT
    start = np.zeros(3)
    target = np.array([args.step_m, 0.0, 0.0])
    desired_step = np.tile(target, (n_step, 1))
    desired_step[0] = start

    step_rows = []
    for gov in build_candidates(pr_mod):
        res = simulate(gov, desired_step, t_step, start=start)
        m = step_metrics(res, t_step, target, start)
        m.update(candidate=gov.name, description=gov.description)
        step_rows.append(m)
        np.savez_compressed(args.output / f'step_{gov.name}.npz',
                            t=t_step, nominal=res['nominal'],
                            reference=res['reference'], plant=res['plant'],
                            plant_vel=res['plant_vel'])

    # Governor-only step response, with the streamer limiter removed, to show
    # which stage actually sets the bandwidth.
    governor_only = []
    for gov in build_candidates(pr_mod):
        gov.reset(start, t_step[0])
        pos = np.zeros((n_step, 3))
        for i in range(n_step):
            dt = CONTROL_DT
            pos[i] = gov.step(desired_step[i], t_step[i], dt)
        progress = pos @ np.array([1.0, 0.0, 0.0])
        reached = np.flatnonzero(progress >= 0.9 * args.step_m)
        vel = np.diff(pos, axis=0) / CONTROL_DT
        governor_only.append(dict(
            candidate=gov.name,
            t90_ms=float(t_step[reached[0]] * 1000.0) if reached.size else None,
            peak_speed_mps=float(np.max(np.linalg.norm(vel, axis=1))) if vel.size else None,
            overshoot_m=float(max(0.0, np.max(progress) - args.step_m)),
        ))

    # ── Sensitivity of the step response to the downstream caps ────────────
    # Offline sensitivity study only.  Velocity/acceleration limits are
    # operator-approved safety settings; nothing here proposes changing them.
    cap_rows = []
    for vmax, amax in [(0.25, 1.00), (0.25, 1.50), (0.35, 1.00),
                       (0.35, 1.50), (0.45, 1.50)]:
        gov = BaselineGovernor(pr_mod)
        gov.reset(start, t_step[0])
        pos, vel, acc = start.copy(), np.zeros(3), np.zeros(3)
        traj, vels, accs = [], [], []
        ref_state = gov
        fb_hist = [(t_step[0], pos.copy())]
        for i in range(n_step):
            cutoff = t_step[i] - FEEDBACK_DELAY_SEC
            fb = fb_hist[0][1]
            for stamp, p in fb_hist:
                if stamp <= cutoff:
                    fb = p
                else:
                    break
            nominal = np.asarray(ref_state.step(desired_step[i], t_step[i], CONTROL_DT), float)
            lead = nominal - fb
            ln = float(np.linalg.norm(lead))
            if ln > COMMAND_LEAD_CAP_M:
                nominal = fb + lead * (COMMAND_LEAD_CAP_M / ln)
            pos, vel, acc = smooth_step(pos, nominal, vel, acc, CONTROL_DT,
                                        max_vel=vmax, max_accel=amax)
            fb_hist.append((t_step[i], pos.copy()))
            fb_hist = fb_hist[-256:]
            traj.append(pos.copy())
            vels.append(vel.copy())
            accs.append(acc.copy())
        traj = np.array(traj)
        progress = traj @ np.array([1.0, 0.0, 0.0])
        reached = np.flatnonzero(progress >= 0.9 * args.step_m)
        jerk = np.linalg.norm(np.diff(np.array(accs), axis=0), axis=1) / CONTROL_DT
        cap_rows.append(dict(
            max_vel_mps=vmax, max_accel_mps2=amax,
            t90_ms=float(t_step[reached[0]] * 1000.0) if reached.size else None,
            kinematic_floor_ms=float(args.step_m / vmax * 1000.0),
            peak_speed_mps=float(np.max(np.linalg.norm(np.array(vels), axis=1))),
            jerk_p95_mps3=float(np.percentile(jerk, 95)) if jerk.size else None,
        ))

    # ── Command-lead budget sweep (the mechanism that actually throttles) ──
    # Validated against the logs: with L=0.040, Q=0.029, a=1.00 this formula
    # predicts 0.097 m/s and trial 20260919_103009 measured 0.095 m/s.
    # Q depends partly on QUEUE_PREBUFFER_POINTS=3; a shallower prebuffer
    # shortens the schedule the robot is executing behind.
    lead_rows = []
    for queue_lag in (0.029, 0.020):
        for lead_cap in (0.04, 0.05, 0.06, 0.08, 0.10):
            for amax in (1.00, 1.50):
                d_eff = max(lead_cap - queue_lag, 0.0)
                v_cmd = min(STREAM_MAX_VEL,
                            math.sqrt(2.0 * amax * d_eff) if d_eff > 0 else 0.0,
                            d_eff / CONTROL_DT)
                lead_rows.append(dict(
                    queue_execution_lag_m=queue_lag, command_lead_cap_m=lead_cap,
                    max_accel_mps2=amax, effective_profile_distance_m=d_eff,
                    predicted_commanded_speed_mps=v_cmd,
                    speed_ratio_vs_current=(v_cmd / 0.097) if v_cmd else None,
                    velocity_cap_reached=bool(v_cmd >= STREAM_MAX_VEL - 1e-9),
                ))

    # ── Replay on real logged GRU output and logged effective force ────────
    replay_rows = []
    for trial in args.replay_trials:
        try:
            data = load_replay_trial(args.workspace, trial)
        except FileNotFoundError:
            print(f'skip missing trial {trial}', file=sys.stderr)
            continue
        for gov in build_candidates(pr_mod):
            res = simulate(gov, data['desired'], data['t'], force_seq=data['force'],
                           adm_mod=adm_mod, start=data['actual'][0])
            row = tracking_metrics(res, data['desired'], data['t'])
            row.update(candidate=gov.name, trial=trial, samples=data['n'])
            replay_rows.append(row)
            print(f"{trial} {gov.name}: lag_med="
                  f"{row['effective_lag_ms_median']} lead_hits={row['command_lead_hits']}",
                  file=sys.stderr)

    summary = dict(
        synthetic_step=dict(
            step_m=args.step_m, duration_sec=6.0,
            full_chain=step_rows, governor_only=governor_only,
            downstream_cap_sensitivity=cap_rows,
            command_lead_budget_sweep=lead_rows,
            note='Full chain = governor + command-lead clamp + streamer '
                 'profile limiter, with a feedback transport delay of '
                 f'{FEEDBACK_DELAY_SEC*1000:.1f} ms taken from the measured '
                 'queue ACK latency. Joint-space IK, soft joint limits and '
                 'MotoROS2 queue admission are not modelled.',
        ),
        replay=replay_rows,
        model_limits=[
            'No IK, no joint velocity clamp, no queue BUSY/retry admission: a '
            'candidate that looks fast here can still be limited on hardware.',
            'The replay drives every candidate with the SAME logged GRU output '
            'and logged effective force; a different reference would in reality '
            'change the robot motion, hence the GRU input itself. This is an '
            'open-loop comparison, not a closed-loop prediction.',
            'Synthetic step metrics are a design comparison, not a robot safety '
            'certification, and were not validated on hardware.',
        ],
        constants=dict(
            control_dt_sec=CONTROL_DT, stream_max_vel_mps=STREAM_MAX_VEL,
            stream_max_accel_mps2=STREAM_MAX_ACCEL,
            stream_max_jerk_mps3=STREAM_MAX_JERK,
            command_lead_cap_m=COMMAND_LEAD_CAP_M,
            nominal_lead_cap_m=NOMINAL_LEAD_CAP_M,
            feedback_delay_sec=FEEDBACK_DELAY_SEC,
        ),
    )
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))

    with (args.output / 'synthetic_step.csv').open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['candidate', 't90_ms', 't95_ms', 'peak_speed_mps',
                    'peak_accel_mps2', 'jerk_p95_mps3', 'jerk_integral',
                    'overshoot_m', 'command_lead_hits', 'governor_only_t90_ms',
                    'governor_only_peak_speed_mps'])
        go = {g['candidate']: g for g in governor_only}
        for r in step_rows:
            g = go.get(r['candidate'], {})
            w.writerow([r['candidate'], r['t90_ms'], r['t95_ms'], r['peak_speed_mps'],
                        r['peak_accel_mps2'], r['jerk_p95_mps3'], r['jerk_integral'],
                        r['overshoot_m'], r['command_lead_hits'],
                        g.get('t90_ms'), g.get('peak_speed_mps')])

    with (args.output / 'replay.csv').open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['trial', 'candidate', 'samples', 'gap_median_m', 'gap_p95_m',
                    'effective_lag_ms_median', 'effective_lag_ms_p95',
                    'unsaturated_lag_ms_median', 'unsaturated_lag_ms_p95',
                    'unsaturated_samples', 'desired_speed_over_cap_fraction',
                    'peak_speed_mps', 'jerk_p95_mps3', 'jerk_integral',
                    'command_lead_hits', 'command_lead_hit_fraction'])
        for r in replay_rows:
            w.writerow([r['trial'], r['candidate'], r['samples'], r['gap_median_m'],
                        r['gap_p95_m'], r['effective_lag_ms_median'],
                        r['effective_lag_ms_p95'], r['unsaturated_lag_ms_median'],
                        r['unsaturated_lag_ms_p95'], r['unsaturated_samples'],
                        r['desired_speed_over_cap_fraction'], r['peak_speed_mps'],
                        r['jerk_p95_mps3'], r['jerk_integral'],
                        r['command_lead_hits'], r['command_lead_hit_fraction']])

    with (args.output / 'lead_budget_sweep.csv').open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['queue_execution_lag_m', 'command_lead_cap_m', 'max_accel_mps2',
                    'effective_profile_distance_m', 'predicted_commanded_speed_mps',
                    'speed_ratio_vs_current', 'velocity_cap_reached'])
        for r in lead_rows:
            w.writerow([r['queue_execution_lag_m'], r['command_lead_cap_m'],
                        r['max_accel_mps2'], round(r['effective_profile_distance_m'], 4),
                        round(r['predicted_commanded_speed_mps'], 4),
                        round(r['speed_ratio_vs_current'], 2) if r['speed_ratio_vs_current'] else None,
                        r['velocity_cap_reached']])

    with (args.output / 'cap_sensitivity.csv').open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['max_vel_mps', 'max_accel_mps2', 't90_ms',
                    'kinematic_floor_ms', 'peak_speed_mps', 'jerk_p95_mps3'])
        for r in cap_rows:
            w.writerow([r['max_vel_mps'], r['max_accel_mps2'], r['t90_ms'],
                        r['kinematic_floor_ms'], r['peak_speed_mps'], r['jerk_p95_mps3']])

    snap = args.output / 'source_snapshot'
    snap.mkdir()
    (snap / 'simulate_hc_p2_reference_candidates.py').write_text(Path(__file__).read_text())
    print(f'Wrote candidate comparison to {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
