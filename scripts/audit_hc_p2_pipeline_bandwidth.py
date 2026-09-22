#!/usr/bin/env python3
"""P2 — offline bandwidth/latency audit of the co-carrying motion pipeline.

Reads only existing CSV logs; never launches ROS, never publishes a topic and
never sends a robot command.  The pure ``PredictionReference`` math is replayed
against logged signals to verify that the offline model matches the runtime
formula, in the same spirit as the P0 F_robot replay check.

IMPORTANT: the modules are imported from the live working tree
(``--workspace``, default ``/home/hungnb/cocarry_ws``) and NOT from this
script's own directory.  The committed HEAD of this repository is older than
the working tree that produced the September 18-19 logs: committed
``prediction_reference.py`` has no velocity-lead term at all, and committed
``cartesian_streamer_hc10dtp.py`` has no ``_publish_motion_diagnostics``.
Auditing against the committed copies would model code that never ran.

    python3 scripts/audit_hc_p2_pipeline_bandwidth.py --output <new_dir>
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

DEFAULT_WORKSPACE = Path('/home/hungnb/cocarry_ws')

# Runtime constants, read from the live working tree (not from HEAD):
#   launch/cocarry_admittance_real_gui.launch.py
#     prediction_reference_tau_sec default 0.5, lead_sec default 0.15
#   cocarry_admittance_control/prediction_reference.py
#     max_lead_m default 0.02, velocity_tau_sec default 0.15
#   config/cocarry_admittance_params.yaml
#     prediction_max_nominal_lead_m 0.05, control_rate_hz 15.0
#
# The launch file OVERRIDES several YAML/source defaults.  Everything below is
# the value the real launch actually passes, verified by reading
# launch/cocarry_admittance_real_gui.launch.py, not the YAML or the streamer
# module constants:
#   streamer arguments: --stream-hz 15 --max-vel 0.25 --max-accel 1.00
#       --max-jerk 10.0 --continuous-cartesian-smoothing --fail-closed
#       --max-joint-vel 0.50 --max-wrist-joint-vel 0.08 --max-j3-joint-vel 0.50
#       --max-j5-joint-vel 0.50 --max-j6-joint-vel 0.40
#   admittance parameters: max_virtual_velocity_mps 0.25,
#       max_virtual_acceleration_mps2 1.00, command_lead_m 0.04
# The YAML values 0.15/0.50/0.03 and the module defaults 0.15/0.50 are NOT what
# ran; the logs saturate the command lead at exactly 0.0400 m, confirming 0.04.
REAL_TAU_SEC = 0.4
REAL_LEAD_SEC = 0.15
REAL_MAX_LEAD_M = 0.02
REAL_NOMINAL_LEAD_CAP_M = 0.05
REAL_COMMAND_LEAD_CAP_M = 0.04
STREAM_MAX_VEL = 0.25
STREAM_MAX_ACCEL = 1.00
STREAM_MAX_JERK = 10.0
CONTROL_RATE_HZ = 15.0
# Streamer joint velocity caps as launched, [S, L, U, R, B, T] rad/s.
JOINT_VEL_LIMITS = [0.50, 0.50, 0.50, 0.08, 0.50, 0.40]
# QUEUE_PREBUFFER_POINTS in the streamer; each point is one control period, so
# the prebuffer alone puts this many ticks of schedule ahead of the robot.
QUEUE_PREBUFFER_POINTS = 3

FLOAT_COLS = [
    'actual_ee_x', 'actual_ee_y', 'actual_ee_z',
    'predicted_xd_relative', 'predicted_yd_relative', 'predicted_zd_relative',
    'nominal_xd', 'nominal_yd', 'nominal_zd',
    'admittance_error_x', 'admittance_error_y', 'admittance_error_z',
    'reference_xr', 'reference_yr', 'reference_zr',
    'f_human_x', 'f_human_y', 'f_human_z',
    'f_effective_x', 'f_effective_y', 'f_effective_z',
    'raw_predicted_xd_relative', 'raw_predicted_yd_relative',
    'raw_predicted_zd_relative',
    'force_age_ms', 'pose_age_ms', 'prediction_age_ms', 'udp_gap_ms',
]


def load_runtime_modules(workspace: Path):
    """Import the pure admittance/reference modules from the live working tree."""
    pkg = workspace / 'src/cocarry_admittance_control/cocarry_admittance_control'
    if not (pkg / 'prediction_reference.py').exists():
        raise SystemExit(f'Cannot find runtime modules under {pkg}')
    sys.path.insert(0, str(pkg))
    import prediction_reference as pr_mod
    import admittance as adm_mod
    return pr_mod, adm_mod, pkg


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(x):
    x = np.asarray(list(x), dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return dict(n=0, median=None, p95=None, p99=None, mean=None, max=None)
    return dict(
        n=int(x.size),
        median=float(np.median(x)),
        p95=float(np.percentile(x, 95)),
        p99=float(np.percentile(x, 99)),
        mean=float(np.mean(x)),
        max=float(np.max(x)),
    )


def load_trial(path: Path):
    """Parse one CSV into numpy arrays plus decoded motion diagnostics."""
    with path.open(newline='') as fh:
        rows = list(csv.DictReader(fh))
    data = {'n': len(rows), 'sha256': digest(path)}
    data['t'] = np.array([int(r['ros_timestamp_ns']) for r in rows], dtype=np.int64)
    for col in FLOAT_COLS:
        vals = []
        for r in rows:
            try:
                vals.append(float(r.get(col, '')))
            except (TypeError, ValueError):
                vals.append(np.nan)
        data[col] = np.array(vals, dtype=float)
    md = []
    for r in rows:
        raw = r.get('motion_diagnostics_json', '')
        try:
            md.append(json.loads(raw) if raw else None)
        except (json.JSONDecodeError, TypeError):
            md.append(None)
    data['motion_diagnostics'] = md
    data['model'] = [r.get('model', '') for r in rows]
    data['role'] = [r.get('role', '') for r in rows]
    data['hybrid_status'] = [r.get('hybrid_status_json', '') for r in rows]
    return data


def xyz(data, prefix_triplet):
    return np.stack([data[c] for c in prefix_triplet], axis=1)


def effective_limits(data):
    """Prefer the limits recorded in the log over this file's constants.

    Schema 2 of motion_diagnostics_json and the controller hybrid status both
    carry the limits actually in force after launch overrides.  Reading the
    YAML or the module constants instead is exactly the mistake this function
    exists to prevent: the launch passes --max-vel 0.25 --max-accel 1.00 and
    command_lead_m 0.04, while the YAML says 0.15 / 0.50 / 0.03.
    """
    found = {}
    for md in data['motion_diagnostics']:
        if isinstance(md, dict) and isinstance(md.get('limits'), dict):
            found.update(md['limits'])
            break
    for raw in data.get('hybrid_status', []):
        if not raw:
            continue
        try:
            status = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            continue
        if isinstance(status, dict) and isinstance(status.get('limits'), dict):
            found.update(status['limits'])
            break
    assumed = dict(
        max_cartesian_velocity=STREAM_MAX_VEL,
        max_cartesian_acceleration=STREAM_MAX_ACCEL,
        max_cartesian_jerk=STREAM_MAX_JERK,
        max_command_lead_m=REAL_COMMAND_LEAD_CAP_M,
        prediction_max_nominal_lead_m=REAL_NOMINAL_LEAD_CAP_M,
        prediction_reference_tau_sec=REAL_TAU_SEC,
        prediction_reference_lead_sec=REAL_LEAD_SEC,
    )
    resolved = dict(assumed)
    resolved.update({k: v for k, v in found.items() if v is not None})
    mismatches = {
        k: dict(assumed=assumed[k], logged=found[k])
        for k in assumed if k in found and found[k] is not None
        and isinstance(found[k], (int, float)) and abs(found[k] - assumed[k]) > 1e-9
    }
    return dict(source='log' if found else 'assumed_constants',
                values=resolved, logged=found, mismatches=mismatches)


def queue_admission(data):
    """Cumulative queue BUSY/retry/reject, available from schema 2 onwards."""
    busy = retry = reject = recovery = None
    for md in data['motion_diagnostics']:
        if not isinstance(md, dict):
            continue
        if md.get('queue_busy_total') is not None:
            busy = md['queue_busy_total']
            retry = md.get('queue_retry_total')
            reject = md.get('queue_reject_total')
            recovery = md.get('auto_recovery_count')
    if busy is None:
        return dict(available=False,
                    note='Log predates motion_diagnostics schema 2; the '
                         'streamer kept BUSY/retry/reject only in 5 s console '
                         'counters, so they cannot be recovered offline.')
    return dict(available=True, queue_busy_total=busy, queue_retry_total=retry,
                queue_reject_total=reject, auto_recovery_count=recovery)


def smoother_state(data):
    """Streamer profile-limiter velocity/acceleration, schema 2 onwards."""
    speeds, accels, jerks = [], [], []
    prev_acc, prev_stamp = None, None
    for md in data['motion_diagnostics']:
        if not isinstance(md, dict):
            continue
        vel, acc = md.get('smoother_velocity'), md.get('smoother_acceleration')
        if vel is None or acc is None:
            continue
        speeds.append(float(np.linalg.norm(vel)))
        accels.append(float(np.linalg.norm(acc)))
        stamp = md.get('stamp_ns')
        if prev_acc is not None and stamp and prev_stamp and stamp > prev_stamp:
            dt = (stamp - prev_stamp) / 1e9
            jerks.append(float(np.linalg.norm(np.asarray(acc) - prev_acc) / dt))
        prev_acc, prev_stamp = np.asarray(acc, dtype=float), stamp
    if not speeds:
        return dict(available=False,
                    note='Log predates schema 2; jerk/acceleration can only be '
                         'estimated as derivatives of the measured EE.')
    return dict(available=True, commanded_speed_mps=stats(speeds),
                commanded_accel_mps2=stats(accels),
                commanded_jerk_mps3=stats(jerks))


# ── Sampling quality ─────────────────────────────────────────────────────

def timing_quality(data):
    dt = np.diff(data['t']) / 1e9
    dt_pos = dt[dt > 0]
    gap_threshold = 3.0 / CONTROL_RATE_HZ
    return dict(
        dt_sec=stats(dt_pos),
        non_monotonic_count=int(np.sum(dt <= 0)),
        large_gap_count=int(np.sum(dt_pos > gap_threshold)),
        large_gap_threshold_sec=gap_threshold,
        implied_rate_hz=float(1.0 / np.median(dt_pos)) if dt_pos.size else None,
    )


# ── Layer 1: raw prediction -> conditioned nominal ───────────────────────

def replay_prediction_reference(data, pr_mod, tau=REAL_TAU_SEC, lead=REAL_LEAD_SEC,
                                 max_lead_m=REAL_MAX_LEAD_M,
                                 nominal_lead_cap=REAL_NOMINAL_LEAD_CAP_M):
    """Replay controller._nominal_position's filter + geometric clamp chain.

    capture_ee is approximated by the first logged actual EE sample (the robot
    is at rest at Start Run).  This is a stated approximation: the controller's
    own _capture_ee is never written to the CSV.
    """
    n = data['n']
    t_sec = data['t'] / 1e9
    actual = xyz(data, ['actual_ee_x', 'actual_ee_y', 'actual_ee_z'])
    raw_pred = xyz(data, ['raw_predicted_xd_relative', 'raw_predicted_yd_relative',
                           'raw_predicted_zd_relative'])
    logged = xyz(data, ['nominal_xd', 'nominal_yd', 'nominal_zd'])
    capture = actual[0].copy()
    ref = pr_mod.PredictionReference(tau, lead_sec=lead, max_lead_m=max_lead_m)
    ref.reset(capture, t_sec[0])
    replay = np.full((n, 3), np.nan)
    limited = 0
    for i in range(n):
        if not np.all(np.isfinite(raw_pred[i])) or not np.all(np.isfinite(actual[i])):
            continue
        out = ref.step(capture + raw_pred[i], t_sec[i])
        delta = out - actual[i]
        dist = float(np.linalg.norm(delta))
        if dist > nominal_lead_cap:
            out = actual[i] + delta * (nominal_lead_cap / dist)
            ref.reset(out, t_sec[i])
            limited += 1
        replay[i] = out
    err = replay - logged
    finite = np.isfinite(err).all(axis=1)
    err_f = err[finite]
    # The only unknown in the replay is capture_ee, which enters additively.
    # Removing its best constant offset separates "wrong capture estimate"
    # from "wrong filter model": a near-zero residual after de-biasing means
    # the offline filter reproduces the runtime formula.
    bias = err_f.mean(axis=0) if err_f.size else np.zeros(3)
    debiased = err_f - bias
    return dict(
        capture_ee_estimate=capture.tolist(),
        capture_ee_bias_correction_m=bias.tolist(),
        compared_samples=int(finite.sum()),
        rmse_axis_m=np.sqrt(np.mean(err_f ** 2, axis=0)).tolist() if err_f.size else None,
        rmse_vector_m=float(np.sqrt(np.mean(np.sum(err_f ** 2, axis=1)))) if err_f.size else None,
        max_abs_err_m=float(np.max(np.abs(err_f))) if err_f.size else None,
        debiased_rmse_vector_m=(
            float(np.sqrt(np.mean(np.sum(debiased ** 2, axis=1)))) if err_f.size else None),
        debiased_max_abs_err_m=float(np.max(np.abs(debiased))) if err_f.size else None,
        nominal_lead_limit_hits=limited,
        tau_sec=tau, lead_sec=lead, max_lead_m=max_lead_m,
    )


def effective_lag_ms(leading, following, t, min_speed_mps=0.02):
    """Convert a spatial gap into a time lag: lag = |gap| / |speed|.

    For a follower that is rate-limited rather than filtered, the distance by
    which it trails its command divided by the command speed is the time by
    which it trails.  Only samples above ``min_speed_mps`` are used, since the
    quotient is meaningless when the command is nearly stationary.
    """
    gap = np.linalg.norm(leading - following, axis=1)
    speed = np.zeros(len(t))
    dt = np.diff(t)
    speed[1:] = np.linalg.norm(np.diff(leading, axis=0), axis=1) / np.maximum(dt, 1e-6)
    usable = np.isfinite(gap) & np.isfinite(speed) & (speed >= min_speed_mps)
    if not usable.any():
        return stats([]), 0
    lag_ms = gap[usable] / speed[usable] * 1000.0
    return stats(lag_ms), int(usable.sum())


def nominal_lag_vs_raw(data):
    """Lag from the raw GRU nominal to the conditioned nominal, from logs only."""
    actual0 = np.array([data['actual_ee_x'][0], data['actual_ee_y'][0],
                        data['actual_ee_z'][0]])
    raw_nominal = actual0 + xyz(data, ['raw_predicted_xd_relative',
                                        'raw_predicted_yd_relative',
                                        'raw_predicted_zd_relative'])
    nominal = xyz(data, ['nominal_xd', 'nominal_yd', 'nominal_zd'])
    dt = np.diff(data['t']) / 1e9
    dt_med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else 1.0 / CONTROL_RATE_HZ
    lags = windowed_lag_ms(raw_nominal, nominal, dt_med)
    gap = np.linalg.norm(raw_nominal - nominal, axis=1)
    eff_lag, eff_n = effective_lag_ms(raw_nominal, nominal, data['t'] / 1e9)
    return dict(lag_ms=stats(lags), windows_used=len(lags),
                raw_to_nominal_gap_m=stats(gap),
                effective_lag_ms=eff_lag, effective_lag_samples=eff_n,
                analytic_first_order_lag_ms=REAL_TAU_SEC * 1000.0,
                analytic_note='A first-order lag with tau has group delay tau '
                              'at low frequency; the velocity-lead term of '
                              f'{REAL_LEAD_SEC}s partially cancels it, capped '
                              f'at {REAL_MAX_LEAD_M} m.')


# ── Layer 2: nominal -> reference (admittance) ───────────────────────────

def check_admittance_identity(data):
    nominal = xyz(data, ['nominal_xd', 'nominal_yd', 'nominal_zd'])
    error = xyz(data, ['admittance_error_x', 'admittance_error_y', 'admittance_error_z'])
    reference = xyz(data, ['reference_xr', 'reference_yr', 'reference_zr'])
    residual = reference - (nominal + error)
    finite = np.isfinite(residual).all(axis=1)
    res = residual[finite]
    return dict(
        compared_samples=int(finite.sum()),
        residual_rms_m=float(np.sqrt(np.mean(res ** 2))) if res.size else None,
        residual_max_m=float(np.max(np.abs(res))) if res.size else None,
        admittance_error_norm_m=stats(np.linalg.norm(error[finite], axis=1)),
    )


def command_lead_events(data, cap=REAL_COMMAND_LEAD_CAP_M):
    reference = xyz(data, ['reference_xr', 'reference_yr', 'reference_zr'])
    actual = xyz(data, ['actual_ee_x', 'actual_ee_y', 'actual_ee_z'])
    lead = np.linalg.norm(reference - actual, axis=1)
    lead = lead[np.isfinite(lead)]
    return dict(
        command_lead_hits=int(np.sum(lead >= cap - 1e-6)),
        command_lead_hit_fraction=float(np.mean(lead >= cap - 1e-6)) if lead.size else None,
        command_lead_m=stats(lead), cap_m=cap,
    )


# ── Layer 3: reference -> actual EE (streamer + IK + queue) ──────────────

def windowed_lag_ms(reference, actual, dt_median_sec, window_sec=2.0,
                     max_lag_sec=1.0, min_corr=0.5):
    """Per-window cross-correlation lag between two XYZ trajectories.

    Correlates the de-meaned displacement magnitude inside each window and
    keeps only windows whose best correlation clears ``min_corr``, so that
    stationary windows cannot contribute a meaningless lag.
    """
    n = reference.shape[0]
    win = max(int(window_sec / dt_median_sec), 20)
    max_lag = max(int(max_lag_sec / dt_median_sec), 1)
    if n <= win + max_lag:
        return []
    lags_ms = []
    for start in range(0, n - win, max(win // 2, 1)):
        r = reference[start:start + win]
        a = actual[start:start + win]
        if not (np.isfinite(r).all() and np.isfinite(a).all()):
            continue
        r_mag = np.linalg.norm(r - r.mean(axis=0), axis=1)
        a_mag = np.linalg.norm(a - a.mean(axis=0), axis=1)
        if np.std(r_mag) < 1e-4 or np.std(a_mag) < 1e-4:
            continue
        r_mag = r_mag - r_mag.mean()
        a_mag = a_mag - a_mag.mean()
        best_lag, best_corr = 0, -np.inf
        for lag in range(0, max_lag + 1):
            rr = r_mag if lag == 0 else r_mag[:-lag]
            aa = a_mag if lag == 0 else a_mag[lag:]
            if rr.size < 5:
                continue
            denom = np.linalg.norm(rr) * np.linalg.norm(aa)
            if denom < 1e-9:
                continue
            corr = float(np.dot(rr, aa) / denom)
            if corr > best_corr:
                best_corr, best_lag = corr, lag
        if best_corr >= min_corr:
            lags_ms.append(best_lag * dt_median_sec * 1000.0)
    return lags_ms


def streamer_layer(data, timing):
    reference = xyz(data, ['reference_xr', 'reference_yr', 'reference_zr'])
    actual = xyz(data, ['actual_ee_x', 'actual_ee_y', 'actual_ee_z'])
    dt_med = timing['dt_sec']['median'] or (1.0 / CONTROL_RATE_HZ)
    lags_ms = windowed_lag_ms(reference, actual, dt_med)
    eff_lag, eff_n = effective_lag_ms(reference, actual, data['t'] / 1e9)

    tracking_err, ik_fail_events, vel_clamp_ticks, motion_ticks = [], 0, 0, 0
    hold_ticks, ack_latency_ms, due_vs_ack_ms = 0, [], []
    prev_ik_fail = False
    clamp_ratio = []
    for md in data['motion_diagnostics']:
        if md is None:
            continue
        te = md.get('tracking_error_m')
        if te is not None:
            tracking_err.append(te)
        fails = md.get('ik_consecutive_failures')
        if fails:
            if not prev_ik_fail:
                ik_fail_events += 1
            prev_ik_fail = True
        else:
            prev_ik_fail = False
        if md.get('point_hold'):
            hold_ticks += 1
        req, sent = md.get('requested_joint_velocity'), md.get('sent_joint_velocity')
        if req and sent and len(req) == len(sent):
            motion_ticks += 1
            if any(abs(r - s) > 1e-6 for r, s in zip(req, sent)):
                vel_clamp_ticks += 1
            req_max = max(abs(v) for v in req)
            sent_max = max(abs(v) for v in sent)
            if req_max > 1e-9:
                clamp_ratio.append(sent_max / req_max)
        send_ns, ack_ns = md.get('send_monotonic_ns'), md.get('ack_monotonic_ns')
        if send_ns and ack_ns and ack_ns >= send_ns:
            ack_latency_ms.append((ack_ns - send_ns) / 1e6)
        due_ns = md.get('due_monotonic_ns')
        if due_ns and ack_ns:
            due_vs_ack_ms.append((due_ns - ack_ns) / 1e6)

    # Numerical-derivative proxies from the logged actual EE.  The streamer's
    # internal velocity/acceleration/jerk state is not logged anywhere.
    t = data['t'] / 1e9
    valid = np.isfinite(actual).all(axis=1) & np.isfinite(t)
    peak_speed = jerk_p95 = jerk_integral = float('nan')
    if valid.sum() > 4:
        tv, av = t[valid], actual[valid]
        keep = np.concatenate([[True], np.diff(tv) > 0])
        tv, av = tv[keep], av[keep]
        if tv.size > 4:
            vel = np.gradient(av, tv, axis=0)
            speed = np.linalg.norm(vel, axis=1)
            peak_speed = float(np.nanmax(speed))
            jerk = np.gradient(np.gradient(vel, tv, axis=0), tv, axis=0)
            jm = np.linalg.norm(jerk, axis=1)
            jm = jm[np.isfinite(jm)]
            if jm.size:
                jerk_p95 = float(np.percentile(jm, 95))
                jerk_integral = float(np.trapz(jm, dx=float(np.median(np.diff(tv)))))
    return dict(
        cross_correlation_lag_ms=stats(lags_ms),
        cross_correlation_windows_used=len(lags_ms),
        cross_correlation_resolution_ms=dt_med * 1000.0,
        effective_lag_ms=eff_lag,
        effective_lag_samples=eff_n,
        tracking_error_m=stats(tracking_err),
        queue_ack_latency_ms=stats(ack_latency_ms),
        due_minus_ack_ms=stats(due_vs_ack_ms),
        ik_consecutive_failure_events=ik_fail_events,
        joint_velocity_clamp_ticks=vel_clamp_ticks,
        motion_ticks=motion_ticks,
        joint_velocity_clamp_fraction=(vel_clamp_ticks / motion_ticks) if motion_ticks else None,
        joint_velocity_sent_over_requested=stats(clamp_ratio),
        hold_point_ticks=hold_ticks,
        actual_ee_peak_speed_mps=peak_speed,
        actual_ee_jerk_p95_proxy_mps3=jerk_p95,
        actual_ee_jerk_integral_proxy=jerk_integral,
        note='peak speed and jerk are numerical-derivative proxies from the '
             'logged actual EE only; the streamer jerk limiter state is not logged.',
    )


def lead_budget(data, cap=REAL_COMMAND_LEAD_CAP_M, v_max=STREAM_MAX_VEL,
                 a_max=STREAM_MAX_ACCEL, dt=1.0 / CONTROL_RATE_HZ):
    """Why the commanded speed is far below the Cartesian velocity cap.

    The controller clamps the reference to ``cap`` metres ahead of the measured
    EE.  The streamer's profile limiter, however, restarts each tick from the
    last ACCEPTED queue pose, which already runs ahead of the measured EE by
    the queue execution lag (the same quantity the tracking watchdog reports).
    What the trapezoidal profile actually sees is therefore

        d_eff = command_lead - queue_lag

    and its deceleration branch commands sqrt(2*a*d_eff), capped by v_max and
    by the continuous-smoothing term d_eff/dt.  A small d_eff throttles the
    commanded speed even when the velocity cap is nowhere near saturated.
    """
    reference = xyz(data, ['reference_xr', 'reference_yr', 'reference_zr'])
    actual = xyz(data, ['actual_ee_x', 'actual_ee_y', 'actual_ee_z'])
    lead = np.linalg.norm(reference - actual, axis=1)

    queue_lag, smoothed_ahead = [], []
    for md in data['motion_diagnostics']:
        if md is None:
            continue
        exp, act = md.get('expected_xyz'), md.get('actual_xyz')
        if exp and act:
            queue_lag.append(float(np.linalg.norm(np.asarray(exp) - np.asarray(act))))
        ik = md.get('ik_request_xyz')
        if ik and act:
            smoothed_ahead.append(float(np.linalg.norm(np.asarray(ik) - np.asarray(act))))

    lead_med = float(np.nanmedian(lead)) if np.isfinite(lead).any() else float('nan')
    queue_med = float(np.median(queue_lag)) if queue_lag else float('nan')

    # Evaluate the profile branch PER SAMPLE and then take the median.  The
    # deceleration branch is sqrt(), which is concave, so evaluating it at the
    # median distance overestimates the median speed (Jensen); the per-sample
    # form is the one that matches the measured speed.
    n_pair = min(len(lead), len(queue_lag))
    if n_pair:
        d_series = np.clip(lead[:n_pair] - np.asarray(queue_lag[:n_pair]), 0.0, None)
        v_series = np.minimum.reduce([
            np.full(n_pair, v_max),
            np.sqrt(2.0 * a_max * d_series),
            d_series / dt,
        ])
        finite = np.isfinite(v_series)
        predicted = float(np.median(v_series[finite])) if finite.any() else float('nan')
        d_eff = float(np.median(d_series[np.isfinite(d_series)]))
    else:
        predicted = float('nan')
        d_eff = float('nan')
    unblocked = min(v_max, math.sqrt(2.0 * a_max * lead_med) if lead_med > 0 else 0.0)

    t = data['t'] / 1e9
    valid = np.isfinite(actual).all(axis=1)
    measured_speed = float('nan')
    if valid.sum() > 3:
        tv, av = t[valid], actual[valid]
        keep = np.concatenate([[True], np.diff(tv) > 0])
        tv, av = tv[keep], av[keep]
        if tv.size > 3:
            sp = np.linalg.norm(np.diff(av, axis=0), axis=1) / np.diff(tv)
            measured_speed = float(np.median(sp[np.isfinite(sp)]))
    return dict(
        command_lead_m=stats(lead),
        queue_execution_lag_m=stats(queue_lag),
        smoothed_target_ahead_of_actual_m=stats(smoothed_ahead),
        effective_profile_distance_m=d_eff,
        predicted_commanded_speed_mps=predicted,
        speed_if_full_lead_budget_mps=unblocked,
        measured_median_speed_mps=measured_speed,
        velocity_cap_mps=v_max,
        velocity_cap_saturated=bool(np.isfinite(measured_speed) and measured_speed >= 0.95 * v_max),
        note='predicted_commanded_speed is the trapezoidal deceleration branch '
             'evaluated at the median effective distance; it is a first-order '
             'explanation of the throttling, not a full simulation of the tick '
             'sequence.',
    )


def displacement_response(data, min_travel_m=0.05):
    """Time for actual EE to cover 90% of a reference displacement segment.

    Segments are detected on the commanded reference: a segment starts when
    the reference starts moving and ends when it settles again.  This is a
    naturalistic-motion estimate, not a controlled step test.
    """
    t = data['t'] / 1e9
    reference = xyz(data, ['reference_xr', 'reference_yr', 'reference_zr'])
    actual = xyz(data, ['actual_ee_x', 'actual_ee_y', 'actual_ee_z'])
    valid = np.isfinite(reference).all(axis=1) & np.isfinite(actual).all(axis=1)
    if valid.sum() < 30:
        return dict(segments=0, t90_ms=stats([]), peak_speed_mps=stats([]))
    t, reference, actual = t[valid], reference[valid], actual[valid]
    dt = np.diff(t)
    speed = np.zeros(len(t))
    speed[1:] = np.linalg.norm(np.diff(reference, axis=0), axis=1) / np.maximum(dt, 1e-6)
    moving = speed > 0.01
    # A segment ends once the reference has been still for a sustained pause,
    # not at the first sample below threshold; otherwise a single trial
    # collapses into one multi-second "segment" and t90 becomes meaningless.
    pause_samples = max(int(0.4 / float(np.median(dt))), 3) if dt.size else 6
    t90_list, peak_list, travel_list, seg_dur = [], [], [], []
    i = 0
    n = len(moving)
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        still = 0
        while j < n and still < pause_samples:
            still = still + 1 if not moving[j] else 0
            j += 1
        end = j - still if still >= pause_samples else j
        seg = slice(i, max(end, i + 2))
        seg_t, seg_ref, seg_act = t[seg], reference[seg], actual[seg]
        if seg_t.size >= 3 and (seg_t[-1] - seg_t[0]) > 0.3:
            start_pos, end_pos = seg_act[0], seg_ref[-1]
            travel = float(np.linalg.norm(end_pos - start_pos))
            if travel >= min_travel_m:
                direction = (end_pos - start_pos) / travel
                progress = (seg_act - start_pos) @ direction
                reached = np.flatnonzero(progress >= 0.9 * travel)
                if reached.size:
                    t90_list.append(float(seg_t[reached[0]] - seg_t[0]) * 1000.0)
                    travel_list.append(travel)
                    seg_dur.append(float(seg_t[-1] - seg_t[0]))
                a_speed = np.linalg.norm(np.diff(seg_act, axis=0), axis=1) / np.maximum(
                    np.diff(seg_t), 1e-6)
                if a_speed.size:
                    peak_list.append(float(np.max(a_speed)))
        i = max(end, i + 1)
    return dict(segments=len(t90_list), t90_ms=stats(t90_list),
                peak_speed_mps=stats(peak_list), travel_m=stats(travel_list),
                segment_duration_sec=stats(seg_dur), min_travel_m=min_travel_m,
                note='Segments come from naturalistic co-carry motion, not a '
                     'controlled step input; t90 mixes human intent with '
                     'pipeline response.')


def analyze_trial(path: Path, pr_mod):
    data = load_trial(path)
    timing = timing_quality(data)
    limits = effective_limits(data)
    values = limits['values']
    return dict(
        trial=path.stem,
        sha256=data['sha256'],
        n_rows=data['n'],
        duration_sec=float((data['t'][-1] - data['t'][0]) / 1e9) if data['n'] > 1 else 0.0,
        models=sorted(set(data['model'])),
        roles=sorted(set(data['role'])),
        effective_limits=limits,
        queue_admission=queue_admission(data),
        smoother_state=smoother_state(data),
        timing=timing,
        ages=dict(
            force_age_ms=stats(data['force_age_ms']),
            pose_age_ms=stats(data['pose_age_ms']),
            prediction_age_ms=stats(data['prediction_age_ms']),
            udp_gap_ms=stats(data['udp_gap_ms']),
        ),
        prediction_reference_replay=replay_prediction_reference(
            data, pr_mod,
            tau=values['prediction_reference_tau_sec'],
            lead=values['prediction_reference_lead_sec'],
            nominal_lead_cap=values['prediction_max_nominal_lead_m']),
        nominal_conditioning=nominal_lag_vs_raw(data),
        admittance=check_admittance_identity(data),
        command_lead=command_lead_events(data, cap=values['max_command_lead_m']),
        lead_budget=lead_budget(
            data, cap=values['max_command_lead_m'],
            v_max=values['max_cartesian_velocity'],
            a_max=values['max_cartesian_acceleration']),
        streamer=streamer_layer(data, timing),
        displacement_response=displacement_response(data),
    )


def median_of(trials, path, key):
    values = []
    for tr in trials:
        node = tr
        for part in path:
            node = node.get(part, {}) if isinstance(node, dict) else {}
        if isinstance(node, dict) and node.get('n') and node.get(key) is not None:
            values.append(node[key])
    return float(np.median(values)) if values else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--workspace', type=Path, default=DEFAULT_WORKSPACE,
                     help='Live workspace providing logs and runtime modules')
    ap.add_argument('--log-glob', default='cocarry_admittance_3d_2026*.csv')
    ap.add_argument('--min-rows', type=int, default=100)
    args = ap.parse_args()
    if args.output.exists():
        raise SystemExit(f'Output dir exists, refusing to overwrite: {args.output}')

    pr_mod, _adm_mod, pkg = load_runtime_modules(args.workspace)
    log_dir = args.workspace / 'cocarry_logs'
    files = sorted(log_dir.glob(args.log_glob))
    if not files:
        raise SystemExit(f'No logs matched {args.log_glob} under {log_dir}')

    args.output.mkdir(parents=True)
    trials, skipped = [], []
    for f in files:
        with f.open() as fh:
            n = sum(1 for _ in fh) - 1
        if n < args.min_rows:
            skipped.append(dict(trial=f.stem, n_rows=n))
            continue
        print(f'Analyzing {f.name} ({n} rows)...', file=sys.stderr)
        trials.append(analyze_trial(f, pr_mod))

    per_trial = args.output / 'per_trial'
    per_trial.mkdir()
    for tr in trials:
        (per_trial / f"{tr['trial']}.json").write_text(json.dumps(tr, indent=2))

    summary = dict(
        n_trials=len(trials),
        trials_used=[t['trial'] for t in trials],
        skipped_short=skipped,
        total_rows=sum(t['n_rows'] for t in trials),
        total_duration_sec=sum(t['duration_sec'] for t in trials),
        pooled=dict(
            replay_rmse_vector_m=stats(
                [t['prediction_reference_replay']['rmse_vector_m'] for t in trials]),
            replay_max_abs_err_m=stats(
                [t['prediction_reference_replay']['max_abs_err_m'] for t in trials]),
            replay_debiased_rmse_vector_m=stats(
                [t['prediction_reference_replay']['debiased_rmse_vector_m'] for t in trials]),
            replay_debiased_max_abs_err_m=stats(
                [t['prediction_reference_replay']['debiased_max_abs_err_m'] for t in trials]),
            raw_to_nominal_effective_lag_ms_median=median_of(
                trials, ['nominal_conditioning', 'effective_lag_ms'], 'median'),
            raw_to_nominal_effective_lag_ms_p95=median_of(
                trials, ['nominal_conditioning', 'effective_lag_ms'], 'p95'),
            streamer_effective_lag_ms_median=median_of(
                trials, ['streamer', 'effective_lag_ms'], 'median'),
            streamer_effective_lag_ms_p95=median_of(
                trials, ['streamer', 'effective_lag_ms'], 'p95'),
            streamer_effective_lag_ms_p99=median_of(
                trials, ['streamer', 'effective_lag_ms'], 'p99'),
            admittance_identity_residual_rms_m=stats(
                [t['admittance']['residual_rms_m'] for t in trials]),
            raw_to_nominal_lag_ms_median=median_of(
                trials, ['nominal_conditioning', 'lag_ms'], 'median'),
            raw_to_nominal_lag_ms_p95=median_of(
                trials, ['nominal_conditioning', 'lag_ms'], 'p95'),
            raw_to_nominal_gap_m_median=median_of(
                trials, ['nominal_conditioning', 'raw_to_nominal_gap_m'], 'median'),
            streamer_lag_ms_median=median_of(
                trials, ['streamer', 'cross_correlation_lag_ms'], 'median'),
            streamer_lag_ms_p95=median_of(
                trials, ['streamer', 'cross_correlation_lag_ms'], 'p95'),
            streamer_lag_ms_p99=median_of(
                trials, ['streamer', 'cross_correlation_lag_ms'], 'p99'),
            tracking_error_m_median=median_of(trials, ['streamer', 'tracking_error_m'], 'median'),
            tracking_error_m_p95=median_of(trials, ['streamer', 'tracking_error_m'], 'p95'),
            queue_ack_latency_ms_median=median_of(
                trials, ['streamer', 'queue_ack_latency_ms'], 'median'),
            queue_ack_latency_ms_p95=median_of(
                trials, ['streamer', 'queue_ack_latency_ms'], 'p95'),
            queue_ack_latency_ms_p99=median_of(
                trials, ['streamer', 'queue_ack_latency_ms'], 'p99'),
            prediction_age_ms_median=median_of(trials, ['ages', 'prediction_age_ms'], 'median'),
            prediction_age_ms_p95=median_of(trials, ['ages', 'prediction_age_ms'], 'p95'),
            prediction_age_ms_p99=median_of(trials, ['ages', 'prediction_age_ms'], 'p99'),
            force_age_ms_median=median_of(trials, ['ages', 'force_age_ms'], 'median'),
            force_age_ms_p95=median_of(trials, ['ages', 'force_age_ms'], 'p95'),
            pose_age_ms_median=median_of(trials, ['ages', 'pose_age_ms'], 'median'),
            pose_age_ms_p95=median_of(trials, ['ages', 'pose_age_ms'], 'p95'),
            udp_gap_ms_median=median_of(trials, ['ages', 'udp_gap_ms'], 'median'),
            t90_ms_median=median_of(trials, ['displacement_response', 't90_ms'], 'median'),
            t90_ms_p95=median_of(trials, ['displacement_response', 't90_ms'], 'p95'),
            actual_peak_speed_mps_median=median_of(
                trials, ['displacement_response', 'peak_speed_mps'], 'median'),
            command_lead_m_median=median_of(trials, ['command_lead', 'command_lead_m'], 'median'),
            command_lead_m_p95=median_of(trials, ['command_lead', 'command_lead_m'], 'p95'),
            command_lead_hits_total=sum(t['command_lead']['command_lead_hits'] for t in trials),
            ik_failure_events_total=sum(
                t['streamer']['ik_consecutive_failure_events'] for t in trials),
            joint_velocity_clamp_ticks_total=sum(
                t['streamer']['joint_velocity_clamp_ticks'] for t in trials),
            motion_ticks_total=sum(t['streamer']['motion_ticks'] for t in trials),
            hold_point_ticks_total=sum(t['streamer']['hold_point_ticks'] for t in trials),
            non_monotonic_timestamp_total=sum(t['timing']['non_monotonic_count'] for t in trials),
            large_gap_total=sum(t['timing']['large_gap_count'] for t in trials),
            displacement_segments_total=sum(
                t['displacement_response']['segments'] for t in trials),
        ),
        known_missing_data=[
            'Queue BUSY/retry/reject counters are never published: the streamer '
            'keeps them in per-5s window counters logged to the console only. '
            'No per-sample queue retry delay distribution can be derived.',
            'CSV rows are a latest-sample-and-hold join at the 15 Hz controller '
            'tick; only *_age_ms and the motion_diagnostics monotonic stamps '
            'carry per-signal timing, so inter-layer delay below one tick '
            '(~66 ms) cannot be resolved directly.',
            'Streamer internal velocity/acceleration/jerk limiter state is not '
            'logged; jerk numbers here are actual-EE derivative proxies.',
            'The controller _capture_ee is not logged; the replay approximates '
            'it with the first actual EE sample of each trial.',
            'No Axia raw/processed stage timing is joined here; P1 already '
            'characterised the force filter separately.',
        ],
        constants_used=dict(
            tau_sec=REAL_TAU_SEC, lead_sec=REAL_LEAD_SEC, max_lead_m=REAL_MAX_LEAD_M,
            nominal_lead_cap_m=REAL_NOMINAL_LEAD_CAP_M,
            command_lead_cap_m=REAL_COMMAND_LEAD_CAP_M,
            stream_max_vel_mps=STREAM_MAX_VEL, stream_max_accel_mps2=STREAM_MAX_ACCEL,
            stream_max_jerk_mps3=STREAM_MAX_JERK, control_rate_hz=CONTROL_RATE_HZ,
        ),
    )
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))

    with (args.output / 'summary.csv').open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['trial', 'n_rows', 'duration_sec', 'replay_rmse_vector_m',
                    'replay_max_abs_err_m', 'admittance_identity_rms_m',
                    'raw_to_nominal_lag_ms_median', 'streamer_lag_ms_median',
                    'streamer_lag_ms_p95', 'tracking_error_m_median',
                    'tracking_error_m_p95', 'queue_ack_ms_median', 'queue_ack_ms_p95',
                    'prediction_age_ms_median', 'prediction_age_ms_p95',
                    't90_ms_median', 'actual_peak_speed_mps',
                    'command_lead_m_p95', 'command_lead_hits', 'ik_failure_events',
                    'joint_vel_clamp_ticks', 'motion_ticks', 'hold_ticks',
                    'non_monotonic', 'large_gaps'])
        for t in trials:
            w.writerow([
                t['trial'], t['n_rows'], round(t['duration_sec'], 2),
                t['prediction_reference_replay']['rmse_vector_m'],
                t['prediction_reference_replay']['max_abs_err_m'],
                t['admittance']['residual_rms_m'],
                t['nominal_conditioning']['lag_ms'].get('median'),
                t['streamer']['cross_correlation_lag_ms'].get('median'),
                t['streamer']['cross_correlation_lag_ms'].get('p95'),
                t['streamer']['tracking_error_m'].get('median'),
                t['streamer']['tracking_error_m'].get('p95'),
                t['streamer']['queue_ack_latency_ms'].get('median'),
                t['streamer']['queue_ack_latency_ms'].get('p95'),
                t['ages']['prediction_age_ms'].get('median'),
                t['ages']['prediction_age_ms'].get('p95'),
                t['displacement_response']['t90_ms'].get('median'),
                t['streamer']['actual_ee_peak_speed_mps'],
                t['command_lead']['command_lead_m'].get('p95'),
                t['command_lead']['command_lead_hits'],
                t['streamer']['ik_consecutive_failure_events'],
                t['streamer']['joint_velocity_clamp_ticks'],
                t['streamer']['motion_ticks'],
                t['streamer']['hold_point_ticks'],
                t['timing']['non_monotonic_count'],
                t['timing']['large_gap_count'],
            ])

    snap = args.output / 'source_snapshot'
    snap.mkdir()
    (snap / 'audit_hc_p2_pipeline_bandwidth.py').write_text(Path(__file__).read_text())
    for name in ('prediction_reference.py', 'admittance.py'):
        (snap / f'working_tree_{name}').write_text((pkg / name).read_text())
    (args.output / 'provenance.json').write_text(json.dumps(dict(
        workspace=str(args.workspace), log_glob=args.log_glob,
        min_rows=args.min_rows,
        runtime_module_sha256={
            name: digest(pkg / name) for name in ('prediction_reference.py', 'admittance.py')},
        note='Runtime modules are taken from the live working tree, which is '
             'ahead of the committed HEAD for these files.',
    ), indent=2))
    print(f'Wrote {len(trials)} trial analyses to {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
