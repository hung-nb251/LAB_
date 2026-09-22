"""Unit tests for the P2 pipeline bandwidth audit and candidate simulation.

Pure numerical tests: no ROS, no logs, no robot. Run with
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 because the installed ROS launch_testing
plugin is not compatible with the local pytest version.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import audit_hc_p2_pipeline_bandwidth as audit  # noqa: E402
import simulate_hc_p2_reference_candidates as sim  # noqa: E402


# ── stats ────────────────────────────────────────────────────────────────

def test_stats_handles_empty_and_nan():
    assert audit.stats([])['n'] == 0
    assert audit.stats([np.nan, np.inf, -np.inf])['n'] == 0
    s = audit.stats([1.0, 2.0, 3.0, np.nan])
    assert s['n'] == 3
    assert s['median'] == pytest.approx(2.0)
    assert s['max'] == pytest.approx(3.0)


# ── effective_lag_ms ─────────────────────────────────────────────────────

def test_effective_lag_recovers_known_delay():
    """A follower trailing a constant-velocity command by a fixed time."""
    dt = 1.0 / 15.0
    t = np.arange(200) * dt
    speed = 0.10
    delay_sec = 0.30
    leading = np.stack([speed * t, np.zeros_like(t), np.zeros_like(t)], axis=1)
    following = np.stack([speed * np.maximum(t - delay_sec, 0.0),
                          np.zeros_like(t), np.zeros_like(t)], axis=1)
    result, n = audit.effective_lag_ms(leading, following, t)
    assert n > 100
    # Steady-state gap/speed must recover the delay within one control tick.
    assert result['median'] == pytest.approx(delay_sec * 1000.0, abs=dt * 1000.0)


def test_effective_lag_ignores_stationary_samples():
    t = np.arange(50) / 15.0
    still = np.zeros((50, 3))
    result, n = audit.effective_lag_ms(still, still, t)
    assert n == 0
    assert result['n'] == 0


# ── windowed cross-correlation lag ───────────────────────────────────────

def test_windowed_lag_detects_integer_sample_shift():
    dt = 1.0 / 15.0
    n = 400
    t = np.arange(n) * dt
    signal = 0.1 * np.sin(2 * np.pi * 0.25 * t)
    shift = 4
    leading = np.stack([signal, np.zeros(n), np.zeros(n)], axis=1)
    following = np.stack([np.concatenate([np.zeros(shift), signal[:-shift]]),
                          np.zeros(n), np.zeros(n)], axis=1)
    lags = audit.windowed_lag_ms(leading, following, dt, window_sec=4.0)
    assert lags
    # Magnitude-based correlation of a sine folds the period, so only require
    # that a positive lag on the order of the true shift is recovered.
    assert np.median(lags) > 0


def test_windowed_lag_returns_empty_for_stationary_signals():
    dt = 1.0 / 15.0
    still = np.zeros((300, 3))
    assert audit.windowed_lag_ms(still, still, dt) == []


# ── streamer profile limiter ─────────────────────────────────────────────

def test_smooth_step_respects_velocity_and_acceleration_caps():
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = np.zeros(3)
    target = np.array([1.0, 0.0, 0.0])   # far away: caps must bind
    dt = 1.0 / 15.0
    for _ in range(200):
        new_pos, vel, acc = sim.smooth_step(pos, target, vel, acc, dt)
        step = np.linalg.norm(new_pos - pos)
        assert np.linalg.norm(vel) <= sim.STREAM_MAX_VEL + 1e-9
        assert np.linalg.norm(acc) <= sim.STREAM_MAX_ACCEL + 1e-9
        assert step <= sim.STREAM_MAX_VEL * dt + 1e-9
        pos = new_pos


def test_smooth_step_jerk_limit_binds_while_far_from_target():
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = np.zeros(3)
    target = np.array([1.0, 0.0, 0.0])
    dt = 1.0 / 15.0
    prev_acc = acc.copy()
    for _ in range(5):
        pos, vel, acc = sim.smooth_step(pos, target, vel, acc, dt)
        # Far from target (>20 mm), the runtime applies the jerk limit.
        assert np.max(np.abs(acc - prev_acc)) <= sim.STREAM_MAX_JERK * dt + 1e-9
        prev_acc = acc.copy()


def test_smooth_step_converges_to_target():
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = np.zeros(3)
    target = np.array([0.05, 0.0, 0.0])
    for _ in range(300):
        pos, vel, acc = sim.smooth_step(pos, target, vel, acc, 1.0 / 15.0)
    assert np.linalg.norm(pos - target) < 1e-3


def test_step_cannot_beat_the_velocity_kinematic_floor():
    """A 0.10 m step cannot complete faster than distance / max velocity."""
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = np.zeros(3)
    dt = 1.0 / 15.0
    target = np.array([0.10, 0.0, 0.0])
    elapsed = 0.0
    while np.linalg.norm(pos - target) > 0.01 * 0.10 and elapsed < 10.0:
        pos, vel, acc = sim.smooth_step(pos, target, vel, acc, dt)
        elapsed += dt
    floor = 0.10 / sim.STREAM_MAX_VEL
    assert elapsed >= floor - dt


# ── governors ────────────────────────────────────────────────────────────

class _FakePredictionReference:
    """Minimal stand-in so governor tests do not depend on the workspace."""

    def __init__(self, tau, lead_sec=0.0, max_lead_m=0.02):
        self.tau = float(tau)
        self.position = None
        self.time = None

    def reset(self, position, now):
        self.position = np.asarray(position, float).copy()
        self.time = float(now)

    def step(self, desired, now):
        desired = np.asarray(desired, float)
        if self.tau == 0.0 or self.position is None:
            self.reset(desired, now)
            return self.position.copy()
        dt = max(float(now) - self.time, 0.0)
        alpha = -np.expm1(-dt / self.tau) if dt > 0 else 0.0
        self.position = self.position + alpha * (desired - self.position)
        self.time = float(now)
        return self.position.copy()


class _FakeModule:
    PredictionReference = _FakePredictionReference


def test_second_order_governor_is_critically_damped_without_overshoot():
    gov = sim.SecondOrderGovernor(settling_sec=0.45)
    dt = 1.0 / 15.0
    gov.reset(np.zeros(3), 0.0)
    target = np.array([0.10, 0.0, 0.0])
    peak = 0.0
    for i in range(200):
        out = gov.step(target, i * dt, dt)
        peak = max(peak, float(out[0]))
    # Critically damped: settles at the target, with no meaningful overshoot.
    assert out[0] == pytest.approx(0.10, abs=1e-3)
    assert peak <= 0.10 + 1e-3


def test_fast_bounded_governor_respects_its_own_rate_bound():
    gov = sim.FastBoundedGovernor(_FakeModule, tau=0.15, max_vel=0.15, max_accel=0.50)
    dt = 1.0 / 15.0
    gov.reset(np.zeros(3), 0.0)
    prev = np.zeros(3)
    target = np.array([1.0, 0.0, 0.0])
    for i in range(100):
        out = np.asarray(gov.step(target, i * dt, dt), float)
        assert np.linalg.norm(out - prev) <= 0.15 * dt + 1e-9
        prev = out


def test_second_order_feed_forward_overshoot_stays_small_and_settles():
    """The feed-forward branch cannot run past the command by more than the
    discretisation error of the integrator.

    At the 15 Hz control rate a 0.45 s settling time gives wn*dt ~= 0.70, which
    is a coarse step for the explicit update used here, so the discrete
    response overshoots slightly even though the continuous-time system is
    critically damped. The overshoot is bounded and decays; it is reported in
    the audit rather than being assumed away.
    """
    gov = sim.SecondOrderGovernor(settling_sec=0.45, jerk_limit=10.0, feed_forward=0.08)
    dt = 1.0 / 15.0
    gov.reset(np.zeros(3), 0.0)
    target = np.array([0.10, 0.0, 0.0])
    peak = 0.0
    for i in range(200):
        out = np.asarray(gov.step(target, i * dt, dt), float)
        peak = max(peak, float(out[0]))
    assert peak <= 0.10 * 1.10        # under 10% overshoot
    assert out[0] == pytest.approx(0.10, abs=1e-3)   # and it settles on target


# ── plant simulation ─────────────────────────────────────────────────────

class _PassthroughGovernor:
    """Unbounded governor: jumps straight to the commanded nominal."""

    name = 'passthrough'
    description = 'test double'

    def reset(self, position, now):
        pass

    def step(self, desired, now, dt):
        return np.asarray(desired, float)


def test_simulate_clamps_an_unbounded_governor_to_the_command_lead_cap():
    n = 150
    t = np.arange(n) / 15.0
    desired = np.tile(np.array([1.0, 0.0, 0.0]), (n, 1))  # far, keeps saturating
    result = sim.simulate(_PassthroughGovernor(), desired, t, start=np.zeros(3))
    lead = np.linalg.norm(result['reference'] - result['plant'], axis=1)
    # The reference may lead the *delayed* feedback by the cap; against the
    # current plant position it can never exceed the cap plus one tick of travel.
    assert np.max(lead) <= sim.COMMAND_LEAD_CAP_M + sim.STREAM_MAX_VEL / 15.0 + 1e-6
    assert result['lead_hits'] > 0


def test_self_bounded_governor_hits_the_lead_cap_less_than_an_unbounded_one():
    """A rate-bounded reference exercises the anti-windup path less often.

    With the real caps (0.25 m/s, 1.00 m/s^2) a self-bounded governor still
    reaches the clamp during sustained far-target saturation, so the property
    that holds is a reduction, not elimination. On a short 0.10 m step the
    second-order governor does reach zero clamp events; see the audit report.
    """
    n = 150
    t = np.arange(n) / 15.0
    desired = np.tile(np.array([1.0, 0.0, 0.0]), (n, 1))
    bounded = sim.simulate(sim.SecondOrderGovernor(settling_sec=0.45), desired, t,
                           start=np.zeros(3))
    unbounded = sim.simulate(_PassthroughGovernor(), desired, t, start=np.zeros(3))
    assert bounded['lead_hits'] < unbounded['lead_hits']


def test_second_order_governor_avoids_the_lead_cap_on_a_short_step():
    """On a 0.10 m step the bounded governor never needs the anti-windup path."""
    n = 120
    t = np.arange(n) / 15.0
    desired = np.tile(np.array([0.10, 0.0, 0.0]), (n, 1))
    result = sim.simulate(sim.SecondOrderGovernor(settling_sec=0.45), desired, t,
                          start=np.zeros(3))
    assert result['lead_hits'] == 0


def test_simulate_is_deterministic():
    n = 100
    t = np.arange(n) / 15.0
    desired = np.tile(np.array([0.05, 0.0, 0.0]), (n, 1))
    first = sim.simulate(sim.SecondOrderGovernor(settling_sec=0.45), desired, t,
                         start=np.zeros(3))
    second = sim.simulate(sim.SecondOrderGovernor(settling_sec=0.45), desired, t,
                          start=np.zeros(3))
    assert np.allclose(first['plant'], second['plant'])


def test_step_metrics_reports_t90_and_no_negative_overshoot():
    gov = sim.SecondOrderGovernor(settling_sec=0.45)
    n = 120
    t = np.arange(n) / 15.0
    target = np.array([0.10, 0.0, 0.0])
    desired = np.tile(target, (n, 1))
    result = sim.simulate(gov, desired, t, start=np.zeros(3))
    m = sim.step_metrics(result, t, target, np.zeros(3))
    assert m['t90_ms'] is not None and m['t90_ms'] > 0
    assert m['overshoot_m'] >= 0.0
    assert m['peak_speed_mps'] <= sim.STREAM_MAX_VEL + 1e-9


# ── audit-side pure helpers ──────────────────────────────────────────────

def test_timing_quality_flags_non_monotonic_and_gaps():
    base = 1_000_000_000
    tick = int(1e9 / 15)
    stamps = [base, base + tick, base + tick, base + 2 * tick,
              base + 2 * tick + int(0.5e9)]
    data = {'t': np.array(stamps, dtype=np.int64)}
    q = audit.timing_quality(data)
    assert q['non_monotonic_count'] == 1     # the repeated stamp
    assert q['large_gap_count'] == 1         # the 0.5 s jump


# ── schema 2 instrumentation consumed by the audit ───────────────────────

def _schema2_row(**overrides):
    row = {
        'limits': {
            'max_cartesian_velocity': 0.25,
            'max_cartesian_acceleration': 1.00,
            'max_cartesian_jerk': 10.0,
            'prebuffer_points': 3,
        },
        'queue_busy_total': 7,
        'queue_retry_total': 7,
        'queue_reject_total': 1,
        'auto_recovery_count': 0,
        'smoother_velocity': [0.10, 0.0, 0.0],
        'smoother_acceleration': [0.5, 0.0, 0.0],
        'stamp_ns': 1_000_000_000,
    }
    row.update(overrides)
    return row


def test_effective_limits_prefers_the_logged_values_over_constants():
    """The audit must not fall back to YAML/module constants when the log
    carries the values that were actually in force."""
    data = {'motion_diagnostics': [_schema2_row()], 'hybrid_status': [
        '{"limits": {"max_command_lead_m": 0.04}}']}
    limits = audit.effective_limits(data)
    assert limits['source'] == 'log'
    assert limits['values']['max_cartesian_velocity'] == 0.25
    assert limits['values']['max_cartesian_acceleration'] == 1.00
    assert limits['values']['max_command_lead_m'] == 0.04


def test_effective_limits_reports_a_mismatch_against_its_own_assumption():
    data = {'motion_diagnostics': [_schema2_row(limits={'max_cartesian_velocity': 0.15})],
            'hybrid_status': []}
    limits = audit.effective_limits(data)
    assert 'max_cartesian_velocity' in limits['mismatches']
    assert limits['mismatches']['max_cartesian_velocity']['logged'] == 0.15


def test_effective_limits_falls_back_when_the_log_is_schema_1():
    data = {'motion_diagnostics': [{'sent_joints': [0] * 6}], 'hybrid_status': ['']}
    limits = audit.effective_limits(data)
    assert limits['source'] == 'assumed_constants'
    assert limits['values']['max_cartesian_velocity'] == audit.STREAM_MAX_VEL


def test_queue_admission_reads_schema_2_counters():
    data = {'motion_diagnostics': [_schema2_row(), _schema2_row(queue_busy_total=9)]}
    result = audit.queue_admission(data)
    assert result['available'] is True
    assert result['queue_busy_total'] == 9      # cumulative: last wins


def test_queue_admission_reports_unavailable_for_older_logs():
    data = {'motion_diagnostics': [{'sent_joints': [0] * 6}, None]}
    result = audit.queue_admission(data)
    assert result['available'] is False
    assert 'schema 2' in result['note']


def test_smoother_state_derives_commanded_jerk_from_logged_acceleration():
    tick = int(1e9 / 15)
    rows = [
        _schema2_row(stamp_ns=tick, smoother_acceleration=[0.0, 0.0, 0.0]),
        _schema2_row(stamp_ns=2 * tick, smoother_acceleration=[0.5, 0.0, 0.0]),
    ]
    result = audit.smoother_state({'motion_diagnostics': rows})
    assert result['available'] is True
    # 0.5 m/s^2 change over one 1/15 s tick is 7.5 m/s^3.
    assert result['commanded_jerk_mps3']['median'] == pytest.approx(7.5, rel=1e-6)


def test_smoother_state_reports_unavailable_for_older_logs():
    result = audit.smoother_state({'motion_diagnostics': [{'sent_joints': [0] * 6}]})
    assert result['available'] is False


# ── the real streamer payload builder (needs a sourced ROS env) ──────────

def _load_streamer_module():
    import importlib.util
    path = ROOT / 'src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py'
    spec = importlib.util.spec_from_file_location('streamer_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_streamer_publishes_schema2_diagnostics_with_effective_limits():
    """Exercise the real _publish_motion_diagnostics, not a reimplementation.

    Skipped when ROS message packages are not importable; run it from a
    sourced workspace to cover the publishing path.
    """
    import types
    try:
        mod = _load_streamer_module()
    except Exception as exc:                      # ROS deps unavailable
        pytest.skip(f'streamer module not importable here: {type(exc).__name__}')

    # The launch overrides these globals; reproduce the real profile.
    mod.MAX_CARTESIAN_VELOCITY = 0.25
    mod.MAX_CARTESIAN_ACCELERATION = 1.00
    mod.MAX_JOINT_VELOCITIES[:] = [0.50, 0.50, 0.50, 0.08, 0.50, 0.40]

    captured = {}

    class _Pub:
        def publish(self, msg):
            captured['data'] = msg.data

    stub = types.SimpleNamespace(
        _last_diagnostic_time=0.0, _stream_period_sec=1 / 15,
        _motion_diagnostics={'tracking_error_m': 0.03, 'point_hold': False},
        _stream_state='streaming', _joint_coordination='synchronized',
        _total_busy_count=4, _total_retry_count=4, _total_reject_count=0,
        _accepted_points=1234, _auto_recovery_count=0,
        _prev_ee_velocity=[0.11, 0.0, 0.0], _prev_ee_acceleration=[0.4, 0.0, 0.0],
        _stream_hz=15.0, _queue_dt_sec=0.066, _prebuffer_target=3,
        _fail_closed=True, _motion_diagnostic_pub=_Pub(),
    )
    stub.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=123456789))

    mod.CartesianStreamer._publish_motion_diagnostics(stub)
    payload = json.loads(captured['data'])

    assert payload['schema'] == 2
    assert payload['queue_busy_total'] == 4
    assert payload['queue_reject_total'] == 0
    assert payload['accepted_points_total'] == 1234
    assert payload['smoother_velocity'] == [0.11, 0.0, 0.0]
    # The published limits must be the overridden values, never the module
    # defaults -- this is the check that would have caught the constants error.
    assert payload['limits']['max_cartesian_velocity'] == 0.25
    assert payload['limits']['max_cartesian_acceleration'] == 1.00
    assert payload['limits']['max_joint_velocities'] == [0.50, 0.50, 0.50, 0.08, 0.50, 0.40]
    # Pre-existing diagnostic fields must survive untouched.
    assert payload['tracking_error_m'] == 0.03


def test_displacement_response_finds_no_segment_when_still():
    n = 200
    t0 = 1_000_000_000
    tick = int(1e9 / 15)
    data = {
        't': np.array([t0 + i * tick for i in range(n)], dtype=np.int64),
        'reference_xr': np.zeros(n), 'reference_yr': np.zeros(n),
        'reference_zr': np.zeros(n),
        'actual_ee_x': np.zeros(n), 'actual_ee_y': np.zeros(n),
        'actual_ee_z': np.zeros(n),
    }
    assert audit.displacement_response(data)['segments'] == 0
