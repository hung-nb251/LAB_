#!/usr/bin/env python3
"""Offline replay of co-carry simulation logs for wrist velocity limits.

The replay keeps the production IK structure: fixed captured orientation,
previous-solution seed, safe joint margins, and the local tool0 transform. It
does not publish ROS topics or change the streamer's production limits.
"""

import argparse
import csv
import importlib.util
import math
from pathlib import Path

import numpy as np


SIM_HOME_JOINTS = np.array([
    1.570742, 0.027735, -0.710960,
    0.000032, -0.847848, -0.000658,
], dtype=np.float64)
BASE_JOINT_VELOCITIES = np.array(
    [0.20, 0.20, 0.20, 0.08, 0.08, 0.08], dtype=np.float64)
SOFT_JOINT_LIMITS = np.array([
    (0.00, 3.14),
    (-0.80, 1.30),
    (-2.00, 1.25),
    (-2.50, 2.50),
    (-2.09, 0.52),
    (-2.50, 2.50),
], dtype=np.float64)
SAFE_JOINT_LIMITS = SOFT_JOINT_LIMITS + np.array([
    [math.radians(3.0), -math.radians(3.0)]
] * 6)

MAX_CARTESIAN_VELOCITY = 0.15
MAX_CARTESIAN_ACCELERATION = 0.50
MAX_CARTESIAN_JERK = 10.0


def _load_solver_class():
    path = Path(__file__).with_name('local_ik_solver.py')
    spec = importlib.util.spec_from_file_location('local_ik_solver', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LocalIKSolver


def _finite_row(row, fields):
    try:
        values = np.array([float(row[name]) for name in fields])
    except (KeyError, TypeError, ValueError):
        return None
    return values if np.all(np.isfinite(values)) else None


def _load_reference(path):
    samples = []
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            position = _finite_row(
                row, ('reference_xr', 'reference_yr', 'reference_zr'))
            actual = _finite_row(
                row, ('actual_ee_x', 'actual_ee_y', 'actual_ee_z'))
            if position is None or actual is None:
                continue
            try:
                timestamp = int(row['ros_timestamp_ns']) * 1e-9
            except (KeyError, TypeError, ValueError):
                continue
            samples.append((timestamp, position, actual))
    return samples


class CartesianSmoother:
    """Position-only equivalent of the production Cartesian smoothing path."""

    def __init__(self, initial_position):
        self.position = np.array(initial_position, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)

    def step(self, target, dt):
        error = np.asarray(target) - self.position
        distance = float(np.linalg.norm(error))
        if distance < 1e-6:
            desired_speed = 0.0
        else:
            decel_distance = (
                MAX_CARTESIAN_VELOCITY ** 2
                / (2.0 * MAX_CARTESIAN_ACCELERATION)
            )
            desired_speed = (
                math.sqrt(2.0 * MAX_CARTESIAN_ACCELERATION * distance)
                if distance < decel_distance else MAX_CARTESIAN_VELOCITY
            )
        desired_speed = min(desired_speed, MAX_CARTESIAN_VELOCITY)
        direction = error / distance if distance > 1e-5 else np.zeros(3)
        desired_velocity = desired_speed * direction

        acceleration = (desired_velocity - self.velocity) / dt
        max_delta_acceleration = (
            math.inf if distance < 0.020 else MAX_CARTESIAN_JERK * dt)
        delta_acceleration = acceleration - self.acceleration
        acceleration = self.acceleration + np.clip(
            delta_acceleration,
            -max_delta_acceleration,
            max_delta_acceleration,
        )
        acceleration_norm = float(np.linalg.norm(acceleration))
        if acceleration_norm > MAX_CARTESIAN_ACCELERATION:
            acceleration *= MAX_CARTESIAN_ACCELERATION / acceleration_norm
        self.acceleration = acceleration

        velocity = self.velocity + acceleration * dt
        speed = float(np.linalg.norm(velocity))
        if speed > MAX_CARTESIAN_VELOCITY:
            velocity *= MAX_CARTESIAN_VELOCITY / speed
        self.velocity = velocity

        overshoot = distance < 0.020 and float(np.dot(error, velocity)) < 0.0
        if overshoot or (distance < 0.002 and speed < 0.010):
            self.position = np.asarray(target).copy()
            self.velocity.fill(0.0)
            self.acceleration.fill(0.0)
        else:
            self.position = self.position + velocity * dt
        return self.position.copy()


def _percentile(values, percentile):
    return float(np.percentile(values, percentile)) if values else math.nan


def replay_file(path, wrist_limits):
    solver = _load_solver_class()()
    samples = _load_reference(path)
    if len(samples) < 2:
        raise ValueError(f'{path}: fewer than two valid samples')

    home_position, fixed_orientation = solver.fk_pose(SIM_HOME_JOINTS)
    initial_position = samples[0][2]
    initial_mismatch = float(np.linalg.norm(initial_position - home_position))
    smoother = CartesianSmoother(initial_position)
    desired_joints = SIM_HOME_JOINTS.copy()
    queued_by_limit = {
        limit: SIM_HOME_JOINTS.copy() for limit in wrist_limits
    }
    errors_by_limit = {limit: [] for limit in wrist_limits}
    gaps_by_limit = {limit: [] for limit in wrist_limits}
    demanded_wrist_rates = []
    ik_failures = 0
    solved = 0
    previous_time = samples[0][0]

    for timestamp, reference, _ in samples[1:]:
        dt = max(0.001, min(timestamp - previous_time, 0.2))
        previous_time = timestamp
        smoothed_position = smoother.step(reference, dt)
        solution = solver.solve_ik(
            smoothed_position,
            fixed_orientation,
            desired_joints,
            joint_limits=SAFE_JOINT_LIMITS,
        )
        if solution is None:
            ik_failures += 1
            continue

        solution = np.asarray(solution)
        demanded_wrist_rates.append(np.abs(solution[3:] - desired_joints[3:]) / dt)
        desired_joints = solution
        solved += 1

        for limit in wrist_limits:
            velocity_limits = BASE_JOINT_VELOCITIES.copy()
            velocity_limits[3:] = limit
            queued = queued_by_limit[limit]
            requested_velocity = (solution - queued) / dt
            queued = queued + np.clip(
                requested_velocity, -velocity_limits, velocity_limits) * dt
            queued_by_limit[limit] = queued
            queued_position = solver.fk_position(queued)
            errors_by_limit[limit].append(
                float(np.linalg.norm(queued_position - smoothed_position)))
            gaps_by_limit[limit].append(float(np.max(np.abs(solution - queued))))

    demanded = (
        np.vstack(demanded_wrist_rates)
        if demanded_wrist_rates else np.empty((0, 3)))
    results = {
        'path': str(path),
        'samples': len(samples),
        'solved': solved,
        'ik_failures': ik_failures,
        'initial_mismatch_m': initial_mismatch,
        'wrist_demand_p95_rad_s': (
            np.percentile(demanded, 95, axis=0).tolist()
            if len(demanded) else [math.nan] * 3),
        'wrist_demand_max_rad_s': (
            np.max(demanded, axis=0).tolist()
            if len(demanded) else [math.nan] * 3),
        'ik_min_singular_value': solver.get_stats()['ik_min_singular_value'],
        'ik_max_damping': solver.get_stats()['ik_max_damping'],
        'limits': {},
    }
    for limit in wrist_limits:
        errors = errors_by_limit[limit]
        gaps = gaps_by_limit[limit]
        results['limits'][limit] = {
            'wrist_demand_exceeded_pct': (
                100.0 * float(np.mean(np.any(demanded > limit, axis=1)))
                if len(demanded) else math.nan),
            'tracking_mean_mm': 1000.0 * (sum(errors) / len(errors)),
            'tracking_p95_mm': 1000.0 * _percentile(errors, 95),
            'tracking_max_mm': 1000.0 * max(errors),
            'joint_gap_p95_rad': _percentile(gaps, 95),
            'joint_gap_max_rad': max(gaps),
        }
    return results


def _print_result(result):
    print(f'\n{result["path"]}')
    print(
        f'  samples={result["samples"]}, solved={result["solved"]}, '
        f'ik_failures={result["ik_failures"]}, '
        f'initial_mismatch={result["initial_mismatch_m"] * 1000.0:.2f} mm')
    p95 = result['wrist_demand_p95_rad_s']
    maximum = result['wrist_demand_max_rad_s']
    print(
        '  unconstrained wrist demand R/B/T [rad/s]: '
        f'p95={np.round(p95, 3)}, max={np.round(maximum, 3)}')
    print(
        f'  adaptive DLS: min_sigma={result["ik_min_singular_value"]:.5f}, '
        f'max_lambda={result["ik_max_damping"]:.5f}')
    for limit, metrics in result['limits'].items():
        print(
            f'  wrist_limit={limit:.2f} rad/s | '
            f'demand_exceeded={metrics["wrist_demand_exceeded_pct"]:.1f}% | '
            f'tracking mean/p95/max='
            f'{metrics["tracking_mean_mm"]:.1f}/'
            f'{metrics["tracking_p95_mm"]:.1f}/'
            f'{metrics["tracking_max_mm"]:.1f} mm | '
            f'joint_gap p95/max={metrics["joint_gap_p95_rad"]:.3f}/'
            f'{metrics["joint_gap_max_rad"]:.3f} rad')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument(
        '--wrist-limits', nargs='+', type=float,
        default=[0.08, 0.12, 0.15], metavar='RAD_S')
    args = parser.parse_args()
    wrist_limits = sorted(set(args.wrist_limits))
    if not wrist_limits or any(limit <= 0.0 for limit in wrist_limits):
        parser.error('wrist limits must be positive')

    for path in args.logs:
        _print_result(replay_file(path, wrist_limits))


if __name__ == '__main__':
    main()
