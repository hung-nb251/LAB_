#!/usr/bin/env python3
"""Summarize a HC static-CoG session and fit tau = r x F + bias."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def latest_trials(static_root):
    result = []
    for pose_dir in sorted(p for p in static_root.iterdir() if p.is_dir()):
        trials = sorted(
            d for d in pose_dir.iterdir()
            if d.is_dir() and (d / 'events.jsonl').exists())
        if trials:
            result.append((pose_dir.name, trials[-1]))
    return result


def analyze_trial(pose, trial_dir, window_sec):
    raw = []
    joints = []
    registers = []
    register_scans = {}
    counts = {}
    calibrated = []
    connected = []
    stop_ns = None
    start_ns = None

    with (trial_dir / 'events.jsonl').open() as stream:
        for line in stream:
            event = json.loads(line)
            kind = event['kind']
            counts[kind] = counts.get(kind, 0) + 1
            if kind == 'start':
                start_ns = event['wall_ns']
            elif kind == 'stop':
                stop_ns = event['wall_ns']
            elif kind == 'register':
                register_scans.setdefault(event['scan_id'], set()).add(
                    event['address'])
                response = event.get('response', {})
                if response.get('success'):
                    registers.append([
                        event['wall_ns'], event['address'], response['value'],
                        event.get('scaled'),
                    ])
            elif kind == 'topic':
                topic = event['topic']
                message = event['message']
                if topic == '/axia/raw_wrench':
                    wrench = message['wrench']
                    raw.append([
                        event['wall_ns'],
                        *[wrench['force'][axis] for axis in 'xyz'],
                        *[wrench['torque'][axis] for axis in 'xyz'],
                    ])
                elif topic == '/joint_states':
                    position = message.get('position', [])
                    if len(position) >= 6:
                        joints.append([event['wall_ns'], *position[:6]])
                elif topic == '/axia/calibrated':
                    calibrated.append(bool(message['data']))
                elif topic == '/axia/connected':
                    connected.append(bool(message['data']))

    row = {
        'pose': pose,
        'trial_dir': str(trial_dir),
        'duration_sec': ((stop_ns - start_ns) / 1e9
                         if start_ns is not None and stop_ns is not None else 0.0),
        'raw_count': len(raw),
        'joint_count': len(joints),
        'full_register_scans': sum(
            len(addresses) == 24 for addresses in register_scans.values()),
        'register_timeouts': counts.get('register_timeout', 0),
        'register_errors': counts.get('register_error', 0),
        'connected_all_true': bool(connected) and all(connected),
        'calibrated_all_true': bool(calibrated) and all(calibrated),
        'usable': False,
        'reason': '',
    }
    if stop_ns is None or not raw:
        row['reason'] = 'missing_stop_or_raw'
        return row
    if not joints:
        row['reason'] = 'missing_joint_states'
        return row

    cutoff_ns = stop_ns - int(window_sec * 1e9)
    raw_window = np.asarray([sample[1:] for sample in raw
                             if sample[0] >= cutoff_ns], dtype=float)
    joint_window = np.asarray([sample[1:] for sample in joints
                               if sample[0] >= cutoff_ns], dtype=float)
    if len(raw_window) < window_sec * 50 or len(joint_window) < window_sec * 20:
        row['reason'] = 'stable_window_too_short'
        return row

    mean = raw_window.mean(axis=0)
    std = raw_window.std(axis=0)
    joint_mean = joint_window.mean(axis=0)
    joint_range = np.ptp(joint_window, axis=0)
    row.update({
        **{f'raw_mean_{name}': float(value) for name, value in zip(
            ('fx', 'fy', 'fz', 'tx', 'ty', 'tz'), mean)},
        **{f'raw_std_{name}': float(value) for name, value in zip(
            ('fx', 'fy', 'fz', 'tx', 'ty', 'tz'), std)},
        **{f'joint_mean_{index + 1}': float(value)
           for index, value in enumerate(joint_mean)},
        **{f'joint_range_{index + 1}': float(value)
           for index, value in enumerate(joint_range)},
        'window_samples': len(raw_window),
        'usable': (row['connected_all_true'] and row['calibrated_all_true']
                   and float(joint_range.max()) < 0.002),
    })
    if registers:
        register_cutoff_ns = registers[-1][0] - int(window_sec * 1e9)
        register_window = [sample for sample in registers
                           if sample[0] >= register_cutoff_ns]
        for address in sorted({sample[1] for sample in register_window}):
            samples = [sample for sample in register_window
                       if sample[1] == address]
            row[f'm{address}_raw_mean'] = float(np.mean(
                [sample[2] for sample in samples]))
            scaled = [sample[3] for sample in samples
                      if sample[3] is not None]
            if scaled:
                row[f'm{address}_scaled_mean'] = float(np.mean(scaled))
    if not row['usable']:
        row['reason'] = 'health_or_motion_check_failed'
    return row


def fit_cog(rows):
    design = []
    target = []
    forces = []
    for row in rows:
        fx, fy, fz = [row[f'raw_mean_{axis}'] for axis in ('fx', 'fy', 'fz')]
        tx, ty, tz = [row[f'raw_mean_t{axis}'] for axis in 'xyz']
        design.extend([
            [0.0, fz, -fy, 1.0, 0.0, 0.0],
            [-fz, 0.0, fx, 0.0, 1.0, 0.0],
            [fy, -fx, 0.0, 0.0, 0.0, 1.0],
        ])
        target.extend([tx, ty, tz])
        forces.append([fx, fy, fz])
    design = np.asarray(design)
    target = np.asarray(target)
    solution, _, rank, singular = np.linalg.lstsq(design, target, rcond=None)
    residual = (design @ solution - target).reshape(-1, 3)

    unit_forces = np.asarray(forces)
    unit_forces /= np.linalg.norm(unit_forces, axis=1, keepdims=True)
    angles = []
    for left in range(len(unit_forces)):
        for right in range(left + 1, len(unit_forces)):
            cosine = float(np.clip(unit_forces[left] @ unit_forces[right], -1, 1))
            angles.append(math.degrees(math.acos(cosine)))

    loo_cog = []
    for omitted in range(len(rows)):
        mask = np.ones(len(target), dtype=bool)
        mask[3 * omitted:3 * omitted + 3] = False
        loo_cog.append(np.linalg.lstsq(
            design[mask], target[mask], rcond=None)[0][:3])
    loo_cog = np.asarray(loo_cog)
    return {
        'model': 'tau_sensor = r_cog_cross_force_sensor + constant_torque_bias',
        'pose_count': len(rows),
        'cog_sensor_m': solution[:3].tolist(),
        'cog_sensor_mm': (solution[:3] * 1000).tolist(),
        'torque_bias_nm': solution[3:].tolist(),
        'torque_rmse_nm_per_axis': np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
        'torque_rmse_nm_all': float(np.sqrt(np.mean(residual ** 2))),
        'rank': int(rank),
        'condition_number': float(singular[0] / singular[-1]),
        'max_force_direction_separation_deg': max(angles),
        'median_force_direction_separation_deg': float(np.median(angles)),
        'loo_cog_std_mm': (loo_cog.std(axis=0) * 1000).tolist(),
        'loo_cog_min_mm': (loo_cog.min(axis=0) * 1000).tolist(),
        'loo_cog_max_mm': (loo_cog.max(axis=0) * 1000).tolist(),
        'provisional_only': max(angles) < 30.0,
        'per_pose_torque_residual_norm_nm': {
            row['pose']: float(np.linalg.norm(error))
            for row, error in zip(rows, residual)
        },
    }


def write_report(path, all_rows, usable_rows, fit, window_sec):
    excluded = [row for row in all_rows if not row['usable']]
    timeout_rows = [row for row in usable_rows if row['register_timeouts']]
    lines = [
        '# Phân tích pose tĩnh CoG', '',
        f'- Cửa sổ phân tích thống nhất: {window_sec:.0f} giây cuối mỗi trial.',
        f'- Pose tìm thấy: {len(all_rows)}; hợp lệ: {len(usable_rows)}; '
        f'loại: {len(excluded)}.',
        f'- Góc lớn nhất giữa các hướng lực Axia: '
        f'{fit["max_force_direction_separation_deg"]:.2f}°.',
        '- Trung bình M310–M345 của cửa sổ cuối được lưu trong '
        '`static_pose_summary.csv`.',
        '',
        '## Chất lượng dữ liệu', '',
    ]
    for row in all_rows:
        status = 'DÙNG' if row['usable'] else f'LOẠI ({row["reason"]})'
        lines.append(
            f'- `{row["pose"]}`: {status}; joint={row["joint_count"]}; '
            f'scan M đầy đủ={row["full_register_scans"]}; '
            f'timeout={row["register_timeouts"]}.')
    lines += ['', '## Fit CoG Axia tạm thời', '',
              'Mô hình dùng trung bình raw wrench tại từng pose:', '',
              '```text',
              'tau_sensor = r_CoG × F_sensor + torque_bias',
              '```', '',
              f'- CoG tạm thời trong frame Axia: '
              f'`[{fit["cog_sensor_mm"][0]:.2f}, '
              f'{fit["cog_sensor_mm"][1]:.2f}, '
              f'{fit["cog_sensor_mm"][2]:.2f}] mm`.',
              f'- Torque bias: `[{fit["torque_bias_nm"][0]:.4f}, '
              f'{fit["torque_bias_nm"][1]:.4f}, '
              f'{fit["torque_bias_nm"][2]:.4f}] Nm`.',
              f'- RMSE torque tổng: {fit["torque_rmse_nm_all"]:.4f} Nm.',
              f'- Condition number: {fit["condition_number"]:.1f}.',
              f'- Độ lệch chuẩn leave-one-pose-out của CoG: '
              f'`[{fit["loo_cog_std_mm"][0]:.2f}, '
              f'{fit["loo_cog_std_mm"][1]:.2f}, '
              f'{fit["loo_cog_std_mm"][2]:.2f}] mm`.', '',
              '## Kết luận', '']
    if fit['provisional_only']:
        lines.append(
            'Kết quả CoG trên chỉ là fit tạm thời. Các hướng lực thay đổi tối đa '
            'dưới 30°, nên bộ pose chưa kích thích orientation đủ rộng để tách '
            'CoG khỏi torque bias một cách chắc chắn. Cần bổ sung các pose thực '
            'sự nghiêng tool quanh ít nhất hai trục; thay đổi XYZ/joint trong khi '
            'giữ orientation tool gần cố định không giải quyết được bài toán.')
    if timeout_rows:
        lines += ['', 'Các pose có timeout M nhưng vẫn còn nhiều scan hoàn chỉnh: '
                  + ', '.join(row['pose'] for row in timeout_rows) + '.']
    path.write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir', type=Path)
    parser.add_argument('--window-sec', type=float, default=60.0)
    args = parser.parse_args()
    static_root = args.session_dir / '01_static_cog'
    if not static_root.is_dir():
        parser.error(f'Không tìm thấy {static_root}')

    rows = [analyze_trial(pose, trial, args.window_sec)
            for pose, trial in latest_trials(static_root)]
    usable = [row for row in rows if row['usable']]
    if len(usable) < 4:
        parser.error('Cần ít nhất 4 pose hợp lệ để fit')
    fit = fit_cog(usable)

    output = args.session_dir / '03_analysis'
    output.mkdir(exist_ok=True)
    keys = sorted({key for row in rows for key in row})
    with (output / 'static_pose_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    (output / 'static_cog_fit.json').write_text(
        json.dumps(fit, indent=2, ensure_ascii=False) + '\n')
    write_report(
        output / 'static_cog_analysis_vi.md', rows, usable, fit, args.window_sec)
    print(output)


if __name__ == '__main__':
    main()
