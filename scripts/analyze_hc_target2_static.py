#!/usr/bin/env python3
"""Reproducible local Target 2 calibration; offline only."""
import argparse
import json
from pathlib import Path
import numpy as np
from calibrate_hc_register_force import parse, fit, metrics


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial', type=Path, required=True)
    p.add_argument('--previous', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    rows, quality = parse(args.trial)
    matrix = fit(rows)
    phases = sorted({r['phase'] for r in rows})
    old = json.loads(args.previous.read_text())
    holdout = {}
    for phase in phases:
        train = [r for r in rows if r['phase'] != phase]
        test = [r for r in rows if r['phase'] == phase]
        holdout[phase] = metrics(test, fit(train))
    result = dict(status='local_static_candidate_not_deployed',
        formula='F_base_N = A @ (0.1*(M320_M322_raw-10000) - baseline_N)',
        quality=quality, matrix=matrix.T.tolist(),
        in_sample=metrics(rows, matrix),
        leave_one_direction_out=holdout,
        transfer_from_previous={name: metrics(rows, np.asarray(v['candidate_matrix_all_repeats']).T)
            for name,v in old['local'].items()},
        limitations=['Only one action phase per direction; no independent repeated trial.',
            'Leave-one-direction-out is harder than repetition testing and is not movement validation.',
            'Recorded TF plus source-default yaw -90 deg; unchanged mounting assumed.',
            'Axia raw baseline subtraction applies only at the fixed pose.',
            'Angles exclude samples where reference or estimate norm is below 4 N.',
            'No pose-dependent baseline model or conflict-classification validation is supplied.'])
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'target2_static_candidate.json').write_text(json.dumps(result,indent=2)+'\n')
    lines = ['# Target 2 static force — 18:05:09', '',
        'Đã fit candidate cục bộ, chưa triển khai hoặc xác nhận calibration toàn hành trình.', '',
        f"Scan hoàn chỉnh: {quality['complete_scans']}; timeout: {quality['timeout_events']}; "
        f"biên độ joint lớn nhất: {quality['joint_range_max_rad']:.8f} rad.",
        f"Baseline M320–M322: {np.round(quality['register_baseline_n'],3).tolist()} N.",
        f"Mẫu active sau lọc biên marker: {len(rows)}. Có đủ sáu hướng, mỗi hướng một pha `_1`.", '',
        'Công thức: `F_base = A @ (decoded_M - baseline)`; A:', '```text',
        np.array2string(matrix.T,precision=6), '```', '',
        '## Đánh giá', '',
        '| Phép kiểm tra | RMSE vector N | Mẫu góc / tổng | Góc trung vị | Góc P95 |',
        '|---|---:|---:|---:|---:|']
    evaluations = {'Fit cùng dữ liệu (không độc lập)':result['in_sample']}
    evaluations.update({'Bỏ pha '+k+' khỏi fit':v for k,v in holdout.items()})
    evaluations.update({'Ma trận cũ '+k:v for k,v in result['transfer_from_previous'].items()})
    for label,m in evaluations.items():
        lines.append(f"| {label} | {m['vector_rmse_n']:.2f} | {m['angle_n']}/{m['n']} | "
                     f"{m['angle_median_deg']:.2f}° | {m['angle_p95_deg']:.2f}° |")
    lines += ['',
        'Các ma trận cũ được giữ nguyên nhưng dùng baseline đo tại Target 2 mới. '
        'Đây là kiểm chứng chuyển giao ma trận, không phải kiểm chứng mô hình baseline.',
        'Norm register sau trừ baseline trong các pha release: P95 '
        f"{quality['release_register_norm_p95_n']:.2f} N; Axia raw tương ứng "
        f"{quality['release_axia_norm_p95_n']:.2f} N.",
        'Không dùng số liệu in-sample để tuyên bố đã calibrate khi chuyển động. '
        'Không ép dấu, không sửa RPY, không cập nhật controller hoặc Tool Data.',
        'Chưa yêu cầu thu thêm ngay: giữ trial này làm dữ liệu Target 2, sử dụng '
        'kết quả kiểm chứng theo pha để quyết định mô hình phù hợp. Lượt chạy động '
        'và mô hình baseline theo pose vẫn là phần chưa được xác nhận.']
    (args.output/'report_vi.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
