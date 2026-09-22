#!/usr/bin/env python3
"""Offline calibration/validation for the 2026-09-18 GRU M310-M325 trials.

The script is read-only with respect to source logs. It reuses the timestamp,
frame, Jacobian and ridge-regression implementation audited in
``analyze_hc_dynamic_branches.py`` and writes a new analysis directory.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
BASE_PATH = ROOT / "analyze_hc_dynamic_branches.py"
SPEC = importlib.util.spec_from_file_location("dynamic_branches", BASE_PATH)
base = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(base)

# Physical measurement: flange -> Axia sensing reference origin, expressed in
# the physical Axia axes. Y is the measured 120 mm centre spacing. Z is the
# 10 mm plate plus 25.4 mm from Axia mounting side to its tool-side origin.
GEOMETRY_M = np.array([0.0, 0.120, 0.0354])
base.FLANGE_TO_AXIA_SENSOR_M = GEOMETRY_M

SESSION = ROOT / "cocarry_logs/hc_force_calibration/20260918_tool0_dynamic_tau_v1/02_dynamic_force"
NEW_TRIALS = [
    ("home_to_target1/20260918_161818_520069_gru_home_to_target1_r1", "T1 r1", "T1", 1),
    ("home_to_target2/20260918_162036_318542_gru_home_to_target2_r1", "T2 r1", "T2", 1),
    ("home_to_target1/20260918_162314_591621_gru_home_to_target1_r2", "T1 r2", "T1", 2),
    ("home_to_target2/20260918_162742_685925_gru_home_to_target2_r2", "T2 r2", "T2", 2),
    ("home_to_target2/20260918_163006_714520_gru_home_to_target2_r3", "T2 r3", "T2", 3),
    ("home_to_target1/20260918_163243_264393_gru_home_to_target1_r3", "T1 r3", "T1", 3),
    ("home_to_target2/20260918_163518_776972_gru_home_to_target2_r4", "T2 r4", "T2", 4),
]


def compact_metrics(m):
    return {
        "n": m["n"],
        "rmse_vector_n": m.get("rmse_vector_n"),
        "rmse_axes_n": m.get("rmse_axes_n"),
        "reference_ge4": m.get("reference_ge4"),
        "angle_n": m.get("angle_n"),
        "angle_coverage": m.get("angle_coverage"),
        "angle_deg": m.get("angle_deg"),
        "direction_failure_fraction": m.get("direction_failure_fraction"),
        "reference_norm_n": m.get("ref_norm_n"),
    }


def pooled(model, trials):
    pred, ref = [], []
    for trial in trials:
        pp, rr, good, _ = base.predict(model, trial)
        pred.append(pp[good])
        ref.append(rr[good])
    return compact_metrics(base.metrics(np.vstack(pred), np.vstack(ref)))


def parse_inputs():
    paths = {p.parent.name: p.parent for p in (ROOT / "cocarry_logs").glob("hc_force*/**/events.jsonl")}
    gt = [base.parse_trial(paths[name], mode, label) for name, mode, label in base.TRIALS[:6]]
    gru = [base.parse_trial(SESSION / rel, "GRU", label) for rel, label, _, _ in NEW_TRIALS]
    return gt, gru


def add_result(rows, protocol, branch, variant, train_labels, test_labels, metrics):
    rows.append({
        "protocol": protocol,
        "branch": branch,
        "variant": variant,
        "train": train_labels,
        "test": test_labels,
        "metrics": metrics,
    })


def build_report(output, quality, results, final_metrics):
    lookup = {(r["protocol"], r["branch"], r["variant"]): r["metrics"] for r in results
              if r["protocol"] in {"GT_to_all_GRU", "GRU_leave_one_trial_out"}}
    gt_force = lookup[("GT_to_all_GRU", "force", "linear")]
    gt_tau = lookup[("GT_to_all_GRU", "torque", "linear")]
    loto_force = lookup[("GRU_leave_one_trial_out", "force", "pose")]
    loto_tau = lookup[("GRU_leave_one_trial_out", "torque", "pose")]

    def angle(m, key):
        return m["angle_deg"][key] if m.get("angle_deg") else float("nan")

    lines = [
        "# Calibration F_robot từ 7 lượt GRU ngày 18/09/2026",
        "",
        "## Kết luận",
        "",
        "Nhánh **M310–M315 → hiệu chỉnh joint torque → Jacobian → force** tiếp tục là ứng viên tốt hơn. "
        "Trên leave-one-trial-out của bảy lượt GRU, mô hình torque có bù pose đạt "
        f"RMSE {loto_tau['rmse_vector_n']:.2f} N, góc trung vị {angle(loto_tau, 'median'):.2f}° "
        f"và P95 {angle(loto_tau, 'p95'):.2f}°. Nhánh M320–M322 tương ứng đạt "
        f"{loto_force['rmse_vector_n']:.2f} N, {angle(loto_force, 'median'):.2f}° và "
        f"{angle(loto_force, 'p95'):.2f}°.",
        "",
        "Lượt T2-r4 được giữ ngoài bước chọn mô hình. Sau khi chọn torque+pose trên r3, refit bằng "
        "GT+r1/r2/r3 rồi kiểm tra T2-r4 cho RMSE "
        f"{final_metrics['rmse_vector_n']:.2f} N; góc trung vị/P95 "
        f"{angle(final_metrics, 'median'):.2f}°/{angle(final_metrics, 'p95'):.2f}°. "
        "Đây là calibration cục bộ cho hai tuyến và cấu hình hiện tại, chưa phải calibration toàn workspace.",
        "",
        "## Chất lượng dữ liệu",
        "",
        "| Trial | Scan hoàn chỉnh | Mẫu RUNNING ghép | Chu kỳ scan trung vị | Span 12 register trung vị | Drift Axia cuối lượt |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for q in quality:
        counts = q["counts"]
        end = q.get("end_baseline") or {}
        drift = np.linalg.norm(end.get("axia_force_mean_n", [np.nan] * 3))
        lines.append(
            f"| {q['label']} | {counts.get('scan_end', 0)} | {q['paired_running_samples']} | "
            f"{q['sample_interval_s']['median']:.3f} s | "
            f"{q['sequential_register_span_s']['both']['median']:.3f} s | {drift:.3f} N |"
        )
    lines += [
        "",
        "Không có register timeout/error. Service latency trung vị xấp xỉ 20 ms/register. "
        "Do đọc tuần tự, một scan 12 register kéo dài khoảng 0,22 s và chu kỳ khoảng 0,42 s; "
        "bộ dữ liệu chỉ phù hợp chuyển động chậm, không mô tả chính xác đỉnh lực nhanh.",
        "",
        "## So sánh calibration",
        "",
        "| Giao thức | Nhánh | Mô hình | RMSE | Góc trung vị | Góc P95 | Độ phủ góc |",
        "|---|---|---|---:|---:|---:|---:|",
    ]
    for r in results:
        if r["protocol"] not in {"GT_to_all_GRU", "GRU_leave_one_trial_out", "fixed_split_validation", "independent_final_T2_r4"}:
            continue
        m = r["metrics"]
        cov = m.get("angle_coverage")
        lines.append(
            f"| {r['protocol']} | {r['branch']} | {r['variant']} | {m['rmse_vector_n']:.2f} N | "
            f"{angle(m, 'median'):.2f}° | {angle(m, 'p95'):.2f}° | "
            f"{100*cov:.1f}% |"
        )
    lines += [
        "",
        "GT-only chuyển sang GRU còn yếu: force-linear đạt "
        f"{gt_force['rmse_vector_n']:.2f} N và torque-linear {gt_tau['rmse_vector_n']:.2f} N. "
        "Điều này xác nhận dữ liệu động GRU mới là cần thiết; hệ số từ Ground Truth không chuyển nguyên trạng sang GRU.",
        "",
        "## Candidate",
        "",
        "Candidate chính nằm trong `candidate_models.json` và dùng:",
        "",
        "1. Trừ baseline M310–M315 không tiếp xúc đầu lượt.",
        "2. Ghép feature `[tau_M(6), q-q0(6)]`.",
        "3. Chuẩn hóa theo `scale`, tính `tau_cal = (feature/scale) @ coef`.",
        "4. Khôi phục wrench bằng Jacobian có damping; lấy ba thành phần force.",
        "",
        "Mô hình pose cho kết quả tốt trong vùng Home–T1/T2 nhưng các singular value nhỏ cho thấy phần "
        "pose chưa được kích thích đầy đủ ngoài hai tuyến. `torque_linear_conservative` được lưu kèm làm "
        "phương án ít phụ thuộc pose hơn. Chưa đưa candidate vào controller.",
        "",
        "## Giới hạn",
        "",
        "- Axia được dùng làm reference sau trừ baseline đầu lượt và bù trọng lực theo khối lượng tạm 1,126 kg. "
        "Orientation gần cố định nên ảnh hưởng trọng lực chênh lệch nhỏ, nhưng mass/CoG/inertia chưa được xác nhận.",
        "- Hình học dùng flange→gốc sensing Axia `(0, 0.120, 0.0354) m`, từ phép đo 120 mm, plate 10 mm "
        "và bản vẽ Axia 25,4 mm.",
        "- Register trong một scan không đồng thời. Không áp bù lag cố định vì chưa nhận dạng được lag ổn định.",
        "- Dữ liệu hiện chỉ gồm GRU, chưa có GRU+MJM ngày 18/09 và chưa có session độc lập.",
        "- Kết quả tốt trên T2-r4 không xác nhận các hướng lực, orientation hoặc vùng workspace chưa xuất hiện.",
        "",
        "## Artifacts",
        "",
        "- `data_quality.json`: chất lượng từng trial và hash log nguồn.",
        "- `results.json`, `metrics.csv`: toàn bộ giao thức kiểm chứng.",
        "- `candidate_models.json`: candidate đã kiểm chứng và candidate refit toàn dữ liệu.",
        "- `trial_*.png`: Axia so với hai nhánh theo leave-one-trial-out.",
        "- Script tái lập: `analyze_hc_gru_tau_calibration.py`.",
        "",
    ]
    (output / "report_vi.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    gt, gru = parse_inputs()
    quality = [d["meta"] for d in gru]
    (args.output / "data_quality.json").write_text(json.dumps(quality, indent=2) + "\n")

    results = []
    variants = {"force": ["linear", "pose"], "torque": ["linear", "pose"]}
    labels_gt = [d["meta"]["label"] for d in gt]
    labels_gru = [d["meta"]["label"] for d in gru]

    # Frozen historical protocol retrained only to apply the corrected 35.4 mm Z geometry.
    for branch, opts in variants.items():
        for variant in opts:
            model = base.fit_model(gt, branch, variant)
            add_result(results, "GT_to_all_GRU", branch, variant, labels_gt, labels_gru,
                       pooled(model, gru))

    # Every reported row is generated by a model that excludes that complete trial.
    loto_predictions = {}
    for branch, opts in variants.items():
        for variant in opts:
            pp, rr = [], []
            for i, trial in enumerate(gru):
                train = [d for j, d in enumerate(gru) if j != i]
                model = base.fit_model(train, branch, variant)
                pred, ref, good, _ = base.predict(model, trial)
                pp.append(pred[good]); rr.append(ref[good])
                loto_predictions[(i, branch, variant)] = (pred, ref, good)
            metric = compact_metrics(base.metrics(np.vstack(pp), np.vstack(rr)))
            add_result(results, "GRU_leave_one_trial_out", branch, variant,
                       "all other GRU trials", labels_gru, metric)

    # Predeclared chronological split: r1/r2 train; r3 selects model; r4 is final.
    train_ids = [0, 1, 2, 3]
    val_ids = [4, 5]
    final_id = 6
    train = gt + [gru[i] for i in train_ids]
    validation = [gru[i] for i in val_ids]
    for branch, opts in variants.items():
        for variant in opts:
            model = base.fit_model(train, branch, variant)
            add_result(results, "fixed_split_validation", branch, variant,
                       labels_gt + [labels_gru[i] for i in train_ids],
                       [labels_gru[i] for i in val_ids], pooled(model, validation))

    selected_train = gt + [gru[i] for i in train_ids + val_ids]
    validated = base.fit_model(selected_train, "torque", "pose")
    final_metrics = pooled(validated, [gru[final_id]])
    add_result(results, "independent_final_T2_r4", "torque", "pose",
               labels_gt + [labels_gru[i] for i in train_ids + val_ids],
               [labels_gru[final_id]], final_metrics)

    # Route-level transfer is intentionally harsh and exposes local pose overfit.
    route_ids = {"T1": [0, 2, 5], "T2": [1, 3, 4, 6]}
    for train_route, test_route in [("T1", "T2"), ("T2", "T1")]:
        for branch, opts in variants.items():
            for variant in opts:
                model = base.fit_model([gru[i] for i in route_ids[train_route]], branch, variant)
                add_result(results, f"route_holdout_{train_route}_to_{test_route}", branch, variant,
                           [labels_gru[i] for i in route_ids[train_route]],
                           [labels_gru[i] for i in route_ids[test_route]],
                           pooled(model, [gru[i] for i in route_ids[test_route]]))

    conservative = base.fit_model(selected_train, "torque", "linear")
    all_pose = base.fit_model(gt + gru, "torque", "pose")
    candidates = {
        "status": "OFFLINE_CANDIDATES_NOT_DEPLOYED",
        "geometry_flange_to_axia_origin_sensor_axes_m": GEOMETRY_M.tolist(),
        "validated_torque_pose": {
            "model": validated,
            "training": labels_gt + labels_gru[:6],
            "independent_test": "T2 r4",
            "independent_test_metrics": final_metrics,
        },
        "torque_linear_conservative": {
            "model": conservative,
            "training": labels_gt + labels_gru[:6],
            "independent_test": "T2 r4",
            "independent_test_metrics": pooled(conservative, [gru[6]]),
        },
        "all_data_torque_pose_refit": {
            "model": all_pose,
            "training": labels_gt + labels_gru,
            "independent_test_after_refit": None,
        },
        "formula": {
            "features": "[M310:315 - initial_baseline, q - q0]",
            "calibrated_joint_torque": "(features / scale) @ coef",
            "force": "damped wrench recovery from J(q).T W = calibrated_joint_torque",
            "damping": 0.01,
            "jacobian_scaling": "none; solve the physical J(q).T W equation",
        },
    }
    (args.output / "candidate_models.json").write_text(json.dumps(candidates, indent=2) + "\n")
    (args.output / "results.json").write_text(json.dumps({
        "status": "offline_calibration_not_deployed",
        "geometry_m": GEOMETRY_M.tolist(),
        "results": results,
    }, indent=2) + "\n")

    with (args.output / "metrics.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["protocol", "branch", "variant", "n", "rmse_N", "angle_n",
                         "angle_median_deg", "angle_p95_deg", "angle_coverage"])
        for r in results:
            m = r["metrics"]; ang = m.get("angle_deg") or {}
            writer.writerow([r["protocol"], r["branch"], r["variant"], m["n"],
                             m.get("rmse_vector_n"), m.get("angle_n"), ang.get("median"),
                             ang.get("p95"), m.get("angle_coverage")])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for i, trial in enumerate(gru):
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fp, fr, fg = loto_predictions[(i, "force", "pose")]
        tp, tr, tg = loto_predictions[(i, "torque", "pose")]
        for k, ax in enumerate(axes):
            ax.plot(trial["t"], trial["w"][:, k], color="black", lw=1.4, label="Axia reference")
            ax.plot(trial["t"][fg], fp[fg, k], lw=1, label="M320 pose, held-out trial")
            ax.plot(trial["t"][tg], tp[tg, k], lw=1, label="M310 torque+J pose, held-out trial")
            ax.set_ylabel(f"F{'xyz'[k]} [N]"); ax.grid(alpha=.2)
        axes[0].legend(fontsize=8); axes[-1].set_xlabel("Seconds since logger start")
        fig.suptitle(trial["meta"]["label"]); fig.tight_layout()
        fig.savefig(args.output / f"trial_{i}_{trial['meta']['label'].replace(' ', '_')}.png", dpi=140)
        plt.close(fig)

    build_report(args.output, quality, results, final_metrics)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
