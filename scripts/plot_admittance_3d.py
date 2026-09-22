#!/usr/bin/env python3
"""Plot the causal co-carrying position flow and measured human force.

The CSV columns are controller nominal ``x_d``, controller command ``x_ref``
and measured robot EE ``x_actual``.  ``nominal_*`` is conditioned/clamped and
must not be labelled as the raw GRU prediction.
"""
import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# ── Font Settings ──────────────────────────────────────────────────────────
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12
plt.rcParams['legend.fontsize'] = 11


def _rmse(a, b):
    return float(np.sqrt(np.mean(np.square(a - b))))


def estimate_tracking_delay(x_ref, x_actual, sample_period, max_lag_sec=1.0):
    """Estimate the causal delay x_ref[k] -> x_actual[k + lag]."""
    if len(x_ref) < 3 or not np.isfinite(sample_period) or sample_period <= 0.0:
        return 0, 0.0, _rmse(x_ref, x_actual)
    max_lag = min(int(round(max_lag_sec / sample_period)), len(x_ref) - 2)
    candidates = []
    for lag in range(max_lag + 1):
        command = x_ref[:-lag] if lag else x_ref
        measured = x_actual[lag:] if lag else x_actual
        candidates.append((_rmse(command, measured), lag))
    error, lag = min(candidates)
    return lag, lag * sample_period, error


def _role_spans(time_array, roles):
    """Return contiguous controller-role intervals for plot backgrounds."""
    roles = np.asarray(roles, dtype=str)
    if len(time_array) == 0 or len(roles) != len(time_array):
        return []
    spans = []
    start = 0
    for index in range(1, len(roles)):
        if roles[index] != roles[start]:
            spans.append((time_array[start], time_array[index], roles[start]))
            start = index
    end = time_array[-1]
    if len(time_array) > 1:
        end += time_array[-1] - time_array[-2]
    spans.append((time_array[start], end, roles[start]))
    return spans


def plot_admittance_csv(csv_path, save_dir, compensate_delay=True,
                        max_lag_sec=1.0, gaussian_sigma=0.0):
    print(f"Đang đọc file: {csv_path} ...")
    df = pd.read_csv(csv_path)

    # Lọc bỏ phần footer (các dòng có timestamp không phải là số)
    if 'ros_timestamp_ns' in df.columns:
        df = df[pd.to_numeric(df['ros_timestamp_ns'], errors='coerce').notnull()].copy()

    # x_d is the conditioned nominal used by the controller.  It may include
    # capture-pose conversion, prediction-reference filtering and lead limits.
    x_d_raw = df[['nominal_xd', 'nominal_yd', 'nominal_zd']].astype(float).values

    # x_ref is the Cartesian position command sent to the streamer.
    x_ref_raw = df[['reference_xr', 'reference_yr', 'reference_zr']].astype(float).values

    # x_actual is measured robot EE feedback (the plot's ground truth).
    x_actual_raw = df[['actual_ee_x', 'actual_ee_y', 'actual_ee_z']].astype(float).values
    roles = (df['role'].astype(str).values if 'role' in df.columns
             else np.full(len(df), 'UNKNOWN', dtype=str))

    # ── Lực: f_human ───────────────────────────────────────────────────────
    f_human_raw = df[['f_human_x', 'f_human_y', 'f_human_z']].astype(float).values

    # Accuracy is the default: do not silently smooth logged measurements.
    # A positive sigma remains available only for presentation copies.
    if gaussian_sigma > 0.0:
        from scipy.ndimage import gaussian_filter1d
        x_d = gaussian_filter1d(x_d_raw, sigma=gaussian_sigma, axis=0)
        x_ref = gaussian_filter1d(x_ref_raw, sigma=gaussian_sigma, axis=0)
        x_actual = gaussian_filter1d(x_actual_raw, sigma=gaussian_sigma, axis=0)
        f_human = gaussian_filter1d(f_human_raw, sigma=gaussian_sigma, axis=0)
    else:
        x_d, x_ref, x_actual, f_human = (
            x_d_raw, x_ref_raw, x_actual_raw, f_human_raw)
    filter_label = (f'light Gaussian σ={gaussian_sigma:g}'
                    if gaussian_sigma > 0.0 else 'raw, no Gaussian filter')
    print(f'Display signal: {filter_label}')

    # ── Tính thời gian thực từ timestamps ──────────────────────────────────
    time_array = None
    if 'ros_timestamp_ns' in df.columns:
        ts = pd.to_numeric(df['ros_timestamp_ns'], errors='coerce')
        time_array = (ts - ts.iloc[0]).values * 1e-9

    n = len(df)
    t = time_array if time_array is not None else np.arange(n) * 0.01
    positive_dt = np.diff(t)
    positive_dt = positive_dt[positive_dt > 0.0]
    sample_period = float(np.median(positive_dt)) if len(positive_dt) else 0.0
    lag_samples, delay_sec, aligned_rmse = estimate_tracking_delay(
        x_ref_raw, x_actual_raw, sample_period, max_lag_sec=max_lag_sec)
    synchronous_rmse = _rmse(x_ref_raw, x_actual_raw)

    # Commands are generated at t but appear in measured EE around t + delay.
    # Shifting their time axis preserves every sample and makes causal tracking
    # visible without rewriting/interpolating logged values.
    command_time = t + delay_sec if compensate_delay else t
    # GRU nominal is not active in LEADER.  Mask it rather than drawing a
    # misleading continuous line across the FOLLOWER -> LEADER handoff.
    leader_mask = roles == 'LEADER'
    # Also hide the two boundary samples which Matplotlib would otherwise use
    # to draw a short connecting segment into/out of the shaded LEADER region.
    expanded_leader_mask = leader_mask.copy()
    expanded_leader_mask[1:] |= leader_mask[:-1]
    expanded_leader_mask[:-1] |= leader_mask[1:]
    x_d_plot = np.array(x_d, copy=True)
    x_d_plot[expanded_leader_mask, :] = np.nan
    # Position commands are shifted by the estimated robot delay above, so the
    # role background must use that same axis to keep x_d outside LEADER.
    role_spans = _role_spans(command_time, roles)

    print(
        f"Tracking x_ref -> x_actual: synchronous RMSE="
        f"{synchronous_rmse * 1000.0:.2f} mm; best causal lag="
        f"{lag_samples} samples ({delay_sec:.3f} s); compensated RMSE="
        f"{aligned_rmse * 1000.0:.2f} mm")

    print("Đang vẽ đồ thị...")
    trajectory_name = os.path.splitext(os.path.basename(csv_path))[0]

    # ══════════════════════════════════════════════════════════════════════
    # Tạo figure: 2 cột × 3 hàng  (trái: vị trí, phải: lực)
    # ══════════════════════════════════════════════════════════════════════
    fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex=True)
    coords = ['X', 'Y', 'Z']

    for i in range(3):
        ax_pos = axes[i, 0]   # Cột trái: vị trí
        ax_frc = axes[i, 1]   # Cột phải: lực

        # White = FOLLOWER/normal background; pale red = direct MJM LEADER.
        # Keep the fill behind all signals and apply it to both columns.
        for start, end, role in role_spans:
            if role == 'LEADER':
                for axis in (ax_pos, ax_frc):
                    axis.axvspan(start, end, color='#f4a3a3', alpha=0.42,
                                 linewidth=0, zorder=0)

        # x_d and x_ref share controller-command time.  With compensation they
        # are displayed at the estimated time their effect reaches the robot.
        ax_pos.plot(command_time, x_d_plot[:, i], 'k--', linewidth=1.5,
                    label=r'$x_d$', zorder=2)
        ax_pos.plot(command_time, x_ref[:, i], 'r-', linewidth=1.2,
                    label=r'$x_a$', zorder=2)
        ax_pos.plot(t, x_actual[:, i], 'b:', linewidth=2.0,
                    label='Ground Truth', zorder=3)
        ax_pos.set_ylabel(f'{coords[i]} (m)')
        ax_pos.set_facecolor('white')
        ax_frc.set_facecolor('white')
        ax_pos.grid(True, alpha=0.3)

        # ── Cột phải: f_human ─────────────────────────────────────────
        ax_frc.plot(t, f_human[:, i], 'b-', linewidth=1.2,
                    label=r'$f_{human}$', zorder=2)
        ax_frc.set_ylabel(f'F_{coords[i]} (N)')
        ax_frc.grid(True, alpha=0.3)

        # ── Legend đặt ở ngoài đồ thị phía trên, có khung viền (border) ──────
        if i == 0:
            handles, labels = ax_pos.get_legend_handles_labels()
            handles.append(Patch(facecolor='#f4a3a3', alpha=0.42,
                                 label='LEADER'))
            fig.legend(handles=handles, loc='upper left',
                       bbox_to_anchor=(0.055, 0.995), ncol=4, frameon=True,
                       edgecolor='gray', facecolor='white', framealpha=0.9)
            force_handles, force_labels = ax_frc.get_legend_handles_labels()
            fig.legend(handles=force_handles, labels=force_labels,
                       loc='upper left', bbox_to_anchor=(0.545, 0.995),
                       ncol=1, frameon=True, edgecolor='gray',
                       facecolor='white', framealpha=0.9)

    # ── Trục x (hàng cuối) ─────────────────────────────────────────────
    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.955))

    # ── Lưu file ───────────────────────────────────────────────────────
    os.makedirs(save_dir, exist_ok=True)
    filter_suffix = (f'_gaussian_sigma_{gaussian_sigma:g}'
                     if gaussian_sigma > 0.0 else '_raw')
    out_path = os.path.join(
        save_dir, f'xd_xref_actual_force_{trajectory_name}{filter_suffix}.png')
    fig.savefig(out_path, dpi=150)
    plt.close('all')
    print(f"Đồ thị đã được lưu tại: {out_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot x_d -> x_ref -> actual EE with causal delay handling')
    parser.add_argument('csv_file')
    parser.add_argument('--save-dir', default='/home/hungnb/cocarry_ws/cocarry_logs')
    parser.add_argument('--max-lag-sec', type=float, default=1.0)
    parser.add_argument('--no-delay-compensation', action='store_true',
                        help='Plot every signal at its publication timestamp')
    filter_group = parser.add_mutually_exclusive_group()
    filter_group.add_argument(
        '--light-filter', action='store_true',
        help='Apply only a very light display filter (Gaussian sigma=0.5)')
    filter_group.add_argument(
        '--gaussian-sigma', type=float, default=None,
        help='Custom display-only smoothing; omit it to keep raw data')
    args = parser.parse_args()
    gaussian_sigma = (0.5 if args.light_filter else
                      0.0 if args.gaussian_sigma is None else
                      max(0.0, args.gaussian_sigma))
    plot_admittance_csv(
        args.csv_file,
        args.save_dir,
        compensate_delay=not args.no_delay_compensation,
        max_lag_sec=max(0.0, args.max_lag_sec),
        gaussian_sigma=gaussian_sigma,
    )
