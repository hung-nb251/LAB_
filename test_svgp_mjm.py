"""
test_svgp_mjm.py
================
Kiểm tra cơ chế HYBRID: SVGP (FOLLOWER) → Minimum Jerk (LEADER)

Từ t = 0 đến t = T_SWITCH  : FOLLOWER mode — dùng SVGP dự đoán quỹ đạo ngắn hạn
                               (bám theo chuyển động người dùng từ dữ liệu CSV).
Từ t = T_SWITCH trở đi       : LEADER mode  — dùng Minimum Jerk Model để dẫn
                               robot từ vị trí hiện tại đến đích cố định.

Không có:
  - Goal Classification (GMM)
  - Disagreement Detection
Chỉ kiểm tra khả năng tạo quỹ đạo ngắn hạn và dài hạn.
"""

import os
import sys
import json
import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# ─── 1. CẤU HÌNH (CÓ THỂ ĐIỀU CHỈNH) ────────────────────────────────────────

# Thư mục chứa dữ liệu quỹ đạo để replay
DATA_FOLDER         = "GroundTruth_Data"
TEST_LIST_FILE      = os.path.join(DATA_FOLDER, "test_file_list.json")
SVGP_CHECKPOINT_DIR = "pHRI_Models/svgp_hri_m52"
T_SWITCH            = 5.0   # (s)
GOAL                = np.array([-0.0578, 0.7138, 0.0])
GOAL_MODE           = 'Default'   # 'Manual' hoặc 'Default'
SVGP_HISTORY_SIZE   = 10
SIGMA               = 0.2
DT                  = 12 / 191
SAVE_DIR            = "svgp_mjm_test_results"
MAX_FILES           = None

# ─── 2. CÁC HÀM TIỆN ÍCH ────────────────────────────────────────────────────

def load_svgp_system(checkpoint_dir):
    """Load SVGP model và scalers từ thư mục checkpoint."""
    try:
        with open(os.path.join(checkpoint_dir, "scaler_x.pkl"), "rb") as f:
            scaler_x = pickle.load(f)
        with open(os.path.join(checkpoint_dir, "scaler_y.pkl"), "rb") as f:
            scaler_y = pickle.load(f)
        with open(os.path.join(checkpoint_dir, "svgp_model.pkl"), "rb") as f:
            model = pickle.load(f)
        print(f"✅ Đã load SVGP model từ: {checkpoint_dir}")
        return scaler_x, scaler_y, model
    except FileNotFoundError as e:
        print(f"❌ Không tìm thấy SVGP checkpoint: {e}")
        return None, None, None


def predict_next_point_svgp(points_history, scaler_x, scaler_y, model,
                             history_size):
    """Dự đoán điểm tiếp theo của quỹ đạo người dùng bằng SVGP.
    
    Args:
        points_history: mảng (N, 3) các điểm lịch sử X,Y,Z gần nhất
        scaler_x, scaler_y, model: SVGP system đã load
        history_size: số điểm cần lấy làm cửa sổ đầu vào
    
    Returns:
        predicted_point: ndarray (3,) - điểm dự đoán tiếp theo
    """
    L = points_history.shape[0]
    padding_value = points_history[0] if L > 0 else np.zeros(3)

    # Tạo cửa sổ đầu vào: padding bằng điểm đầu nếu chưa đủ history
    current_window = np.tile(padding_value, (history_size, 1)).astype(np.float64)
    num_actual = min(L, history_size)
    if num_actual > 0:
        current_window[history_size - num_actual:] = points_history[L - num_actual:]

    # Scale và predict
    X_input_scaled = scaler_x.transform(current_window).flatten().reshape(1, -1)
    mean_scaled, _ = model.predict_f(X_input_scaled)
    predicted_point = scaler_y.inverse_transform(mean_scaled.numpy()).flatten()
    return predicted_point


def fitts_law_duration(x_current, x_goal, dt_file, a=4.4455, b=1.4248, w=0.3):
    """Ước lượng thời gian di chuyển bằng Fitts' Law."""
    D = np.linalg.norm(x_current - x_goal)
    t_f = a + b * np.log2(2 * D / w)
    return max(t_f, dt_file * 5)


def dynamic_minimum_jerk_trajectory(x_0, v_0, x_f, t_total, dt, a_0=None):
    """Tạo toàn bộ quỹ đạo Minimum Jerk từ x_0 đến x_f trong t_total giây.
    
    Sử dụng phương trình đa thức bậc 5 kinh điển:
        x(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    
    Returns:
        trajectory: ndarray (N, 3)
    """
    if a_0 is None:
        a_0 = np.zeros_like(x_0)

    t_total = max(t_total, dt)
    n_steps = int(t_total / dt) + 1
    t_array = np.linspace(0, t_total, n_steps)
    t = t_array[:, np.newaxis]

    T = t_total
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    delta_x = x_f - x_0

    c0 = x_0
    c1 = v_0
    c2 = a_0 / 2.0
    c3 = (20 * delta_x - (12 * v_0) * T - (3 * a_0) * T2) / (2 * T3)
    c4 = (-30 * delta_x + (16 * v_0) * T + (3 * a_0) * T2) / (2 * T4)
    c5 = (12 * delta_x - (6 * v_0) * T - a_0 * T2) / (2 * T5)

    trajectory = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
    return trajectory

# ─── 3. HÀM XỬ LÝ CHÍNH CHO TỪNG FILE ──────────────────────────────────────

def run_hybrid_test(filepath, svgp_scaler_x, svgp_scaler_y, svgp_model,
                    save_dir=None):
    """
    Chạy thử nghiệm hybrid SVGP → MJM trên một file quỹ đạo.

    Returns:
        dict chứa kết quả (mae_follower, mae_leader, etc.) hoặc None nếu lỗi
    """
    fname = os.path.basename(filepath)

    # --- Đọc và làm mịn dữ liệu ---
    try:
        df = pd.read_csv(filepath).dropna()
    except Exception as e:
        print(f"  ❌ Lỗi đọc file {fname}: {e}")
        return None

    required_cols = {'X', 'Y', 'Z', 'wall_time'}
    if not required_cols.issubset(df.columns):
        print(f"  ⚠️ Bỏ qua {fname}: thiếu cột X/Y/Z/wall_time")
        return None

    # Parse timestamps để lấy đúng DT của file
    df['wall_time'] = pd.to_datetime(df['wall_time'])
    timestamps = (df['wall_time'] - df['wall_time'].iloc[0]).dt.total_seconds().values
    
    dt_array = np.diff(timestamps)
    dt_file = np.median(dt_array) if len(dt_array) > 0 else DT

    # Áp dụng Gaussian Filter làm mịn
    X_raw = gaussian_filter1d(df['X'].values, sigma=SIGMA)
    Y_raw = gaussian_filter1d(df['Y'].values, sigma=SIGMA)
    Z_raw = gaussian_filter1d(df['Z'].values, sigma=SIGMA)
    ground_truth = np.stack([X_raw, Y_raw, Z_raw], axis=1)  # (N, 3)

    # ─── Lọc nhiễu / cắt đuôi rác ở cuối (như test_fitts_law_cocarry.py) ───
    V_THRESH = 0.02
    # Tính velocity dùng smoothed array sigma=1 giống test_fitts_law để đỡ nhiễu
    smoothed_for_vel = gaussian_filter1d(ground_truth, sigma=1, axis=0)
    velocities = np.linalg.norm(np.diff(smoothed_for_vel, axis=0), axis=1) / dt_array
    
    active_indices = np.where(velocities > V_THRESH)[0]
    if len(active_indices) > 0:
        last_active = int(active_indices[-1])
        ground_truth = ground_truth[:last_active + 2]

    N = len(ground_truth)

    if GOAL_MODE == 'Manual':
        current_goal = ground_truth[-1].copy()
    else:
        current_goal = GOAL.copy()

    if N < SVGP_HISTORY_SIZE + 2:
        print(f"  ⚠️ Bỏ qua {fname}: quỹ đạo quá ngắn ({N} điểm).")
        return None

    # Tính số bước chuyển đổi
    n_switch = int(T_SWITCH / dt_file)
    n_switch = min(n_switch, N - 1)

    # --- Khởi tạo kết quả ---
    x_ref_list = []     # Quỹ đạo reference tạo ra từ thuật toán hybrid
    modes_list = []     # Ghi lại mode tại từng bước
    history    = []     # Buffer lịch sử điểm thực (ground truth, dùng cho SVGP)

    # ─── Giai đoạn FOLLOWER (SVGP) ─────────────────────────────────────────
    print(f"  🔵 FOLLOWER (SVGP): bước 0 → {n_switch} (t=0s → {n_switch*dt_file:.2f}s)")

    x_ref = ground_truth[0].copy()
    x_ref_list.append(x_ref.copy())
    history.append(ground_truth[0].copy())
    modes_list.append("FOLLOWER")

    for step in range(1, n_switch):
        history.append(ground_truth[step].copy())
        history_arr = np.array(history)

        # SVGP predict điểm tiếp theo
        x_ref = predict_next_point_svgp(
            history_arr, svgp_scaler_x, svgp_scaler_y, svgp_model,
            SVGP_HISTORY_SIZE
        )
        x_ref_list.append(x_ref.copy())
        modes_list.append("FOLLOWER")

    # ─── Tính vận tốc tại thời điểm chuyển đổi ─────────────────────────────
    x_switch = x_ref.copy()   # Vị trí cuối FOLLOWER = X_0 của MJM

    # ─── Giai đoạn LEADER (Current MJM) ─────────────────────────────
    n_leader = N - n_switch
    t_remaining = n_leader * dt_file
    # Tính t_f theo Fitts' Law (KHÔNG cấp min với t_remaining để hiện thị sự chênh lệch thời gian)
    t_fitts = fitts_law_duration(x_switch, current_goal, dt_file)

    print(f"  🔴 LEADER (Current MJM): bước {n_switch} → {N} (t={n_switch*dt_file:.2f}s → {N*dt_file:.2f}s)")
    print(f"     X_0 (x_switch) = ({x_switch[0]:.4f}, {x_switch[1]:.4f}, {x_switch[2]:.4f})")
    print(f"     X_f (GOAL)     = ({current_goal[0]:.4f}, {current_goal[1]:.4f}, {current_goal[2]:.4f})")
    print(f"     Thời gian CÒN LẠI dự đoán (Fitts) = {t_fitts:.2f}s | CÒN LẠI thực tế = {t_remaining:.2f}s")

    # Tạo toàn bộ quỹ đạo MJM (v_0 = 0 giống như outer_loop.py)
    mjm_traj = dynamic_minimum_jerk_trajectory(x_switch, np.zeros(3), current_goal, t_fitts, dt_file)

    # Nối toàn bộ quỹ đạo MJM vào kết quả
    for pt in mjm_traj:
        x_ref_list.append(pt.copy())
        modes_list.append("LEADER")

    # Lấy bản sao chưa pad để vẽ đồ thị
    gt_unpadded = ground_truth.copy()
    x_ref_unpadded = np.array(x_ref_list)
    modes_unpadded = modes_list.copy()

    # ─── Căn chỉnh độ dài (Pad để tính MAE) ─────────────────────────────────
    max_L = max(len(ground_truth), len(x_ref_list))

    # Pad ground_truth bằng vị trí cuối cùng của người dùng (người dừng lại)
    if len(ground_truth) < max_L:
        pad_len = max_L - len(ground_truth)
        pad_arr = np.tile(ground_truth[-1], (pad_len, 1))
        ground_truth_L = np.vstack([ground_truth, pad_arr])
    else:
        ground_truth_L = ground_truth.copy()

    # Pad x_ref_list bằng vị trí current_goal (robot dừng tại đích)
    x_ref_padded_list = x_ref_list.copy()
    modes_padded_list = modes_list.copy()
    if len(x_ref_list) < max_L:
        pad_len = max_L - len(x_ref_list)
        for _ in range(pad_len):
            x_ref_padded_list.append(current_goal.copy())
            modes_padded_list.append("LEADER")

    x_ref_arr = np.array(x_ref_padded_list)
    modes_L   = modes_padded_list

    # ─── Tính MAE ───────────────────────────────────────────────────────────
    errors = np.linalg.norm(x_ref_arr - ground_truth_L, axis=1)

    follower_mask = np.array([m == "FOLLOWER" for m in modes_L])
    leader_mask   = np.array([m == "LEADER"   for m in modes_L])

    mae_total    = np.mean(errors)
    mae_follower = np.mean(errors[follower_mask]) if follower_mask.any() else 0.0
    mae_leader   = np.mean(errors[leader_mask])   if leader_mask.any()   else 0.0

    dist_final = np.linalg.norm(x_ref_arr[-1] - current_goal)
    print(f"  📊 MAE total={mae_total:.4f}m | FOLLOWER={mae_follower:.4f}m | LEADER={mae_leader:.4f}m")
    print(f"  🎯 Khoảng cách đến đích ở điểm cuối: {dist_final:.4f}m")

    # ─── Vẽ đồ thị ──────────────────────────────────────────────────────────
    if save_dir:
        # Truyền mảng chưa pad để vẽ đồ thị
        _plot_result(gt_unpadded, x_ref_unpadded, modes_unpadded, fname, save_dir,
                     mae_follower, mae_leader, dt_file, current_goal)

    return {
        'file'        : fname,
        'mae_total'   : mae_total,
        'mae_follower': mae_follower,
        'mae_leader'  : mae_leader,
        'dist_final'  : dist_final,
        'n_steps'     : max_L,
        'n_switch'    : n_switch,
    }


def _plot_result(gt_unpadded, x_ref_unpadded, modes, fname, save_dir, mae_f, mae_l, dt_file, current_goal):
    """Vẽ và lưu 2 biểu đồ: (a) quỹ đạo 3D, (b) sai số theo thời gian."""
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size']   = 11

    # Tạo mảng thời gian riêng biệt
    t_gt  = np.arange(len(gt_unpadded)) * dt_file
    t_ref = np.arange(len(x_ref_unpadded)) * dt_file

    # Pad để tính Error
    max_L = max(len(gt_unpadded), len(x_ref_unpadded))
    gt_pad = np.vstack([gt_unpadded, np.tile(gt_unpadded[-1], (max_L - len(gt_unpadded), 1))]) if len(gt_unpadded) < max_L else gt_unpadded
    x_ref_pad = np.vstack([x_ref_unpadded, np.tile(current_goal, (max_L - len(x_ref_unpadded), 1))]) if len(x_ref_unpadded) < max_L else x_ref_unpadded
    
    t_pad  = np.arange(max_L) * dt_file
    errors = np.linalg.norm(x_ref_pad - gt_pad, axis=1)
    
    follower_m = np.array([m == "FOLLOWER" for m in modes])
    leader_m   = np.array([m == "LEADER"   for m in modes])

    fig = plt.figure(figsize=(16, 6))

    # ── Subplot 1: Quỹ đạo 3D ──
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(gt_unpadded[:, 0], gt_unpadded[:, 1], gt_unpadded[:, 2], 'b-', alpha=0.5, linewidth=1.2, label='Ground Truth')
    
    if follower_m.any():
        ax1.plot(x_ref_unpadded[follower_m, 0], x_ref_unpadded[follower_m, 1], x_ref_unpadded[follower_m, 2],
                 'g-', linewidth=1.5, label='SVGP (FOLLOWER)')
    if leader_m.any():
        ax1.plot(x_ref_unpadded[leader_m, 0], x_ref_unpadded[leader_m, 1], x_ref_unpadded[leader_m, 2],
                 'r-', linewidth=1.5, label='MJM (LEADER)')
                 
    ax1.scatter(*current_goal, color='black', s=80, zorder=5, marker='*', label='Goal (Robot)')
    ax1.scatter(*gt_unpadded[-1], color='purple', s=80, zorder=5, marker='X', label='GT End')
    ax1.scatter(*gt_unpadded[0], color='green', s=40, zorder=5, marker='o')
    ax1.scatter(*x_ref_unpadded[follower_m][-1] if follower_m.any() else gt_unpadded[0],
                color='orange', s=40, zorder=5, marker='D', label='Switch Point')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory')
    ax1.legend(fontsize=8, loc='upper left')

    # ── Subplot 2: Sai số theo thời gian ──
    ax2 = fig.add_subplot(132)
    ax2.plot(t_pad, errors, 'k-', linewidth=1.2, label='Error (‖x_ref − x_h‖)')
    if follower_m.any():
        ax2.axvspan(t_ref[0], t_ref[follower_m].max(),
                    alpha=0.15, color='green', label=f'FOLLOWER (MAE={mae_f:.4f}m)')
    if leader_m.any():
        ax2.axvspan(t_ref[leader_m].min(), t_ref[-1],
                    alpha=0.15, color='red', label=f'LEADER (MAE={mae_l:.4f}m)')
    ax2.axvline(x=T_SWITCH, color='gray', linestyle='--', linewidth=1, label=f'Switch @ {T_SWITCH}s')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Euclidean Error (m)')
    ax2.set_title('Tracking Error vs Time')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.4)

    # ── Subplot 3: Từng trục XYZ ──
    ax3 = fig.add_subplot(133)
    labels = ['X', 'Y', 'Z']
    colors = ['tab:red', 'tab:green', 'tab:blue']
    for i, (lbl, col) in enumerate(zip(labels, colors)):
        ax3.plot(t_gt, gt_unpadded[:, i], color=col, alpha=0.4, linewidth=1, label=f'GT {lbl}')
        ax3.plot(t_ref, x_ref_unpadded[:, i], color=col, linewidth=1.2, label=f'Ref {lbl}')
        # Đánh dấu thời điểm robot chạm đích bằng dấu X
        ax3.scatter(t_ref[-1], x_ref_unpadded[-1, i], color=col, marker='X', s=50, zorder=5)
        
    if follower_m.any():
        ax3.axvspan(t_ref[0], t_ref[follower_m].max(), alpha=0.08, color='green')
    if leader_m.any():
        ax3.axvspan(t_ref[leader_m].min(), t_ref[-1], alpha=0.08, color='red')
    ax3.axvline(x=T_SWITCH, color='gray', linestyle='--', linewidth=1)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Per-Axis Position')
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.4)

    title_str = fname.replace('.csv', '')
    fig.suptitle(f'Hybrid SVGP→MJM | {title_str} | T_switch={T_SWITCH}s', fontsize=12)
    plt.tight_layout()

    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, fname.replace('.csv', '_hybrid.png'))
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()


# ─── 4. MAIN ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 70)
    print(" TEST HYBRID: SVGP (FOLLOWER) → Minimum Jerk Model (LEADER)")
    print("=" * 70)
    print(f"  T_SWITCH       = {T_SWITCH} s")
    print(f"  GOAL           = {GOAL}")
    print(f"  SVGP HISTORY   = {SVGP_HISTORY_SIZE} điểm")
    print(f"  DATA_FOLDER    = {DATA_FOLDER}")
    print(f"  SAVE_DIR       = {SAVE_DIR}")
    print("=" * 70)

    # ─── Load SVGP ───────────────────────────────────────────────────────────
    if not os.path.exists(SVGP_CHECKPOINT_DIR):
        print(f"❌ Không tìm thấy thư mục SVGP: {SVGP_CHECKPOINT_DIR}")
        sys.exit(1)

    svgp_scaler_x, svgp_scaler_y, svgp_model = load_svgp_system(SVGP_CHECKPOINT_DIR)
    if svgp_model is None:
        sys.exit(1)

    # ─── Liệt kê file CSV từ test_file_list.json ────────────────────────────
    if not os.path.exists(TEST_LIST_FILE):
        print(f"❌ Không tìm thấy danh sách file test: {TEST_LIST_FILE}")
        sys.exit(1)

    with open(TEST_LIST_FILE, 'r', encoding='utf-8') as f:
        csv_files = json.load(f)

    if not csv_files:
        print(f"❌ Danh sách file test trống trong: {TEST_LIST_FILE}")
        sys.exit(1)

    if MAX_FILES is not None:
        csv_files = csv_files[:MAX_FILES]

    print(f"\n📂 Tìm thấy {len(csv_files)} file CSV. Bắt đầu kiểm tra...\n")
    os.makedirs(SAVE_DIR, exist_ok=True)

    # ─── Vòng lặp chính ──────────────────────────────────────────────────────
    all_results = []
    for i, fname in enumerate(csv_files):
        fpath = os.path.join(DATA_FOLDER, fname)
        print(f"[{i+1:03d}/{len(csv_files)}] 📄 {fname}")
        result = run_hybrid_test(fpath, svgp_scaler_x, svgp_scaler_y,
                                 svgp_model, save_dir=SAVE_DIR)
        if result:
            all_results.append(result)
        print()

    # ─── Tổng kết ────────────────────────────────────────────────────────────
    print("=" * 70)
    print("  SUMMARY")
    print("=" * 70)

    if all_results:
        n = len(all_results)
        avg_total    = np.mean([r['mae_total']    for r in all_results])
        avg_follower = np.mean([r['mae_follower'] for r in all_results])
        avg_leader   = np.mean([r['mae_leader']   for r in all_results])
        avg_final    = np.mean([r['dist_final']   for r in all_results])

        print(f"\n  Số file đã test          : {n}")
        print(f"  T_SWITCH                 : {T_SWITCH} s")
        print(f"  ─────────────────────────────────────────")
        print(f"  MAE tổng trung bình      : {avg_total:.4f} m")
        print(f"  MAE FOLLOWER (SVGP)      : {avg_follower:.4f} m")
        print(f"  MAE LEADER   (MJM)       : {avg_leader:.4f} m")
        print(f"  Dist. đến đích (cuối)    : {avg_final:.4f} m")

        # Lưu kết quả tổng kết ra CSV
        df_out = pd.DataFrame(all_results)
        summary_path = os.path.join(SAVE_DIR, "summary.csv")
        df_out.to_csv(summary_path, index=False)
        print(f"\n  📁 Kết quả lưu tại       : {SAVE_DIR}/")
        print(f"  📊 Tổng kết CSV          : {summary_path}")
    else:
        print("  ⚠️  Không có kết quả nào hợp lệ.")

    print(f"\n✅ Hoàn tất kiểm tra Hybrid SVGP → MJM.")
