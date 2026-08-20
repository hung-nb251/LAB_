import sys
import os
import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d

# Thêm đường dẫn tới thư mục chứa shared_control_lib
sys.path.append('/home/hungnb/simulation_hri')
from shared_control_lib import plot_xd_vs_xr, ControlMode

def plot_experiment_csv(csv_path, save_dir):
    print(f"Đang đọc file: {csv_path} ...")
    df = pd.read_csv(csv_path)
    # Lọc bỏ phần footer (các dòng có timestamp không phải là số)
    if 'ros_timestamp_ns' in df.columns:
        df = df[pd.to_numeric(df['ros_timestamp_ns'], errors='coerce').notnull()].copy()
    
    # Xác định các chế độ điều khiển (LEADER/FOLLOWER/READY)
    if 'role' in df.columns:
        modes = df['role'].tolist()
    else:
        modes = [ControlMode.LEADER if ms == 0.0 else ControlMode.FOLLOWER for ms in df['inference_ms']]
        
    # Lấy dữ liệu thực tế của robot (robot_ee) và mục tiêu dự đoán (target)
    x_r_raw = df[['robot_ee_x', 'robot_ee_y', 'robot_ee_z']].astype(float).values
    x_d_raw = df[['target_x', 'target_y', 'target_z']].astype(float).values
    
    # Làm phẳng "đoạn bị thừa ở đầu" (khi role là READY)
    # Tìm index đầu tiên không phải READY (thường là FOLLOWER hoặc LEADER)
    start_idx = 0
    for i, m in enumerate(modes):
        # Kiểm tra nếu role là chữ 'READY' (kiểu string) hoặc ControlMode.READY
        if str(m) != 'READY':
            start_idx = i
            break
            
    # Gán toàn bộ đoạn đầu bằng giá trị ổn định đầu tiên (tại start_idx)
    # Điều này giúp loại bỏ gai nhiễu ở index 0
    if start_idx > 0:
        x_d_raw[:start_idx] = x_d_raw[start_idx]
        x_r_raw[:start_idx] = x_r_raw[start_idx]
        
    # Nếu người dùng muốn x[0] của toàn bộ mảng:
    # Ở đây x_d_raw[0] đã được gán bằng x_d_raw[start_idx] nên mảng sẽ phẳng từ 0 đến start_idx
    
    # 1. Dữ liệu TARGET (tương đương với Pred cho robot)
    sigma_pred = 0.5
    x_d = np.zeros_like(x_d_raw)
    x_d[:, 0] = gaussian_filter1d(x_d_raw[:, 0], sigma=sigma_pred)
    x_d[:, 1] = gaussian_filter1d(x_d_raw[:, 1], sigma=sigma_pred)
    x_d[:, 2] = gaussian_filter1d(x_d_raw[:, 2], sigma=sigma_pred)
    
    # 2. Dữ liệu ROBOT EE (tương đương với Meas)
    sigma = 0.5
    x_r = np.zeros_like(x_r_raw)
    x_r[:, 0] = gaussian_filter1d(x_r_raw[:, 0], sigma=sigma)
    x_r[:, 1] = gaussian_filter1d(x_r_raw[:, 1], sigma=sigma)
    x_r[:, 2] = gaussian_filter1d(x_r_raw[:, 2], sigma=sigma)
    
    # Xác định các chế độ điều khiển (LEADER/FOLLOWER) để vẽ nền màu
    if 'role' in df.columns:
        modes = df['role'].tolist()
    else:
        # Nếu inference = 0.0 nghĩa là MJM (LEADER)
        modes = [ControlMode.LEADER if ms == 0.0 else ControlMode.FOLLOWER for ms in df['inference_ms']]
        
    # Tính thời gian thực từ timestamps nếu có
    time_array = None
    if 'ros_timestamp_ns' in df.columns:
        ts = pd.to_numeric(df['ros_timestamp_ns'], errors='coerce')
        time_array = (ts - ts.iloc[0]).values * 1e-9
    elif 'wall_time' in df.columns:
        time_array = (pd.to_datetime(df['wall_time']) - pd.to_datetime(df['wall_time'].iloc[0])).dt.total_seconds().values

    print("Đang vẽ đồ thị...")
    trajectory_name = os.path.splitext(os.path.basename(csv_path))[0] + "_robot_ee"
    
    # Gọi hàm vẽ từ thư viện
    plot_xd_vs_xr(
        x_d=x_d,
        x_r=x_r,
        modes=modes,
        trajectory_name=trajectory_name,
        save_dir=save_dir,
        label_d='Target (Pred)',
        label_r='Robot EE (Meas)',
        title_prefix='Target vs Robot EE Trajectory',
        time_array=time_array
    )
    print(f"Đồ thị đã được lưu tại: {save_dir}")

if __name__ == '__main__':
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "/home/hungnb/cocarry_ws/cocarry_logs/experiment_SVGP+MJM_20260818_112648.csv"
    save_folder = "/home/hungnb/cocarry_ws/cocarry_logs/"
    plot_experiment_csv(csv_file, save_folder)
