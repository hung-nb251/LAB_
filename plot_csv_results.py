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
    
    # Lấy dữ liệu thực tế (tay người - meas) và dự đoán (pred)
    x_r_raw = df[['filt_x', 'filt_y', 'filt_z']].astype(float).values
    x_d_raw = df[['pred_x', 'pred_y', 'pred_z']].astype(float).values
    
    # 1. Dữ liệu DỰ ĐOÁN (pred): Lọc thêm offline để đồ thị mượt hơn
    # Mặc dù trong ROS đã có EMA filter, nhưng để hiển thị đẹp hơn ta lọc thêm một chút
    sigma_pred = 0.5  # Dùng sigma nhỏ hơn để không làm lệch pha quá nhiều
    x_d = np.zeros_like(x_d_raw)
    x_d[:, 0] = gaussian_filter1d(x_d_raw[:, 0], sigma=sigma_pred)
    x_d[:, 1] = gaussian_filter1d(x_d_raw[:, 1], sigma=sigma_pred)
    x_d[:, 2] = gaussian_filter1d(x_d_raw[:, 2], sigma=sigma_pred)
    
    # 2. Dữ liệu THỰC TẾ (meas): File CSV đang ghi lại raw camera (Kinect noise cao).
    # Trong khi UI thì vẽ filtered_hand_position. Nên ở đây ta bắt buộc phải lọc
    # meas bằng Gaussian Filter để đồ thị mượt như trên UI.
    sigma = 0.5  # Độ mượt
    print(f"Đang lọc dữ liệu đo đạc (meas) với Gaussian Filter (sigma={sigma}) ...")
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
    trajectory_name = os.path.splitext(os.path.basename(csv_path))[0]
    
    # Gọi hàm vẽ từ thư viện
    plot_xd_vs_xr(
        x_d=x_d,
        x_r=x_r,
        modes=modes,
        trajectory_name=trajectory_name,
        save_dir=save_dir,
        label_d='Predicted',
        label_r='Measured',
        title_prefix='SVGP Prediction vs Measured Trajectory',
        time_array=time_array
    )
    print(f"Đồ thị đã được lưu tại: {save_dir}")

if __name__ == '__main__':
    csv_file = "/home/hungnb/cocarry_ws/cocarry_logs/Experiment_14_8_26/experiment_SVGP+MJM_20260814_161131.csv"
    save_folder = "/home/hungnb/cocarry_ws/cocarry_logs/"
    plot_experiment_csv(csv_file, save_folder)
