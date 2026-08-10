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
    
    # Lấy dữ liệu thực tế (tay người - meas) và dự đoán (pred)
    x_r_raw = df[['meas_x', 'meas_y', 'meas_z']].values
    x_d_raw = df[['pred_x', 'pred_y', 'pred_z']].values
    
    # 1. Dữ liệu DỰ ĐOÁN (pred): Lọc thêm offline để đồ thị mượt hơn
    # Mặc dù trong ROS đã có EMA filter, nhưng để hiển thị đẹp hơn ta lọc thêm một chút
    sigma_pred = 1.5  # Dùng sigma nhỏ hơn để không làm lệch pha quá nhiều
    x_d = np.zeros_like(x_d_raw)
    x_d[:, 0] = gaussian_filter1d(x_d_raw[:, 0], sigma=sigma_pred)
    x_d[:, 1] = gaussian_filter1d(x_d_raw[:, 1], sigma=sigma_pred)
    x_d[:, 2] = gaussian_filter1d(x_d_raw[:, 2], sigma=sigma_pred)
    
    # 2. Dữ liệu THỰC TẾ (meas): File CSV đang ghi lại raw camera (Kinect noise cao).
    # Trong khi UI thì vẽ filtered_hand_position. Nên ở đây ta bắt buộc phải lọc
    # meas bằng Gaussian Filter để đồ thị mượt như trên UI.
    sigma = 3.0  # Độ mượt
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
        
    print("Đang vẽ đồ thị...")
    trajectory_name = os.path.splitext(os.path.basename(csv_path))[0]
    
    # Gọi hàm vẽ từ thư viện
    plot_xd_vs_xr(
        x_d=x_d,
        x_r=x_r,
        modes=modes,
        trajectory_name=trajectory_name,
        save_dir=save_dir,
        label_d='Predicted (Filtered in ROS)',
        label_r='Measured (Filtered offline)',
        title_prefix='SVGP Prediction vs Measured Trajectory'
    )
    print(f"Đồ thị đã được lưu tại: {save_dir}")

if __name__ == '__main__':
    csv_file = "/home/hungnb/cocarry_ws/cocarry_logs/Minh/experiment_GRU_20260616_194523.csv"
    save_folder = "/home/hungnb/cocarry_ws/cocarry_logs/"
    plot_experiment_csv(csv_file, save_folder)
