import os
import glob
import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d
import sys
sys.path.append('/home/hungnb/cocarry_ws')
sys.path.append('/home/hungnb/simulation_hri')
from plot_csv_results import plot_experiment_csv
from shared_control_lib import ControlMode

def main():
    data_dir = "/home/hungnb/cocarry_ws/cocarry_logs/Experiment_14_8_26"
    csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
    
    results = []
    
    for csv_file in csv_files:
        if "MAE_Summary.csv" in csv_file:
            continue
        print(f"==================================================")
        print(f"Processing {csv_file}")
        
        # 1. Plot
        plot_experiment_csv(csv_file, data_dir)
        
        # 2. Metric calculation
        df = pd.read_csv(csv_file)
        if 'ros_timestamp_ns' in df.columns:
            df = df[pd.to_numeric(df['ros_timestamp_ns'], errors='coerce').notnull()].copy()
            
        x_r_raw = df[['filt_x', 'filt_y', 'filt_z']].astype(float).values
        x_d_raw = df[['pred_x', 'pred_y', 'pred_z']].astype(float).values
        
        sigma_pred = 0.5
        x_d = np.zeros_like(x_d_raw)
        x_d[:, 0] = gaussian_filter1d(x_d_raw[:, 0], sigma=sigma_pred)
        x_d[:, 1] = gaussian_filter1d(x_d_raw[:, 1], sigma=sigma_pred)
        x_d[:, 2] = gaussian_filter1d(x_d_raw[:, 2], sigma=sigma_pred)
        
        sigma = 0.5
        x_r = np.zeros_like(x_r_raw)
        x_r[:, 0] = gaussian_filter1d(x_r_raw[:, 0], sigma=sigma)
        x_r[:, 1] = gaussian_filter1d(x_r_raw[:, 1], sigma=sigma)
        x_r[:, 2] = gaussian_filter1d(x_r_raw[:, 2], sigma=sigma)
        
        mae_total = np.abs(x_d - x_r).mean(axis=0)
        
        if 'role' in df.columns:
            roles = df['role'].str.strip().values
        else:
            roles = np.array([ControlMode.LEADER if ms == 0.0 else ControlMode.FOLLOWER for ms in df['inference_ms']])
            
        mask_follower = (roles == ControlMode.FOLLOWER)
        mask_leader = (roles == ControlMode.LEADER)
        
        mae_follower = np.full(3, np.nan)
        if mask_follower.sum() > 0:
            mae_follower = np.abs(x_d[mask_follower] - x_r[mask_follower]).mean(axis=0)
            
        mae_leader = np.full(3, np.nan)
        if mask_leader.sum() > 0:
            mae_leader = np.abs(x_d[mask_leader] - x_r[mask_leader]).mean(axis=0)
            
        avg_inf_time_follower = np.nan
        if 'inference_ms' in df.columns and mask_follower.sum() > 0:
            avg_inf_time_follower = df.loc[mask_follower, 'inference_ms'].astype(float).mean()
            
        traj_name = os.path.basename(csv_file)
        results.append({
            'Trajectory': traj_name,
            'MAE_Total_X': mae_total[0],
            'MAE_Total_Y': mae_total[1],
            'MAE_Total_Z': mae_total[2],
            'MAE_FOLLOWER_X': mae_follower[0],
            'MAE_FOLLOWER_Y': mae_follower[1],
            'MAE_FOLLOWER_Z': mae_follower[2],
            'MAE_LEADER_X': mae_leader[0],
            'MAE_LEADER_Y': mae_leader[1],
            'MAE_LEADER_Z': mae_leader[2],
            'Avg_Inference_Time_FOLLOWER_ms': avg_inf_time_follower
        })
    
    res_df = pd.DataFrame(results)
    summary_path = os.path.join(data_dir, "MAE_Summary.csv")
    res_df.to_csv(summary_path, index=False)
    print(f"\nSummary successfully saved to {summary_path}")

if __name__ == '__main__':
    main()
