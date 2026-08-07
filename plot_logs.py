import pandas as pd
import matplotlib.pyplot as plt
import sys

def analyze_and_plot(file_path, title, out_path):
    df = pd.read_csv(file_path)
    
    # Calculate relative time in seconds
    df['time_sec'] = (df.iloc[:, 0] - df.iloc[0, 0]) / 1e9
    
    plt.figure(figsize=(15, 8))
    
    # Subplot 1: Y Trajectory
    plt.subplot(2, 1, 1)
    plt.plot(df['time_sec'], df['meas_y'], label='Measured Y', color='blue', alpha=0.6)
    
    # Plot SVGP predictions
    mask_svgp = df['inference_ms'] > 0
    if mask_svgp.any():
        plt.scatter(df.loc[mask_svgp, 'time_sec'], df.loc[mask_svgp, 'pred_y'], 
                    color='orange', s=10, label='SVGP Prediction', zorder=5)
                    
    # Plot MJM predictions
    mask_mjm = df['inference_ms'] == 0
    if mask_mjm.any():
        plt.scatter(df.loc[mask_mjm, 'time_sec'], df.loc[mask_mjm, 'pred_y'], 
                    color='green', s=10, label='MJM Prediction', zorder=6)
                    
    plt.title(f"{title} - Trajectory Y")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    
    # Subplot 2: Inference Time (to check interleaving)
    plt.subplot(2, 1, 2)
    plt.plot(df['time_sec'], df['inference_ms'], color='purple', label='Inference Time (ms)')
    plt.title("Inference Time over Time (0ms = MJM, >0ms = SVGP)")
    plt.xlabel("Time (s)")
    plt.ylabel("Time (ms)")
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    
    print(f"Stats for {title}:")
    print(f"  Total points: {len(df)}")
    print(f"  SVGP points: {mask_svgp.sum()}")
    print(f"  MJM points: {mask_mjm.sum()}")
    
analyze_and_plot('/home/hungnb/cocarry_ws/cocarry_logs/experiment_SVGP_20260807_161454.csv', 'Experiment 16:14:54', '/home/hungnb/.gemini/antigravity-ide/brain/50c22735-4cd2-46b2-aa95-869a0a8ff521/plot_161454.png')
analyze_and_plot('/home/hungnb/cocarry_ws/cocarry_logs/experiment_SVGP_20260807_161653.csv', 'Experiment 16:16:53', '/home/hungnb/.gemini/antigravity-ide/brain/50c22735-4cd2-46b2-aa95-869a0a8ff521/plot_161653.png')

