#!/usr/bin/env python3
import os
import sys
import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.interpolate import interp1d

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

def generate_aggregated_imu_plot():
    cache_path = os.path.join(project_root, "dev_logs", "analysis", "database", "imu_cache.pkl")
    
    if not os.path.exists(cache_path):
        print(f"Error: Cache {cache_path} not found. Run db_cache_imu.py first.")
        return
        
    with open(cache_path, 'rb') as f:
        cached_data = pickle.load(f)
        
    if not cached_data:
        print("Error: Cache is empty.")
        return
        
    print(f"Loaded {len(cached_data)} IMU traces.")
    
    t_common = np.linspace(-1.0, 2.0, 300)
    
    a_dev_rot, g_mag_rot = [], []
    a_dev_fix, g_mag_fix = [], []
    
    for trace in cached_data:
        t = trace['t_rel']
        if len(t) < 5: continue
        
        # Deduplicate time axis
        t_unique, idx = np.unique(t, return_index=True)
        if len(t_unique) < 5: continue
        
        # Interpolate to common grid
        f_a = interp1d(t_unique, trace['a_deviation'][idx], bounds_error=False, fill_value=np.nan)
        f_g = interp1d(t_unique, trace['g_mag'][idx], bounds_error=False, fill_value=np.nan)
        
        a_interp = f_a(t_common)
        g_interp = f_g(t_common)
        
        if trace['condition'] == 'Rotating Cage':
            a_dev_rot.append(a_interp)
            g_mag_rot.append(g_interp)
        else:
            a_dev_fix.append(a_interp)
            g_mag_fix.append(g_interp)
            
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True, dpi=150)
    
    def plot_panel(ax, a_list, g_list, title):
        a_arr = np.nan_to_num(np.array(a_list))
        g_arr = np.nan_to_num(np.array(g_list))
        
        ax2 = ax.twinx()
        
        # Plot individual thin lines
        for a_tr in a_arr:
            ax.plot(t_common, a_tr, color='#D62728', alpha=0.03, linewidth=0.8)
        for g_tr in g_arr:
            ax2.plot(t_common, g_tr, color='#1F77B4', alpha=0.03, linewidth=0.8)
            
        # Plot Median (Average Trend)
        a_median = np.median(a_arr, axis=0)
        g_median = np.median(g_arr, axis=0)
        
        l1 = ax.plot(t_common, a_median, color='#D62728', linewidth=2.5, label='Median Linear Accel Deviation (m/s²)')
        l2 = ax2.plot(t_common, g_median, color='#1F77B4', linestyle='-.', linewidth=2.5, label='Median Gyro Rotational Surge (rad/s)')
        
        # Threshold
        l3 = ax.axhline(5.0, color='orange', linestyle='--', linewidth=1.5, label='Severe Impact Threshold (5.0 m/s²)')
        
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_ylabel('Acceleration Deviation (m/s²)', color='#D62728')
        ax2.set_ylabel('Gyro Magnitude (rad/s)', color='#1F77B4')
        
        ax.tick_params(axis='y', labelcolor='#D62728')
        ax2.tick_params(axis='y', labelcolor='#1F77B4')
        
        ax.set_ylim(-1.0, 20.0)
        ax.yaxis.set_major_locator(ticker.MultipleLocator(2.0))
        
        max_g = np.max(g_arr) if g_arr.size > 0 else 10.0
        ax2.set_ylim(0, max(10.0, max_g * 1.1))
        
        ax.grid(True, linestyle=':', alpha=0.6)
        
        # Red Impact Line
        ax.axvline(0.0, color='#D32F2F', linestyle='--', linewidth=2, zorder=10)
        ax.text(0.0, 18, ' Column Impact', color='#D32F2F', fontweight='bold', ha='left')
        
        # Legends
        lines = l1 + [l3]
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper right', bbox_to_anchor=(0.85, 0.98), frameon=True, facecolor='white', edgecolor='#EAEAEA', fontsize=8)
        
        lines2 = l2
        labels2 = [l.get_label() for l in lines2]
        ax2.legend(lines2, labels2, loc='upper right', bbox_to_anchor=(0.85, 0.85), frameon=True, facecolor='white', edgecolor='#EAEAEA', fontsize=8)

        # Draw N count
        ax.text(0.98, 0.02, f"n = {len(a_list)} flights", transform=ax.transAxes,
                ha='right', va='bottom', fontsize=9, fontweight='bold', alpha=0.8,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    plot_panel(axs[0], a_dev_fix, g_mag_fix, "IMU Collision Dynamics (Aggregated) - Fixed Cage")
    plot_panel(axs[1], a_dev_rot, g_mag_rot, "IMU Collision Dynamics (Aggregated) - Rotating Cage")
    
    axs[1].set_xlabel('Time relative to Impact (seconds)', fontweight='bold')
    axs[1].set_xlim(-1.0, 2.0)
    axs[1].xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    
    fig.suptitle('Time-Normalized IMU Dynamics Overview\n(Overlaid traces with Median profiles)', fontsize=14, y=0.98, fontweight='bold')
    plt.tight_layout()
    
    out_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", "plot_aggregated_imu_dynamics.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"Aggregated IMU plot saved to {out_path}")
    plt.show()

if __name__ == "__main__":
    generate_aggregated_imu_plot()
