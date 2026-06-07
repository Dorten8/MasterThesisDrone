import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_imu_spread():
    db_path = 'dev_logs/analysis/experiments_summary.db'
    if not os.path.exists(db_path):
        print(f"Database not found at {db_path}")
        return

    conn = sqlite3.connect(db_path)
    
    query = """
    SELECT 
        condition,
        imu_ax_spread_impact, imu_ay_spread_impact, imu_az_spread_impact,
        imu_ax_spread_regular, imu_ay_spread_regular, imu_az_spread_regular
    FROM flights_summary
    WHERE impact_detected = 1
    """
    
    df = pd.read_sql_query(query, conn)
    conn.close()
    
    if df.empty:
        print("No IMU spread data found in the database.")
        return

    # Extract data grouped by condition
    data_rot = df[df['condition'] == 'Rotating Cage']
    data_fix = df[df['condition'] == 'Fixed Cage']

    fig, axes = plt.subplots(1, 3, figsize=(15, 6), sharey=True)
    
    axes_list = [
        ('X', 'imu_ax_spread_regular', 'imu_ax_spread_impact'),
        ('Y', 'imu_ay_spread_regular', 'imu_ay_spread_impact'),
        ('Z', 'imu_az_spread_regular', 'imu_az_spread_impact')
    ]
    
    colors = {'Rotating Cage': '#1f77b4', 'Fixed Cage': '#ff7f0e'}
    labels = ['Regular (1.0s)', 'Impact (400ms)']
    
    for i, (ax_name, col_reg, col_imp) in enumerate(axes_list):
        ax = axes[i]
        
        # Prepare data arrays (drop NaNs)
        reg_rot = data_rot[col_reg].dropna().values
        imp_rot = data_rot[col_imp].dropna().values
        reg_fix = data_fix[col_reg].dropna().values
        imp_fix = data_fix[col_imp].dropna().values
        
        # Positions for the boxes:
        # X=1 for Regular, X=2 for Impact
        pos_reg_rot = 0.85
        pos_reg_fix = 1.15
        pos_imp_rot = 1.85
        pos_imp_fix = 2.15
        
        box_data = [reg_rot, reg_fix, imp_rot, imp_fix]
        positions = [pos_reg_rot, pos_reg_fix, pos_imp_rot, pos_imp_fix]
        
        bplot = ax.boxplot(
            box_data, positions=positions, widths=0.2, patch_artist=True,
            showfliers=False, medianprops=dict(color='black', linewidth=1.5)
        )
        
        # Color the boxes
        for patch, pos in zip(bplot['boxes'], positions):
            if pos in [pos_reg_rot, pos_imp_rot]:
                patch.set_facecolor(colors['Rotating Cage'])
                patch.set_alpha(0.8)
            else:
                patch.set_facecolor(colors['Fixed Cage'])
                patch.set_alpha(0.8)
                
        ax.set_xticks([1, 2])
        ax.set_xticklabels(labels, fontsize=12)
        ax.set_title(f'{ax_name}-Axis Acceleration Spread', fontsize=14, fontweight='bold')
        
        if i == 0:
            ax.set_ylabel('Spread (Standard Deviation) [g]', fontsize=12)
            
            # Custom legend
            from matplotlib.patches import Patch
            legend_elements = [
                Patch(facecolor=colors['Rotating Cage'], label='Rotating Cage', alpha=0.8),
                Patch(facecolor=colors['Fixed Cage'], label='Fixed Cage', alpha=0.8)
            ]
            ax.legend(handles=legend_elements, loc='upper left')

    plt.suptitle('High-Frequency IMU Acceleration Spread: Impact vs. Regular Flight', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    
    output_dir = 'dev_logs/analysis/graphics'
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'plot_16_imu_vibration_spread.png')
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Successfully generated and saved plot to {output_path}")

if __name__ == '__main__':
    plot_imu_spread()
