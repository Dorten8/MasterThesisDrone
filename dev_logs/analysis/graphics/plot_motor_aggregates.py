#!/usr/bin/env python3
import sqlite3
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Define DB Path
DB_PATH = "/home/dorten/MasterThesisDrone/dev_logs/analysis/experiments_summary.db"
OUTPUT_DIR = "/home/dorten/MasterThesisDrone/dev_logs/analysis/graphics"

def plot_motor_aggregates(df_impacts=None):
    """
    Aggregated Motor Performance Analysis boxplot: Fixed vs. Rotating Cage.

    Parameters
    ----------
    df_impacts : pd.DataFrame or None
        Must contain columns: condition, motor_avg_before, motor_avg_after,
        motor_thrust_surge, motor_imbalance_after.
        If None, queries experiments_summary.db directly (CLI fallback).
    """
    if df_impacts is not None:
        required = ['condition', 'motor_avg_before', 'motor_avg_after',
                    'motor_thrust_surge', 'motor_imbalance_after']
        missing = [c for c in required if c not in df_impacts.columns]
        if missing:
            raise KeyError(f"df_impacts missing required columns: {missing}")
        df = df_impacts[required].dropna(subset=['motor_avg_before', 'motor_avg_after'])
    else:
        if not os.path.exists(DB_PATH):
            print(f"[ERROR] Database not found at {DB_PATH}")
            return
        conn = sqlite3.connect(DB_PATH)
        query = """
            SELECT condition, motor_avg_before, motor_avg_after, motor_thrust_surge, motor_imbalance_after
            FROM flights_summary
            WHERE impact_detected = 1
              AND motor_avg_before IS NOT NULL
              AND motor_avg_after IS NOT NULL
        """
        df = pd.read_sql_query(query, conn)
        conn.close()

    if df.empty:
        print("[WARN] No valid motor data found for impact flights.")
        return

    print(f"📊 Loaded {len(df)} passes with valid motor telemetry.")

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Sleek harmonious colors
    colors = {"Rotating Cage": "#4B8BFF", "Fixed Cage": "#FF4B4B"}
    conditions = ["Fixed Cage", "Rotating Cage"]

    metrics = [
        ("motor_avg_after", "Sustained Post-Impact Motor Average", "Average Motor Command (0-1)"),
        ("motor_thrust_surge", "Transient Recovery Thrust Surge", "Thrust Surge (Delta Command)"),
        ("motor_imbalance_after", "Post-Impact Motor Imbalance (Aerodynamic Dev)", "Standard Deviation (Command)")
    ]

    for idx, (col, title, ylabel) in enumerate(metrics):
        ax = axes[idx]
        
        # Prepare data groups
        data_groups = [df[df["condition"] == cond][col].dropna().values for cond in conditions]
        
        # Plot boxplot
        bp = ax.boxplot(data_groups, labels=conditions, patch_artist=True, widths=0.4)
        
        # Color customizing
        for patch, cond in zip(bp['boxes'], conditions):
            patch.set_facecolor(colors[cond])
            patch.set_alpha(0.8)
            patch.set_edgecolor('#333333')
            patch.set_linewidth(1.5)
            
        for median in bp['medians']:
            median.set_color('#222222')
            median.set_linewidth(2)
            
        for whisker in bp['whiskers']:
            whisker.set_color('#555555')
            whisker.set_linewidth(1.2)
            
        for cap in bp['caps']:
            cap.set_color('#555555')
            cap.set_linewidth(1.2)

        ax.set_title(title, fontsize=13, fontweight='bold', pad=12)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.grid(True, linestyle='--', alpha=0.5, axis='y')
        
        # Style spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#cccccc')
        ax.spines['bottom'].set_color('#cccccc')
        ax.tick_params(labelsize=10)

    plt.suptitle("Aggregated Motor Performance Analysis: Fixed vs. Rotating Cage", fontsize=15, fontweight='bold', y=1.02)
    plt.tight_layout()
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, "motor_aggregates_comparison.png")
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"🎉 Aggregated comparison plot saved to: {out_path}")

if __name__ == "__main__":
    plot_motor_aggregates()
