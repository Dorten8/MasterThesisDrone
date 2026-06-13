import os
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_imu_spread():
    """
    Generate a grouped boxplot comparing IMU acceleration spread (standard deviation
    of high-frequency accelerometer readings) between the impact window and regular
    flight, separately for Rotating Cage and Fixed Cage conditions.

    Concept -- IMU Acceleration Spread:
      The accelerometer on the Pixhawk IMU produces readings at several hundred Hz.
      During a collision (impact), the drone experiences violent, rapidly-changing
      accelerations that register as high-variance ("spread") IMU data. By contrast,
      steady flight produces low-variance readings. The spread metric is defined as
      the standard deviation of all accelerometer samples within a defined time window:
        - Impact window: [t_impact - 50 ms,  t_impact + 350 ms] -- captures the
          collision event plus the immediate post-impact shake. This is a 400 ms
          window TOTAL, starting 50 ms BEFORE impact and extending 350 ms AFTER.
        - Regular flight window: [t_impact - 1050 ms, t_impact - 50 ms] -- a 1.0 s
          window ending 50 ms before the impact, representing nominal flight vibrations.
      By comparing these two windows per axis (X, Y, Z) and per cage condition, we can
      quantify how much the collision shakes the drone and whether the Rotating Cage
      attenuates or amplifies the shock relative to the Fixed Cage.
    """
    db_path = 'dev_logs/analysis/experiments_summary.db'
    if not os.path.exists(db_path):
        print(f"Database not found at {db_path}")
        return

    conn = sqlite3.connect(db_path)

    # ---- Database query ----
    # We retrieve per-flight IMU spread values that were precomputed during the data
    # processing pipeline (kin_calculator.py §§10.4.5). Only flights with a detected
    # structural impact (impact_detected=1) are included because the impact-vs-regular
    # comparison is meaningless for flights that never collided. The six columns
    # capture spread in g for each of the three axes (X, Y, Z) in both the impact
    # and regular windows.
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

    # ---- Split data by cage condition ----
    data_rot = df[df['condition'] == 'Rotating Cage']
    data_fix = df[df['condition'] == 'Fixed Cage']

    n_rot = len(data_rot)
    n_fix = len(data_fix)

    # ---- Print numerical summary (mean ± std) ----
    print(f"{'='*72}")
    print(f"  IMU Acceleration Spread — Numerical Summary")
    print(f"  Data: {n_rot} Rotating Cage + {n_fix} Fixed Cage  | "
          f"impact_detected=1 only")
    print(f"  Impact window: [t_imp − 50 ms, t_imp + 350 ms]  "
          f"(400 ms total, centred asymmetrically)")
    print(f"  Regular window: [t_imp − 1050 ms, t_imp − 50 ms]  "
          f"(1.0 s pre-impact baseline)")
    print(f"{'='*72}")
    for ax_label, col_reg, col_imp in [
        ('X', 'imu_ax_spread_regular', 'imu_ax_spread_impact'),
        ('Y', 'imu_ay_spread_regular', 'imu_ay_spread_impact'),
        ('Z', 'imu_az_spread_regular', 'imu_az_spread_impact'),
    ]:
        for cond_label, sub in [('Rotating Cage', data_rot), ('Fixed Cage', data_fix)]:
            reg = sub[col_reg].dropna()
            imp = sub[col_imp].dropna()
            # % change from regular to impact
            pct_chg = ((imp.mean() - reg.mean()) / reg.mean()) * 100 if reg.mean() > 0 else float('inf')
            print(f"  [{ax_label}] {cond_label:>15s}:  Regular μ = {reg.mean():.4f} ± {reg.std():.4f} g  "
                  f"|  Impact μ = {imp.mean():.4f} ± {imp.std():.4f} g  "
                  f"({pct_chg:+.0f}%)")
        print()

    # ---- Plot ----
    fig, axes = plt.subplots(1, 3, figsize=(10, 6.5), sharey=True)

    axes_list = [
        ('X', 'imu_ax_spread_regular', 'imu_ax_spread_impact'),
        ('Y', 'imu_ay_spread_regular', 'imu_ay_spread_impact'),
        ('Z', 'imu_az_spread_regular', 'imu_az_spread_impact')
    ]

    # Standard colours per skill file §4: Rotating = blue, Fixed = red
    colors = {'Rotating Cage': '#1f77b4', 'Fixed Cage': '#D62728'}

    # Window labels — simple; full definitions are in the figure footnote below
    labels = ['Steady Flight', 'Impact']

    # Storage for mean values per axis (used for subplot-footnote table)
    mean_tables = []

    for i, (ax_name, col_reg, col_imp) in enumerate(axes_list):
        ax = axes[i]

        reg_rot = data_rot[col_reg].dropna().values
        imp_rot = data_rot[col_imp].dropna().values
        reg_fix = data_fix[col_reg].dropna().values
        imp_fix = data_fix[col_imp].dropna().values

        # Collect means for the subplot-footnote table
        mu_reg_rot = float(np.mean(reg_rot))
        mu_reg_fix = float(np.mean(reg_fix))
        mu_imp_rot = float(np.mean(imp_rot))
        mu_imp_fix = float(np.mean(imp_fix))
        mean_tables.append((ax_name, mu_reg_rot, mu_reg_fix, mu_imp_rot, mu_imp_fix))

        # Within-group pairs have a visible gap; groups are well separated
        pos_reg_rot = 0.80
        pos_reg_fix = 1.20
        pos_imp_rot = 2.20
        pos_imp_fix = 2.60

        box_data = [reg_rot, reg_fix, imp_rot, imp_fix]
        positions = [pos_reg_rot, pos_reg_fix, pos_imp_rot, pos_imp_fix]

        bplot = ax.boxplot(
            box_data, positions=positions, widths=0.28, patch_artist=True,
            showfliers=False, medianprops=dict(color='black', linewidth=1.5)
        )

        for patch, pos in zip(bplot['boxes'], positions):
            if pos in [pos_reg_rot, pos_imp_rot]:
                patch.set_facecolor(colors['Rotating Cage'])
                patch.set_alpha(0.8)
            else:
                patch.set_facecolor(colors['Fixed Cage'])
                patch.set_alpha(0.8)

        ax.set_xticks([1, 2.4])
        ax.set_xticklabels(labels, fontsize=11, fontweight='bold')
        ax.set_xlim([0.40, 3.0])
        # RGB colour-coding per axis: X=red, Y=green, Z=blue (traditional)
        title_color = {'X': '#E74C3C', 'Y': '#27AE60', 'Z': '#3498DB'}
        ax.set_title(f'{ax_name}-Axis', fontsize=14, fontweight='bold',
                     color=title_color[ax_name], pad=10)

        if i == 0:
            ax.set_ylabel('Spread (Std. Deviation) [g]', fontsize=12)

            from matplotlib.patches import Patch
            legend_elements = [
                Patch(facecolor=colors['Rotating Cage'], label='Rotating Cage', alpha=0.8),
                Patch(facecolor=colors['Fixed Cage'], label='Fixed Cage', alpha=0.8)
            ]
            ax.legend(handles=legend_elements, loc='upper left')

    # ── Mean-value table per subplot, placed below the x-axis labels ──
    # Using axes coordinates (y negative) so they sit in the figure margin
    # without compressing the boxplot data
    for i, (ax_name, mu_reg_rot, mu_reg_fix, mu_imp_rot, mu_imp_fix) in enumerate(mean_tables):
        ax = axes[i]
        # Steady flight means on the left, impact means on the right — clean one-liner
        ax.text(0.05, -0.10,
                f'μ: Rot={mu_reg_rot:.3f}\n    Fix={mu_reg_fix:.3f}',
                transform=ax.transAxes, ha='left', va='top',
                fontsize=8, fontfamily='monospace', color='#444444',
                linespacing=1.3)
        ax.text(0.95, -0.10,
                f'μ: Rot={mu_imp_rot:.3f}\n    Fix={mu_imp_fix:.3f}',
                transform=ax.transAxes, ha='right', va='top',
                fontsize=8, fontfamily='monospace', color='#444444',
                linespacing=1.3)

    plt.suptitle('High-Frequency IMU Acceleration Spread: Impact vs. Regular Flight',
                 fontsize=16, fontweight='bold', y=1.00)

    # ---- Figure-bottom annotations (per §4 data-origin rule) ----
    # Window definition footnote (left side) — multiline, larger font
    fig.text(
        0.02, 0.025,
        "Impact window:  400 ms  (50 ms pre + 350 ms post)\n"
        "Regular window: 1.0 s  (ending 50 ms before impact)",
        ha='left', va='bottom', fontsize=8, fontstyle='italic', color='#555555',
        linespacing=1.5
    )
    # Data source (right side)
    fig.text(
        0.98, 0.025,
        f"Source: flights_summary (impact_detected={n_rot}r + {n_fix}f)",
        ha='right', va='bottom', fontsize=8, fontstyle='italic', color='#555555'
    )

    plt.tight_layout(rect=[0, 0.07, 1, 0.97])

    output_dir = 'dev_logs/analysis/graphics'
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'plot_16_imu_vibration_spread.png')

    thesis_output_dir = 'thesis/plots'
    os.makedirs(thesis_output_dir, exist_ok=True)
    thesis_output_path = os.path.join(thesis_output_dir,
                                      'IMU Vibration Spread by Cage Configuration.png')

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.savefig(thesis_output_path, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"Generated: {output_path}")
    print(f"  → Also saved to thesis: {thesis_output_path}")


if __name__ == '__main__':
    plot_imu_spread()
