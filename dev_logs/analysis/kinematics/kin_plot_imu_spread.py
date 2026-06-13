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
          collision event plus the immediate post-impact shake.
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
    # processing pipeline. Only flights where an impact was detected (impact_detected=1)
    # are included, because the impact-vs-regular comparison is meaningless for flights
    # that never collided. The six columns capture spread in milli-g for each of the
    # three axes (X, Y, Z) in both the impact and regular windows.
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
    # Each condition yields its own DataFrame slice so we can extract the four
    # data arrays (2 windows x 2 conditions) per axis independently.
    data_rot = df[df['condition'] == 'Rotating Cage']
    data_fix = df[df['condition'] == 'Fixed Cage']

    # ---- Layout: three axes side-by-side (X, Y, Z), sharing the y-axis scale ----
    # `sharey=True` ensures the spread magnitude can be compared directly across axes
    # without distortion from different y-axis ranges.
    fig, axes = plt.subplots(1, 3, figsize=(15, 6), sharey=True)

    # Each entry maps an axis letter to the database column names for its regular and
    # impact spread values.
    axes_list = [
        ('X', 'imu_ax_spread_regular', 'imu_ax_spread_impact'),
        ('Y', 'imu_ay_spread_regular', 'imu_ay_spread_impact'),
        ('Z', 'imu_az_spread_regular', 'imu_az_spread_impact')
    ]

    colors = {'Rotating Cage': '#1f77b4', 'Fixed Cage': '#ff7f0e'}
    # x-tick labels: "Regular (1.0s)" for the pre-impact flight window,
    #                "Impact (400ms)" for the collision window.
    labels = ['Regular (1.0s)', 'Impact (400ms)']

    for i, (ax_name, col_reg, col_imp) in enumerate(axes_list):
        ax = axes[i]

        # ---- Extract data arrays with NaN removal ----
        # Some flights may have missing spread values (e.g., sensor dropout, short logs).
        # `dropna()` discards those NaN entries so they do not propagate into the boxplot
        # statistics (median, quartiles, whiskers). This is safe because missing IMU data
        # is rare and random, not condition-specific, so filtering them does not bias the
        # comparison.
        reg_rot = data_rot[col_reg].dropna().values
        imp_rot = data_rot[col_imp].dropna().values
        reg_fix = data_fix[col_reg].dropna().values
        imp_fix = data_fix[col_imp].dropna().values

        # ---- Manual position layout for grouped boxplots ----
        # Rather than using matplotlib's built-in grouped positioning (which would
        # place all four boxes at uniform spacing), we manually offset the Rotating
        # and Fixed boxes slightly to the left and right of each nominal x-position.
        # This creates a compact "paired" look:
        #   Regular group at x approx 1: Rotating at 0.85 (left),  Fixed at 1.15 (right)
        #   Impact  group at x approx 2: Rotating at 1.85 (left),  Fixed at 2.15 (right)
        # The 0.3 gap between the two members of a pair is small enough to read them
        # as belonging to the same window category, while the 0.7 gap between the
        # Regular and Impact clusters separates the two window types clearly.
        pos_reg_rot = 0.85
        pos_reg_fix = 1.15
        pos_imp_rot = 1.85
        pos_imp_fix = 2.15
        
        # Assemble the four data arrays and their corresponding x-positions in the
        # order [Rot Regular, Fix Regular, Rot Impact, Fix Impact] for each axis.
        box_data = [reg_rot, reg_fix, imp_rot, imp_fix]
        positions = [pos_reg_rot, pos_reg_fix, pos_imp_rot, pos_imp_fix]

        # ---- Build the grouped boxplot ----
        # `positions` places each box at the manually computed x-coordinate.
        # `widths=0.2` keeps the individual boxes narrow so neighbouring pairs do not
        # overlap. `showfliers=False` suppresses outlier markers because IMU spread
        # distributions can have extreme outliers from violent collisions that would
        # compress the y-axis and obscure the box structure; the IQR-based box still
        # faithfully represents the bulk of the distribution.
        # `medianprops` makes the median line bold (1.5 pt) so it is easy to locate.
        bplot = ax.boxplot(
            box_data, positions=positions, widths=0.2, patch_artist=True,
            showfliers=False, medianprops=dict(color='black', linewidth=1.5)
        )

        # ---- Colour-code boxes by cage condition ----
        # We iterate over the box patches and their corresponding positions. Boxes
        # at the Rotating Cage positions (0.85 and 1.85) get the blue colour; boxes
        # at the Fixed Cage positions (1.15 and 2.15) get the orange colour. The
        # alpha=0.8 transparency is slightly higher than the statistics plot because
        # the fliers are hidden and we want the boxes to read more solidly.
        for patch, pos in zip(bplot['boxes'], positions):
            if pos in [pos_reg_rot, pos_imp_rot]:
                patch.set_facecolor(colors['Rotating Cage'])
                patch.set_alpha(0.8)
            else:
                patch.set_facecolor(colors['Fixed Cage'])
                patch.set_alpha(0.8)

        # ---- Axis labelling ----
        # The x-axis uses two nominal tick positions (1 and 2) labelled "Regular" and
        # "Impact", with the two cage-condition boxes clustered around each tick.
        ax.set_xticks([1, 2])
        ax.set_xticklabels(labels, fontsize=12)
        ax.set_title(f'{ax_name}-Axis Acceleration Spread', fontsize=14, fontweight='bold')

        # ---- Y-axis label and custom legend (only on the leftmost subplot) ----
        # The spread is expressed in units of g (gravitational acceleration, 9.81 m/s^2),
        # computed as the standard deviation of the raw accelerometer readings. A custom
        # legend using coloured Patch handles is necessary because the boxplot does not
        # produce labeled artists that matplotlib's auto-legend can use directly.
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
