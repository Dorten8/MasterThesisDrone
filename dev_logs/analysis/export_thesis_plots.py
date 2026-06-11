#!/usr/bin/env python3
"""
Export all experiment notebook plots to thesis/plots/ with proper title-based filenames.

Copies:
  1. Existing generated PNGs from graphics/ → thesis/plots/ (renamed to match plot titles)
  2. Runs flight_loader plots for representative flights and saves them
  3. Runs the full mission geometry plot and saves it
"""

import sys, os, shutil, importlib

# -- Paths --
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
GRAPHICS_DIR  = os.path.join(PROJECT_ROOT, "dev_logs", "analysis", "graphics")
PLOTS_DIR     = os.path.join(PROJECT_ROOT, "thesis", "plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

# ================================================================
# MAP 1: Existing graphics/ → thesis/plots/ with title-based names
# ================================================================
# Format: (source_filename, dest_filename_without_extension)
EXISTING_MAP = [
    # ── experiments_analysis_summary.ipynb ──
    ("recovery_area_comparison",
     "Rotating Cage vs Fixed Cage Trajectory Recovery Area"),
    ("comparative_cage_deviation_overlay",
     "Rotating Cage vs Fixed Cage Stabilization Dynamics"),
    ("deceleration_vs_battery_angle",
     "Deceleration vs Battery State (per cage configuration)"),
    ("deceleration_vs_battery_global",
     "Rotating Cage vs Fixed Cage Global Deceleration vs Battery State"),
    ("plot_13_mission_pies",
     "Mission Composition Pie Charts by Cage Configuration"),
    ("plot_10a_flying_duration",
     "Rotating Cage vs Fixed Cage Active Flying Duration Comparison"),
    ("plot_10b_capacity_drain_rate",
     "Rotating Cage vs Fixed Cage Battery Capacity Drain Rate"),
    ("plot_10c_voltage_vs_duration",
     "Rotating Cage vs Fixed Cage Average Flying Voltage vs Flight Duration"),
    ("plot_14_angle_vs_deviation",
     "Rotating Cage vs Fixed Cage Post-Impact Max Deviation vs Impact Angle"),
    ("plot_15_recovery_distribution",
     "Rotating Cage vs Fixed Cage Recovery Area Distribution by Angle Bins"),
    ("plot_16_path_heatmap",
     "Flight Path Heatmap by Cage Configuration"),
    ("advanced_thesis_highlights",
     "Rotating Cage vs Fixed Cage Attitude-Shock Phase Portrait and Vibration Spread"),
    ("plot_17_allocator_saturation_comparison",
     "Control Allocator Saturation Comparison"),
    ("plot_18_pid_tracking_comparison",
     "PID Rate Controller Tracking Comparison"),
    ("plot_A_imu_z_vs_rpm",
     "IMU Z-Axis Peak Acceleration vs Motor RPM"),
    ("plot_aggregated_imu_dynamics",
     "Aggregated IMU Collision Dynamics Across All Flights"),
    ("plot_16_imu_vibration_spread",
     "IMU Vibration Spread by Cage Configuration"),

    # ── experiments_angle_prediction.ipynb (EDA) ──
    ("eda_correlation_heatmap",
     "Fixed Cage IMU Feature Correlation Heatmap with Impact Angle"),
    ("eda_top3_scatter",
     "Top-3 IMU Features vs Impact Angle with Huber Trendlines"),
    ("eda_parallel_coordinates",
     "Parallel Coordinates of IMU Features by Impact Angle Group"),

    # ── rf_angle_prediction ──
    ("rf_actual_vs_predicted",
     "Fixed Cage 5-Fold CV Predicted vs Actual Impact Angle"),
    ("rf_feature_importance",
     "Fixed Cage Random Forest Feature Importance (MDI)"),
    ("rf_cross_condition",
     "Fixed Cage to Rotating Cage Cross-Condition Transfer"),
    ("rf_permutation_importance",
     "Fixed Cage Permutation Feature Importance"),
    ("rf_learning_curve",
     "Fixed Cage Learning Curve"),
    ("rf_huber_baseline",
     "Model Comparison Huber Baseline vs Random Forest"),
]


def copy_existing_graphics():
    """Copy existing PNGs with title-based names."""
    copied = 0
    for src_base, title in EXISTING_MAP:
        src = os.path.join(GRAPHICS_DIR, src_base + ".png")
        if not os.path.exists(src):
            print(f"⚠️  MISSING: {src}")
            continue
        # Derive a safe filename from the title
        safe_name = title.replace("/", "-").replace(":", "-").replace("?", "")
        dst = os.path.join(PLOTS_DIR, safe_name + ".png")
        shutil.copy2(src, dst)
        print(f"  ✓ {safe_name}.png")
        copied += 1
    print(f"\nCopied {copied} existing plots\n")


# ================================================================
# MAP 2: Flight-loader generated plots (run with savefig)
# ================================================================
def run_flight_loader_plots():
    """Generate and save per-flight plots for representative flights."""
    sys.path.insert(0, PROJECT_ROOT)

    from dev_logs.analysis.database.flight_loader import (
        load_flight_data,
        plot_trajectory_from,
        plot_velocity_profile_from,
        plot_battery_sag_from,
        plot_imu_dynamics_from,
        plot_imu_xyz_from,
        compute_ekf_velocity,
        plot_ekf_velocity_from,
        plot_ekf_kinetic_from,
        plot_ekf_dual_comparison,
    )

    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')

    # Professional plotting style (thesis)
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'grid.linestyle': '--',
        'grid.alpha': 0.5,
        'figure.titlesize': 14,
        'figure.dpi': 150,
    })

    # Load representative flights
    print("Loading Rotating Cage flight ...")
    rot = load_flight_data(
        "flight_20260601-1756_45°_column_collision_loop_rotating_cage",
        "Pass-05"
    )
    print("Loading Fixed Cage flight ...")
    fix = load_flight_data(
        "flight_20260524-1904_75°_column_collision_loop_fixed_cage",
        "Pass-01"
    )

    def save_current_fig(title, dpi=300):
        """Save current matplotlib figure with title as filename."""
        safe = title.replace("/", "-").replace(":", "-").replace("?", "").replace("\n", " ").strip()
        path = os.path.join(PLOTS_DIR, f"{safe}.png")
        plt.savefig(path, dpi=dpi, bbox_inches='tight')
        print(f"  ✓ {safe[:60]}...")
        plt.close()

    # -- Trajectory plots --
    print("\nGenerating trajectory plots ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_trajectory_from(data)
        save_current_fig(f"2D Top-Down Trajectory — {cond} 45° Collision Loop")

    # -- Velocity profiles (raw) --
    print("\nGenerating raw velocity profiles ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_velocity_profile_from(data, raw=True)
        save_current_fig(f"Raw Kinetic Profile — {cond} 45° Collision Loop")

    # -- Velocity profiles (splined) --
    print("\nGenerating splined velocity profiles ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_velocity_profile_from(data)
        save_current_fig(f"Kinetic Profile — {cond} 45° Collision Loop")

    # -- Battery --
    print("\nGenerating battery sag plots ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_battery_sag_from(data)
        save_current_fig(f"Battery Voltage Sag — {cond} 45° Collision Loop")

    # -- IMU dynamics --
    print("\nGenerating IMU dynamics plots ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_imu_dynamics_from(data)
        save_current_fig(f"IMU Collision Dynamics — {cond} 45° Impact")

    # -- IMU XYZ --
    print("\nGenerating IMU XYZ plots ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_imu_xyz_from(data)
        save_current_fig(f"IMU XYZ Raw Components — {cond} 45° Impact")

    # -- EKF kinetics --
    print("\nGenerating EKF kinetic profiles ...")
    for data, cond in [(rot, "Rotating Cage"), (fix, "Fixed Cage")]:
        plot_ekf_kinetic_from(data)
        save_current_fig(f"EKF Kinetic Profile — {cond} 45° Collision Loop")

    # -- EKF velocity (Fixed Cage only — has MoCap dropouts) --
    print("\nGenerating EKF vs MoCap velocity plots ...")
    plot_ekf_velocity_from(fix)
    save_current_fig("EKF vs MoCap Velocity — Fixed Cage Representative Flight")

    # -- EKF dual comparison --
    print("\nGenerating EKF dual comparison ...")
    plot_ekf_dual_comparison(rot, fix)
    save_current_fig("EKF Dual Comparison Fixed vs Rotating Cage Representative Flights")


# ================================================================
# MAP 3: Full mission geometry plot
# ================================================================
def run_geometry_plot():
    """Generate the 45° full mission geometry plot and save it."""
    print("\nGenerating full mission geometry plot ...")

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np

    # Add project to path for graphics_loader
    sys.path.insert(0, PROJECT_ROOT)
    from dev_logs.analysis.graphics.graphics_loader import draw_vector_drone

    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 11,
        'figure.dpi': 150,
    })

    # Coordinate rotation
    def rotate_coords(x_val, y_val):
        return -y_val, x_val

    # Mission waypoints (ENU frame)
    WP_stage = (0.248, 1.200)
    WP1      = (0.248, 0.950)
    WP2      = (0.248, -1.200)
    WP3      = (0.000, 0.300)
    COLUMN   = (0.408, 0.358)
    coll_pt  = (0.248, 0.358)

    COL_DIAMETER  = 0.09
    COL_RADIUS    = COL_DIAMETER / 2.0
    CAGE_DIAMETER = 0.358
    CAGE_RADIUS   = CAGE_DIAMETER / 2.0

    r_stage = rotate_coords(*WP_stage)
    r_wp1   = rotate_coords(*WP1)
    r_wp2   = rotate_coords(*WP2)
    r_wp3   = rotate_coords(*WP3)
    r_col   = rotate_coords(*COLUMN)
    r_coll  = rotate_coords(*coll_pt)

    enu_loop = [WP_stage, WP1, WP2, WP3, WP_stage]
    rot_loop = [rotate_coords(x, y) for x, y in enu_loop]

    fig, ax = plt.subplots(figsize=(11, 7))

    # Dotted red path
    path_x = [p[0] for p in rot_loop]
    path_y = [p[1] for p in rot_loop]
    ax.plot(path_x, path_y, color='#D62728', linestyle=':', linewidth=2.5, zorder=5,
            label='Target Track')

    # Directional arrows at midpoints
    for i in range(len(enu_loop) - 1):
        se, ee = enu_loop[i], enu_loop[i+1]
        mid     = ((se[0] + ee[0]) / 2, (se[1] + ee[1]) / 2)
        r_mid   = rotate_coords(*mid)
        r_dir   = rotate_coords(ee[0] - se[0], ee[1] - se[1])
        ln      = np.hypot(*r_dir)
        if ln > 1e-6:
            ax.annotate('', xy=(r_mid[0] + r_dir[0]/ln*0.08, r_mid[1] + r_dir[1]/ln*0.08),
                        xytext=(r_mid[0], r_mid[1]),
                        arrowprops=dict(arrowstyle="->", color='#D62728', lw=2,
                                        shrinkA=0, shrinkB=0))

    # Column obstacle
    col_fill = plt.Circle(r_col, COL_RADIUS, color='#FFCC00', alpha=0.3, zorder=5)
    col_edge = plt.Circle(r_col, COL_RADIUS, fill=False, color='#CC9900', linewidth=2.0, zorder=5)
    ax.add_patch(col_fill)
    ax.add_patch(col_edge)
    ax.scatter(r_col[0], r_col[1], color='black', marker='+', zorder=5)
    ax.annotate(f'Column Obstacle\nX: {COLUMN[0]:.3f}m\nY: {COLUMN[1]:.3f}m',
                xy=r_col, xytext=(0, 50), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#CC9900', ha='center', va='bottom',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#CC9900', lw=0.8))

    # Command waypoints
    wps = [r_stage, r_wp1, r_wp2, r_wp3]
    ax.scatter([p[0] for p in wps], [p[1] for p in wps],
               color='#D62728', s=120, facecolors='none', edgecolors='#D62728',
               linewidth=2.5, zorder=6, label='Command Waypoints')

    # WP labels
    wp_info = [
        (r_stage, f"WP_stage (U-turn)\nX: {WP_stage[0]:.3f}m\nY: {WP_stage[1]:.3f}m", (-70, 5)),
        (r_wp1,   f"Exp. Start-point\nX: {WP1[0]:.3f}m\nY: {WP1[1]:.3f}m",           (-70, -18)),
        (r_wp2,   f"Exp. End-point\nX: {WP2[0]:.3f}m\nY: {WP2[1]:.3f}m",             (15, 50)),
        (r_wp3,   f"Recovery (WP3)\nX: {WP3[0]:.3f}m\nY: {WP3[1]:.3f}m",            (15, -60)),
    ]
    for (x, y), lbl, off in wp_info:
        ax.annotate(lbl, xy=(x, y), xytext=off, textcoords='offset points',
                    fontsize=8, fontweight='bold', color='#444444', ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                    arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8))

    # Drone ghosts at waypoints
    for (x, y) in [r_stage, r_wp1, r_wp2, r_wp3]:
        draw_vector_drone(ax, x, y, CAGE_RADIUS, rotation_deg=0,
                          color_mode='gray', alpha_multiplier=0.3)

    # Full-colour drone at collision point
    draw_vector_drone(ax, r_coll[0], r_coll[1], CAGE_RADIUS, rotation_deg=0,
                      color_mode='full')
    safety_cage = plt.Circle(r_coll, CAGE_RADIUS, fill=False, color='#1F77B4',
                              linestyle='--', linewidth=2.0, zorder=5)
    ax.add_patch(safety_cage)
    ax.plot([], [], color='#1F77B4', linestyle='--', linewidth=2.0,
            label=f'Safety Cage (D={CAGE_DIAMETER*100:.1f}cm)')

    # Closest approach annotation
    clearance = np.hypot(COLUMN[0]-coll_pt[0], COLUMN[1]-coll_pt[1]) - COL_RADIUS - CAGE_RADIUS
    ax.annotate(f"Closest Approach\nClearance: {clearance*100:+.1f} cm",
                xy=r_coll, xytext=(0, -75), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#1F77B4', ha='center', va='top',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#1F77B4', lw=0.8))

    # 45° collision-angle annotation
    v_len = 0.28
    ax.quiver(r_coll[0], r_coll[1], v_len, 0,
              angles='xy', scale_units='xy', scale=1, color='crimson',
              width=0.005, headwidth=4.5, zorder=8,
              label='Velocity Vector at Contact')
    arc_r = 0.18
    arc = mpatches.Arc(r_coll, 2*arc_r, 2*arc_r,
                       angle=0, theta1=0, theta2=45,
                       color='crimson', linewidth=2.0, zorder=9)
    ax.add_patch(arc)
    import math
    theta_mid = math.radians(22.5)
    lr = arc_r + 0.07
    ax.text(r_coll[0] + lr * math.cos(theta_mid),
            r_coll[1] + lr * math.sin(theta_mid),
            "45°", color='crimson', fontweight='bold',
            fontsize=9, ha='center', va='center', zorder=10,
            bbox=dict(facecolor='white', alpha=0.95, edgecolor='crimson',
                      boxstyle='round,pad=0.15'))

    # Separation vector
    ax.plot([r_coll[0], r_col[0]], [r_coll[1], r_col[1]],
            color='purple', linestyle=':', linewidth=2, zorder=5,
            label='Separation Vector')

    # Axis setup
    ax.set_xlabel('Y coordinate, meters', color='#2CA02C', fontweight='bold')
    ax.set_ylabel('X coordinate, meters', color='#D62728', fontweight='bold')
    ax.tick_params(axis='x', colors='#2CA02C')
    ax.tick_params(axis='y', colors='#D62728')

    disc_text = ("X and Y axes have been rotated on this plot\n"
                 "to strictly adhere to the physical mapping of X-Y\n"
                 "in the motion capture system used")
    ax.text(0.02, 0.02, disc_text, transform=ax.transAxes,
            ha='left', va='bottom', fontsize=7, style='italic', alpha=0.8, zorder=10)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-0.5, 1.0)
    ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
    ax.set_yticks(np.arange(-0.5, 1.1, 0.5))

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc='upper left', prop={'family': 'monospace', 'size': 8.5})
    ax.grid(True, color='#EAEAEA')

    ax.set_title('45° Column Collision Loop — Full Mission Geometry',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()

    out_path = os.path.join(PLOTS_DIR, "45° Column Collision Loop Full Mission Geometry.png")
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"  ✓ 45° Column Collision Loop — Full Mission Geometry.png")
    plt.close()


# ================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("📤 Exporting thesis plots to thesis/plots/")
    print("=" * 60)

    copy_existing_graphics()

    print("=" * 60)
    print("🔄 Running flight-loader plots (may take a while) ...")
    print("=" * 60)
    run_flight_loader_plots()

    print("\n" + "=" * 60)
    print("🗺️  Generating full mission geometry plot ...")
    print("=" * 60)
    run_geometry_plot()

    print("\n" + "=" * 60)
    total = len([f for f in os.listdir(PLOTS_DIR) if f.endswith('.png')])
    print(f"✅ Done. {total} plots in thesis/plots/")
    print("=" * 60)
