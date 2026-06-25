#!/usr/bin/env python3
"""summary_plots.py — Aggregate Comparative Dashboard Plot Functions.

Thin, DataFrame-driven plotting functions for the experiments_analysis_summary
notebook.  Every function follows the same contract:

    plot_xxx(df_xxx, output_path=None, show_plot=True)

DataFrames are passed *in* (never queried internally) so the notebook's
data-loading cells remain the single source of truth for filtering.

All functions adhere to experiments_analysis_skill.md §4 (Universal Plotting
Standards): Rotating Cage = blue (#1F77B4) on the left, Fixed Cage = red
(#D62728) on the right, bold titles (no <> wrapping), data-origin footnotes,
trendline annotations per §4.1B, and hardcoded axis limits per §4.2.

Usage (from notebook)::

    from dev_logs.analysis.summary_plots import (
        plot_recovery_area_boxplot,
        plot_deviation_vs_angle_overlay,
        plot_battery_efficiency_comparison,
        plot_deceleration_vs_battery_split,
        plot_deceleration_vs_battery_global,
        plot_mission_outcome_pies,
        plot_mission_outcome_table,
        render_comparison_table_html,
        plot_sunburst_impact_distribution,
        plot_peak_accel_rotational_energy,
        plot_imu_z_vs_motor_rpm,
        plot_impact_angle_vs_max_deviation,
        plot_recovery_area_distribution,
        plot_2d_path_overlay,
        plot_attitude_shock_phase_portrait,
        plot_allocator_saturation,
        plot_pid_tracking_error,
    )

    plot_recovery_area_boxplot(df_impacts)
"""

import os
import sys
import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Path resolution ──────────────────────────────────────────────────────────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_GRAPHICS_DIR = os.path.join(_THIS_DIR, "graphics")
_project_root = os.path.abspath(os.path.join(_THIS_DIR, "..", ".."))
os.makedirs(_GRAPHICS_DIR, exist_ok=True)

# ── Module-level constants (§4.0) ────────────────────────────────────────────
C_ROT = '#1F77B4'   # Rotating Cage — blue
C_FIX = '#D62728'   # Fixed Cage — red
C_NO_IMPACT = '#B0B0B0'  # Neutral grey for No Impact (§4.4)

# Battery bin colours (§4.3)
BATTERY_BINS = [
    (0.0, 40.0, '#D62728', '0-40%'),
    (40.0, 60.0, '#FF7F0E', '40-60%'),
    (60.0, 100.0, '#2CA02C', '60-100%'),
]

# RdYlGn angle colormap: 0° = red (direct), 90° = green (glancing) — §4.4
ANGLE_CMAP = plt.cm.RdYlGn
ANGLE_NORM = plt.Normalize(0, 90)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  1.  Recovery Area Boxplot  (Cell 14)                                      ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_recovery_area_boxplot(df_impacts, output_path=None, show_plot=True):
    """Boxplot comparing recovery_area (cm²) between Rotating and Fixed cages.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Must contain columns: condition, recovery_area.
    output_path : str or None
        If given, saves the figure to this path.
    show_plot : bool
        If True, calls plt.show().
    """
    plot_data = df_impacts.dropna(subset=['recovery_area'])

    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=150)
    box_groups = ['Rotating Cage', 'Fixed Cage']
    box_data = [plot_data[plot_data['condition'] == name]['recovery_area']
                for name in box_groups]

    bp = ax.boxplot(box_data, tick_labels=box_groups, patch_artist=True,
                    widths=0.4, zorder=3)

    for patch, color in zip(bp['boxes'], [C_ROT, C_FIX]):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)
        patch.set_edgecolor(color)
        patch.set_linewidth(1.5)

    for median in bp['medians']:
        median.set(color='#333333', linewidth=2.0)
    for cap in bp['caps']:
        cap.set(color='#888888', linewidth=1.2)
    for whisker in bp['whiskers']:
        whisker.set(color='#888888', linewidth=1.2, linestyle='--')

    # Jittered scatter overlay
    for i, name in enumerate(box_groups, 1):
        vals = plot_data[plot_data['condition'] == name]['recovery_area']
        if not vals.empty:
            x_jitter = np.random.normal(i, 0.04, size=len(vals))
            ax.scatter(x_jitter, vals, color='#333333', s=35, alpha=0.6,
                       edgecolor='w', linewidth=0.8, zorder=4)

    ax.grid(True, linestyle=':', alpha=0.6, zorder=0)
    ax.set_title('Rotating Cage vs Fixed Cage — Trajectory Recovery Area',
                 fontsize=11, fontweight='bold', pad=12)
    ax.set_xlabel('Enclosure Type', fontsize=9, fontweight='bold')
    ax.set_ylabel('Recovery Envelope Area ($cm^2$)', fontsize=9, fontweight='bold')
    ax.set_ylim(0, 270)
    ax.set_yticks(np.arange(0, 280, 50))

    # Data origin footnote (§4.1A)
    n_rot = int((plot_data['condition'] == 'Rotating Cage').sum())
    n_fix = int((plot_data['condition'] == 'Fixed Cage').sum())
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))

    plt.tight_layout()
    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'recovery_area_comparison.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  2.  Deviation vs Angle Overlay  (Cell 17)                                 ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_deviation_vs_angle_overlay(df_rot, df_fix, output_path=None,
                                     show_plot=True):
    """Overlay scatter: avg_dev_after vs impact_angle, per-battery-bin trendlines.

    Parameters
    ----------
    df_rot : pd.DataFrame  Rotating Cage impacts.
    df_fix : pd.DataFrame  Fixed Cage impacts.
        Both need: avg_dev_after, impact_angle, battery_at_start.
    """
    plot_rot = df_rot.dropna(subset=['avg_dev_after', 'impact_angle',
                                      'battery_at_start'])
    plot_fix = df_fix.dropna(subset=['avg_dev_after', 'impact_angle',
                                      'battery_at_start'])

    fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=150)

    # ── Rotating Cage (dashed trendlines) ──
    for low, high, color, label_range in BATTERY_BINS:
        if low == 0:
            sub = plot_rot[(plot_rot['battery_at_start'] >= low) &
                           (plot_rot['battery_at_start'] <= high)]
        else:
            sub = plot_rot[(plot_rot['battery_at_start'] > low) &
                           (plot_rot['battery_at_start'] <= high)]
        if sub.empty:
            continue
        ax.scatter(sub['impact_angle'], sub['avg_dev_after'] / 10.0,
                   color=color, s=60, alpha=0.25, edgecolor='none', zorder=2)
        if len(sub) > 1:
            m, c = np.polyfit(sub['impact_angle'],
                              sub['avg_dev_after'] / 10.0, 1)
            xg = np.linspace(0, 90, 100)
            ax.plot(xg, m * xg + c, color=color, linestyle='--', linewidth=2.0,
                    label=f"Rot {label_range:<6} (m={m:+.3f})", zorder=4)

    # ── Fixed Cage (solid trendlines) ──
    for low, high, color, label_range in BATTERY_BINS:
        if low == 0:
            sub = plot_fix[(plot_fix['battery_at_start'] >= low) &
                           (plot_fix['battery_at_start'] <= high)]
        else:
            sub = plot_fix[(plot_fix['battery_at_start'] > low) &
                           (plot_fix['battery_at_start'] <= high)]
        if sub.empty:
            continue
        ax.scatter(sub['impact_angle'], sub['avg_dev_after'] / 10.0,
                   color=color, s=60, alpha=0.25, edgecolor='none', zorder=2)
        if len(sub) > 1:
            m, c = np.polyfit(sub['impact_angle'],
                              sub['avg_dev_after'] / 10.0, 1)
            xg = np.linspace(0, 90, 100)
            ax.plot(xg, m * xg + c, color=color, linestyle='-', linewidth=2.0,
                    label=f"Fix {label_range:<6} (m={m:+.3f})", zorder=4)

    ax.grid(True, linestyle=':', alpha=0.6, zorder=0)
    ax.set_xlim(0, 90)
    ax.set_ylim(0, 15)
    ax.set_title('Rotating Cage vs Fixed Cage — Stabilization Dynamics',
                 fontsize=11, fontweight='bold', pad=12)
    ax.set_xlabel('Impact Angle (deg)', fontsize=9, fontweight='bold')
    ax.set_ylabel('Average Deviation After Contact (cm)', fontsize=9,
                  fontweight='bold')
    ax.legend(title='Enclosure & Battery Trends', title_fontsize='9',
              loc='upper right', framealpha=0.95,
              prop={'family': 'monospace', 'size': 8.0})

    # Trendline footnote (§4.1B)
    fig.text(0.02, 0.01,
             "Note: Trendlines computed via ordinary least squares (y = mx + c).",
             fontsize=7, fontstyle='italic', color='#555555', ha='left',
             va='bottom')

    n_rot = len(plot_rot); n_fix = len(plot_fix)
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()
    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'comparative_cage_deviation_overlay.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  3.  Battery Efficiency Comparison  (Cell 19)                              ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_battery_efficiency_comparison(df_eff, output_dir=None, show_plot=True):
    """3-panel battery efficiency: duration boxplot, drain rate boxplot,
    voltage-vs-duration scatter.

    Parameters
    ----------
    df_eff : pd.DataFrame
        From flights_battery_efficiency table.
        Needs: condition, total_flying_time, capacity_drain_rate_flying,
        voltage_drop_rate_flying, avg_voltage_during_flight.
    """
    colors = [C_ROT, C_FIX]
    out = output_dir or _GRAPHICS_DIR
    n_rot = int((df_eff['condition'] == 'Rotating Cage').sum())
    n_fix = int((df_eff['condition'] == 'Fixed Cage').sum())
    data_origin = (f"Comparison of {n_rot}× Rotating Cage and "
                   f"{n_fix}× Fixed Cage flights")

    # ── Panel A: Active Flying Duration ────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(6, 4.5), dpi=150)
    box1 = ax1.boxplot([
        df_eff[df_eff['condition'] == 'Rotating Cage']['total_flying_time'],
        df_eff[df_eff['condition'] == 'Fixed Cage']['total_flying_time'],
    ], patch_artist=True, widths=0.4,
       medianprops=dict(color='black', linewidth=1.5))
    for patch, color in zip(box1['boxes'], colors):
        patch.set_facecolor(color); patch.set_alpha(0.7)
    ax1.set_xticklabels(['Rotating Cage', 'Fixed Cage'], fontweight='bold')
    ax1.set_ylabel('Active Flying Duration (seconds)', fontsize=11, fontweight='bold')
    ax1.set_title('Rotating Cage vs Fixed Cage — Active Flying Duration',
                  fontsize=12, fontweight='bold', pad=15)
    ax1.grid(True, linestyle='--', alpha=0.3)
    fig1.text(0.98, 0.01, data_origin, ha='right', va='bottom',
              fontsize=7.5, fontweight='bold',
              bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                        edgecolor='none'))
    plt.tight_layout()
    _save_and_show(fig1, os.path.join(out, 'plot_10a_flying_duration.png'),
                   show_plot)

    # ── Panel B: Capacity Drain Rate ───────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(6, 4.5), dpi=150)
    box2 = ax2.boxplot([
        df_eff[df_eff['condition'] == 'Rotating Cage']['capacity_drain_rate_flying'],
        df_eff[df_eff['condition'] == 'Fixed Cage']['capacity_drain_rate_flying'],
    ], patch_artist=True, widths=0.4,
       medianprops=dict(color='black', linewidth=1.5))
    for patch, color in zip(box2['boxes'], colors):
        patch.set_facecolor(color); patch.set_alpha(0.7)
    ax2.set_xticklabels(['Rotating Cage', 'Fixed Cage'], fontweight='bold')
    ax2.set_ylabel('Capacity Consumption Rate (%/min)', fontsize=11, fontweight='bold')
    ax2.set_title('Rotating Cage vs Fixed Cage — Battery Capacity Drain Rate',
                  fontsize=12, fontweight='bold', pad=15)
    ax2.grid(True, linestyle='--', alpha=0.3)
    fig2.text(0.98, 0.01, data_origin, ha='right', va='bottom',
              fontsize=7.5, fontweight='bold',
              bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                        edgecolor='none'))
    plt.tight_layout()
    _save_and_show(fig2, os.path.join(out, 'plot_10b_capacity_drain_rate.png'),
                   show_plot)

    # ── Panel C: Voltage vs Duration Scatter ───────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(7, 5), dpi=150)
    df_rs = df_eff[df_eff['condition'] == 'Rotating Cage']
    df_fs = df_eff[df_eff['condition'] == 'Fixed Cage']
    ax3.scatter(df_rs['total_flying_time'], df_rs['avg_voltage_during_flight'],
                color=C_ROT, alpha=0.8, edgecolors='none', s=50,
                label='Rotating Cage')
    ax3.scatter(df_fs['total_flying_time'], df_fs['avg_voltage_during_flight'],
                color=C_FIX, alpha=0.8, edgecolors='none', s=50,
                label='Fixed Cage')
    for df_sub, color in [(df_rs, C_ROT), (df_fs, C_FIX)]:
        if len(df_sub) > 1:
            z = np.polyfit(df_sub['total_flying_time'],
                           df_sub['avg_voltage_during_flight'], 1)
            xp = np.linspace(df_sub['total_flying_time'].min(),
                             df_sub['total_flying_time'].max(), 100)
            ax3.plot(xp, np.poly1d(z)(xp), color=color, linestyle='--',
                     alpha=0.6, linewidth=1.5)
    ax3.set_xlabel('Active Flying Duration (seconds)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Average Flying Voltage (V)', fontsize=11, fontweight='bold')
    ax3.set_title('Rotating Cage vs Fixed Cage — Voltage vs Duration',
                  fontsize=12, fontweight='bold', pad=15)
    ax3.legend(frameon=True, facecolor='white', edgecolor='none')
    ax3.grid(True, linestyle='--', alpha=0.3)
    fig3.text(0.02, 0.01,
             'Note: Trendlines computed via ordinary least squares (y = mx + c).',
             fontsize=7, fontstyle='italic', color='#555555', ha='left',
             va='bottom')
    fig3.text(0.98, 0.01, data_origin, ha='right', va='bottom',
              fontsize=7.5, fontweight='bold',
              bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                        edgecolor='none'))
    plt.tight_layout()
    _save_and_show(fig3, os.path.join(out, 'plot_10c_voltage_vs_duration.png'),
                   show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  4.  Deceleration vs Battery — Split Panel  (Cell 21)                      ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_deceleration_vs_battery_split(df_impacts, output_path=None,
                                        show_plot=True):
    """Side-by-side scatter: battery vs deceleration colored by impact angle.

    Data Provenance
    --------------
    Data flows through the following pipeline before reaching this plot:

    1. **Raw telemetry** — ``/fmu/out/vehicle_local_position`` (EKF2 velocity)
       logged by PX4 at ~50 Hz during each collision flight.

    2. **Velocity → acceleration** — ``kin_calculator.py`` applies a
       Savitzky-Golay (SG) derivative filter (window=5, order=2) to the
       velocity magnitude time series, producing ``impact_accel``, the
       deceleration experienced at the moment of closest approach to the
       column.  The SG filter suppresses high-frequency noise from numerical
       differentiation while preserving the impact-jerk signature.

    3. **Battery state** — ``battery_at_start`` is the LiPo percentage at
       mission commencement, recorded by ``db_pipeline.py`` from the
       ``flights_summary`` table.  This is the *starting* state of charge for
       each individual pass (not an instantaneous reading at impact).

    4. **Impact angle** — Computed as the angle between the drone's velocity
       vector and the column surface normal at the point of closest approach
       (the "collision normal").  See the notebook §4 (Trigonometric
       Derivation) for the full geometric derivation.

    5. **Filter** — Only flights with ``impact_detected == 1`` are included
       (127 flights after manual validation against the cage radius).

    6. **Regression** — The dashed/solid trendlines use Huber regression
       (``huber_regressor`` from ``eda_angle_prediction.py``), which is robust
       to outliers.  Unlike OLS, Huber loss transitions from quadratic to
       linear for residuals beyond δ = 1.35, preventing a few high-deceleration
       outliers from dominating the slope estimate.

    Interpretation Notes
    -------------------
    - The color gradient (Red → Green = 0° → 90° impact angle) reveals
      whether direct head-on collisions (red) or glancing brushes (green)
      dominate a given battery range.
    - The improvement annotation below the panels is computed from the
      *pooled* means of all flights per condition — it should be read as an
      aggregate operational characteristic, not a claim of causality from
      battery to deceleration.
    - The plot exposes whether deceleration severity is uniform across
      battery states, or whether low-battery flights (where voltage sag may
      reduce available motor thrust) cluster at different deceleration
      magnitudes.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs columns: condition, impact_accel, battery_at_start,
        impact_angle.
    output_path : str or None
        If provided, save the figure to this path.
    show_plot : bool
        Whether to display the figure interactively.
    """
    from dev_logs.analysis.eda.eda_angle_prediction import huber_regressor

    plot_data = df_impacts.copy()
    plot_data['deceleration'] = plot_data['impact_accel'].abs()
    plot_data = plot_data.dropna(subset=['deceleration', 'battery_at_start',
                                          'impact_angle'])
    df_rot = plot_data.query("condition == 'Rotating Cage'")
    df_fix = plot_data.query("condition == 'Fixed Cage'")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True,
                                    dpi=150)
    norm = plt.Normalize(0, 90)
    cmap = plt.cm.RdYlGn

    # Rotating Cage
    sc1 = ax1.scatter(df_rot['battery_at_start'], df_rot['deceleration'],
                      c=df_rot['impact_angle'], cmap=cmap, norm=norm,
                      s=90, alpha=0.85, edgecolor='w', linewidth=0.8, zorder=3)
    if len(df_rot) > 1:
        m, c = huber_regressor(df_rot['battery_at_start'],
                               df_rot['deceleration'])
        xr = np.linspace(df_rot['battery_at_start'].min(),
                         df_rot['battery_at_start'].max(), 100)
        ax1.plot(xr, m * xr + c, color='#444444', linestyle='--', linewidth=1.5,
                 alpha=0.7, label=f'Huber Trend (m={m:.3f})')
        ax1.legend(loc='upper left', fontsize=8.5, framealpha=0.9)
    ax1.set_title('Rotating Cage', fontsize=11, fontweight='bold', pad=12,
                  color='black')
    ax1.set_xlabel('Battery at Start (%)', fontsize=9.5, fontweight='bold')
    ax1.set_ylabel('Maximum Deceleration (m/s²)', fontsize=9.5, fontweight='bold')
    ax1.set_xticks(range(0, 101, 10))
    ax1.grid(True, linestyle=':', alpha=0.6, zorder=0)

    # Fixed Cage
    sc2 = ax2.scatter(df_fix['battery_at_start'], df_fix['deceleration'],
                      c=df_fix['impact_angle'], cmap=cmap, norm=norm,
                      s=90, alpha=0.85, edgecolor='w', linewidth=0.8, zorder=3)
    if len(df_fix) > 1:
        m, c = huber_regressor(df_fix['battery_at_start'],
                               df_fix['deceleration'])
        xf = np.linspace(df_fix['battery_at_start'].min(),
                         df_fix['battery_at_start'].max(), 100)
        ax2.plot(xf, m * xf + c, color='#444444', linestyle='-', linewidth=1.5,
                 alpha=0.7, label=f'Huber Trend (m={m:.3f})')
        ax2.legend(loc='upper left', fontsize=8.5, framealpha=0.9)
    ax2.set_title('Fixed Cage', fontsize=11, fontweight='bold', pad=12,
                  color='black')
    ax2.set_xlabel('Battery at Start (%)', fontsize=9.5, fontweight='bold')
    ax2.set_xticks(range(0, 101, 10))
    ax2.grid(True, linestyle=':', alpha=0.6, zorder=0)

    fig.suptitle('Impact Deceleration vs. Start Battery %', fontsize=13,
                 fontweight='bold', y=0.95)
    ax1.set_ylim(0, 3.5)
    ax1.set_yticks(np.arange(0, 4.0, 0.5))
    fig.subplots_adjust(left=0.08, right=0.86, bottom=0.18, top=0.83,
                        wspace=0.15)

    # Colourbar
    cax = fig.add_axes([0.90, 0.15, 0.02, 0.68])
    cbar = fig.colorbar(sc2, cax=cax)
    cbar.set_label('Collision Impact Angle (deg)', fontsize=9.5, fontweight='bold')
    cbar.ax.tick_params(labelsize=8.5)

    # Improvement annotation — REMOVED per user request (misleading aggregate number)
    # rot_mean = df_rot['deceleration'].dropna().mean()
    # fix_mean = df_fix['deceleration'].dropna().mean()
    # improvement = (fix_mean - rot_mean) / fix_mean * 100
    # direction = 'reduces' if improvement > 0 else 'increases'
    # fig.text(0.5, 0.07,
    #          f"Rotating Cage {direction} impact deceleration by "
    #          f"{abs(improvement):.1f}% vs Fixed Cage",
    #          ha='center', va='center', fontsize=9.5, fontweight='bold')

    # Footnotes (§4.1B, §4.1A)
    fig.text(0.08, 0.02,
             "Note: Trendlines computed via robust Huber regression "
             "(y = mx + c).",
             fontsize=8.5, fontstyle='italic', color='#555555')
    n_rot = len(df_rot); n_fix = len(df_fix)
    fig.text(0.98, 0.02,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=8.5, color='#555555')

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'deceleration_vs_battery_angle.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  5.  Deceleration vs Battery — Global  (Cell 22)                           ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_deceleration_vs_battery_global(df_impacts, output_path=None,
                                         show_plot=True):
    """Single-panel overlay: battery vs deceleration, both conditions.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, impact_accel, battery_at_start.
    """
    from dev_logs.analysis.eda.eda_angle_prediction import huber_regressor

    plot_data = df_impacts.copy()
    plot_data['deceleration'] = plot_data['impact_accel'].abs()
    plot_data = plot_data.dropna(subset=['deceleration', 'battery_at_start'])
    df_rot = plot_data.query("condition == 'Rotating Cage'")
    df_fix = plot_data.query("condition == 'Fixed Cage'")

    fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=150)

    ax.scatter(df_rot['battery_at_start'], df_rot['deceleration'],
               color=C_ROT, s=90, alpha=0.75, edgecolor='w', linewidth=0.8,
               label=f'Rotating Cage (N = {len(df_rot)})', zorder=3)
    if len(df_rot) > 1:
        m, c = huber_regressor(df_rot['battery_at_start'],
                               df_rot['deceleration'])
        xr = np.linspace(df_rot['battery_at_start'].min(),
                         df_rot['battery_at_start'].max(), 100)
        ax.plot(xr, m * xr + c, color=C_ROT, linestyle='--', linewidth=2.0,
                label=f'Rotating Trend (m={m:+.3f})', zorder=4)

    ax.scatter(df_fix['battery_at_start'], df_fix['deceleration'],
               color=C_FIX, s=90, alpha=0.75, edgecolor='w', linewidth=0.8,
               label=f'Fixed Cage (N = {len(df_fix)})', zorder=3)
    if len(df_fix) > 1:
        m, c = huber_regressor(df_fix['battery_at_start'],
                               df_fix['deceleration'])
        xf = np.linspace(df_fix['battery_at_start'].min(),
                         df_fix['battery_at_start'].max(), 100)
        ax.plot(xf, m * xf + c, color=C_FIX, linestyle='-', linewidth=2.0,
                label=f'Fixed Trend (m={m:+.3f})', zorder=4)

    ax.set_title('Rotating Cage vs Fixed Cage — Impact Deceleration vs. Battery',
                 fontsize=11, fontweight='bold', pad=12, color='black')
    ax.set_xlabel('Battery at Start (%)', fontsize=9.5, fontweight='bold')
    ax.set_ylabel('Maximum Deceleration (m/s²)', fontsize=9.5, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6, zorder=0)
    ax.set_ylim(0, 3.5)
    ax.set_yticks(np.arange(0, 4.0, 0.5))
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)

    # Improvement annotation
    rot_mean = df_rot['deceleration'].dropna().mean()
    fix_mean = df_fix['deceleration'].dropna().mean()
    improvement = (fix_mean - rot_mean) / fix_mean * 100
    direction = 'reduces' if improvement > 0 else 'increases'
    ax.text(0.02, 0.98,
            f"Rotating Cage {direction} impact deceleration by "
            f"{abs(improvement):.1f}% vs Fixed Cage",
            transform=ax.transAxes, ha='left', va='top', fontsize=9,
            fontweight='bold')

    fig.text(0.08, 0.02,
             "Note: Trendlines computed via robust Huber regression "
             "(y = mx + c).",
             fontsize=8.5, fontstyle='italic', color='#555555')
    n_rot = len(df_rot); n_fix = len(df_fix)
    fig.text(0.98, 0.02,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    fig.subplots_adjust(top=0.90, bottom=0.12)

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'deceleration_vs_battery_global.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  6.  Mission Outcome Pies + Comparison Table  (Cell 26)                    ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_mission_outcome_pies(df_all, output_path=None, show_plot=True):
    """3×2 pie grid: Rotating|Fixed × (Total, 45° mission, 75° mission).

    Parameters
    ----------
    df_all : pd.DataFrame
        Full flights_summary. Needs: condition, impact_detected,
        impact_angle, flight_name.
    """
    df = df_all.copy()
    df['mission_type'] = df['flight_name'].apply(
        lambda x: '45°' if '45°' in x else ('75°' if '75°' in x else 'Other'))

    def _categorize(row):
        if row['impact_detected'] == 0:
            return 'No Impact'
        angle = row['impact_angle']
        if pd.isna(angle):
            return 'Impact (Unknown Angle)'
        if angle < 30:   return 'Impact (<30°)'
        elif angle < 40: return 'Impact (30-40°)'
        elif angle < 50: return 'Impact (40-50°)'
        elif angle < 60: return 'Impact (50-60°)'
        else:            return 'Impact (60-90°)'

    df['outcome'] = df.apply(_categorize, axis=1)

    colors_map = {
        'No Impact': '#e0e0e0', 'Impact (<30°)': '#aec7e8',
        'Impact (30-40°)': '#1f77b4', 'Impact (40-50°)': '#ffbb78',
        'Impact (50-60°)': '#2ca02c', 'Impact (60-90°)': '#ff9896',
        'Impact Detected': '#aec7e8',
    }

    fig, axes = plt.subplots(3, 2, figsize=(12, 15), dpi=150)

    # Row 1: Total
    for col, cond in enumerate(['Rotating Cage', 'Fixed Cage']):
        sub = df[df['condition'] == cond]
        cnt = sub['impact_detected'].map({1: 'Impact Detected',
                                           0: 'No Impact'}).value_counts()
        axes[0, col].pie([cnt.get('Impact Detected', 0), cnt.get('No Impact', 0)],
                         labels=['Impact Detected', 'No Impact'],
                         autopct='%1.1f%%', startangle=90,
                         colors=[colors_map['Impact Detected'],
                                 colors_map['No Impact']],
                         textprops={'fontsize': 9})
        axes[0, col].set_title(f'{cond} — Total Flights (n={len(sub)})',
                               fontsize=11, fontweight='bold')

    # Rows 2-3: 45° / 75° missions
    for row, mission in enumerate([('45°', 1), ('75°', 2)]):
        mtype, r = mission
        for col, cond in enumerate(['Rotating Cage', 'Fixed Cage']):
            sub = df[(df['condition'] == cond) &
                     (df['mission_type'] == mtype)]
            cnt = sub['outcome'].value_counts()
            lbls = [k for k in colors_map if k in cnt and k != 'Impact Detected']
            vals = [cnt[k] for k in lbls]
            axes[r, col].pie(vals, labels=lbls, autopct='%1.1f%%', startangle=90,
                             colors=[colors_map[k] for k in lbls],
                             textprops={'fontsize': 9})
            axes[r, col].set_title(
                f'{cond} — {mtype} Mission (n={len(sub)})',
                fontsize=11, fontweight='bold')

    fig.suptitle('Rotating Cage vs Fixed Cage — Mission Outcomes & '
                 'Impact Angle Distributions', fontsize=14, fontweight='bold',
                 y=0.98)
    plt.tight_layout()

    n_rot = int((df['condition'] == 'Rotating Cage').sum())
    n_fix = int((df['condition'] == 'Fixed Cage').sum())
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_13_mission_pies.png'), show_plot)


def plot_mission_outcome_table(df_all, output_path=None, show_plot=True):
    """Generate a LaTeX table of mission outcome distribution for \\input{} inclusion.

    Produces a clean ``.tex`` file with a ``tabular`` environment (no float
    wrapper) that can be picked up by the thesis via::

        \\begin{table}[htbp]
        \\centering
        \\caption{...}
        \\label{tab:mission_outcomes}
        \\input{tables/mission_outcome_table.tex}
        \\end{table}

    Requires ``\\usepackage{booktabs}`` in the preamble (already loaded in
    ``main.tex``).  No other packages needed.

    Parameters
    ----------
    df_all : pd.DataFrame
        Full flights_summary. Needs: condition, impact_detected,
        impact_angle, flight_name.
    output_path : str or None
        Base output path (without extension — .tex appended).
        Defaults to ``thesis/tables/mission_outcome_table``.
    show_plot : bool
        If True, print the generated LaTeX to stdout.
    """
    df = df_all.copy()
    df['mission_type'] = df['flight_name'].apply(
        lambda x: '45°' if '45°' in x else ('75°' if '75°' in x else 'Other'))

    def _categorize(row):
        if row['impact_detected'] == 0:
            return 'No Impact'
        angle = row['impact_angle']
        # All 127 impact-detected flights have a valid measured angle;
        # no NaN / Unknown cases exist in the current dataset.
        if angle < 30:
            return '{<\,}30°'
        elif angle < 40:
            return '30–40°'
        elif angle < 50:
            return '40–50°'
        elif angle < 60:
            return '50–60°'
        else:
            return '60–90°'

    df['outcome'] = df.apply(_categorize, axis=1)

    # ── Build table data ────────────────────────────────────────────────────
    # No "Unknown" bin — all 168 flights have a known outcome
    # (either No Impact or a measured angle ≥ 0°)
    angle_bins = ['No Impact', '{<\\,}30°', '30–40°', '40–50°', '50–60°',
                  '60–90°']
    conditions = ['Rotating Cage', 'Fixed Cage']
    missions = ['45°', '75°', 'Total']

    rows_data = []  # list of (label, is_total, [N, *counts_per_bin])
    for cond in conditions:
        for mission in missions:
            label = ('45°' if mission == '45°' else
                     '75°' if mission == '75°' else 'All')
            is_total = (mission == 'Total')
            if mission == 'Total':
                sub = df[df['condition'] == cond]
            else:
                sub = df[(df['condition'] == cond) &
                         (df['mission_type'] == mission)]
            n = len(sub)
            counts = [int((sub['outcome'] == b).sum()) for b in angle_bins]
            rows_data.append((cond, label, is_total, [n] + counts))

    # ── Assemble LaTeX ──────────────────────────────────────────────────────
    col_spec = 'l' + 'r' * (len(angle_bins) + 1)  # label + N + 7 angle bins
    header_bins = ['N', 'No Impact', '{<\\,}30°', '30–40°', '40–50°',
                   '50–60°', '60–90°']

    lines = []
    lines.append('% Auto-generated by plot_mission_outcome_table()')
    lines.append('% Requires: \\usepackage{booktabs}')
    lines.append(r'\begin{tabular}{%s}' % col_spec)
    lines.append(r'\toprule')
    # Vertically oriented headers to reduce table width
    header_tex = ' & ' + ' & '.join(
        r'\rotatebox{90}{\textsf{%s}}' % h for h in header_bins
    ) + r' \\'
    lines.append(header_tex)
    lines.append(r'\midrule')

    for cond in conditions:
        lines.append(r'\multicolumn{%d}{l}{\textbf{%s}} \\' %
                     (len(header_bins) + 1, cond))
        lines.append(r'\midrule')
        for c, label, is_total, vals in rows_data:
            if c != cond:
                continue
            row_label = (r'\quad \textbf{%s}' % label) if is_total else (
                        r'\quad %s' % label)
            row_str = ' & '.join([row_label] + [str(v) for v in vals])
            lines.append(row_str + r' \\')
            if is_total:
                lines.append(r'\addlinespace')

    # Remove trailing \addlinespace (cosmetic — doesn't affect compile)
    if lines[-1] == r'\addlinespace':
        lines.pop()

    lines.append(r'\bottomrule')
    lines.append(r'\end{tabular}')

    latex_str = '\n'.join(lines) + '\n'

    # ── Source comment ──────────────────────────────────────────────────────
    n_rot = int((df['condition'] == 'Rotating Cage').sum())
    n_fix = int((df['condition'] == 'Fixed Cage').sum())
    source_comment = (
        f'%% Data: experiments_summary.db → flights_summary '
        f'({n_rot}× Rotating, {n_fix}× Fixed flights)\n'
    )

    # ── Save ────────────────────────────────────────────────────────────────
    tables_dir = os.path.join(_project_root, 'thesis', 'tables')
    base = output_path or os.path.join(tables_dir, 'mission_outcome_table')
    tex_path = base + '.tex'
    os.makedirs(os.path.dirname(tex_path), exist_ok=True)

    with open(tex_path, 'w') as f:
        f.write(latex_str + source_comment)

    print(f'[INFO] Saved → {os.path.relpath(tex_path, _project_root)}')

    if show_plot:
        # Build a clean HTML preview directly from the data (no LaTeX
        # parsing).  KaTeX in Jupyter cannot render tabular/table
        # environments, so we show HTML instead.  The actual .tex file
        # is ingested by the thesis inside a table float + caption.
        try:
            from IPython.display import display, HTML

            n_cols = len(header_bins) + 1  # +1 for row-label column
            parts = ['<table style="border-collapse:collapse;'
                     'font-size:0.85em;margin:0.5em 0;">']

            # ── Header row ──
            parts.append('<tr style="border-bottom:2px solid #333;">')
            parts.append('<th style="padding:4px 10px;"></th>')
            for h in header_bins:
                parts.append(
                    f'<th style="padding:4px 8px;text-align:right;'
                    f'writing-mode:vertical-rl;font-size:0.85em;'
                    f'font-weight:bold;color:#444;">{h}</th>')
            parts.append('</tr>')

            # ── Data rows ──
            for cond in conditions:
                # Section label row
                parts.append(
                    f'<tr><th colspan="{n_cols}" '
                    f'style="padding:8px 10px 2px 10px;text-align:left;'
                    f'font-weight:bold;">{cond}</th></tr>')
                for c, label, is_total, vals in rows_data:
                    if c != cond:
                        continue
                    display_label = (' ' + label) if not is_total \
                                    else (' ' + label)
                    tag = 'th' if is_total else 'td'
                    style = ('padding:2px 10px;text-align:left;'
                             'font-weight:bold;') if is_total else \
                            ('padding:2px 10px;text-align:left;')
                    parts.append('<tr>')
                    parts.append(f'<{tag} style="{style}">{display_label}'
                                 f'</{tag}>')
                    for v in vals:
                        parts.append(
                            f'<td style="padding:2px 8px;text-align:right;'
                            f'font-family:monospace;">{v}</td>')
                    parts.append('</tr>')
                # spacer
                parts.append(
                    '<tr><td colspan="{}" style="padding:0;"></td></tr>'
                    .format(n_cols))

            parts.append('</table>')

            # Source line
            parts.append(
                f'<p style="color:#888;font-size:0.8em;margin-top:0.3em;">'
                f'{n_rot}× Rotating, {n_fix}× Fixed flights &nbsp;|&nbsp; '
                f'<code>thesis/tables/mission_outcome_table.tex</code> → '
                f'ingested by thesis via <code>\\input</code></p>')

            display(HTML('\n'.join(parts)))
        except ImportError:
            print(f'\n{latex_str}')


def render_comparison_table_html(df_all):
    """Return an HTML string for the styled angle-bin comparison table.

    Uses IPython.display.HTML in the notebook cell.  Can be called standalone
    to get the raw HTML string.

    Parameters
    ----------
    df_all : pd.DataFrame
        Full flights_summary. Needs: condition, impact_detected, impact_angle.

    Returns
    -------
    str : Styled HTML table.
    """
    df_impacts = df_all.query("impact_detected == 1").copy()
    bins = [30, 40, 50, 60, 90]
    labels_table = ['30-40°', '40-50°', '50-60°', '60-90°']

    df_v = df_impacts.dropna(subset=['impact_angle']).copy()
    df_v['angle_bin'] = pd.cut(df_v['impact_angle'], bins=bins,
                                labels=labels_table, right=False)
    df_v.loc[df_v['impact_angle'] == 90, 'angle_bin'] = '60-90°'
    df_t = df_v.dropna(subset=['angle_bin']).copy()

    # Placeholder columns
    df_t['Avg RPM max'] = np.nan
    df_t['Avg Amp Draw'] = np.nan
    df_t['Avg Voltage Drop'] = np.nan

    metrics = {
        'flight_name': 'count', 'Avg RPM max': 'mean',
        'Avg Amp Draw': 'mean', 'Avg Voltage Drop': 'mean',
        'imu_peak_accel': 'mean', 'imu_peak_gyro': 'mean',
        'max_dev_after': 'mean', 'recovery_area': 'mean',
        'motor_thrust_surge': 'mean', 'motor_imbalance_after': 'mean',
    }
    df_avg = df_t.groupby(['angle_bin', 'condition'],
                           observed=False).agg(metrics).reset_index()
    df_avg.rename(columns={
        'flight_name': 'N-Flights',
        'imu_peak_accel': 'IMU Max Spikes (Accel)',
        'imu_peak_gyro': 'IMU Max Spikes (Gyro)',
        'max_dev_after': 'Average Deviation Max (mm)',
        'recovery_area': 'Average Recovery Area (mm*m)',
        'motor_thrust_surge': 'Avg Thrust Surge',
        'motor_imbalance_after': 'Avg Motor Imbalance',
    }, inplace=True)
    df_avg = df_avg.sort_values(by=['angle_bin', 'condition']
                                ).reset_index(drop=True)
    df_avg = df_avg[[
        'angle_bin', 'condition', 'N-Flights',
        'Avg RPM max', 'Avg Amp Draw', 'Avg Voltage Drop',
        'IMU Max Spikes (Accel)', 'IMU Max Spikes (Gyro)',
        'Average Deviation Max (mm)', 'Average Recovery Area (mm*m)',
        'Avg Thrust Surge', 'Avg Motor Imbalance',
    ]]

    cols_to_compare = [
        'IMU Max Spikes (Accel)', 'IMU Max Spikes (Gyro)',
        'Average Deviation Max (mm)', 'Average Recovery Area (mm*m)',
        'Avg Thrust Surge', 'Avg Motor Imbalance',
    ]

    html = ('<table border="1" class="dataframe" '
            'style="border-collapse:collapse;text-align:right;'
            'font-family:monospace;font-size:13px;">')
    html += ('<tr style="background-color:#f2f2f2;font-weight:bold;">')
    for col in df_avg.columns:
        html += f'<th style="padding:8px;border:1px solid #ddd;">{col}</th>'
    html += '</tr>'

    for _, row in df_avg.iterrows():
        bin_val = row['angle_bin']; cond_val = row['condition']
        sibling_cond = 'Fixed Cage' if cond_val == 'Rotating Cage' else 'Rotating Cage'
        sibling_rows = df_avg[(df_avg['angle_bin'] == bin_val) &
                               (df_avg['condition'] == sibling_cond)]
        has_sib = len(sibling_rows) > 0
        if has_sib:
            sib_row = sibling_rows.iloc[0]

        html += '<tr>'
        for col_idx, col in enumerate(df_avg.columns):
            val = row[col]; style = 'padding:8px;border:1px solid #ddd;'
            if col == 'N-Flights':
                val_str = f"{val:.0f}"
            elif col in ['angle_bin', 'condition']:
                val_str = str(val)
            else:
                val_str = 'N/A' if pd.isna(val) else f"{val:.2f}"

            if col_idx < 4:
                if cond_val == 'Rotating Cage':
                    style += (' background-color:#d2e4f6;color:#0d233a;'
                              'font-weight:500;')
                else:
                    style += (' background-color:#f7d2d2;color:#3d0b0b;'
                              'font-weight:500;')
            elif col in cols_to_compare and has_sib:
                sib_val = sib_row[col]
                if not pd.isna(val) and not pd.isna(sib_val):
                    better = min(val, sib_val); worse = max(val, sib_val)
                    ratio = (1.0 - better / worse) if worse > 0 else 0.0
                    ratio = min(max(ratio, 0.0), 0.8)
                    if val == sib_val:
                        style += ' background-color:#f9f9f9;color:#333;'
                    elif val < sib_val:
                        lg = int(95 - 25 * ratio); tg = int(30 - 15 * ratio)
                        style += (f' background-color:hsl(134,50%,{lg}%);'
                                  f'color:hsl(134,60%,{tg}%);font-weight:bold;')
                    else:
                        lr = int(95 - 20 * ratio); tr = int(30 - 15 * ratio)
                        style += (f' background-color:hsl(0,65%,{lr}%);'
                                  f'color:hsl(0,75%,{tr}%);')
            html += f'<td style="{style}">{val_str}</td>'
        html += '</tr>'
    html += '</table>'
    return html


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  7.  Sunburst Impact Distribution  (Cell 27 → replaces _sunburst_cell)     ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

# Hardcoded aggregated percentages (as of 2026-06-09)
_SUNBURST_DATA = {
    'Rotating Cage': {
        'total_n': 81,
        'total': {'Impact Detected': 74.1, 'No Impact': 25.9},
        '45°': {'n': 50, '<30°': 18.0, '30-40°': 42.0, '40-50°': 26.0,
                '50-60°': 10.0, '60-90°': 0.0, 'No Impact': 4.0},
        '75°': {'n': 31, '<30°': 0.0, '30-40°': 6.5, '40-50°': 6.5,
                '50-60°': 9.7, '60-90°': 16.1, 'No Impact': 61.3},
    },
    'Fixed Cage': {
        'total_n': 87,
        'total': {'Impact Detected': 77.0, 'No Impact': 23.0},
        '45°': {'n': 56, '<30°': 5.4, '30-40°': 32.1, '40-50°': 23.2,
                '50-60°': 17.9, '60-90°': 12.5, 'No Impact': 8.9},
        '75°': {'n': 31, '<30°': 6.5, '30-40°': 0.0, '40-50°': 12.9,
                '50-60°': 16.1, '60-90°': 16.1, 'No Impact': 48.4},
    },
}
_SUNBURST_ANGLE_BINS = ['<30°', '30-40°', '40-50°', '50-60°', '60-90°']
_SUNBURST_BIN_MIDPOINTS = {'<30°': 15, '30-40°': 35, '40-50°': 45,
                           '50-60°': 55, '60-90°': 75}


def plot_sunburst_impact_distribution(output_path=None, show_plot=True):
    """Nested radial (sunburst) chart: one panel per cage condition.

    Uses hardcoded aggregated percentage data.  Regenerate when DB is updated.
    """
    import matplotlib.patches as mpatches

    data = _SUNBURST_DATA
    angle_cmap = plt.cm.RdYlGn
    angle_norm = plt.Normalize(0, 90)

    def _bin_color(label):
        if label == 'No Impact':
            return C_NO_IMPACT
        return angle_cmap(angle_norm(_SUNBURST_BIN_MIDPOINTS[label]))

    order = _SUNBURST_ANGLE_BINS + ['No Impact']

    def _pct_filter(p):
        return f'{p:.1f}%' if p >= 5.0 else ''

    fig, axes = plt.subplots(1, 2, figsize=(20, 12), dpi=150,
                             subplot_kw=dict(aspect='equal'))
    plt.subplots_adjust(left=0.05, right=0.95, top=0.85, bottom=0.2,
                        wspace=0.3)

    for ax, cage in zip(axes, ['Rotating Cage', 'Fixed Cage']):
        d = data[cage]
        cond_color = C_ROT if cage == 'Rotating Cage' else C_FIX

        # Outer ring — Total Impact vs No Impact
        impact_pct = d['total']['Impact Detected']
        noimpact_pct = d['total']['No Impact']
        ax.pie([impact_pct, noimpact_pct], labels=None,
               colors=[cond_color, C_NO_IMPACT],
               startangle=90, counterclock=True, radius=1.4,
               wedgeprops=dict(width=0.3, edgecolor='white', linewidth=1.5),
               autopct=None)

        # Middle ring — 45° mission
        vals_45 = [d['45°'][cat] for cat in order]
        colors_45 = [_bin_color(cat) for cat in order]
        ax.pie(vals_45, labels=None, colors=colors_45,
               startangle=90, counterclock=True, radius=1.05,
               wedgeprops=dict(width=0.15, edgecolor='black', linewidth=1.0),
               autopct=_pct_filter, pctdistance=0.975,
               textprops={'fontsize': 9, 'fontweight': 'bold',
                          'color': 'black', 'ha': 'center', 'va': 'center'})

        # Inner ring — 75° mission
        vals_75 = [d['75°'][cat] for cat in order]
        colors_75 = [_bin_color(cat) for cat in order]
        ax.pie(vals_75, labels=None, colors=colors_75,
               startangle=90, counterclock=True, radius=0.7,
               wedgeprops=dict(width=0.15, edgecolor='black', linewidth=1.0),
               autopct=_pct_filter, pctdistance=0.625,
               textprops={'fontsize': 9, 'fontweight': 'bold',
                          'color': 'black', 'ha': 'center', 'va': 'center'})

        # Centre label
        ax.text(0, 0, f'{cage.split()[0]}\nn = {d["total_n"]}',
                ha='center', va='center', fontsize=11, fontweight='bold')
        ax.set_title(f'{cage}', fontsize=12, fontweight='bold', pad=20)

        # Data table in bottom corners
        gl45 = d['45°']['50-60°'] + d['45°']['60-90°']
        gl75 = d['75°']['50-60°'] + d['75°']['60-90°']
        table_text = (
            f"{'Mission':>11s}  45°    75°\n"
            f"{'n':>11s}  {d['45°']['n']:>4d}  {d['75°']['n']:>4d}\n"
            f"{'Impact%':>11s}  "
            f"{100 - d['45°']['No Impact']:>5.1f}"
            f"  {100 - d['75°']['No Impact']:>5.1f}\n"
            f"{'Glancing>=50%':>11s}  {gl45:>5.1f}  {gl75:>5.1f}")
        if cage == 'Rotating Cage':
            fig.text(0.02, 0.01, table_text, fontfamily='monospace',
                     fontsize=7, ha='left', va='bottom',
                     bbox=dict(boxstyle='round,pad=0.2', facecolor='#F0F4FF',
                               alpha=0.85, edgecolor='#CCCCCC'))
        else:
            fig.text(0.98, 0.01, table_text, fontfamily='monospace',
                     fontsize=7, ha='right', va='bottom',
                     bbox=dict(boxstyle='round,pad=0.2', facecolor='#FFF0F0',
                               alpha=0.85, edgecolor='#CCCCCC'))

        # Educational arrows
        ax.annotate('45° Mission', xy=(0, 1.05), xycoords='data',
                    xytext=(0, 0.95), textcoords='axes fraction',
                    ha='center', va='bottom',
                    fontsize=8.5, fontstyle='italic', color='#444444',
                    arrowprops=dict(arrowstyle='->', color='#777777', lw=0.8,
                                   connectionstyle='arc3,rad=-0.25'))
        ax.annotate('75° Mission', xy=(0, 0.70), xycoords='data',
                    xytext=(0, 0.87), textcoords='axes fraction',
                    ha='center', va='bottom',
                    fontsize=8.5, fontstyle='italic', color='#444444',
                    arrowprops=dict(arrowstyle='->', color='#777777', lw=0.8,
                                   connectionstyle='arc3,rad=0.25'))

    # Shared legend
    legend_els = [mpatches.Patch(facecolor=C_NO_IMPACT, label='No Impact')]
    for label in reversed(_SUNBURST_ANGLE_BINS):
        legend_els.append(mpatches.Patch(facecolor=_bin_color(label),
                                          label=label))
    legend_els.append(mpatches.Patch(facecolor=C_ROT, label='Rotating — Outer'))
    legend_els.append(mpatches.Patch(facecolor=C_FIX, label='Fixed — Outer'))
    fig.legend(handles=legend_els, loc='lower center', ncol=5,
               fontsize=9, framealpha=0.9, handlelength=2, handleheight=2,
               bbox_to_anchor=(0.5, -0.15))

    fig.text(0.50, -0.19,
             "Source: experiments_summary.db → flights_summary "
             "(hard-coded aggregated percentages)",
             ha='center', va='bottom', fontsize=7, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    fig.suptitle('Nested Radial (Sunburst): Impact Angle Distribution '
                 'by Cage and Mission', fontsize=13, fontweight='bold', y=0.98)

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_14_sunburst_revised.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  8.  Peak Accel + Rotational Energy Boxplots  (Cell 28)                    ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_peak_accel_rotational_energy(df_impacts, output_path=None,
                                       show_plot=True):
    """1×2 boxplots: (A) imu_peak_accel (g), (B) imu_gyro_energy (rad).

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, imu_peak_accel, imu_gyro_energy.
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=150)
    box_groups = ['Rotating Cage', 'Fixed Cage']
    colors = [C_ROT, C_FIX]

    # Panel A: Peak Acceleration
    bd_acc = [df_impacts[df_impacts['condition'] == n]['imu_peak_accel'].dropna()
              for n in box_groups]
    bp0 = axes[0].boxplot(bd_acc, tick_labels=box_groups, patch_artist=True,
                          widths=0.4)
    axes[0].set_title('Rotating Cage vs Fixed Cage — Peak Acceleration (g)',
                      fontsize=12, fontweight='bold')
    axes[0].set_ylabel('Acceleration (g)', fontsize=10, fontweight='bold')
    axes[0].grid(True, linestyle=':', alpha=0.6)
    for patch, color in zip(bp0['boxes'], colors):
        patch.set_facecolor(color); patch.set_alpha(0.65)
        patch.set_edgecolor(color); patch.set_linewidth(1.5)
    for median in bp0['medians']:
        median.set(color='#333333', linewidth=2.0)

    # Panel B: Integrated Rotational Energy
    bd_en = [df_impacts[df_impacts['condition'] == n]['imu_gyro_energy'].dropna()
             for n in box_groups]
    bp1 = axes[1].boxplot(bd_en, tick_labels=box_groups, patch_artist=True,
                          widths=0.4)
    axes[1].set_title('Rotating Cage vs Fixed Cage — '
                      'Integrated Rotational Energy', fontsize=12,
                      fontweight='bold')
    axes[1].set_ylabel('Integrated Rotational Energy [rad]', fontsize=10,
                       fontweight='bold')
    axes[1].grid(True, linestyle=':', alpha=0.6)
    for patch, color in zip(bp1['boxes'], colors):
        patch.set_facecolor(color); patch.set_alpha(0.65)
        patch.set_edgecolor(color); patch.set_linewidth(1.5)
    for median in bp1['medians']:
        median.set(color='#333333', linewidth=2.0)

    n_rot = len(bd_acc[0]); n_fix = len(bd_acc[1])
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()
    _save_and_show(fig, output_path, show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  9.  IMU Z vs Motor RPM — Plot A  (Cell 31)                                ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_imu_z_vs_motor_rpm(df_impacts, output_path=None, show_plot=True):
    """Side-by-side scatter: estimated RPM vs imu_peak_accel_z.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, motor_max_after, imu_peak_accel_z, impact_angle.
    """
    plot_data = df_impacts.copy()
    plot_data['estimated_rpm'] = 2000 + 10000 * plot_data['motor_max_after']
    plot_data = plot_data.dropna(subset=['estimated_rpm', 'imu_peak_accel_z',
                                          'impact_angle'])
    df_rot = plot_data.query("condition == 'Rotating Cage'")
    df_fix = plot_data.query("condition == 'Fixed Cage'")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=150, sharey=True)
    norm = plt.Normalize(30, 90)
    cmap = plt.cm.coolwarm

    for ax, df_sub, cond, marker, color in [
        (ax1, df_rot, 'Rotating Cage', 's', C_ROT),
        (ax2, df_fix, 'Fixed Cage', 'o', C_FIX),
    ]:
        sc = ax.scatter(df_sub['estimated_rpm'], df_sub['imu_peak_accel_z'],
                        c=df_sub['impact_angle'], cmap=cmap, norm=norm,
                        marker=marker, s=90, alpha=0.85, edgecolor='black',
                        linewidth=0.8, zorder=3)
        mean_val = df_sub['imu_peak_accel_z'].mean()
        ax.axhline(mean_val, color='black', linestyle='--', linewidth=2,
                   label=f'Mean: {mean_val:.2f} g')
        if len(df_sub) > 1:
            z = np.polyfit(df_sub['estimated_rpm'],
                           df_sub['imu_peak_accel_z'], 1)
            xp = np.linspace(df_sub['estimated_rpm'].min(),
                             df_sub['estimated_rpm'].max(), 100)
            ax.plot(xp, np.poly1d(z)(xp), color=color, linestyle='--' if cond == 'Rotating Cage' else '-',
                    linewidth=2, label='Linear Trend')
        ax.set_title(f'{cond}', fontsize=12, fontweight='bold')
        ax.set_xlabel('Commanded Motor Speed (Estimated RPM)', fontsize=10,
                      fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper left', frameon=True, facecolor='white',
                  edgecolor='none')
        ax.text(0.98, 0.02, f"n = {len(df_sub)} flights",
                transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5,
                fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                          edgecolor='none'))

    if df_rot['estimated_rpm'].max() > 0:
        x_min = 8000; x_max = 13000
        ax1.set_xlim(x_min, x_max); ax2.set_xlim(x_min, x_max)
        ax1.set_xticks(np.arange(x_min, x_max + 1, 1000))
        ax2.set_xticks(np.arange(x_min, x_max + 1, 1000))

    ax1.set_ylabel('Peak IMU Acceleration Z (g)', fontsize=10, fontweight='bold')
    fig.suptitle('Rotating Cage vs Fixed Cage — IMU Peak Accel Z vs Motor Speed',
                 fontsize=14, fontweight='bold', y=0.98)
    fig.text(0.05, -0.05,
             "Note: Commanded speed is estimated from actuator output "
             "(2000 + 10000 × control_signal).\n"
             "Colorbar indicates the normal contact angle (degrees).",
             fontsize=9, fontstyle='italic', color='#555555')
    fig.subplots_adjust(left=0.08, right=0.88, bottom=0.15, top=0.85,
                        wspace=0.25)
    cax = fig.add_axes([0.895, 0.15, 0.025, 0.70])
    cbar = fig.colorbar(sc, cax=cax)
    cbar.set_label('Impact Angle (degrees)', fontsize=10, fontweight='bold')

    n_rot = len(df_rot); n_fix = len(df_fix)
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_A_imu_z_vs_rpm.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  10.  Impact Angle vs Max Deviation — Plot B  (Cell 32)                    ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_impact_angle_vs_max_deviation(df_impacts, output_path=None,
                                        show_plot=True):
    """Single-panel scatter: impact_angle vs max_dev_after, colored by battery.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, max_dev_after, battery_at_start, impact_angle.
    """
    plot_data = df_impacts.dropna(subset=['max_dev_after', 'battery_at_start',
                                           'impact_angle'])
    df_rot = plot_data.query("condition == 'Rotating Cage'")
    df_fix = plot_data.query("condition == 'Fixed Cage'")

    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
    norm = plt.Normalize(0, 100)
    cmap = plt.cm.coolwarm

    sc_rot = ax.scatter(df_rot['impact_angle'], df_rot['max_dev_after'],
                        c=df_rot['battery_at_start'], cmap=cmap, norm=norm,
                        marker='s', s=90, alpha=0.85, edgecolor='black',
                        linewidth=0.8, zorder=3, label='Rotating Cage')
    sc_fix = ax.scatter(df_fix['impact_angle'], df_fix['max_dev_after'],
                        c=df_fix['battery_at_start'], cmap=cmap, norm=norm,
                        marker='o', s=90, alpha=0.85, edgecolor='black',
                        linewidth=0.8, zorder=3, label='Fixed Cage')

    # Trendlines: Rotating=dashed, Fixed=solid (§4.0)
    if len(df_rot) > 1:
        m, c = np.polyfit(df_rot['impact_angle'], df_rot['max_dev_after'], 1)
        xr = np.linspace(df_rot['impact_angle'].min(),
                         df_rot['impact_angle'].max(), 100)
        ax.plot(xr, m * xr + c, color='#444444', linestyle='--', linewidth=1.8,
                alpha=0.9, label=f'Rotating Trend: y = {m:.3f}x + {c:.3f}')
    if len(df_fix) > 1:
        m, c = np.polyfit(df_fix['impact_angle'], df_fix['max_dev_after'], 1)
        xf = np.linspace(df_fix['impact_angle'].min(),
                         df_fix['impact_angle'].max(), 100)
        ax.plot(xf, m * xf + c, color='#444444', linestyle='-', linewidth=1.8,
                alpha=0.9, label=f'Fixed Trend: y = {m:.3f}x + {c:.3f}')

    ax.set_title('Rotating Cage vs Fixed Cage — Post-Impact Max Deviation '
                 'vs. Impact Angle', fontsize=12, fontweight='bold', pad=12)
    ax.set_xlabel('Impact Angle (deg)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Maximum Deviation (mm)', fontsize=10, fontweight='bold')
    ax.set_xlim(0, 90); ax.set_ylim(0, 500)
    ax.grid(True, linestyle=':', alpha=0.6, zorder=0)
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.9)

    cbar = fig.colorbar(sc_rot, ax=ax)
    cbar.set_label('Start Battery Percentage (%)', fontsize=10, fontweight='bold')
    cbar.ax.tick_params(labelsize=8.5)

    ax.text(0.02, -0.12,
            "Note: Trendlines computed via ordinary least squares (y = mx + c).",
            transform=ax.transAxes, fontsize=8.5, fontstyle='italic',
            color='#555555')

    n_rot = len(df_rot); n_fix = len(df_fix)
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_14_angle_vs_deviation.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  11.  Recovery Area Distribution — Plot C  (Cell 34)                       ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_recovery_area_distribution(df_impacts, output_path=None,
                                     show_plot=True):
    """Horizontal boxplots: recovery_area by angle bins.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, impact_angle, recovery_area.
    """
    from matplotlib.patches import Patch

    bins = [30, 40, 50, 60, 90]
    labels_bl = ['30-40°', '40-50°', '50-60°', '60-90°']
    df_v = df_impacts.dropna(subset=['impact_angle']).copy()
    df_v['angle_bin'] = pd.cut(df_v['impact_angle'], bins=bins,
                                labels=labels_bl, right=False)
    df_v.loc[df_v['impact_angle'] == 90, 'angle_bin'] = '60-90°'
    df_p = df_v.dropna(subset=['recovery_area', 'angle_bin']).copy()

    box_data_rot = [df_p[(df_p['angle_bin'] == l) &
                          (df_p['condition'] == 'Rotating Cage'
                          )]['recovery_area'].values for l in labels_bl]
    box_data_fix = [df_p[(df_p['angle_bin'] == l) &
                          (df_p['condition'] == 'Fixed Cage'
                          )]['recovery_area'].values for l in labels_bl]

    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    positions = np.arange(len(labels_bl))
    width = 0.35

    bp_rot = ax.boxplot(box_data_rot, positions=positions - width / 2,
                        vert=False, widths=width, patch_artist=True,
                        boxprops=dict(facecolor=C_ROT, color='black'),
                        medianprops=dict(color='black', linewidth=1.5))
    bp_fix = ax.boxplot(box_data_fix, positions=positions + width / 2,
                        vert=False, widths=width, patch_artist=True,
                        boxprops=dict(facecolor=C_FIX, color='black'),
                        medianprops=dict(color='black', linewidth=1.5))

    ax.set_yticks(positions)
    ytl = []
    for i, l in enumerate(labels_bl):
        n_r = len(box_data_rot[i]); n_f = len(box_data_fix[i])
        ytl.append(f"{l}\n(Rot: n={n_r}, Fix: n={n_f})")
    ax.set_yticklabels(ytl)
    ax.set_title('Rotating Cage vs Fixed Cage — Recovery Area Distribution '
                 'by Angle Bins', fontsize=12, fontweight='bold', pad=12)
    ax.set_ylabel('Impact Angle Bins', fontsize=10, fontweight='bold')
    ax.set_xlabel('Recovery Area (mm·m)', fontsize=10, fontweight='bold')
    ax.set_ylim(-0.6, len(labels_bl) - 0.4)
    ax.grid(True, linestyle=':', alpha=0.6, axis='x')
    ax.legend(handles=[
        Patch(facecolor=C_ROT, edgecolor='black', label='Rotating Cage'),
        Patch(facecolor=C_FIX, edgecolor='black', label='Fixed Cage'),
    ], title='Configuration', loc='upper right',
       prop={'family': 'monospace', 'size': 8.5})

    n_rot = int((df_p['condition'] == 'Rotating Cage').sum())
    n_fix = int((df_p['condition'] == 'Fixed Cage').sum())
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_15_recovery_distribution.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  12.  2D Path Overlay — Plot D  (Cell 35)                                  ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_2d_path_overlay(cache_path=None, output_path=None, show_plot=True):
    """Side-by-side trajectory overlay: all flights per cage condition.

    Parameters
    ----------
    cache_path : str or None
        Path to trajectory_cache.pkl. Auto-resolved if None.
    """
    from dev_logs.analysis.kinematics.kin_plot_trajectory import rotate_coords
    from dev_logs.analysis.graphics.graphics_loader import draw_vector_drone

    if cache_path is None:
        cache_path = os.path.join(_THIS_DIR, 'database', 'trajectory_cache.pkl')
    if not os.path.exists(cache_path):
        print(f"[ERROR] Trajectory cache not found at {cache_path}")
        return

    with open(cache_path, 'rb') as f:
        cache_flights = pickle.load(f)

    flights_rot = [f for f in cache_flights
                   if f['condition'] == 'Rotating Cage']
    flights_fix = [f for f in cache_flights
                   if f['condition'] == 'Fixed Cage']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4), sharey=True,
                                    dpi=150)

    column_x, column_y = 0.408, 0.358
    column_radius = 0.045
    rot_col_x, rot_col_y = rotate_coords(column_x, column_y)
    x_lane = 0.248
    wps = np.array([[x_lane, 0.950], [x_lane, -1.200]])
    rot_wps_x, rot_wps_y = rotate_coords(wps[:, 0], wps[:, 1])

    for flight in flights_rot:
        x_new, y_new = rotate_coords(flight['x'], flight['y'])
        ax1.plot(x_new, y_new, color=C_ROT, alpha=0.15, linewidth=0.5, zorder=2)
    for flight in flights_fix:
        x_new, y_new = rotate_coords(flight['x'], flight['y'])
        ax2.plot(x_new, y_new, color=C_FIX, alpha=0.15, linewidth=0.5, zorder=2)

    for ax, flights_list, name, color in [
        (ax1, flights_rot, 'Rotating Cage', C_ROT),
        (ax2, flights_fix, 'Fixed Cage', C_FIX),
    ]:
        ax.add_patch(plt.Circle((rot_col_x, rot_col_y), column_radius,
                                color='#FFCC00', alpha=0.6,
                                label='Column Obstacle', zorder=4))
        ax.scatter(rot_col_x, rot_col_y, color='black', marker='+', zorder=5)
        ax.plot(rot_wps_x, rot_wps_y, color='#444444', linestyle=':',
                linewidth=2, zorder=5, label='Command Path')
        draw_vector_drone(ax, rot_wps_x[0], rot_wps_y[0], 0.179,
                          rotation_deg=0, color_mode='gray')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(-1.6, 1.6); ax.set_ylim(-0.35, 0.85)
        ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
        ax.set_yticks(np.arange(-0.35, 0.86, 0.25))
        ax.set_xlabel('Y coordinate, meters', color='#2CA02C', fontweight='bold')
        ax.grid(True, color='#EAEAEA', linestyle=':', alpha=0.6)
        ax.set_title(f'{name} (n = {len(flights_list)})', fontsize=12,
                     fontweight='bold', color=color)
        ax.legend(loc='upper left', prop={'family': 'monospace', 'size': 8.5})

    ax1.set_ylabel('X coordinate, meters', color='#D62728', fontweight='bold')
    ax1.tick_params(axis='y', colors='#D62728')
    for ax in (ax1, ax2):
        ax.tick_params(axis='x', colors='#2CA02C')

    fig.suptitle('Rotating Cage vs Fixed Cage — 2D Path Overlay & '
                 'Obstacle Clearance', fontsize=14, fontweight='bold', y=0.98)

    n_rot = len(flights_rot); n_fix = len(flights_fix)
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_16_path_heatmap.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  13.  Attitude-Shock Phase Portrait + Vibration  (Cell 36)                 ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_attitude_shock_phase_portrait(df_impacts, output_path=None,
                                        show_plot=True):
    """1×2: (A) Phase portrait, (B) Boxplot imu_std_ay.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, imu_peak_accel, imu_gyro_energy, imu_std_ay.
    """
    colors = [C_ROT, C_FIX]
    fig = plt.figure(figsize=(14, 5.5), dpi=150)

    # Panel A
    ax1 = fig.add_subplot(121)
    for cond, color in zip(['Rotating Cage', 'Fixed Cage'], colors):
        sub = df_impacts[df_impacts['condition'] == cond]
        ax1.scatter(sub['imu_peak_accel'], sub['imu_gyro_energy'],
                    label=cond, color=color, alpha=0.7)
    ax1.set_title('Rotating Cage vs Fixed Cage — Attitude-Shock Phase Portrait',
                  fontsize=11, fontweight='bold')
    ax1.set_xlabel('Peak Acceleration (g)', fontsize=9.5, fontweight='bold')
    ax1.set_ylabel('Rotational Energy', fontsize=9.5, fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6); ax1.legend()

    # Panel B
    ax2 = fig.add_subplot(122)
    bd = [df_impacts[df_impacts['condition'] == n]['imu_std_ay'].dropna()
          for n in ['Rotating Cage', 'Fixed Cage']]
    bp = ax2.boxplot(bd, tick_labels=['Rotating Cage', 'Fixed Cage'],
                     patch_artist=True, widths=0.4)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color); patch.set_alpha(0.65)
        patch.set_edgecolor(color); patch.set_linewidth(1.5)
    for median in bp['medians']:
        median.set(color='#333333', linewidth=2.0)
    ax2.set_title('Rotating Cage vs Fixed Cage — Post-Impact Y-Axis '
                  'Vibration Spread', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Std Dev of ay (g)', fontsize=9.5, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)

    n_rot = int((df_impacts['condition'] == 'Rotating Cage').sum())
    n_fix = int((df_impacts['condition'] == 'Fixed Cage').sum())
    fig.text(0.98, 0.01,
             f"Comparison of {n_rot}× Rotating Cage and {n_fix}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'advanced_thesis_highlights.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  14.  Allocator Saturation Boxplot  (Cell 39)                              ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_allocator_saturation(df_impacts, output_path=None, show_plot=True):
    """1×3 boxplots: saturation duration, unallocated torque, thrust achieved.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, allocator_saturation_duration_sec,
        max_unallocated_torque, thrust_setpoint_achieved_pct.
    """
    df_p = df_impacts.dropna(subset=['allocator_saturation_duration_sec',
                                      'max_unallocated_torque',
                                      'thrust_setpoint_achieved_pct'])
    colors = [C_ROT, C_FIX]
    df_rot = df_p[df_p['condition'] == 'Rotating Cage']
    df_fix = df_p[df_p['condition'] == 'Fixed Cage']

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), dpi=150)

    configs = [
        (0, 'allocator_saturation_duration_sec',
         'Saturation Duration (seconds)',
         'Control Allocator Saturation Duration'),
        (1, 'max_unallocated_torque',
         'Max Unallocated Torque (N·m)',
         'Maximum Unallocated Torque'),
        (2, 'thrust_setpoint_achieved_pct',
         'Thrust Setpoint Achieved (%)',
         'Thrust Setpoint Achieved Percentage'),
    ]
    for idx, col, ylabel, title in configs:
        bd = [df_rot[col].dropna(), df_fix[col].dropna()]
        bp = axes[idx].boxplot(bd, tick_labels=['Rotating Cage', 'Fixed Cage'],
                               patch_artist=True, widths=0.4,
                               medianprops=dict(color='black', linewidth=1.5))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color); patch.set_alpha(0.7)
        axes[idx].set_ylabel(ylabel, fontsize=10, fontweight='bold')
        axes[idx].set_title(title, fontsize=11, fontweight='bold')
        axes[idx].grid(True, linestyle='--', alpha=0.3)

    fig.suptitle('Rotating Cage vs Fixed Cage — Control Allocator Performance',
                 fontsize=13, fontweight='bold', y=0.98)
    fig.text(0.98, 0.01,
             f"Comparison of {len(df_rot)}× Rotating Cage and "
             f"{len(df_fix)}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_17_allocator_saturation_comparison.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  15.  PID Tracking Error Boxplot  (Cell 40)                                ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def plot_pid_tracking_error(df_impacts, output_path=None, show_plot=True):
    """1×3 boxplots: roll/pitch/yaw rate tracking error RMS.

    Parameters
    ----------
    df_impacts : pd.DataFrame
        Needs: condition, roll_rate_error_rms, pitch_rate_error_rms,
        yaw_rate_error_rms.
    """
    colors = [C_ROT, C_FIX]
    df_rot = df_impacts[df_impacts['condition'] == 'Rotating Cage']
    df_fix = df_impacts[df_impacts['condition'] == 'Fixed Cage']

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), dpi=150)

    configs = [
        (0, 'roll_rate_error_rms', 'Roll Rate Error RMS (rad/s)',
         'Roll Angular Rate Tracking Error'),
        (1, 'pitch_rate_error_rms', 'Pitch Rate Error RMS (rad/s)',
         'Pitch Angular Rate Tracking Error'),
        (2, 'yaw_rate_error_rms', 'Yaw Rate Error RMS (rad/s)',
         'Yaw Angular Rate Tracking Error'),
    ]
    for idx, col, ylabel, title in configs:
        bd = [df_rot[col].dropna(), df_fix[col].dropna()]
        bp = axes[idx].boxplot(bd, tick_labels=['Rotating Cage', 'Fixed Cage'],
                               patch_artist=True, widths=0.4,
                               medianprops=dict(color='black', linewidth=1.5))
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color); patch.set_alpha(0.7)
        axes[idx].set_ylabel(ylabel, fontsize=10, fontweight='bold')
        axes[idx].set_title(title, fontsize=11, fontweight='bold')
        axes[idx].grid(True, linestyle='--', alpha=0.3)

    fig.suptitle('Rotating Cage vs Fixed Cage — PID Rate Controller '
                 'Tracking Error', fontsize=13, fontweight='bold', y=0.98)
    fig.text(0.98, 0.01,
             f"Comparison of {len(df_rot)}× Rotating Cage and "
             f"{len(df_fix)}× Fixed Cage flights",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8,
                       edgecolor='none'))
    plt.tight_layout()

    _save_and_show(fig, output_path or os.path.join(_GRAPHICS_DIR,
                    'plot_18_pid_tracking_comparison.png'), show_plot)


# ╔═════════════════════════════════════════════════════════════════════════════╗
# ║  Helper — Save & Display                                                   ║
# ╚═════════════════════════════════════════════════════════════════════════════╝

def _save_and_show(fig, output_path, show_plot):
    """Internal: save figure to disk and optionally show."""
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"[INFO] Saved → {os.path.relpath(output_path, _THIS_DIR)}")
    if show_plot:
        plt.show()
    plt.close(fig)
