#!/usr/bin/env python3
"""Aggregated IMU Collision Dynamics — side-by-side Rotating vs Fixed Cage.

Time-aligned overlay of all flights' IMU traces (accel deviation + gyro magnitude)
with median profiles, interpolated to a common time grid relative to Column Impact.

Layout (§4.0): 1×2 side-by-side — Rotating Cage left, Fixed Cage right.
Timeline: [−0.5 s, +1.0 s] from Column Impact.
Axis split: Accel Y-labels on left panel only; Gyro Y-labels on right panel only.
Legend on right panel, upper right.  Per-column source labels at figure bottom.
"""
import os
import sys
import pickle
import numpy as np
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

    # Timeline: −0.5 s before impact to +1.0 s after impact
    t_common = np.linspace(-0.5, 1.0, 300)

    a_dev_rot, g_mag_rot = [], []
    a_dev_fix, g_mag_fix = [], []

    for trace in cached_data:
        t = trace['t_rel']
        if len(t) < 5:
            continue

        # Deduplicate time axis (SCIPY interp1d requires strictly increasing x)
        t_unique, idx = np.unique(t, return_index=True)
        if len(t_unique) < 5:
            continue

        # Interpolate each trace onto the common time grid
        f_a = interp1d(t_unique, trace['a_deviation'][idx],
                       bounds_error=False, fill_value=np.nan)
        f_g = interp1d(t_unique, trace['g_mag'][idx],
                       bounds_error=False, fill_value=np.nan)

        a_interp = f_a(t_common)
        g_interp = f_g(t_common)

        if trace['condition'] == 'Rotating Cage':
            a_dev_rot.append(a_interp)
            g_mag_rot.append(g_interp)
        else:
            a_dev_fix.append(a_interp)
            g_mag_fix.append(g_interp)

    n_rot = len(a_dev_rot)
    n_fix = len(a_dev_fix)
    print(f"Rotating: {n_rot} traces, Fixed: {n_fix} traces")

    # Compute shared gyro y-limit so both panels have identical scales (§4.2)
    all_g = np.concatenate([np.array(g_mag_rot).ravel(), np.array(g_mag_fix).ravel()])
    global_max_g = np.nanmax(all_g) if len(all_g) > 0 else 10.0
    gyro_ylim = max(10.0, global_max_g * 1.1)

    # ---- Side-by-side layout: Rotating left, Fixed right (§4.0) ----
    fig, (ax_rot, ax_fix) = plt.subplots(1, 2, figsize=(14, 7), sharey=True, dpi=150)

    def _draw_panel(ax, a_list, g_list, title, show_accel_labels, show_gyro_labels,
                    show_legend):
        """Draw one panel: individual thin traces + median profiles.

        Parameters
        ----------
        show_accel_labels : bool
            If True, show accel Y-axis labels on the left spine.
        show_gyro_labels : bool
            If True, show gyro Y-axis labels on the right spine.
        show_legend : bool
            If True, draw combined accel + gyro + threshold legend.
        """
        a_arr = np.nan_to_num(np.array(a_list))
        g_arr = np.nan_to_num(np.array(g_list))

        ax2 = ax.twinx()

        # Individual thin traces — faint enough to see the median through them,
        # visible enough to assess trace density and spread
        for a_tr in a_arr:
            ax.plot(t_common, a_tr, color='#D62728', alpha=0.06, linewidth=0.8)
        for g_tr in g_arr:
            ax2.plot(t_common, g_tr, color='#1F77B4', alpha=0.06, linewidth=0.8)

        # Median profiles
        a_median = np.median(a_arr, axis=0)
        g_median = np.median(g_arr, axis=0)

        l1 = ax.plot(t_common, a_median, color='#D62728', linewidth=2.5,
                     label='Median Linear Accel Deviation (m/s²)')
        l2 = ax2.plot(t_common, g_median, color='#1F77B4', linestyle='-.',
                      linewidth=2.5, label='Median Gyro Rotational Surge (rad/s)')

        # Severe impact threshold
        l3 = ax.axhline(5.0, color='orange', linestyle='--', linewidth=1.5,
                        label='Severe Impact Threshold (5.0 m/s²)')

        ax.set_title(title, fontsize=12, fontweight='bold')

        # ---- Accel (left) Y-axis ----
        if show_accel_labels:
            ax.set_ylabel('Accel Deviation from Gravity (m/s²)', color='#D62728')
            ax.tick_params(axis='y', labelcolor='#D62728')
        else:
            ax.set_ylabel('')
            ax.tick_params(axis='y', labelleft=False)
        ax.set_ylim(-1.0, 20.0)
        ax.yaxis.set_major_locator(ticker.MultipleLocator(2.0))

        # ---- Gyro (right) Y-axis ----
        if show_gyro_labels:
            ax2.set_ylabel('Gyro Magnitude (rad/s)', color='#1F77B4')
            ax2.tick_params(axis='y', labelcolor='#1F77B4')
        else:
            ax2.set_ylabel('')
            ax2.tick_params(axis='y', labelright=False)
        ax2.set_ylim(0, gyro_ylim)
        ax2.yaxis.set_major_locator(ticker.MultipleLocator(1.0))

        ax.grid(True, linestyle=':', alpha=0.6)

        # Column Impact reference line at t = 0
        ax.axvline(0.0, color='#D32F2F', linestyle='--', linewidth=1.2, zorder=10)
        ax.text(0.02, 0.94, 'Column Impact', color='#D32F2F', fontweight='bold',
                ha='left', transform=ax.transAxes, fontsize=9)

        # X-axis
        ax.set_xlim(-0.5, 1.0)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))

        # Legend — only on left panel
        if show_legend:
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right',
                      frameon=True, facecolor='white', edgecolor='#EAEAEA', fontsize=8)

    # Left panel: Rotating Cage — accel Y-labels, no legend (§4.0)
    _draw_panel(ax_rot, a_dev_rot, g_mag_rot, "Rotating Cage",
                show_accel_labels=True, show_gyro_labels=False, show_legend=False)

    # Right panel: Fixed Cage — gyro Y-labels, legend at upper right (§4.0)
    _draw_panel(ax_fix, a_dev_fix, g_mag_fix, "Fixed Cage",
                show_accel_labels=False, show_gyro_labels=True, show_legend=True)

    # Shared x-axis label
    fig.text(0.5, 0.015, 'Time relative to Impact (seconds)',
             ha='center', fontweight='bold', fontsize=10)

    # Suptitle spanning both panels
    fig.suptitle('Time-Normalized IMU Dynamics Overview\n'
                 '(Overlaid traces with Median profiles)',
                 fontsize=14, y=0.98, fontweight='bold')

    # Per-column source labels (§4.1A convention)
    fig.text(0.02, 0.015,
             f"imu_cache (N={n_rot} Rotating)",
             ha='left', va='bottom', fontsize=7.5, color='#555555',
             transform=fig.transFigure)
    fig.text(0.98, 0.015,
             f"imu_cache (N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555',
             transform=fig.transFigure)

    plt.tight_layout(rect=[0, 0.035, 1, 0.96])

    # ── Dual output: graphics/ (dev) + thesis/plots/ (LaTeX-ready) ────────
    out_path = os.path.join(project_root, "dev_logs", "analysis", "graphics",
                            "plot_19_aggregated_imu_dynamics.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"Aggregated IMU plot saved to {os.path.relpath(out_path, project_root)}")

    thesis_path = os.path.join(project_root, "thesis", "plots",
                               "Aggregated IMU Collision Dynamics Across All Flights.png")
    os.makedirs(os.path.dirname(thesis_path), exist_ok=True)
    fig.savefig(thesis_path, dpi=300, bbox_inches='tight')
    print(f"Aggregated IMU plot saved to {os.path.relpath(thesis_path, project_root)}")
    plt.show()


if __name__ == "__main__":
    generate_aggregated_imu_plot()
