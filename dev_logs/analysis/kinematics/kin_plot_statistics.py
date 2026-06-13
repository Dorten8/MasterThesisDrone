import numpy as np
import matplotlib.pyplot as plt

C_ROTATING = '#1F77B4'  # Steel Blue (Rotating Cage)
C_FIXED = '#FF7F0E'     # Muted Orange (Fixed Cage)

def plot_angle_boxplots(metrics_rotating, metrics_fixed, label="90° Column Collision"):
    """Generates a premium 2x2 grid of box plots comparing performance metrics
    across all passes for Rotating Cage vs. Fixed Cage conditions.
    Handles empty lists gracefully if one condition has not been run yet.
    """
    # Create a 2x2 grid of subplots and flatten into a 1D array for easy indexing
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.ravel()

    # ---- Extract the four performance metrics from the results dictionaries ----
    # Metric 1: Minimum Surface Clearance (cm)
    #   The smallest distance ever recorded between the drone and any surface during the sweep,
    #   converted from meters to centimeters by multiplying by 100. A higher value means the
    #   drone maintained greater safety margins.
    # Metric 2: Sweep Transit Speed (m/s)
    #   The average translational speed of the drone between waypoints WP2 and WP3, i.e. the
    #   segment where the drone passes through the column gap. Kept in m/s (native units).
    # Metric 3: Mean Path Tracking Error (cm)
    #   The average deviation between the drone's actual position and its commanded trajectory
    #   across the entire sweep. Converted from meters to cm. Lower values indicate the drone
    #   followed the planned path more faithfully.
    # Metric 4: Maximum Lateral Displacement (cm)
    #   The largest single-frame lateral (sideways) deviation from the commanded path. This
    #   captures the worst-case excursion and is the most safety-critical metric because a
    #   single large deviation can cause a collision even if the mean error is small.
    clearance_rotating = [m['closest_clearance'] * 100.0 for m in metrics_rotating]
    clearance_fixed = [m['closest_clearance'] * 100.0 for m in metrics_fixed]

    speed_rotating = [m['avg_speed_wp2_wp3'] for m in metrics_rotating]
    speed_fixed = [m['avg_speed_wp2_wp3'] for m in metrics_fixed]

    mean_err_rotating = [m['mean_tracking_error'] * 100.0 for m in metrics_rotating]
    mean_err_fixed = [m['mean_tracking_error'] * 100.0 for m in metrics_fixed]

    max_err_rotating = [m['max_tracking_error'] * 100.0 for m in metrics_rotating]
    max_err_fixed = [m['max_tracking_error'] * 100.0 for m in metrics_fixed]

    # Bundle each metric's rotating and fixed data together with its display title and axis label.
    # Each tuple is: (rotating_data_list, fixed_data_list, subplot_title, y_axis_label).
    datasets = [
        (clearance_rotating, clearance_fixed, "Minimum Surface Clearance (cm)", "Clearance (cm)"),
        (speed_rotating, speed_fixed, "Sweep Transit Speed (m/s)", "Speed (m/s)"),
        (mean_err_rotating, mean_err_fixed, "Mean Path Tracking Error (cm)", "Error (cm)"),
        (max_err_rotating, max_err_fixed, "Maximum Lateral Displacement (cm)", "Displacement (cm)")
    ]

    for idx, (data_rotating, data_fixed, title, ylabel) in enumerate(datasets):
        ax = axes[idx]
        
        # Prepare boxplot data structure
        plot_data = []
        labels = []
        colors = []
        
        if data_rotating:
            plot_data.append(data_rotating)
            labels.append("Rotating Cage")
            colors.append(C_ROTATING)
        if data_fixed:
            plot_data.append(data_fixed)
            labels.append("Fixed Cage")
            colors.append(C_FIXED)
            
        if not plot_data:
            ax.text(0.5, 0.5, "No Data Available", ha='center', va='center', fontsize=12, color='gray')
            ax.set_title(title, pad=10)
            continue

        # ---- Boxplot construction ----
        # `patch_artist=True` turns each box from a bare wireframe into a filled,
        # coloured rectangle patch, enabling the per-condition colour coding below.
        # `widths=0.4` keeps the boxes narrow so the two conditions sit clearly apart.
        bp = ax.boxplot(plot_data, patch_artist=True, widths=0.4)

        # ---- Style each box: colour, transparency, and outline ----
        # Each box (IQR rectangle) is filled with the condition's brand colour at
        # alpha=0.6 so that the gridlines remain faintly visible through the box,
        # adding depth without sacrificing readability. A solid black edge (1.2 pt)
        # ensures the box boundary is crisp even when the fill is translucent.
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)
            patch.set_edgecolor('black')
            patch.set_linewidth(1.2)

        # ---- Whiskers: the vertical lines extending from the box to the data range ----
        # By default whiskers reach the farthest datum within 1.5 x IQR. Painted in
        # solid black with a continuous line for a clean, publication-ready look.
        for whisker in bp['whiskers']:
            whisker.set(color='black', linewidth=1.2, linestyle='-')

        # ---- Caps: the short horizontal bars at the whisker tips ----
        # These anchor the whisker endpoints visually and match the whisker colour.
        for cap in bp['caps']:
            cap.set(color='black', linewidth=1.2)

        # ---- Median line: the bold horizontal bar inside each box ----
        # Rendered thicker (1.8 pt) than the whiskers so the central tendency stands
        # out immediately when scanning the figure.
        for median in bp['medians']:
            median.set(color='black', linewidth=1.8, linestyle='-')

        # ---- Outlier fliers: individual points beyond the whisker range ----
        # Plotted as small semi-transparent grey circles (marker='o', alpha=0.5) so
        # they are visible but do not distract from the main box structure.
        for flier in bp['fliers']:
            flier.set(marker='o', markerfacecolor='gray', alpha=0.5, markersize=5)

        ax.set_title(title, pad=12, fontsize=12, fontweight='bold')
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xticklabels(labels, fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.5)

    plt.suptitle(f"📊 Statistical Aggregate Performance Analysis\n{label}", fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def compare_all_angles(results_list):
    """Generates a premium cross-angle comparison chart displaying how
    performance metrics evolve as a function of the column collision angle.
    Plot shows trend lines with standard error bars for both conditions.
    """
    if not results_list:
        print("[WARN] No results provided for cross-angle comparison.")
        return None

    # ---- Sort results by collision angle so the x-axis progresses monotonically ----
    # This ensures the line plots connect data points in a meaningful left-to-right
    # order (e.g., 15°, 30°, 45°, 60°, 75°, 90°) rather than whatever order the
    # results happened to arrive in.
    sorted_results = sorted(results_list, key=lambda r: r['angle_deg'])
    angles = [r['angle_deg'] for r in sorted_results]

    # ---- Create a 2x2 panel grid, one subplot per metric ----
    # All four subplots share the same x-axis domain (collision sweep angle), which
    # makes it easy to compare how each metric evolves with angle by scanning across
    # the grid horizontally.
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.ravel()

    # Each tuple defines: (dictionary_key, scale_factor, subplot_title, y_axis_label).
    # The scale factor converts from the native metre unit to centimetre where needed.
    metrics_keys = [
        ('closest_clearance', 100.0, "Minimum Surface Clearance (cm)", "Clearance (cm)"),
        ('avg_speed_wp2_wp3', 1.0, "Sweep Transit Speed (m/s)", "Speed (m/s)"),
        ('mean_tracking_error', 100.0, "Mean Path Tracking Error (cm)", "Error (cm)"),
        ('max_tracking_error', 100.0, "Maximum Lateral Displacement (cm)", "Displacement (cm)")
    ]

    for idx, (key, scale, title, ylabel) in enumerate(metrics_keys):
        ax = axes[idx]
        
        # Calculate stats across angles for both Rotating and Fixed conditions
        rotating_means, rotating_sems = [], []
        fixed_means, fixed_sems = [], []
        
        has_rotating_any = False
        has_fixed_any = False

        for r in sorted_results:
            vals_rotating = [m[key] * scale for m in r.get('metrics_rotating_cage', [])]
            vals_fixed = [m[key] * scale for m in r.get('metrics_fixed_cage', [])]
            
            # ---- Per-angle aggregate statistics ----
            # For each collision angle we have N passes. We compute the mean of the
            # metric across those N passes and the Standard Error of the Mean (SEM),
            # which equals std / sqrt(N). SEM is the appropriate error-bar measure
            # when the statistic of interest is the mean itself (rather than the spread
            # of individual data points), because it quantifies how precisely we have
            # estimated the population mean from this finite sample.
            # When N=1, SEM is set to 0.0 because we cannot estimate variability from
            # a single observation.
            if vals_rotating:
                rotating_means.append(np.mean(vals_rotating))
                rotating_sems.append(np.std(vals_rotating) / np.sqrt(len(vals_rotating)) if len(vals_rotating) > 1 else 0.0)
                has_rotating_any = True
            else:
                rotating_means.append(np.nan)
                rotating_sems.append(0.0)

            if vals_fixed:
                fixed_means.append(np.mean(vals_fixed))
                fixed_sems.append(np.std(vals_fixed) / np.sqrt(len(vals_fixed)) if len(vals_fixed) > 1 else 0.0)
                has_fixed_any = True
            else:
                fixed_means.append(np.nan)
                fixed_sems.append(0.0)

        # ---- Plot trend lines with SEM error bars ----
        # Rotating Cage uses circle markers ('-o'); Fixed Cage uses square markers
        # ('-s'). Both are drawn on the same subplot so the two cage conditions can
        # be compared directly at each angle. The error bars (yerr=sem) show +/- 1 SEM
        # around each mean, giving a visual indication of within-angle variability.
        # Plot Rotating Cage
        if has_rotating_any:
            ax.errorbar(angles, rotating_means, yerr=rotating_sems, fmt='-o', color=C_ROTATING,
                        linewidth=2.5, elinewidth=1.8, capsize=4, label="Rotating Cage")
        # Plot Fixed Cage
        if has_fixed_any:
            ax.errorbar(angles, fixed_means, yerr=fixed_sems, fmt='-s', color=C_FIXED,
                        linewidth=2.5, elinewidth=1.8, capsize=4, label="Fixed Cage")

        ax.set_title(title, pad=12, fontsize=12, fontweight='bold')
        ax.set_xlabel("Collision Sweep Angle (degrees)", fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xticks(angles)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(loc='best', frameon=True, facecolor='white', edgecolor='#EAEAEA')

    plt.suptitle("Thesis Cross-Angle Collision Metrics Progression Summary", fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
