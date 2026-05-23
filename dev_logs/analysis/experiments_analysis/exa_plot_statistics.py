import numpy as np
import matplotlib.pyplot as plt

C_CAGE = '#1F77B4'      # Steel Blue (With Cage)
C_NOCAGE = '#FF7F0E'    # Muted Orange (Without Cage)

def plot_angle_boxplots(metrics_cage, metrics_no_cage, label="90° Column Collision"):
    """Generates a premium 2x2 grid of box plots comparing performance metrics
    across all passes for With Cage vs. Without Cage conditions.
    Handles empty lists gracefully if one condition has not been run yet.
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.ravel()

    # Extract metrics list
    clearance_cage = [m['closest_clearance'] * 100.0 for m in metrics_cage]
    clearance_no_cage = [m['closest_clearance'] * 100.0 for m in metrics_no_cage]

    speed_cage = [m['avg_speed_wp2_wp3'] for m in metrics_cage]
    speed_no_cage = [m['avg_speed_wp2_wp3'] for m in metrics_no_cage]

    mean_err_cage = [m['mean_tracking_error'] * 100.0 for m in metrics_cage]
    mean_err_no_cage = [m['mean_tracking_error'] * 100.0 for m in metrics_no_cage]

    max_err_cage = [m['max_tracking_error'] * 100.0 for m in metrics_cage]
    max_err_no_cage = [m['max_tracking_error'] * 100.0 for m in metrics_no_cage]

    datasets = [
        (clearance_cage, clearance_no_cage, "Minimum Surface Clearance (cm)", "Clearance (cm)"),
        (speed_cage, speed_no_cage, "Sweep Transit Speed (m/s)", "Speed (m/s)"),
        (mean_err_cage, mean_err_no_cage, "Mean Path Tracking Error (cm)", "Error (cm)"),
        (max_err_cage, max_err_no_cage, "Maximum Lateral Displacement (cm)", "Displacement (cm)")
    ]

    for idx, (data_cage, data_nocage, title, ylabel) in enumerate(datasets):
        ax = axes[idx]
        
        # Prepare boxplot data structure
        plot_data = []
        labels = []
        colors = []
        
        if data_cage:
            plot_data.append(data_cage)
            labels.append("With Rotating Cage")
            colors.append(C_CAGE)
        if data_nocage:
            plot_data.append(data_nocage)
            labels.append("Without Cage")
            colors.append(C_NOCAGE)
            
        if not plot_data:
            ax.text(0.5, 0.5, "No Data Available", ha='center', va='center', fontsize=12, color='gray')
            ax.set_title(title, pad=10)
            continue

        bp = ax.boxplot(plot_data, patch_artist=True, widths=0.4)
        
        # Style the boxes professionally
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)
            patch.set_edgecolor('black')
            patch.set_linewidth(1.2)
            
        for whisker in bp['whiskers']:
            whisker.set(color='black', linewidth=1.2, linestyle='-')
            
        for cap in bp['caps']:
            cap.set(color='black', linewidth=1.2)
            
        for median in bp['medians']:
            median.set(color='black', linewidth=1.8, linestyle='-')
            
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

    # Sort results by angle
    sorted_results = sorted(results_list, key=lambda r: r['angle_deg'])
    angles = [r['angle_deg'] for r in sorted_results]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.ravel()

    metrics_keys = [
        ('closest_clearance', 100.0, "Minimum Surface Clearance (cm)", "Clearance (cm)"),
        ('avg_speed_wp2_wp3', 1.0, "Sweep Transit Speed (m/s)", "Speed (m/s)"),
        ('mean_tracking_error', 100.0, "Mean Path Tracking Error (cm)", "Error (cm)"),
        ('max_tracking_error', 100.0, "Maximum Lateral Displacement (cm)", "Displacement (cm)")
    ]

    for idx, (key, scale, title, ylabel) in enumerate(metrics_keys):
        ax = axes[idx]
        
        # Calculate stats across angles for With Cage
        cage_means, cage_sems = [], []
        nocage_means, nocage_sems = [], []
        
        has_cage_any = False
        has_nocage_any = False

        for r in sorted_results:
            vals_cage = [m[key] * scale for m in r.get('metrics_cage', [])]
            vals_nocage = [m[key] * scale for m in r.get('metrics_no_cage', [])]
            
            if vals_cage:
                cage_means.append(np.mean(vals_cage))
                cage_sems.append(np.std(vals_cage) / np.sqrt(len(vals_cage)) if len(vals_cage) > 1 else 0.0)
                has_cage_any = True
            else:
                cage_means.append(np.nan)
                cage_sems.append(0.0)
                
            if vals_nocage:
                nocage_means.append(np.mean(vals_nocage))
                nocage_sems.append(np.std(vals_nocage) / np.sqrt(len(vals_nocage)) if len(vals_nocage) > 1 else 0.0)
                has_nocage_any = True
            else:
                nocage_means.append(np.nan)
                nocage_sems.append(0.0)

        # Plot With Cage
        if has_cage_any:
            ax.errorbar(angles, cage_means, yerr=cage_sems, fmt='-o', color=C_CAGE,
                        linewidth=2.5, elinewidth=1.8, capsize=4, label="With Rotating Cage")
        # Plot Without Cage
        if has_nocage_any:
            ax.errorbar(angles, nocage_means, yerr=nocage_sems, fmt='-s', color=C_NOCAGE,
                        linewidth=2.5, elinewidth=1.8, capsize=4, label="Without Cage")

        ax.set_title(title, pad=12, fontsize=12, fontweight='bold')
        ax.set_xlabel("Collision Sweep Angle (degrees)", fontsize=10)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xticks(angles)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(loc='best', frameon=True, facecolor='white', edgecolor='#EAEAEA')

    plt.suptitle("📐 Thesis Cross-Angle Collision Metrics Progression Summary", fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
