import numpy as np
import matplotlib.pyplot as plt

C_ROTATING = '#1F77B4'  # Steel Blue (Rotating Cage)
C_FIXED = '#FF7F0E'     # Muted Orange (Fixed Cage)

def plot_angle_boxplots(metrics_rotating, metrics_fixed, label="90° Column Collision"):
    """Generates a premium 2x2 grid of box plots comparing performance metrics
    across all passes for Rotating Cage vs. Fixed Cage conditions.
    Handles empty lists gracefully if one condition has not been run yet.
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.ravel()

    # Extract metrics lists
    clearance_rotating = [m['closest_clearance'] * 100.0 for m in metrics_rotating]
    clearance_fixed = [m['closest_clearance'] * 100.0 for m in metrics_fixed]

    speed_rotating = [m['avg_speed_wp2_wp3'] for m in metrics_rotating]
    speed_fixed = [m['avg_speed_wp2_wp3'] for m in metrics_fixed]

    mean_err_rotating = [m['mean_tracking_error'] * 100.0 for m in metrics_rotating]
    mean_err_fixed = [m['mean_tracking_error'] * 100.0 for m in metrics_fixed]

    max_err_rotating = [m['max_tracking_error'] * 100.0 for m in metrics_rotating]
    max_err_fixed = [m['max_tracking_error'] * 100.0 for m in metrics_fixed]

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
        
        # Calculate stats across angles for both Rotating and Fixed conditions
        rotating_means, rotating_sems = [], []
        fixed_means, fixed_sems = [], []
        
        has_rotating_any = False
        has_fixed_any = False

        for r in sorted_results:
            vals_rotating = [m[key] * scale for m in r.get('metrics_rotating_cage', [])]
            vals_fixed = [m[key] * scale for m in r.get('metrics_fixed_cage', [])]
            
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

    plt.suptitle("📐 Thesis Cross-Angle Collision Metrics Progression Summary", fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
