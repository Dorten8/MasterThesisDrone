import os
import sys
import numpy as np
import matplotlib.pyplot as plt

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)

def rotate_coords(x_val, y_val):
    """Rotate coordinate system by 90 degrees CCW (x_new = -y, y_new = x)"""
    return -y_val, x_val

def plot_trajectory(df_mocap, wp_events, column_x=0.500, column_y=0.000, 
                    cage_diameter=0.358, column_diameter=0.09, output_path=None):
    """Plots 2D top-down spatial trajectory, showing the flight sweep, waypoints,
    safety cage, minimum clearance vector, and the column.
    """
    if df_mocap.empty:
        print("[WARN] MoCap DataFrame is empty, skipping trajectory plot.")
        return None

    # Load vector graphics loader dynamically
    current_dir = os.path.dirname(os.path.abspath(__file__))
    analysis_dir = os.path.abspath(os.path.join(current_dir, ".."))
    
    if 'graphics' in sys.modules:
        graphics_module = sys.modules['graphics']
        if not getattr(graphics_module, '__file__', '').startswith(analysis_dir):
            del sys.modules['graphics']

    if analysis_dir not in sys.path:
        sys.path.insert(0, analysis_dir)
        
    try:
        from graphics import draw_vector_drone
    except ImportError as e:
        print(f"[WARN] Failed to import draw_vector_drone from graphics package: {e}")
        draw_vector_drone = None

    cage_radius = cage_diameter / 2.0
    column_radius = column_diameter / 2.0

    # Focus solely on the active first-pass sweep (WP1 to WP4)
    t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
    t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
    df_mocap_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]

    if df_mocap_sweep.empty:
        df_mocap_sweep = df_mocap

    # Query exact coordinates where the drone was when it registered WP2 and WP3
    wp2_t = wp_events.get('WP2')
    if wp2_t is not None:
        idx_wp2 = np.searchsorted(df_mocap['t'], wp2_t)
        idx_wp2 = min(max(0, idx_wp2), len(df_mocap) - 1)
        wp2_x, wp2_y = df_mocap['x'].iloc[idx_wp2], df_mocap['y'].iloc[idx_wp2]
    else:
        wp2_x, wp2_y = 0.100, 1.200

    wp3_t = wp_events.get('WP3')
    if wp3_t is not None:
        idx_wp3 = np.searchsorted(df_mocap['t'], wp3_t)
        idx_wp3 = min(max(0, idx_wp3), len(df_mocap) - 1)
        wp3_x, wp3_y = df_mocap['x'].iloc[idx_wp3], df_mocap['y'].iloc[idx_wp3]
    else:
        wp3_x, wp3_y = 0.100, -1.200

    # Calculate minimum spatial distance to column center during active sweep
    dist_to_col_center = np.sqrt((df_mocap_sweep['x'] - column_x)**2 + (df_mocap_sweep['y'] - column_y)**2)
    dist_to_col_surface = dist_to_col_center - column_radius
    min_distance_idx = np.argmin(dist_to_col_surface)

    closest_t = df_mocap_sweep['t'].iloc[min_distance_idx]
    closest_x = df_mocap_sweep['x'].iloc[min_distance_idx]
    closest_y = df_mocap_sweep['y'].iloc[min_distance_idx]
    closest_clearance = dist_to_col_surface.iloc[min_distance_idx] - cage_radius

    fig, ax = plt.subplots(figsize=(10, 10))

    rot_col_x, rot_col_y = rotate_coords(column_x, column_y)
    rot_traj_x, rot_traj_y = rotate_coords(df_mocap_sweep['x'], df_mocap_sweep['y'])
    rot_closest_x, rot_closest_y = rotate_coords(closest_x, closest_y)
    rot_wp2_x, rot_wp2_y = rotate_coords(wp2_x, wp2_y)
    rot_wp3_x, rot_wp3_y = rotate_coords(wp3_x, wp3_y)

    # 1. Plot the Central Column Obstacle (to scale, rotated)
    column_circle_outer = plt.Circle((rot_col_x, rot_col_y), column_radius, color='#FFCC00', alpha=0.3, 
                                     label=f'Column Obstacle (D={column_diameter*100:.1f}cm)')
    column_circle_edge = plt.Circle((rot_col_x, rot_col_y), column_radius, fill=False, color='#CC9900', linewidth=2.0)
    ax.add_patch(column_circle_outer)
    ax.add_patch(column_circle_edge)
    ax.scatter(rot_col_x, rot_col_y, color='black', marker='+', zorder=5)

    # 2. Plot Drone Actual Flight Path (from OptiTrack active sweep, rotated)
    ax.plot(rot_traj_x, rot_traj_y, color=C_MOCAP, alpha=0.8, linewidth=2.5, label='Actual Path (MoCap ENU)')

    # 3. Draw waypoints from column sweep definition (rotated)
    wps = np.array([
        [0.0, 1.20],
        [0.10, 1.20],
        [0.10, -1.20],
        [0.0, -1.20]
    ])
    rot_wps_x, rot_wps_y = rotate_coords(wps[:, 0], wps[:, 1])
    ax.scatter(rot_wps_x, rot_wps_y, color=C_CMD, s=120, facecolors='none', edgecolors=C_CMD, linewidth=2, zorder=6, label='Command Waypoints')
    for i, wp in enumerate(wps, 1):
        r_wp_x, r_wp_y = rotate_coords(wp[0], wp[1])
        ax.annotate(f"WP{i}", (r_wp_x+0.05, r_wp_y+0.02), fontweight='bold', color=C_CMD)

    # Connect waypoints with dotted red line and arrows (regularized along segments)
    for i in range(len(wps) - 1):
        x1, y1 = rot_wps_x[i], rot_wps_y[i]
        x2, y2 = rot_wps_x[i+1], rot_wps_y[i+1]
        ax.plot([x1, x2], [y1, y2], color=C_CMD, linestyle=':', linewidth=2, zorder=5)
        
        # Define arrow placement (ratios) along segments
        if i == 1: # The long segment between WP2 and WP3
            ratios = [0.33, 0.67]
        else:
            ratios = [0.5]
            
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx**2 + dy**2)
        if length > 0:
            ux = dx / length
            uy = dy / length
            for r in ratios:
                arrow_x = x1 + r * dx
                arrow_y = y1 + r * dy
                ax.annotate('', xy=(arrow_x + ux * 0.05, arrow_y + uy * 0.05), xytext=(arrow_x - ux * 0.05, arrow_y - ux * 0.05),
                             arrowprops=dict(arrowstyle="->", color=C_CMD, lw=2, shrinkA=0, shrinkB=0))

    # 4. Plot drone safety cage grayed out at WP2 and WP3 positions natively from drone_top.svg
    if draw_vector_drone:
        draw_vector_drone(ax, rot_wp2_x, rot_wp2_y, cage_radius, rotation_deg=0, color_mode='gray')
        draw_vector_drone(ax, rot_wp3_x, rot_wp3_y, cage_radius, rotation_deg=0, color_mode='gray')

    # 5. Plot drone safety cage boundary & schematic at closest approach (in color!, rotated)
    ax.plot([], [], color='#1F77B4', linestyle='--', label=f'Safety Cage (D={cage_diameter*100:.1f}cm)')
    if draw_vector_drone:
        draw_vector_drone(ax, rot_closest_x, rot_closest_y, cage_radius, rotation_deg=0, color_mode='full')
    else:
        # Fallback circle if draw_vector_drone import failed
        schematic_circle = plt.Circle((rot_closest_x, rot_closest_y), cage_radius, fill=False, color='#1F77B4', linestyle='--', linewidth=2.0)
        ax.add_patch(schematic_circle)

    # Draw line showing minimum separation vector (rotated)
    ax.plot([rot_closest_x, rot_col_x], [rot_closest_y, rot_col_y], color='purple', linestyle=':', linewidth=2, label='Separation Vector')

    ax.set_title('📐 Top-Down 2D Spatial Trajectory & Column Clearance Alignment (Rotated 90° CCW)')
    ax.set_xlabel('Rotated Lab -Y Coordinate (ENU - South/North, meters)')
    ax.set_ylabel('Rotated Lab X Coordinate (ENU - East/West, meters)')

    # Strict Equal Aspect Ratio and boundary truncation
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-0.5, 1.0)

    # Normalize grid to exact squares of 0.5m side length
    ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
    ax.set_yticks(np.arange(-0.5, 1.1, 0.5))

    ax.legend(loc='upper left')
    ax.grid(True)
    plt.tight_layout()
    
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] Trajectory plot saved successfully to: {output_path}")

    plt.show()

    print(f"📐 Physical Separation Metrics (SSoT Configured):")
    print(f"   - Time of closest approach:                     t = {closest_t:.2f}s")
    print(f"   - Minimum spatial distance to column center:   {dist_to_col_center.min():.3f} m")
    print(f"   - Minimum physical cage clearance to surface:  {closest_clearance*100:.1f} cm (Cage diameter = {cage_diameter:.3f}m, Column diameter = {column_diameter:.3f}m)")
    
    return {
        'closest_t': closest_t,
        'min_dist_center': dist_to_col_center.min(),
        'closest_clearance': closest_clearance
    }
