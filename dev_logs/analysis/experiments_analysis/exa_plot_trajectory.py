import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)

def rotate_coords(x_val, y_val):
    """Rotate coordinate system by 90 degrees CCW (x_new = -y, y_new = x)"""
    return -y_val, x_val

def plot_trajectory(df_mocap, wp_events, column_x=0.408, column_y=0.358, 
                    cage_diameter=0.358, column_diameter=0.09, output_path=None, flight_name=None):
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
                ax.annotate('', xy=(arrow_x + ux * 0.05, arrow_y + uy * 0.05), xytext=(arrow_x - ux * 0.05, arrow_y - uy * 0.05),
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

    # Calculate achieved 2D spatial trajectory angle at exact contact point
    achieved_angle = None
    impact_t = wp_events.get('Column Impact')
    if impact_t is not None and not df_mocap.empty:
        idx_impact = (df_mocap['t'] - impact_t).abs().idxmin()
        impact_row = df_mocap.iloc[idx_impact]
        rx = column_x - impact_row['x']
        ry = column_y - impact_row['y']
        r_len = np.sqrt(rx**2 + ry**2)
        vx = impact_row.get('vx', 0.0)
        vy = impact_row.get('vy', -1.0)
        v_len = np.sqrt(vx**2 + vy**2)
        
        if r_len > 1e-3 and v_len > 1e-3:
            cos_theta = (rx * vx + ry * vy) / (r_len * v_len)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            achieved_angle = np.degrees(np.arccos(cos_theta))
            if achieved_angle > 90.0:
                achieved_angle = 180.0 - achieved_angle

    # Draw achieved angle dimension and straight velocity quiver at contact point
    if achieved_angle is not None:
        idx_impact = (df_mocap['t'] - impact_t).abs().idxmin()
        impact_row = df_mocap.iloc[idx_impact]
        vx = impact_row.get('vx', 0.0)
        vy = impact_row.get('vy', -1.0)
        rot_vx, rot_vy = rotate_coords(vx, vy)
        
        # Rays from drone contact center:
        # Ray 1: Separation vector pointing to column center
        angle_ray1 = np.arctan2(rot_col_y - rot_closest_y, rot_col_x - rot_closest_x)
        # Ray 2: Drone's velocity vector
        angle_ray2 = np.arctan2(rot_vy, rot_vx)
        
        deg_ray1 = np.degrees(angle_ray1)
        deg_ray2 = np.degrees(angle_ray2)
        
        # Draw clean straight velocity arrow from drone center
        v_norm = np.sqrt(rot_vx**2 + rot_vy**2)
        if v_norm > 0:
            arrow_scale = 0.28 / v_norm  # Scale to 28cm length
            ax.quiver(rot_closest_x, rot_closest_y, rot_vx * arrow_scale, rot_vy * arrow_scale,
                      angles='xy', scale_units='xy', scale=1, color='crimson', width=0.005, headwidth=4.5, zorder=8,
                      label='Velocity Vector at Contact')
            
        # Draw angular arc
        arc_radius = 0.22  # 22cm radius
        t1 = min(deg_ray1, deg_ray2)
        t2 = max(deg_ray1, deg_ray2)
        if t2 - t1 > 180:
            t1, t2 = t2, t1 + 360
            
        arc = patches.Arc((rot_closest_x, rot_closest_y), 2*arc_radius, 2*arc_radius,
                          angle=0, theta1=t1, theta2=t2, color='crimson', linewidth=2.0, zorder=9)
        ax.add_patch(arc)
        
        # Place label at the bisector
        bisector = np.radians((t1 + t2) / 2.0)
        label_r = arc_radius + 0.09
        label_x = rot_closest_x + label_r * np.cos(bisector)
        label_y = rot_closest_y + label_r * np.sin(bisector)
        
        ax.text(label_x, label_y, f"{achieved_angle:.1f}°", color='crimson', fontweight='bold',
                fontsize=9, ha='center', va='center', zorder=10,
                bbox=dict(facecolor='white', alpha=0.95, edgecolor='crimson', boxstyle='round,pad=0.15'))

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

    # Lower right corner text for flight name and pass identification
    if flight_name:
        ax.text(0.98, 0.02, f"{flight_name}", transform=ax.transAxes,
                ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

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
        'closest_clearance': closest_clearance,
        'achieved_impact_angle': achieved_angle
    }
