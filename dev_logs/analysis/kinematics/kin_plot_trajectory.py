# ==============================================================================
# ⚠️ CRITICAL NOTICE: DO NOT TOUCH OR ALTER THE 2D TRAJECTORY PLOT OR FORMATTING!
# The layout, colors, styles, labels, CAD drone rendering, and mathematical
# overlays (e.g., max deviation, recovery area hatching) must NOT be modified.
# If any adjustments are necessary, ALWAYS consult the user first.
# ==============================================================================

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

def perpendicular_distance(p, p1, p2):
    """Calculates perpendicular distance from point p to line segment (p1 -> p2)."""
    x0, y0 = p
    x1, y1 = p1
    x2, y2 = p2
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    den = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return num / den if den > 0 else 0.0

# ==============================================================================
# ⚠️ CRITICAL NOTICE: DO NOT ALTER OR RENAME THE STYLING OR LABELS OF THIS PLOT!
# ==============================================================================
def plot_trajectory(df_mocap, wp_events, column_x=0.408, column_y=0.358, 
                    cage_diameter=0.358, column_diameter=0.09, output_path=None, flight_name=None,
                    dynamic_waypoints=None, df_column=None, df_setpoint=None, condition=None, show_plot=True):
    """Plots 2D top-down spatial trajectory, showing the flight sweep, waypoints,
    safety cage, minimum clearance vector, maximum deviation, and hatched recovery area.
    """
    if condition is None and flight_name:
        if "rotating_cage" in flight_name.lower():
            condition = "Rotating Cage"
        elif "fixed_cage" in flight_name.lower():
            condition = "Fixed Cage"
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

    # Dynamic target attractor waypoint sequence (theoretical commands)
    if dynamic_waypoints and len(dynamic_waypoints) == 4:
        wps = np.array([dynamic_waypoints[1][1], dynamic_waypoints[2][1]])
    else:
        is_collision = False
        is_45 = False
        if flight_name:
            fn_lower = flight_name.lower()
            if "collision" in fn_lower or "75" in fn_lower or "45" in fn_lower:
                is_collision = True
            if "45" in fn_lower:
                is_45 = True
        
        if is_collision:
            x_lane = 0.248 if is_45 else 0.186
            wps = np.array([[x_lane, 0.950], [x_lane, -1.200]])
        else:
            wps = np.array([[0.100, 1.200], [0.100, -1.200]])

    wp3_ideal_x, wp3_ideal_y = wps[1][0], wps[1][1]

    # CRITICAL SPEC: Truncate sweep strictly between Exp. Start-point (WP2) and Exp. End-point (WP3)
    wp2_t = wp_events.get('WP2')
    wp3_t = wp_events.get('WP3')
    
    t_start = wp2_t if wp2_t is not None else df_mocap['t'].iloc[0]
    t_end = wp3_t if wp3_t is not None else df_mocap['t'].iloc[-1]
    
    df_mocap_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
    if df_mocap_sweep.empty:
        df_mocap_sweep = df_mocap

    # Query exact registered MoCap coordinates at Exp. Start-point (WP2)
    if wp2_t is not None:
        idx_wp2 = np.searchsorted(df_mocap['t'], wp2_t)
        idx_wp2 = min(max(0, idx_wp2), len(df_mocap) - 1)
        wp2_x, wp2_y = df_mocap['x'].iloc[idx_wp2], df_mocap['y'].iloc[idx_wp2]
    else:
        is_45 = flight_name and "45" in flight_name.lower()
        x_lane = 0.248 if is_45 else 0.186
        wp2_x, wp2_y = x_lane, 0.950

    # Query exact registered MoCap coordinates at Exp. End-point (WP3)
    if wp3_t is not None:
        idx_wp3 = np.searchsorted(df_mocap['t'], wp3_t)
        idx_wp3 = min(max(0, idx_wp3), len(df_mocap) - 1)
        wp3_x, wp3_y = df_mocap['x'].iloc[idx_wp3], df_mocap['y'].iloc[idx_wp3]
    else:
        is_45 = flight_name and "45" in flight_name.lower()
        x_lane = 0.248 if is_45 else 0.186
        wp3_x, wp3_y = x_lane, -1.200

    # Calculate minimum spatial distance to column center during active sweep
    dist_to_col_center = np.sqrt((df_mocap_sweep['x'] - column_x)**2 + (df_mocap_sweep['y'] - column_y)**2)
    dist_to_col_surface = dist_to_col_center - column_radius
    min_distance_idx = np.argmin(dist_to_col_surface)

    closest_t = df_mocap_sweep['t'].iloc[min_distance_idx]
    closest_x = df_mocap_sweep['x'].iloc[min_distance_idx]
    closest_y = df_mocap_sweep['y'].iloc[min_distance_idx]
    closest_clearance = dist_to_col_surface.iloc[min_distance_idx] - cage_radius

    # Extract speed and acceleration at closest approach (impact time)
    idx_closest_t = (df_mocap['t'] - closest_t).abs().idxmin()
    closest_row = df_mocap.iloc[idx_closest_t]
    impact_speed = closest_row.get('speed', 0.0)
    impact_accel = closest_row.get('accel', 0.0)

    # Dynamic Column coordinates at the exact timestamp of closest approach
    if df_column is not None and not df_column.empty:
        idx_col_closest = (df_column['t'] - closest_t).abs().idxmin()
        col_dyn_x = df_column['x'].iloc[idx_col_closest]
        col_dyn_y = df_column['y'].iloc[idx_col_closest]
    else:
        col_dyn_x = column_x
        col_dyn_y = column_y

    fig, ax = plt.subplots(figsize=(11, 7))

    rot_col_x, rot_col_y = rotate_coords(col_dyn_x, col_dyn_y)
    rot_traj_x, rot_traj_y = rotate_coords(df_mocap_sweep['x'], df_mocap_sweep['y'])
    rot_closest_x, rot_closest_y = rotate_coords(closest_x, closest_y)
    rot_wp2_x, rot_wp2_y = rotate_coords(wp2_x, wp2_y)
    rot_wp3_x, rot_wp3_y = rotate_coords(wp3_x, wp3_y)

    # A. Calculate and Plot Hatched Recovery Area (after collision impact)
    impact_t = wp_events.get('Column Impact')
    t_collision = impact_t if impact_t is not None else closest_t
    df_recovery = df_mocap_sweep[df_mocap_sweep['t'] >= t_collision]
    
    if not df_recovery.empty and len(df_recovery) >= 2:
        rot_rec_x, rot_rec_y = rotate_coords(df_recovery['x'], df_recovery['y'])
        
        # Calculate nominal path projection relative to Hybrid tracking segment (Actual Start -> Ideal End)
        dx_line = wp3_ideal_x - wp2_x
        dy_line = wp3_ideal_y - wp2_y
        line_len_sq = dx_line**2 + dy_line**2
        if line_len_sq > 0:
            ap_x = df_recovery['x'] - wp2_x
            ap_y = df_recovery['y'] - wp2_y
            t_proj = (ap_x * dx_line + ap_y * dy_line) / line_len_sq
            proj_x = wp2_x + t_proj * dx_line
            proj_y = wp2_y + t_proj * dy_line
            rot_proj_rec_x, rot_proj_rec_y = rotate_coords(proj_x, proj_y)
            
            # Form recovery polygon
            poly_x = np.concatenate([rot_rec_x, rot_proj_rec_x[::-1]])
            poly_y = np.concatenate([rot_rec_y, rot_proj_rec_y[::-1]])
            
            # Integral area (SIAE) using Trapezoidal Rule along travel path distance
            line_len = np.sqrt(line_len_sq)
            ux, uy = dx_line / line_len, dy_line / line_len
            s_rec = ap_x * ux + ap_y * uy
            d_rec = np.array([perpendicular_distance((row['x'], row['y']), (wp2_x, wp2_y), (wp3_ideal_x, wp3_ideal_y)) for _, row in df_recovery.iterrows()])
            
            sort_idx = np.argsort(s_rec)
            s_sorted = s_rec.iloc[sort_idx]
            d_sorted = d_rec[sort_idx]
            area_m2 = np.trapz(d_sorted, s_sorted)
            area_cm2 = area_m2 * 10000.0
            
            # Fill with diagonal hatch
            ax.fill(poly_x, poly_y, color='purple', alpha=0.15, hatch='//', edgecolor='purple', linewidth=0.5,
                    label=f'Recovery Area ({area_cm2:.1f} cm²)', zorder=3)

    # B. Calculate and Plot Perpendicular Maximum Deviation (strictly post-impact to isolate recovery rebound)
    # Projected relative to the Hybrid tracking segment (Actual Start -> Ideal End)
    dx_line = wp3_ideal_x - wp2_x
    dy_line = wp3_ideal_y - wp2_y
    line_len_sq = dx_line**2 + dy_line**2
    if line_len_sq > 0:
        # Bounded strictly to post-collision samples
        df_dev_search = df_mocap_sweep[df_mocap_sweep['t'] >= t_collision]
        if df_dev_search.empty:
            df_dev_search = df_mocap_sweep
            
        ap_x = df_dev_search['x'] - wp2_x
        ap_y = df_dev_search['y'] - wp2_y
        t_proj = (ap_x * dx_line + ap_y * dy_line) / line_len_sq
        proj_x = wp2_x + t_proj * dx_line
        proj_y = wp2_y + t_proj * dy_line
        perp_dists = np.sqrt((df_dev_search['x'] - proj_x)**2 + (df_dev_search['y'] - proj_y)**2)
        
        max_dev_idx = np.argmax(perp_dists)
        p_max_x, p_max_y = df_dev_search['x'].iloc[max_dev_idx], df_dev_search['y'].iloc[max_dev_idx]
        h_max_x, h_max_y = proj_x.iloc[max_dev_idx], proj_y.iloc[max_dev_idx]
        dev_max_val = perp_dists.iloc[max_dev_idx]
        
        rot_p_max_x, rot_p_max_y = rotate_coords(p_max_x, p_max_y)
        rot_h_max_x, rot_h_max_y = rotate_coords(h_max_x, h_max_y)
        
        # Plot maximum deviation perpendicular dotted line
        ax.plot([rot_p_max_x, rot_h_max_x], [rot_p_max_y, rot_h_max_y], color='orange', linestyle='--', linewidth=2.0,
                label=f'Max Deviation ({dev_max_val*1000:.0f} mm)', zorder=7)
        # Position label on the line
        mid_x = 0.5 * (rot_p_max_x + rot_h_max_x)
        mid_y = 0.5 * (rot_p_max_y + rot_h_max_y)
        ax.annotate(f"dev_max = {dev_max_val*1000:.0f} mm", xy=(mid_x, mid_y), xytext=(12, 0), textcoords='offset points',
                    fontsize=8, fontweight='bold', color='orange', ha='left', va='center',
                    bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=1.0))

    # 1. Plot the Central Column Obstacle (to scale, rotated)
    column_circle_outer = plt.Circle((rot_col_x, rot_col_y), column_radius, color='#FFCC00', alpha=0.3, 
                                     label=f'Column Obstacle (D={column_diameter*100:.1f}cm)')
    column_circle_edge = plt.Circle((rot_col_x, rot_col_y), column_radius, fill=False, color='#CC9900', linewidth=2.0)
    ax.add_patch(column_circle_outer)
    ax.add_patch(column_circle_edge)
    ax.scatter(rot_col_x, rot_col_y, color='black', marker='+', zorder=5)

    # 2. Plot Drone Actual Flight Path (ONLY truncated active sweep path, rotated)
    ax.plot(rot_traj_x, rot_traj_y, color=C_MOCAP, alpha=0.8, linewidth=2.5, label='Actual Path [MoCap ENU]')

    # Plot PX4 Target Setpoint Trajectory if provided
    if df_setpoint is not None and not df_setpoint.empty and 'x_cmd' in df_setpoint.columns and 'y_cmd' in df_setpoint.columns:
        df_sp_sweep = df_setpoint[(df_setpoint['t'] >= t_start) & (df_setpoint['t'] <= t_end)]
        if not df_sp_sweep.empty:
            rot_sp_x, rot_sp_y = rotate_coords(df_sp_sweep['x_cmd'], df_sp_sweep['y_cmd'])
            ax.plot(rot_sp_x, rot_sp_y, color='#9467BD', linestyle='-.', alpha=0.8, linewidth=1.8, 
                    label='PX4 Setpoint (Target track)', zorder=4)

    # 3. Draw only 2 command waypoints: Exp. Start-point (WP2) and Exp. End-point (WP3) (rotated)

    rot_wps_x, rot_wps_y = rotate_coords(wps[:, 0], wps[:, 1])
    ax.scatter(rot_wps_x, rot_wps_y, color=C_CMD, s=120, facecolors='none', edgecolors=C_CMD, linewidth=2, zorder=6, label='Command Waypoints')
    
    # 4. Connect waypoints with dotted red line and perfectly aligned arrow (no skew!)
    x1, y1 = rot_wps_x[0], rot_wps_y[0]
    x2, y2 = rot_wps_x[1], rot_wps_y[1]
    ax.plot([x1, x2], [y1, y2], color=C_CMD, linestyle=':', linewidth=2, zorder=5)
    
    dx = x2 - x1
    dy = y2 - y1
    length = np.sqrt(dx**2 + dy**2)
    if length > 0:
        ux, uy = dx / length, dy / length
        arrow_x = x1 + 0.5 * dx
        arrow_y = y1 + 0.5 * dy
        ax.annotate('', xy=(arrow_x + ux * 0.05, arrow_y + uy * 0.05), 
                     xytext=(arrow_x - ux * 0.05, arrow_y - uy * 0.05),
                     arrowprops=dict(arrowstyle="->", color=C_CMD, lw=2, shrinkA=0, shrinkB=0))

    # 5. Plot drone safety cage grayed out at actual registered MoCap positions at Start & End points
    if draw_vector_drone:
        draw_vector_drone(ax, rot_wp2_x, rot_wp2_y, cage_radius, rotation_deg=0, color_mode='gray')
        draw_vector_drone(ax, rot_wp3_x, rot_wp3_y, cage_radius, rotation_deg=0, color_mode='gray')

    # 6. Plot drone safety cage boundary & schematic at closest approach (in color!, rotated)
    ax.plot([], [], color='#1F77B4', linestyle='--', label=f'Safety Cage (D={cage_diameter*100:.1f}cm)')
    if draw_vector_drone:
        draw_vector_drone(ax, rot_closest_x, rot_closest_y, cage_radius, rotation_deg=0, color_mode='full')
    else:
        schematic_circle = plt.Circle((rot_closest_x, rot_closest_y), cage_radius, fill=False, color='#1F77B4', linestyle='--', linewidth=2.0)
        ax.add_patch(schematic_circle)

    # 7. Unified borderless text boxes with subtle pointer lines to completely prevent overlaps
    # Exp. Start-point
    start_text = f"Exp. Start-point\nX: {wp2_x:.3f}m\nY: {wp2_y:.3f}m"
    ax.annotate(start_text, xy=(rot_wp2_x, rot_wp2_y), xytext=(-1.4, -0.25), textcoords='data',
                fontsize=8, fontweight='bold', color='#444444', ha='left', va='center',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8))

    # Exp. End-point
    end_text = f"Exp. End-point\nX: {wp3_x:.3f}m\nY: {wp3_y:.3f}m"
    ax.annotate(end_text, xy=(rot_wp3_x, rot_wp3_y), xytext=(0, 50), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#444444', ha='center', va='bottom',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8))

    # Column coordinate label positioned directly ABOVE the column center
    col_text = f"Column Obstacle\nX: {col_dyn_x:.3f}m\nY: {col_dyn_y:.3f}m"
    ax.annotate(col_text, xy=(rot_col_x, rot_col_y), xytext=(0, 50), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#CC9900', ha='center', va='bottom',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#CC9900', lw=0.8))

    # Closest approach annotation positioned directly UNDER the drone
    closest_text = f"Closest Approach\n" \
                   f"X: {closest_x:.3f}m\n" \
                   f"Y: {closest_y:.3f}m\n" \
                   f"Clearance: {closest_clearance*100:+.1f} cm\n" \
                   f"Speed: {impact_speed:.3f} m/s\n" \
                   f"Acc: {impact_accel:.3f} m/s²"
    ax.annotate(closest_text, xy=(rot_closest_x, rot_closest_y), xytext=(0, -75), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#1F77B4', ha='center', va='top',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#1F77B4', lw=0.8))

    # Draw line showing minimum separation vector (rotated)
    ax.plot([rot_closest_x, rot_col_x], [rot_closest_y, rot_col_y], color='purple', linestyle=':', linewidth=2, label='Separation Vector')

    # Calculate achieved 2D spatial trajectory angle at exact contact point
    achieved_angle = None
    if t_collision is not None and not df_mocap.empty:
        idx_impact = (df_mocap['t'] - t_collision).abs().idxmin()
        impact_row = df_mocap.iloc[idx_impact]
        rx = col_dyn_x - impact_row['x']
        ry = col_dyn_y - impact_row['y']
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
        idx_impact = (df_mocap['t'] - t_collision).abs().idxmin()
        impact_row = df_mocap.iloc[idx_impact]
        vx = impact_row.get('vx', 0.0)
        vy = impact_row.get('vy', -1.0)
        rot_vx, rot_vy = rotate_coords(vx, vy)
        
        # Rays from drone contact center:
        angle_ray1 = np.arctan2(rot_col_y - rot_closest_y, rot_col_x - rot_closest_x)
        angle_ray2 = np.arctan2(rot_vy, rot_vx)
        deg_ray1 = np.degrees(angle_ray1)
        deg_ray2 = np.degrees(angle_ray2)
        
        # Draw straight velocity arrow from drone center
        v_norm = np.sqrt(rot_vx**2 + rot_vy**2)
        if v_norm > 0:
            arrow_scale = 0.28 / v_norm  # Scale to 28cm length
            ax.quiver(rot_closest_x, rot_closest_y, rot_vx * arrow_scale, rot_vy * arrow_scale,
                      angles='xy', scale_units='xy', scale=1, color='crimson', width=0.005, headwidth=4.5, zorder=8,
                      label='Velocity Vector at Contact')
            
        # Draw angular arc
        arc_radius = 0.22
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

    cond_suffix = f" <{condition}>" if condition else ""
    ax.set_title(f'Experiment 2D horizontal visualization{cond_suffix}', fontsize=12, fontweight='bold')

    # Professional SSoT color coding of grid axes (X = Crimson Red, Y = Green)
    ax.set_xlabel('Y coordinate, meters', color='#2CA02C', fontweight='bold')
    ax.set_ylabel('X coordinate, meters', color='#D62728', fontweight='bold')
    ax.tick_params(axis='x', colors='#2CA02C')
    ax.tick_params(axis='y', colors='#D62728')

    # Lower bottom corner laboratory rotation disclaimer
    disclaimer_text = (
        "X and Y axes have been rotated on this plot\n"
        "to strictly adhere to the physical mapping of X-Y\n"
        "in the motion capture system used"
    )
    ax.text(0.02, 0.02, disclaimer_text,
            transform=ax.transAxes, ha='left', va='bottom', fontsize=7, style='italic', alpha=0.8, zorder=10)

    # Strict Equal Aspect Ratio and boundary truncation
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-0.5, 1.0)

    # Normalize grid to exact squares of 0.5m side length
    ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
    ax.set_yticks(np.arange(-0.5, 1.1, 0.5))

    # Flight name label (below x-axis, figure-bottom)
    if flight_name:
        fig.text(0.98, 0.01, f"{flight_name}", transform=fig.transFigure,
                ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    # Custom table-like monospace legend
    handles, labels = ax.get_legend_handles_labels()
    aligned_labels = []
    for lbl in labels:
        if "Recovery Area" in lbl:
            try:
                val_str = lbl.split('(')[1].split(')')[0]
                aligned_labels.append(f"{'Recovery Area':<18}({val_str})")
            except Exception:
                aligned_labels.append(lbl)
        elif "Max Deviation" in lbl:
            try:
                val_str = lbl.split('(')[1].split(')')[0]
                aligned_labels.append(f"{'Max Deviation':<18}({val_str})")
            except Exception:
                aligned_labels.append(lbl)
        elif "Column Obstacle" in lbl:
            try:
                val_str = lbl.split('(')[1].split(')')[0]
                aligned_labels.append(f"{'Column Obstacle':<18}({val_str})")
            except Exception:
                aligned_labels.append(lbl)
        elif "Safety Cage" in lbl:
            try:
                val_str = lbl.split('(')[1].split(')')[0]
                aligned_labels.append(f"{'Safety Cage':<18}({val_str})")
            except Exception:
                aligned_labels.append(lbl)
        elif "Actual Path" in lbl:
            aligned_labels.append(f"{'Actual Path':<18}[MoCap ENU]")
        elif "PX4 Setpoint" in lbl:
            aligned_labels.append(f"{'PX4 Setpoint':<18}(Target track)")
        elif "Separation Vector" in lbl:
            aligned_labels.append(f"{'Separation Vector':<18}")
        else:
            aligned_labels.append(lbl)

    ax.legend(handles, aligned_labels, loc='upper left', prop={'family': 'monospace', 'size': 8.5})
    ax.grid(True, color='#EAEAEA')
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] Trajectory plot saved successfully to: {output_path}")

    if show_plot:
        plt.show()
    plt.close()

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


# ==============================================================================
# Full Mission Loop — Theoretical Geometry  (no flight data required)
# ==============================================================================

# 45° waypoints (from ExpCollision45Deg)
_WP_45 = {
    'WP_stage': (0.248, 1.200),
    'WP1':      (0.248, 0.950),
    'WP2':      (0.248, -1.200),
    'WP3':      (0.000, 0.300),
    'COLUMN':   (0.408, 0.358),
    'coll_pt':  (0.2496, 0.5164),  # exact 45° geometry
}
# 75° waypoints (from ExpCollision75Deg)
_WP_75 = {
    'WP_stage': (0.186, 1.200),
    'WP1':      (0.186, 1.100),  # 75° has WP1 at Y=1.100 (45° has 0.950)
    'WP2':      (0.186, -1.200),
    'WP3':      (0.000, 0.300),
    'COLUMN':   (0.408, 0.358),
    'coll_pt':  (0.2400, 0.4296),  # approximate 75° geometry
}
_WPS = {45: _WP_45, 75: _WP_75}
_COL_DIAMETER = 0.09
_CAGE_DIAMETER = 0.358


def _rotate_coords(x_val, y_val):
    """Rotate coordinate system 90° CCW — matches plot_trajectory()."""
    return -y_val, x_val


def plot_full_loop_geometry(angle_deg=45, output_path=None, show_plot=True):
    """Theoretical 2D top-down loop geometry — no flight data needed.

    Plots the full closed-loop path, waypoints, shaded drones, column
    obstacle, full-colour drone at the collision point with heading vector,
    separation vector, and safety cage.  Uses the same rotated coordinate
    system, canvas, drone rendering, and styling as ``plot_trajectory()``.

    Parameters
    ----------
    angle_deg : int, optional
        Impact angle (45 or 75).  Selects the corresponding mission waypoints.
    output_path : str, optional
        Save to file instead of showing interactively.
    show_plot : bool, optional
        Call ``plt.show()``.
    """
    # ── Late imports (avoids circular deps, matches plot_trajectory pattern) ──
    current_dir = os.path.dirname(os.path.abspath(__file__))
    analysis_dir = os.path.abspath(os.path.join(current_dir, ".."))
    if 'graphics' in sys.modules:
        gmod = sys.modules['graphics']
        if not getattr(gmod, '__file__', '').startswith(analysis_dir):
            del sys.modules['graphics']
    if analysis_dir not in sys.path:
        sys.path.insert(0, analysis_dir)
    try:
        from graphics import draw_vector_drone
    except ImportError as e:
        print(f"[WARN] draw_vector_drone unavailable: {e}")
        draw_vector_drone = None

    # ── Pick the right waypoint set ──
    wp = _WPS.get(angle_deg, _WPS[45])
    WP_stage, WP1 = wp['WP_stage'], wp['WP1']
    WP2, WP3      = wp['WP2'], wp['WP3']
    COLUMN        = wp['COLUMN']
    coll_pt       = wp['coll_pt']

    COL_RADIUS    = _COL_DIAMETER / 2.0
    CAGE_RADIUS   = _CAGE_DIAMETER / 2.0

    # Rotate everything
    r_stage = _rotate_coords(*WP_stage)
    r_wp1   = _rotate_coords(*WP1)
    r_wp2   = _rotate_coords(*WP2)
    r_wp3   = _rotate_coords(*WP3)
    r_col   = _rotate_coords(*COLUMN)
    r_coll  = _rotate_coords(*coll_pt)

    enu_loop = [WP_stage, WP1, WP2, WP3, WP_stage]
    rot_loop = [_rotate_coords(x, y) for x, y in enu_loop]

    fig, ax = plt.subplots(figsize=(11, 7))

    # ── Dotted red closed-loop path ──
    ax.plot([p[0] for p in rot_loop], [p[1] for p in rot_loop],
            color='#D62728', linestyle=':', linewidth=2.5, zorder=5,
            label='Target Track')

    # Directional arrows at midpoints
    for i in range(len(enu_loop) - 1):
        se, ee = enu_loop[i], enu_loop[i+1]
        mid   = ((se[0] + ee[0]) / 2, (se[1] + ee[1]) / 2)
        r_mid = _rotate_coords(*mid)
        r_dir = _rotate_coords(ee[0] - se[0], ee[1] - se[1])
        ln    = np.hypot(*r_dir)
        if ln > 1e-6:
            ax.annotate('', xy=(r_mid[0] + r_dir[0]/ln*0.08,
                                r_mid[1] + r_dir[1]/ln*0.08),
                        xytext=(r_mid[0], r_mid[1]),
                        arrowprops=dict(arrowstyle="->", color='#D62728',
                                        lw=2, shrinkA=0, shrinkB=0))

    # ── Column obstacle ──
    col_fill = plt.Circle(r_col, COL_RADIUS, color='#FFCC00', alpha=0.3, zorder=5)
    col_edge = plt.Circle(r_col, COL_RADIUS, fill=False, color='#CC9900',
                           linewidth=2.0, zorder=5)
    ax.add_patch(col_fill)
    ax.add_patch(col_edge)
    ax.scatter(r_col[0], r_col[1], color='black', marker='+', zorder=5)
    ax.annotate(f'Column Obstacle\nX: {COLUMN[0]:.3f}m\nY: {COLUMN[1]:.3f}m',
                xy=r_col, xytext=(0, 50), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#CC9900',
                ha='center', va='bottom',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#CC9900', lw=0.8))

    # ── Command waypoints (hollow red circles) ──
    wps = [r_stage, r_wp1, r_wp2, r_wp3]
    ax.scatter([p[0] for p in wps], [p[1] for p in wps],
               color='#D62728', s=120, facecolors='none', edgecolors='#D62728',
               linewidth=2.5, zorder=6, label='Command Waypoints')

    # ── Waypoint labels — strictly vertical above/below ──
    wp_info = [
        (r_stage, f"WP_stage (U-turn)\nX: {WP_stage[0]:.3f}m\nY: {WP_stage[1]:.3f}m",
         0, -60, 'center', 'top'),
        (r_wp1,   f"Exp. Start-point\nX: {WP1[0]:.3f}m\nY: {WP1[1]:.3f}m",
         0, 55, 'center', 'bottom'),
        (r_wp2,   f"Exp. End-point\nX: {WP2[0]:.3f}m\nY: {WP2[1]:.3f}m",
         0, 55, 'center', 'bottom'),
        (r_wp3,   f"Recovery (WP3)\nX: {WP3[0]:.3f}m\nY: {WP3[1]:.3f}m",
         0, -55, 'center', 'top'),
    ]
    for (x, y), lbl, dx, dy, ha, va in wp_info:
        ax.annotate(lbl, xy=(x, y), xytext=(dx, dy),
                    textcoords='offset points',
                    fontsize=8, fontweight='bold', color='#444444',
                    ha=ha, va=va,
                    bbox=dict(facecolor='white', alpha=0.9,
                              edgecolor='none', pad=2),
                    arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8))

    # ── Gray drone ghosts at all waypoints ──
    if draw_vector_drone:
        for (x, y) in [r_stage, r_wp1, r_wp2, r_wp3]:
            draw_vector_drone(ax, x, y, CAGE_RADIUS, rotation_deg=0,
                              color_mode='gray')

    # ── Full-colour drone at collision point + safety cage ──
    if draw_vector_drone:
        draw_vector_drone(ax, r_coll[0], r_coll[1], CAGE_RADIUS,
                          rotation_deg=0, color_mode='full')
    safety_cage = plt.Circle(r_coll, CAGE_RADIUS, fill=False,
                              color='#1F77B4', linestyle='--',
                              linewidth=2.0, zorder=5)
    ax.add_patch(safety_cage)
    ax.plot([], [], color='#1F77B4', linestyle='--', linewidth=2.0,
            label=f'Safety Cage (D={_CAGE_DIAMETER*100:.1f}cm)')

    # Collision-point label above
    clearance = (np.hypot(COLUMN[0] - coll_pt[0], COLUMN[1] - coll_pt[1])
                 - COL_RADIUS - CAGE_RADIUS)
    ax.annotate(f"Collision Point\nClearance: {clearance*100:+.1f} cm",
                xy=r_coll, xytext=(0, 65), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#1F77B4',
                ha='center', va='bottom',
                bbox=dict(facecolor='white', alpha=0.9,
                          edgecolor='none', pad=2),
                arrowprops=dict(arrowstyle="->", color='#1F77B4', lw=0.8))

    # ── Heading vector (red quiver, southward in ENU) ──
    ax.quiver(r_coll[0], r_coll[1], 0.28, 0,
              angles='xy', scale_units='xy', scale=1, color='crimson',
              width=0.005, headwidth=4.5, zorder=8,
              label='Velocity Vector at Contact')

    # ── Separation vector (purple dotted) ──
    ax.plot([r_coll[0], r_col[0]], [r_coll[1], r_col[1]],
            color='purple', linestyle=':', linewidth=2, zorder=5,
            label='Separation Vector')

    # ── Axis setup ──
    ax.set_xlabel('Y coordinate, meters', color='#2CA02C', fontweight='bold')
    ax.set_ylabel('X coordinate, meters', color='#D62728', fontweight='bold')
    ax.tick_params(axis='x', colors='#2CA02C')
    ax.tick_params(axis='y', colors='#D62728')

    ax.text(0.02, 0.02,
            "X and Y axes have been rotated on this plot\n"
            "to strictly adhere to the physical mapping of X-Y\n"
            "in the motion capture system used",
            transform=ax.transAxes, ha='left', va='bottom',
            fontsize=7, style='italic', alpha=0.8, zorder=10)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-0.5, 1.0)
    ax.set_xticks(np.arange(-1.5, 1.6, 0.5))
    ax.set_yticks(np.arange(-0.5, 1.1, 0.5))

    # ── Monospace legend ──
    handles, labels = ax.get_legend_handles_labels()
    aligned = []
    for lbl in labels:
        matched = False
        for key in ['Column Obstacle', 'Safety Cage',
                     'Velocity Vector', 'Separation Vector']:
            if key in lbl:
                try:
                    val = lbl.split('(')[1].split(')')[0]
                    aligned.append(f'{key:<18}({val})')
                except Exception:
                    aligned.append(lbl)
                matched = True
                break
        if not matched:
            aligned.append(lbl)
    ax.legend(handles, aligned, loc='upper left',
              prop={'family': 'monospace', 'size': 8.5})
    ax.grid(True, color='#EAEAEA')
    ax.set_title(f'{angle_deg}° Column Collision Loop — Full Mission Geometry',
                 fontsize=12, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO]  Full-loop geometry saved to:  {output_path}")
    if show_plot:
        plt.show()
    plt.close()
