import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

def compute_velocity(df_mocap, window=19, polyorder=3):
    """Computes velocity derivatives from position signals using Savitzky-Golay filtering.
    Handles edge cases where df_mocap has fewer points than the requested window size.
    """
    if df_mocap.empty or len(df_mocap) < 3:
        df_mocap['vx'] = 0.0
        df_mocap['vy'] = 0.0
        df_mocap['vz'] = 0.0
        df_mocap['speed'] = 0.0
        return df_mocap

    # Unique and monotonically sorted
    df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)
    dt = df_mocap['t'].diff()
    median_dt = dt.median()
    if median_dt is None or np.isnan(median_dt) or median_dt == 0:
        median_dt = 0.01

    # Adjust window length to be odd and smaller than len(df_mocap)
    n_points = len(df_mocap)
    adjusted_window = window
    if adjusted_window >= n_points:
        adjusted_window = n_points // 2 * 2 - 1
        if adjusted_window < 3:
            adjusted_window = 3

    if adjusted_window > polyorder:
        try:
            vx_filtered = savgol_filter(df_mocap['x'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vy_filtered = savgol_filter(df_mocap['y'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vz_filtered = savgol_filter(df_mocap['z'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            # Fallback to basic finite difference
            vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
            vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
            vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])
    else:
        vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
        vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
        vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])

    df_mocap['vx'] = vx_filtered
    df_mocap['vy'] = vy_filtered
    df_mocap['vz'] = vz_filtered
    df_mocap['speed'] = np.sqrt(vx_filtered**2 + vy_filtered**2 + vz_filtered**2)
    
    # Compute derivative accelerations from MoCap velocity signals
    df_mocap['ax'] = np.gradient(df_mocap['vx'], df_mocap['t'])
    df_mocap['ay'] = np.gradient(df_mocap['vy'], df_mocap['t'])
    df_mocap['az'] = np.gradient(df_mocap['vz'], df_mocap['t'])
    df_mocap['accel'] = np.gradient(df_mocap['speed'], df_mocap['t'])
    return df_mocap

def find_waypoint_events(df_mocap, takeoff_time, label=None, column_x=0.408, column_y=0.358, column_radius=0.045, cage_radius=0.179, return_all=False):
    """Detects all completed passes within predefined waypoint coordinates chronologically,
    returning a list of dictionaries (one for each detected pass) or the first pass dict.
    """
    if df_mocap.empty:
        return [] if return_all else {}

    # 1. Determine which mission was run by inspecting the flight label
    is_75deg = False
    if label:
        lbl_lower = label.lower()
        if "75" in lbl_lower or "collision" in lbl_lower:
            is_75deg = True

    # 2. Dynamically import and instantiate the mission class
    try:
        import sys
        import os
        # Resolve project root dynamically
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
        if project_root not in sys.path:
            sys.path.append(project_root)

        if is_75deg:
            from drone_control.missions.exp_collision_75deg import ExpCollision75Deg
            mission = ExpCollision75Deg(target_z=0.5)
        else:
            from drone_control.missions.column_sweep_loop import ColumnSweepLoop
            mission = ColumnSweepLoop(target_z=0.5)

        # 3. Simulate takeoff to populate the mission waypoints
        # Get actual takeoff coordinate from telemetry
        takeoff_samples = df_mocap[df_mocap['t'] >= takeoff_time]
        if not takeoff_samples.empty:
            x0 = takeoff_samples['x'].iloc[0]
            y0 = takeoff_samples['y'].iloc[0]
            z0 = takeoff_samples['z'].iloc[0]
        else:
            x0, y0, z0 = 0.0, 0.0, 0.5

        class DummyPose:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z
        
        mission.on_start(DummyPose(x0, y0, z0))
        
        # 4. Build sequence dynamically from the mission class variables
        wp_sequence = [
            ('WP1', (mission.wp1[0], mission.wp1[1])),
            ('WP2', (mission.wp2[0], mission.wp2[1])),
            ('WP3', (mission.wp3[0], mission.wp3[1])),
            ('WP4', (mission.wp4[0], mission.wp4[1]))
        ]
    except Exception as e:
        # Fallback to standard ColumnSweepLoop if import fails
        wp_sequence = [
            ('WP1', (0.000, 1.200)),
            ('WP2', (0.100, 1.200)),
            ('WP3', (0.100, -1.200)),
            ('WP4', (0.000, -1.200))
        ]

    # Find ALL loop passes sequentially
    passes_events = []
    current_search_time = takeoff_time
    max_time = df_mocap['t'].max()

    while current_search_time < max_time:
        wp_events = {}
        t_ptr = current_search_time
        
        # Sequentially search for WP1 -> WP2 -> WP3 -> WP4 in a single pass loop
        found_all = True
        for wp_name, (wx, wy) in wp_sequence:
            dist = np.sqrt((df_mocap['x'] - wx)**2 + (df_mocap['y'] - wy)**2)
            after_ptr = df_mocap['t'] > t_ptr
            arrived_mask = (dist < 0.08) & after_ptr  # 8cm in analysis (vs 5cm on-drone) — robust for 100Hz MoCap replay
            if arrived_mask.any():
                arrival_t = df_mocap.loc[arrived_mask, 't'].iloc[0]
                wp_events[wp_name] = arrival_t
                # Advance search time pointer by 1.0 second for the next waypoint in sequence
                t_ptr = arrival_t + 1.0
            else:
                found_all = False
                break
        
        if found_all:
            # Detect exact Column Y Crossing (Sweep intersection) inside this specific pass
            t_wp2 = wp_events['WP2']
            t_wp3 = wp_events['WP3']
            sweep_data = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
            if not sweep_data.empty:
                idx_min = (sweep_data['y'] - column_y).abs().idxmin()
                t_cross = sweep_data.loc[idx_min, 't']
                wp_events['Column Passed'] = t_cross
                
                # Detect exact Column Impact Event inside this specific pass
                dist_profile = np.sqrt((sweep_data['x'] - column_x)**2 + (sweep_data['y'] - column_y)**2)
                impact_threshold = column_radius + cage_radius + 0.015  # 0.224m + 0.015m = 0.239m
                impact_mask = dist_profile <= impact_threshold
                if impact_mask.any():
                    idx_impact = dist_profile[impact_mask].index[0]
                    t_geom = sweep_data.loc[idx_impact, 't']
                    
                    # Onset of deceleration
                    window_mask = (sweep_data['t'] >= t_geom - 0.2) & (sweep_data['t'] <= t_geom + 0.1)
                    window_data = sweep_data[window_mask]
                    if not window_data.empty:
                        idx_peak_speed = window_data['speed'].idxmax()
                        t_impact = window_data.loc[idx_peak_speed, 't']
                    else:
                        t_impact = t_geom
                    
                    wp_events['Column Impact'] = t_impact
            
            passes_events.append(wp_events)
            # Advance search pointer past WP4 by 1.0 second to begin search for next loop
            current_search_time = wp_events['WP4'] + 1.0
        else:
            # If we couldn't complete the full sequence, stop searching
            break

    # Fallback: if we didn't find any fully completed loops, but have partial waypoints, return at least one
    if not passes_events:
        wp_events = {}
        t_ptr = takeoff_time
        for wp_name, (wx, wy) in wp_sequence:
            dist = np.sqrt((df_mocap['x'] - wx)**2 + (df_mocap['y'] - wy)**2)
            after_ptr = df_mocap['t'] > t_ptr
            arrived_mask = (dist < 0.08) & after_ptr
            if arrived_mask.any():
                arrival_t = df_mocap.loc[arrived_mask, 't'].iloc[0]
                wp_events[wp_name] = arrival_t
                t_ptr = arrival_t + 1.0
        if wp_events:
            passes_events.append(wp_events)

    if return_all:
        return passes_events
    else:
        return passes_events[0] if passes_events else {}

def query_battery(df_bat, t_query):
    """Helper to query battery remaining percentage and voltage at a specific timestamp."""
    if df_bat is None or df_bat.empty:
        return "N/A", "N/A"
    idx = np.searchsorted(df_bat['t'], t_query)
    idx = min(max(0, idx), len(df_bat) - 1)
    return f"{df_bat['remaining'].iloc[idx]:.1f}%", f"{df_bat['voltage'].iloc[idx]:.2f}V"

def build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=None):
    """Compiles a chronological flight events table for display."""
    events_log = []
    last_t_abs = None

    def add_event(name, t_abs):
        nonlocal last_t_abs
        if t_abs is None or np.isnan(t_abs):
            return
        t_arm_rel = t_abs - arming_time
        bat_pct, bat_v = query_battery(df_bat, t_abs)
        
        # Query position
        if not df_mocap.empty:
            idx = np.searchsorted(df_mocap['t'], t_abs)
            idx = min(max(0, idx), len(df_mocap) - 1)
            pos_str = f"({df_mocap['x'].iloc[idx]:.3f}, {df_mocap['y'].iloc[idx]:.3f}, {df_mocap['z'].iloc[idx]:.3f})m"
        else:
            pos_str = "N/A"
            
        # Calculate segment average velocity
        if last_t_abs is None:
            avg_vel_str = "N/A"
        else:
            mask = (df_mocap['t'] >= last_t_abs) & (df_mocap['t'] <= t_abs)
            if mask.any():
                avg_vel = df_mocap.loc[mask, 'speed'].mean()
                avg_vel_str = f"{avg_vel:.3f} m/s"
            else:
                avg_vel_str = "0.000 m/s"
                
        events_log.append({
            'Event Name': name,
            'Absolute Time (s)': f"{t_abs:.2f}s",
            'Time Since Arming (s)': f"{t_arm_rel:+.2f}s" if t_abs >= arming_time else "N/A",
            'Average Velocity': avg_vel_str,
            'Battery Remaining': bat_pct,
            'Voltage (V)': bat_v,
            'Mocap Coordinate (ENU)': pos_str
        })
        last_t_abs = t_abs

    if not df_mocap.empty:
        add_event("1. Log Start", df_mocap['t'].iloc[0])
    if arming_time is not None and arming_time > 0:
        add_event("2. System Armed", arming_time)
    if takeoff_time is not None:
        add_event("3. Takeoff Detected", takeoff_time)
    
    # Sort keys chronologically based on their timestamps to keep events in order
    sorted_wp_names = sorted(wp_events.keys(), key=lambda k: wp_events[k])
    for wp in sorted_wp_names:
        if wp == 'Column Impact':
            angle_str = f" (Achieved Angle: {achieved_angle:.1f}°)" if achieved_angle is not None else ""
            add_event(f"💥 Column Impact{angle_str}", wp_events[wp])
        elif wp == 'Column Passed':
            add_event("🟢 Column Center Passed", wp_events[wp])
        else:
            add_event(f"4. Arrived {wp}", wp_events[wp])
            
    if disarming_time is not None:
        add_event("5. Disarmed / Landed", disarming_time)
    if not df_mocap.empty:
        add_event("6. Log End", df_mocap['t'].iloc[-1])

    return events_log

def perpendicular_distance(p, p1, p2):
    """Calculates perpendicular distance from point p to line segment (p1 -> p2)."""
    x0, y0 = p
    x1, y1 = p1
    x2, y2 = p2
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    den = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return num / den if den > 0 else 0.0

def calculate_metrics(df_mocap, wp_events, column_x, column_y, column_radius, cage_radius):
    """Computes all key collision and trajectory performance metrics for a single flight.
    Analyzes physical clearances, velocities, and commanded tracking errors.
    """
    if df_mocap.empty:
        return {
            'closest_t': 0.0,
            'min_dist_center': 0.0,
            'closest_clearance': 0.0,
            'avg_speed_wp2_wp3': 0.0,
            'max_tracking_error': 0.0,
            'mean_tracking_error': 0.0,
            'max_lateral_displacement': 0.0
        }

    # Segment from WP1 to WP4 (active sweep)
    t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
    t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
    df_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
    if df_sweep.empty:
        df_sweep = df_mocap

    # Calculate closest approach
    dist_to_col_center = np.sqrt((df_sweep['x'] - column_x)**2 + (df_sweep['y'] - column_y)**2)
    min_idx = np.argmin(dist_to_col_center)
    closest_t = df_sweep['t'].iloc[min_idx]
    min_dist_center = dist_to_col_center.iloc[min_idx]
    closest_clearance = min_dist_center - column_radius - cage_radius

    # Segment from WP2 to WP3 (column transit sweep)
    t_wp2 = wp_events.get('WP2', t_start)
    t_wp3 = wp_events.get('WP3', t_end)
    df_wp2_wp3 = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
    if df_wp2_wp3.empty:
        df_wp2_wp3 = df_sweep

    avg_speed = df_wp2_wp3['speed'].mean() if not df_wp2_wp3.empty else 0.0

    # Get WP coordinates or fallbacks
    def get_wp_coords(wp_name, fallback):
        t_wp = wp_events.get(wp_name)
        if t_wp is not None:
            idx = np.searchsorted(df_mocap['t'], t_wp)
            idx = min(max(0, idx), len(df_mocap) - 1)
            return df_mocap['x'].iloc[idx], df_mocap['y'].iloc[idx]
        return fallback

    wp2_pos = get_wp_coords('WP2', (0.100, 1.200))
    wp3_pos = get_wp_coords('WP3', (0.100, -1.200))

    # Perpendicular tracking errors relative to ideal path WP2 -> WP3
    if not df_wp2_wp3.empty:
        errors = [perpendicular_distance((row['x'], row['y']), wp2_pos, wp3_pos) for _, row in df_wp2_wp3.iterrows()]
        mean_err = np.mean(errors)
        max_err = np.max(errors)
    else:
        mean_err = 0.0
        max_err = 0.0

    # Calculate achieved 2D spatial trajectory angle at exact contact point
    achieved_angle = None
    impact_speed = None
    impact_accel = None
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
        
        impact_speed = impact_row.get('speed', 0.0)
        impact_accel = impact_row.get('accel', 0.0)
        
        if r_len > 1e-3 and v_len > 1e-3:
            cos_theta = (rx * vx + ry * vy) / (r_len * v_len)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            achieved_angle = np.degrees(np.arccos(cos_theta))
            if achieved_angle > 90.0:
                achieved_angle = 180.0 - achieved_angle

    return {
        'closest_t': closest_t,
        'min_dist_center': min_dist_center,
        'closest_clearance': closest_clearance,
        'avg_speed_wp2_wp3': avg_speed,
        'max_tracking_error': max_err,
        'mean_tracking_error': mean_err,
        'max_lateral_displacement': max_err,
        'achieved_impact_angle': achieved_angle,
        'impact_speed': impact_speed,
        'impact_accel': impact_accel
    }
