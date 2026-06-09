import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline, interp1d

def resample_and_interpolate_mocap(df_mocap, target_freq=100.0, filter_half_window=10):
    """Resamples MoCap coordinates onto a uniform grid using simple linear interpolation.
    Detects dropout gaps and marks the "ringing regions" (gap + SG window margin) so they
    can be surgically removed from the velocity/acceleration arrays later.
    """
    if df_mocap.empty or len(df_mocap) < 4:
        return df_mocap

    # Ensure unique and monotonically sorted timestamps
    df_clean = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)
    t_start = df_clean['t'].min()
    t_end = df_clean['t'].max()

    # Generate perfectly uniform timeline
    dt = 1.0 / target_freq
    t_uniform = np.arange(t_start, t_end + dt/2.0, dt)

    resampled_data = {'t': t_uniform}
    
    # Simple linear interpolation for all columns (preserves raw data feel, no PCHIP jitter)
    for col in df_clean.columns:
        if col != 't':
            try:
                resampled_data[col] = np.interp(t_uniform, df_clean['t'], df_clean[col])
            except Exception:
                resampled_data[col] = df_clean[col].iloc[0]

    df_resampled = pd.DataFrame(resampled_data)
    
    # --- ADAPTIVE JITTER LOGIC ---
    # Rotating Cage has very clean tracking (~7 drops). Fixed Cage has severe jitter (~146 drops).
    raw_t = df_clean['t'].values
    raw_dt = np.diff(raw_t)
    
    strict_drops = np.where(raw_dt > (1.0 / 30.0))[0]
    is_high_jitter = len(strict_drops) > 20

    if is_high_jitter:
        # FIXED CAGE PROFILE: Ignore micro-jitter. Only repair massive structural gaps (> 100ms).
        dropout_indices = np.where(raw_dt > 0.1)[0]
        margin = 2  # Small margin to prevent overlapping repair windows
    else:
        # ROTATING CAGE PROFILE: The exact parameters you approved.
        dropout_indices = strict_drops
        margin = filter_half_window  # Full 9-point margin
    
    # Build a mask of the "ringing regions" (the gap itself + the calculated margin)
    ringing_mask = np.zeros(len(t_uniform), dtype=bool)
    
    for idx in dropout_indices:
        t0, t1 = raw_t[idx], raw_t[idx + 1]
        gap_indices = np.where((t_uniform >= t0) & (t_uniform <= t1))[0]
        if len(gap_indices) == 0:
            continue
        
        start_idx = max(0, gap_indices[0] - margin)
        end_idx = min(len(t_uniform) - 1, gap_indices[-1] + margin)
        
        ringing_mask[start_idx:end_idx + 1] = True

    df_resampled['_ringing_mask'] = ringing_mask
    df_resampled['_is_high_jitter'] = is_high_jitter

    # Resilient fallback for any floating point boundary NaNs
    df_resampled = df_resampled.ffill().bfill()
    return df_resampled

def compute_velocity(df_mocap, window=19, polyorder=3, resample=True, prefilter_position_fc=None):
    """Computes velocity derivatives from position signals.

    Uses standard SG differentiation on linearly interpolated positions.
    To avoid SG filter ringing spikes at dropout gap boundaries, it surgically
    snips out the ringing regions and replaces them with a clean straight line.

    Parameters
    ----------
    prefilter_position_fc : float or None
        If set (e.g. 12.0), applies a 2nd-order Butterworth zero-phase low-pass
        filter to x, y, z positions BEFORE SG differentiation. This band-limits
        the position signal to physically realizable drone dynamics, preventing
        linear interpolation kinks from being amplified into velocity spikes.
        Default None (off) preserves original pipeline behavior.
    """
    if df_mocap.empty or len(df_mocap) < 3:
        df_mocap['vx'] = 0.0
        df_mocap['vy'] = 0.0
        df_mocap['vz'] = 0.0
        df_mocap['speed'] = 0.0
        return df_mocap

    if resample and len(df_mocap) >= 4:
        df_mocap = resample_and_interpolate_mocap(df_mocap, target_freq=100.0, filter_half_window=window//2)

    # Extract adaptive jitter flag
    is_high_jitter = False
    if '_is_high_jitter' in df_mocap.columns:
        is_high_jitter = bool(df_mocap['_is_high_jitter'].iloc[0])
        df_mocap = df_mocap.drop(columns=['_is_high_jitter'])

    # Unique and monotonically sorted
    df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    # === DISABLED: Position Pre-Filtering (2026-06-08) ===
    # Attempt 1 (FAILED): 12 Hz Butterworth position pre-filter before SG diff.
    # The 12 Hz cutoff is too close to the ~10 Hz dropout kink frequency to
    # effectively attenuate them, and relaxing the velocity Butterworth from
    # 4 Hz → 20 Hz removed the only thing that was actually smoothing the data.
    # Result: way worse than original. Keeping code for future experimentation.
    # if prefilter_position_fc is not None:
    #     from scipy.signal import butter, filtfilt
    #     b_pre, a_pre = butter(2, prefilter_position_fc, fs=100.0, btype='low')
    #     for col in ['x', 'y', 'z']:
    #         df_mocap[col] = filtfilt(b_pre, a_pre, df_mocap[col])

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

    # === SG DIFFERENTIATION PATH ===
    # Traditional approach: differentiate position with SG deriv=1
    if adjusted_window > polyorder:
        try:
            vx_filtered = savgol_filter(df_mocap['x'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vy_filtered = savgol_filter(df_mocap['y'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vz_filtered = savgol_filter(df_mocap['z'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
            vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
            vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])
    else:
        vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
        vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
        vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])

    # --- BUTTERWORTH LOW-PASS FILTER FOR HIGH JITTER ---
    if is_high_jitter:
        from scipy.signal import butter, filtfilt
        try:
            # 2nd order Butterworth filter with 4Hz cutoff (fs=100Hz)
            b, a = butter(2, 4.0, fs=100.0, btype='low')
            vx_filtered = filtfilt(b, a, vx_filtered)
            vy_filtered = filtfilt(b, a, vy_filtered)
            vz_filtered = filtfilt(b, a, vz_filtered)
        except Exception as e:
            print(f"[Warning] Failed to apply Butterworth low-pass filter: {e}")

    df_mocap['is_high_jitter'] = is_high_jitter
    df_mocap['_prefilter_fc'] = prefilter_position_fc if prefilter_position_fc is not None else 0.0
    df_mocap['vx'] = vx_filtered
    df_mocap['vy'] = vy_filtered
    df_mocap['vz'] = vz_filtered
    df_mocap['speed'] = np.sqrt(vx_filtered**2 + vy_filtered**2 + vz_filtered**2)
    
    # Compute acceleration: SG deriv=1 on the velocity signal
    if adjusted_window > polyorder:
        try:
            df_mocap['ax'] = savgol_filter(df_mocap['vx'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['ay'] = savgol_filter(df_mocap['vy'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['az'] = savgol_filter(df_mocap['vz'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['accel'] = savgol_filter(df_mocap['speed'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            df_mocap['ax'] = np.gradient(df_mocap['vx'], df_mocap['t'])
            df_mocap['ay'] = np.gradient(df_mocap['vy'], df_mocap['t'])
            df_mocap['az'] = np.gradient(df_mocap['vz'], df_mocap['t'])
            df_mocap['accel'] = np.gradient(df_mocap['speed'], df_mocap['t'])
    else:
        df_mocap['ax'] = np.gradient(df_mocap['vx'], df_mocap['t'])
        df_mocap['ay'] = np.gradient(df_mocap['vy'], df_mocap['t'])
        df_mocap['az'] = np.gradient(df_mocap['vz'], df_mocap['t'])
        df_mocap['accel'] = np.gradient(df_mocap['speed'], df_mocap['t'])

    # === SURGICAL RINGING REMOVAL ===
    # Linearly interpolate over the ringing gap boundaries to remove SG spikes
    if '_ringing_mask' in df_mocap.columns:
        is_valid = ~df_mocap['_ringing_mask'].values
        valid_t = df_mocap['t'].values[is_valid]
        
        if len(valid_t) > 0:
            for col in ['vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel']:
                valid_vals = df_mocap[col].values[is_valid]
                df_mocap[col] = np.interp(df_mocap['t'].values, valid_t, valid_vals)
                
        df_mocap['is_doctored'] = df_mocap['_ringing_mask']
        df_mocap = df_mocap.drop(columns=['_ringing_mask'])
    else:
        df_mocap['is_doctored'] = False
            
    return df_mocap

def compute_ekf_kinematics(df_odom, df_mocap, target_freq=100.0, window=19, polyorder=3):
    """Computes velocity, speed, and acceleration from PX4 EKF odometry (vehicle_odometry).

    The PX4 EKF fuses MoCap position + IMU acceleration at up to 250 Hz, producing
    a velocity estimate that is inherently smooth even during MoCap dropouts.
    This function applies the same NED→ENU coordinate alignment as db_loader.py.

    Parameters
    ----------
    df_odom : pd.DataFrame
        EKF odometry with columns 't', 'vx_ekf_raw', 'vy_ekf_raw', 'vz_ekf_raw'.
        Generated by build_dataframes() in db_loader.py.
    df_mocap : pd.DataFrame
        MoCap DataFrame (used for time baseline / t_start, t_end).
    target_freq : float
        Target resample frequency (default 100 Hz, matching MoCap pipeline).
    window : int
        Savitzky-Golay window length for acceleration differentiation.
    polyorder : int
        Savitzky-Golay polynomial order.

    Returns
    -------
    pd.DataFrame or None
        DataFrame with columns 't', 'vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel'
        on the resampled time grid. Returns None if df_odom is empty or missing velocity columns.
    """
    if df_odom.empty or 'vx_ekf_raw' not in df_odom.columns:
        return None

    # Coordinate alignment: same convention as build_dataframes in db_loader.py
    # db_loader uses: x_ekf=position[0], y_ekf=-position[1], z_ekf=-position[2]
    # For velocity: vx=v[0], vy=-v[1], vz=-v[2]
    df_vel = df_odom[['t', 'vx_ekf_raw', 'vy_ekf_raw', 'vz_ekf_raw']].copy()
    df_vel = df_vel.dropna().sort_values('t').reset_index(drop=True)
    df_vel['vx'] = df_vel['vx_ekf_raw']
    df_vel['vy'] = -df_vel['vy_ekf_raw']
    df_vel['vz'] = -df_vel['vz_ekf_raw']

    # Resample to uniform time grid matching MoCap pipeline
    t_start = df_vel['t'].min()
    t_end = df_vel['t'].max()

    # Use MoCap time bounds if available (to avoid extrapolation)
    if not df_mocap.empty:
        t_start = max(t_start, df_mocap['t'].min())
        t_end = min(t_end, df_mocap['t'].max())

    if t_end <= t_start:
        return None

    dt = 1.0 / target_freq
    t_uniform = np.arange(t_start, t_end + dt / 2.0, dt)

    if len(t_uniform) < 3:
        return None

    # Interpolate EKF velocity onto the uniform grid
    ekf_t = df_vel['t'].values
    vx_uniform = interp1d(ekf_t, df_vel['vx'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)
    vy_uniform = interp1d(ekf_t, df_vel['vy'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)
    vz_uniform = interp1d(ekf_t, df_vel['vz'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)

    speed_uniform = np.sqrt(vx_uniform**2 + vy_uniform**2 + vz_uniform**2)

    # === ACCELERATION: SG deriv=1 on EKF velocity ===
    # Use same SG parameters as compute_velocity() for consistency
    median_dt = np.median(np.diff(t_uniform))
    if median_dt is None or np.isnan(median_dt) or median_dt == 0:
        median_dt = dt

    n_points = len(t_uniform)
    adj_window = window
    if adj_window >= n_points:
        adj_window = n_points // 2 * 2 - 1
        if adj_window < 3:
            adj_window = 3

    if adj_window > polyorder:
        try:
            ax = savgol_filter(vx_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            ay = savgol_filter(vy_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            az = savgol_filter(vz_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            accel = savgol_filter(speed_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            ax = np.gradient(vx_uniform, t_uniform)
            ay = np.gradient(vy_uniform, t_uniform)
            az = np.gradient(vz_uniform, t_uniform)
            accel = np.gradient(speed_uniform, t_uniform)
    else:
        ax = np.gradient(vx_uniform, t_uniform)
        ay = np.gradient(vy_uniform, t_uniform)
        az = np.gradient(vz_uniform, t_uniform)
        accel = np.gradient(speed_uniform, t_uniform)

    df_ekf = pd.DataFrame({
        't': t_uniform,
        'vx': vx_uniform,
        'vy': vy_uniform,
        'vz': vz_uniform,
        'speed': speed_uniform,
        'ax': ax,
        'ay': ay,
        'az': az,
        'accel': accel,
    })
    return df_ekf


def detect_mission_class(df_setpoint, label=None, target_z=0.5):
    """Dynamically determines and instantiates the correct mission class based on label or setpoint coordinates."""
    is_75deg = False
    is_45deg = False
    if label:
        lbl_lower = label.lower()
        if "45" in lbl_lower:
            is_45deg = True
        elif "75" in lbl_lower or "collision" in lbl_lower:
            is_75deg = True

    import sys
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
    if project_root not in sys.path:
        sys.path.append(project_root)

    if is_45deg:
        try:
            from drone_control.missions.exp_collision_45deg import ExpCollision45Deg
            return ExpCollision45Deg(target_z=target_z)
        except Exception:
            pass

    if not is_75deg and not is_45deg:
        try:
            from drone_control.missions.column_sweep_loop import ColumnSweepLoop
            return ColumnSweepLoop(target_z=target_z)
        except Exception:
            return None

    # Check if we should use ExpCollision75DegV2
    use_v2 = False
    if label and "v2" in label.lower():
        use_v2 = True
    elif df_setpoint is not None and not df_setpoint.empty and 'y_cmd' in df_setpoint.columns:
        # Check if Y=1.10 is commanded close to the sweep lane X=0.186
        sp_v2_mask = (df_setpoint['x_cmd'] - 0.186).abs() < 0.05
        if sp_v2_mask.any():
            y_cmds = df_setpoint.loc[sp_v2_mask, 'y_cmd']
            if ((y_cmds - 1.100).abs() < 0.05).any():
                use_v2 = True

    if use_v2:
        try:
            from drone_control.missions.exp_collision_75deg_v2 import ExpCollision75DegV2
            return ExpCollision75DegV2(target_z=target_z)
        except Exception:
            pass
    try:
        from drone_control.missions.exp_collision_75deg import ExpCollision75Deg
        return ExpCollision75Deg(target_z=target_z)
    except Exception:
        return None


def find_waypoint_events(df_mocap, df_setpoint, takeoff_time=None, label=None, column_x=0.408, column_y=0.358, 
                         column_radius=0.045, cage_radius=0.179, return_all=False, dynamic_waypoints=None):
    """Detects all completed passes within predefined waypoint coordinates chronologically,
    returning a list of dictionaries (one for each detected pass) or the first pass dict.
    """
    if df_mocap.empty:
        return [] if return_all else {}

    # Backward compatibility handler for legacy signatures:
    # Some older scripts call find_waypoint_events(df_mocap, takeoff_time, label=...)
    # where df_setpoint is omitted entirely.
    if not isinstance(df_setpoint, pd.DataFrame):
        # The second argument is actually takeoff_time!
        takeoff_time = df_setpoint
        df_setpoint = pd.DataFrame()

    # 1. Determine waypoint sequence from the mission class SSoT.
    # The mission file defines the exact 4 nominal waypoints. Dynamic waypoints
    # extracted from segmented pass MCAPs are unreliable (PX4 trajectory
    # interpolation produces 15+ intermediate setpoints, not the real 4 WPs).
    wp_sequence = None

    # PRIORITY 1: Always try mission class import first (this is the SSoT)
    mission = detect_mission_class(df_setpoint, label, target_z=0.5)
    if mission is not None:
        try:
            # Simulate takeoff to populate the mission waypoints
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
            
            # Check if this is a collision loop or regular loop
            is_collision = False
            if label:
                lbl_lower = label.lower()
                if "collision" in lbl_lower or "75" in lbl_lower or "45" in lbl_lower:
                    is_collision = True

            if is_collision:
                wp_sequence = [
                    ('WP1', (mission.wp_stage[0], mission.wp_stage[1])),
                    ('WP2', (mission.exp_sp[0], mission.exp_sp[1])),
                    ('WP3', (mission.exp_ep[0], mission.exp_ep[1])),
                    ('WP4', (mission.wp3[0], mission.wp3[1]))
                ]
            else:
                wp_sequence = [
                    ('WP1', (mission.wp1[0], mission.wp1[1])),
                    ('WP2', (mission.wp2[0], mission.wp2[1])),
                    ('WP3', (mission.wp3[0], mission.wp3[1])),
                    ('WP4', (mission.wp4[0], mission.wp4[1]))
                ]
        except Exception:
            pass

    # PRIORITY 2: Fall back to dynamic_waypoints ONLY if exactly 4 were found
    # (>4 means PX4 trajectory interpolation noise, not real waypoints)
    if wp_sequence is None and dynamic_waypoints and len(dynamic_waypoints) == 4:
        wp_sequence = []
        for i, (_, coord) in enumerate(dynamic_waypoints):
            wp_sequence.append((f"WP{i+1}", coord))

    # Absolute hardcoded fallback
    if wp_sequence is None:
        if is_collision:
            # Detect which angle from label
            is_45 = label and "45" in label.lower()
            x_lane = 0.248 if is_45 else 0.186
            wp_sequence = [
                ('WP1', (x_lane, 1.200)),
                ('WP2', (x_lane, 0.950)),
                ('WP3', (x_lane, -1.200)),
                ('WP4', (0.000, 0.300))
            ]
        else:
            wp_sequence = [
                ('WP1', (0.000, 1.200)),
                ('WP2', (0.100, 1.200)),
                ('WP3', (0.100, -1.200)),
                ('WP4', (0.000, -1.200))
            ]

    # Use setpoint transitions to identify when the flight director marked waypoints as reached!
    # df_setpoint contains the ACTIVE command. When WP1 is reached, the active command switches to WP2.
    if not df_setpoint.empty and 'x_cmd' in df_setpoint.columns and 'y_cmd' in df_setpoint.columns:
        df_sp = df_setpoint.dropna(subset=['x_cmd', 'y_cmd'])
    else:
        df_sp = pd.DataFrame(columns=['t', 'x_cmd', 'y_cmd'])
    
    # Find ALL loop passes sequentially
    passes_events = []
    current_search_time = takeoff_time
    max_time = df_sp['t'].max() if not df_sp.empty else df_mocap['t'].max()

    while current_search_time < max_time and not df_sp.empty:
        wp_events = {}
        t_ptr = current_search_time
        
        # Sequentially search for transitions between commands in the loop sequence
        found_all = True
        for i, (wp_name, (wx, wy)) in enumerate(wp_sequence):
            # To find when the drone reached `wp_name` (e.g. WP1), we look for when the Flight Director 
            # transitioned to commanding the NEXT waypoint in the sequence!
            if i + 1 < len(wp_sequence):
                next_wp_name, (next_wx, next_wy) = wp_sequence[i + 1]
                # Look for the exact timestamp the command changes to (next_wx, next_wy)
                dist_to_next = np.sqrt((df_sp['x_cmd'] - next_wx)**2 + (df_sp['y_cmd'] - next_wy)**2)
                
                # Match when the commanded setpoint gets within a 5cm mathematical tolerance 
                # (commands are published exactly, but float precision requires small tolerance)
                match_mask = (dist_to_next < 0.05) & (df_sp['t'] > t_ptr)
                if match_mask.any():
                    arrival_t = df_sp.loc[match_mask, 't'].iloc[0]
                    wp_events[wp_name] = arrival_t
                    # Advance search pointer slightly to prevent duplicate matches on same transition
                    t_ptr = arrival_t + 0.1
                else:
                    found_all = False
                    break
            else:
                # For the absolute final waypoint (e.g. WP4), there is no 'next' command in the sequence.
                # Since we only use WP2 -> WP3 for the sweep, we just assign it a dummy completion time.
                wp_events[wp_name] = t_ptr + 2.0
                break
        
        if found_all:
            passes_events.append(wp_events)
            # Advance search pointer past the end of the loop
            current_search_time = wp_events[wp_sequence[-1][0]] + 1.0
        else:
            # If a pass aborted mid-loop, advance search pointer to the NEXT time WP1 is commanded!
            if len(wp_sequence) >= 2:
                # The start of a new loop happens when WP1 (stage) is reached, which means 
                # the command transitions to WP2 (exp_sp).
                next_wx, next_wy = wp_sequence[1][1]
                dist_to_next = np.sqrt((df_sp['x_cmd'] - next_wx)**2 + (df_sp['y_cmd'] - next_wy)**2)
                # Look at least 5.0s ahead to skip the current failed attempt
                next_loop_mask = (dist_to_next < 0.05) & (df_sp['t'] > current_search_time + 5.0)
                if next_loop_mask.any():
                    current_search_time = df_sp.loc[next_loop_mask, 't'].iloc[0] - 0.5
                else:
                    break
            else:
                break

    # Fallback: if we didn't find any fully completed loops via setpoint transitions
    # (very common in truncated/segmented pass files or legacy bags), use optitrack/mocap
    # proximity to detect the single sweep pass.
    if not passes_events and not df_mocap.empty and wp_sequence is not None:
        try:
            # We search for the timestamps where the drone is closest to the nominal WP2 (entry) and WP3 (exit)
            wp2_coords = wp_sequence[1][1]  # WP2 (exp_sp)
            wp3_coords = wp_sequence[2][1]  # WP3 (exp_ep)
            
            dist_to_wp2 = np.sqrt((df_mocap['x'] - wp2_coords[0])**2 + (df_mocap['y'] - wp2_coords[1])**2)
            dist_to_wp3 = np.sqrt((df_mocap['x'] - wp3_coords[0])**2 + (df_mocap['y'] - wp3_coords[1])**2)
            
            t_wp2 = df_mocap.loc[dist_to_wp2.idxmin(), 't']
            t_wp3 = df_mocap.loc[dist_to_wp3.idxmin(), 't']
            
            # Ensure chronological order
            if t_wp2 < t_wp3:
                wp_evs = {
                    'WP1': t_wp2 - 1.0,
                    'WP2': t_wp2,
                    'WP3': t_wp3,
                    'WP4': t_wp3 + 1.0
                }
                passes_events.append(wp_evs)
        except Exception as e:
            print(f"   ⚠️  Fallback pass detection failed: {e}")

    # Post-process: ALWAYS run Column Passed and Column Impact detection for all passes that have both WP2 and WP3
    for wp_evs in passes_events:
        if 'WP2' in wp_evs and 'WP3' in wp_evs:
            t_wp2_cmd = wp_evs['WP2']
            t_wp3 = wp_evs['WP3']
            
            # Refine WP2 to be the actual start of sweep (when it locks and goes) rather than the command transition time.
            df_window = df_mocap[(df_mocap['t'] >= t_wp2_cmd) & (df_mocap['t'] <= t_wp3)]
            if not df_window.empty:
                # Find when the drone crosses Y = 0.70m towards WP3 (Y is decreasing towards WP3 = -1.2)
                cross_mask = df_window['y'] <= 0.70
                t_cross = df_window.loc[cross_mask, 't'].iloc[0] if cross_mask.any() else t_wp3
                
                df_pre_cross = df_window[df_window['t'] <= t_cross]
                if not df_pre_cross.empty:
                    # Find the last time speed is below 0.10 m/s (ignoring NaNs)
                    valid_speeds = df_pre_cross['speed'].dropna()
                    low_speed_mask = valid_speeds < 0.10
                    
                    if low_speed_mask.any():
                        t_start_sweep = df_pre_cross.loc[valid_speeds[low_speed_mask].index[-1], 't']
                    elif not valid_speeds.empty:
                        # Fallback to minimum speed in pre-cross window
                        idx_min = valid_speeds.idxmin()
                        t_start_sweep = df_pre_cross.loc[idx_min, 't']
                    else:
                        # Fallback if entire window is NaN-masked
                        t_start_sweep = df_pre_cross['t'].iloc[-1]
                    
                    # Update WP2 to the refined timestamp
                    wp_evs['WP2'] = t_start_sweep
            
            t_wp2 = wp_evs['WP2']
            sweep_data = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
            if not sweep_data.empty:
                idx_min = (sweep_data['y'] - column_y).abs().idxmin()
                wp_evs['Column Passed'] = sweep_data.loc[idx_min, 't']
                
                # Detect exact Column Impact Event inside this specific pass (closest approach)
                dist_profile = np.sqrt((sweep_data['x'] - column_x)**2 + (sweep_data['y'] - column_y)**2)
                if not dist_profile.empty:
                    idx_min_dist = dist_profile.idxmin()
                    min_dist = dist_profile.loc[idx_min_dist]
                    
                    # Proximity check: ensure the pass is within a close proximity envelope of 0.38m
                    if min_dist <= 0.38:
                        t_geom = sweep_data.loc[idx_min_dist, 't']
                        
                        # Onset of deceleration (speed peak search around closest approach t_geom)
                        window_mask = (sweep_data['t'] >= t_geom - 0.25) & (sweep_data['t'] <= t_geom + 0.1)
                        window_data = sweep_data[window_mask]
                        if not window_data.empty:
                            idx_peak_speed = window_data['speed'].idxmax()
                            wp_evs['Column Impact'] = window_data.loc[idx_peak_speed, 't']
                        else:
                            wp_evs['Column Impact'] = t_geom

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

def calculate_metrics(df_mocap, wp_events, column_x, column_y, column_radius, cage_radius, df_imu=None, df_ekf_kin=None):
    """Computes all key collision and trajectory performance metrics for a single flight.
    Analyzes physical clearances, velocities, and commanded tracking errors.

    Parameters
    ----------
    df_ekf_kin : pd.DataFrame or None
        EKF kinematics DataFrame from compute_ekf_kinematics(). When provided,
        velocity/acceleration values are read from EKF instead of MoCap-derived columns.
        This eliminates Fixed Cage dropout kinks in velocity and acceleration metrics.
        Default None = use MoCap-derived velocity (original pipeline behavior).
    """
    if df_mocap.empty:
        return {
            'closest_t': 0.0,
            'min_dist_center': 0.0,
            'closest_clearance': 0.0,
            'avg_speed_wp2_wp3': 0.0,
            'max_tracking_error': 0.0,
            'mean_tracking_error': 0.0,
            'max_lateral_displacement': 0.0,
            'imu_peak_accel': None, 'imu_peak_accel_x': None, 'imu_peak_accel_y': None, 'imu_peak_accel_z': None,
            'imu_peak_gyro': None, 'imu_peak_gyro_x': None, 'imu_peak_gyro_y': None, 'imu_peak_gyro_z': None,
            'imu_accel_energy': None, 'imu_accel_energy_x': None, 'imu_accel_energy_y': None, 'imu_accel_energy_z': None,
            'imu_gyro_energy': None, 'imu_gyro_energy_x': None, 'imu_gyro_energy_y': None, 'imu_gyro_energy_z': None,
            'imu_accel_settling': None, 'imu_gyro_settling': None
        }

    # Segment from WP1 to WP4 (active sweep)
    t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
    t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
    df_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
    if df_sweep.empty:
        df_sweep = df_mocap

    # === ENABLED: EKF Velocity (2026-06-09) ===
    # When df_ekf_kin is provided, replace MoCap-derived velocity/acceleration
    # columns with EKF-based values interpolated onto the MoCap time grid.
    # EKF velocity is inherently smooth — PX4 fuses IMU at 100-250 Hz to bridge
    # MoCap dropouts, eliminating Fixed Cage kinks without post-processing.
    if df_ekf_kin is not None and not df_ekf_kin.empty:
        ekf_t = df_ekf_kin['t'].values
        mocap_t = df_mocap['t'].values
        for col in ['vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel']:
            if col in df_ekf_kin.columns:
                df_mocap[col] = np.interp(mocap_t, ekf_t, df_ekf_kin[col].values,
                                          left=np.nan, right=np.nan)
        # Fallback: forward-fill any NaN edges
        df_mocap = df_mocap.ffill().bfill()
        # Re-compute df_sweep and df_wp2_wp3 with updated columns
        df_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
        if df_sweep.empty:
            df_sweep = df_mocap
    # === RETIRED: MoCap-derived velocity (used when df_ekf_kin=None) ===
    # Original pipeline: compute_velocity() SG-differentiates MoCap positions.
    # Works well for Rotating Cage (clean 120 Hz MoCap) but produces
    # dropout kinks for Fixed Cage (as low as ~10 Hz).
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
    # Perpendicular tracking errors relative to ideal path WP2 -> WP3
    if not df_wp2_wp3.empty:
        errors = [perpendicular_distance((row['x'], row['y']), wp2_pos, wp3_pos) for _, row in df_wp2_wp3.iterrows()]
        mean_err = np.mean(errors)
        max_err = np.max(errors)
        sdld = np.std(errors) * 1000.0  # mm
    else:
        mean_err = 0.0
        max_err = 0.0
        sdld = 0.0

    # Calculate speed, accel, and before_impact_accel at closest approach (always available)
    achieved_angle = None
    impact_speed = None
    impact_accel = None
    before_impact_accel = None
    
    if not df_mocap.empty:
        # Always compute speed and accel at closest_t (the point of minimum clearance)
        idx_closest = (df_mocap['t'] - closest_t).abs().idxmin()
        closest_row = df_mocap.iloc[idx_closest]
        impact_speed = closest_row.get('speed', 0.0)
        impact_accel = closest_row.get('accel', 0.0)
        
        # before_impact_accel: average accel in window [closest_t - 0.4, closest_t - 0.2]
        window_before = df_mocap[(df_mocap['t'] >= closest_t - 0.4) & (df_mocap['t'] <= closest_t - 0.2)]
        if not window_before.empty:
            before_impact_accel = window_before['accel'].mean()
    
    # Calculate achieved 2D spatial trajectory angle at contact point (if Column Impact detected)
    impact_t = wp_events.get('Column Impact')
    ref_t = impact_t if impact_t is not None else closest_t
    if not df_mocap.empty:
        idx_ref = (df_mocap['t'] - ref_t).abs().idxmin()
        ref_row = df_mocap.iloc[idx_ref]
        rx = column_x - ref_row['x']
        ry = column_y - ref_row['y']
        r_len = np.sqrt(rx**2 + ry**2)
        vx = ref_row.get('vx', 0.0)
        vy = ref_row.get('vy', -1.0)
        v_len = np.sqrt(vx**2 + vy**2)
        
        if r_len > 1e-3 and v_len > 1e-3:
            cos_theta = (rx * vx + ry * vy) / (r_len * v_len)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            achieved_angle = np.degrees(np.arccos(cos_theta))
            if achieved_angle > 90.0:
                achieved_angle = 180.0 - achieved_angle

    # Find active recovery segment (after collision/closest approach until WP3)
    t_collision = wp_events.get('Column Impact', closest_t)
    df_recovery = df_wp2_wp3[df_wp2_wp3['t'] >= t_collision]
    
    avg_dev_after = 0.0
    max_dev_after = 0.0
    recovery_area = 0.0

    if not df_recovery.empty and len(df_recovery) >= 2:
        # Calculate perpendicular distances for each point in recovery
        d_recovery = [perpendicular_distance((row['x'], row['y']), wp2_pos, wp3_pos) for _, row in df_recovery.iterrows()]
        avg_dev_after = np.mean(d_recovery) * 1000.0  # mm
        max_dev_after = np.max(d_recovery) * 1000.0  # mm
        
        # Calculate recovery area using spatial trapezoidal integration along the nominal path!
        dx_line = wp3_pos[0] - wp2_pos[0]
        dy_line = wp3_pos[1] - wp2_pos[1]
        line_len = np.sqrt(dx_line**2 + dy_line**2)
        if line_len > 0:
            ux = dx_line / line_len
            uy = dy_line / line_len
            # Project each recovery point onto the nominal line to get the travel distance s from WP2
            s_recovery = []
            for _, row in df_recovery.iterrows():
                ap_x = row['x'] - wp2_pos[0]
                ap_y = row['y'] - wp2_pos[1]
                s = ap_x * ux + ap_y * uy
                s_recovery.append(s)
            
            s_recovery = np.array(s_recovery)
            d_recovery = np.array(d_recovery)
            # Sort by s to ensure monotonic integration along the path of travel
            sort_idx = np.argsort(s_recovery)
            s_sorted = s_recovery[sort_idx]
            d_sorted = d_recovery[sort_idx]
            
            # Trapezoidal integration along nominal path of travel
            recovery_area = float(np.trapz(d_sorted, s_sorted) * 1000.0)  # mm * m
            
    # Structural IMU kinematics & energy calculations (SSoT Section 4)
    imu_metrics = {
        'imu_peak_accel': None, 'imu_peak_accel_x': None, 'imu_peak_accel_y': None, 'imu_peak_accel_z': None,
        'imu_peak_gyro': None, 'imu_peak_gyro_x': None, 'imu_peak_gyro_y': None, 'imu_peak_gyro_z': None,
        'imu_accel_energy': None, 'imu_accel_energy_x': None, 'imu_accel_energy_y': None, 'imu_accel_energy_z': None,
        'imu_gyro_energy': None, 'imu_gyro_energy_x': None, 'imu_gyro_energy_y': None, 'imu_gyro_energy_z': None,
        'imu_accel_settling': None, 'imu_gyro_settling': None,
        'imu_ax_spread_impact': None, 'imu_ay_spread_impact': None, 'imu_az_spread_impact': None,
        'imu_ax_spread_regular': None, 'imu_ay_spread_regular': None, 'imu_az_spread_regular': None
    }

    if df_imu is not None and not df_imu.empty:
        t_impact = wp_events.get('Column Impact', closest_t)
        # Reverted back to MoCap closest-approach alignment as requested by user


        # 1. Slice contact window strictly to [t_impact - 0.05, t_impact + 0.35]
        df_contact = df_imu[(df_imu['t'] >= t_impact - 0.05) & (df_imu['t'] <= t_impact + 0.35)]
        if df_contact.empty:
            df_contact = df_imu
            
        # 1b. Slice regular flight window strictly to [t_impact - 1.05, t_impact - 0.05]
        df_regular = df_imu[(df_imu['t'] >= t_impact - 1.05) & (df_imu['t'] < t_impact - 0.05)]

        # 2. Peak linear and rotational components
        imu_metrics['imu_peak_accel'] = float(df_contact['a_deviation'].max())
        imu_metrics['imu_peak_accel_x'] = float(df_contact['ax'].abs().max())
        imu_metrics['imu_peak_accel_y'] = float(df_contact['ay'].abs().max())
        imu_metrics['imu_peak_accel_z'] = float((df_contact['az'] + 9.81).abs().max())

        imu_metrics['imu_peak_gyro'] = float(df_contact['g_mag'].max())
        imu_metrics['imu_peak_gyro_x'] = float(df_contact['gx'].abs().max())
        imu_metrics['imu_peak_gyro_y'] = float(df_contact['gy'].abs().max())
        
        # Calculate IMU Spreads (Standard Deviation)
        if not df_contact.empty:
            imu_metrics['imu_ax_spread_impact'] = float(df_contact['ax'].std() / 9.81)
            imu_metrics['imu_ay_spread_impact'] = float(df_contact['ay'].std() / 9.81)
            imu_metrics['imu_az_spread_impact'] = float((df_contact['az'] + 9.81).std() / 9.81)
            
        if not df_regular.empty:
            imu_metrics['imu_ax_spread_regular'] = float(df_regular['ax'].std() / 9.81)
            imu_metrics['imu_ay_spread_regular'] = float(df_regular['ay'].std() / 9.81)
            imu_metrics['imu_az_spread_regular'] = float((df_regular['az'] + 9.81).std() / 9.81)
        imu_metrics['imu_peak_gyro_z'] = float(df_contact['gz'].abs().max())

        # 3. Trapezoidal integration over the contact window
        if len(df_contact) >= 2:
            t_vals = df_contact['t'].values
            imu_metrics['imu_accel_energy'] = float(np.trapz(df_contact['a_deviation'].values, t_vals))
            imu_metrics['imu_accel_energy_x'] = float(np.trapz(df_contact['ax'].abs().values, t_vals))
            imu_metrics['imu_accel_energy_y'] = float(np.trapz(df_contact['ay'].abs().values, t_vals))
            imu_metrics['imu_accel_energy_z'] = float(np.trapz((df_contact['az'] + 9.81).abs().values, t_vals))

            imu_metrics['imu_gyro_energy'] = float(np.trapz(df_contact['g_mag'].values, t_vals))
            imu_metrics['imu_gyro_energy_x'] = float(np.trapz(df_contact['gx'].abs().values, t_vals))
            imu_metrics['imu_gyro_energy_y'] = float(np.trapz(df_contact['gy'].abs().values, t_vals))
            imu_metrics['imu_gyro_energy_z'] = float(np.trapz(df_contact['gz'].abs().values, t_vals))
        else:
            imu_metrics['imu_accel_energy'] = 0.0
            imu_metrics['imu_accel_energy_x'] = 0.0
            imu_metrics['imu_accel_energy_y'] = 0.0
            imu_metrics['imu_accel_energy_z'] = 0.0
            imu_metrics['imu_gyro_energy'] = 0.0
            imu_metrics['imu_gyro_energy_x'] = 0.0
            imu_metrics['imu_gyro_energy_y'] = 0.0
            imu_metrics['imu_gyro_energy_z'] = 0.0

        # 4. Settling times starting from t_impact
        df_after = df_imu[df_imu['t'] >= t_impact]
        if not df_after.empty:
            high_accel_times = df_after[df_after['a_deviation'] >= 1.5]['t']
            if not high_accel_times.empty:
                imu_metrics['imu_accel_settling'] = float(high_accel_times.max() - t_impact)
            else:
                imu_metrics['imu_accel_settling'] = 0.0

            high_gyro_times = df_after[df_after['g_mag'] >= 0.5]['t']
            if not high_gyro_times.empty:
                imu_metrics['imu_gyro_settling'] = float(high_gyro_times.max() - t_impact)
            else:
                imu_metrics['imu_gyro_settling'] = 0.0

            # 5. Vibration spread over post-impact stabilization window [t_impact + 0.2, t_impact + 3.0]
            # We skip the first 0.2s to isolate the post-impact vibration tail from the initial shock spike
            df_vib = df_imu[(df_imu['t'] >= t_impact + 0.2) & (df_imu['t'] <= t_impact + 3.0)]
            if not df_vib.empty and len(df_vib) >= 5:
                imu_metrics['imu_vib_ax'] = float(df_vib['ax'].std())
                imu_metrics['imu_vib_ay'] = float(df_vib['ay'].std())
                imu_metrics['imu_vib_az'] = float((df_vib['az'] + 9.81).std())
                imu_metrics['imu_vib_gx'] = float(df_vib['gx'].std())
                imu_metrics['imu_vib_gy'] = float(df_vib['gy'].std())
                imu_metrics['imu_vib_gz'] = float(df_vib['gz'].std())
            else:
                imu_metrics['imu_vib_ax'] = 0.0
                imu_metrics['imu_vib_ay'] = 0.0
                imu_metrics['imu_vib_az'] = 0.0
                imu_metrics['imu_vib_gx'] = 0.0
                imu_metrics['imu_vib_gy'] = 0.0
                imu_metrics['imu_vib_gz'] = 0.0
        else:
            imu_metrics['imu_accel_settling'] = 0.0
            imu_metrics['imu_gyro_settling'] = 0.0
            imu_metrics['imu_vib_ax'] = 0.0
            imu_metrics['imu_vib_ay'] = 0.0
            imu_metrics['imu_vib_az'] = 0.0
            imu_metrics['imu_vib_gx'] = 0.0
            imu_metrics['imu_vib_gy'] = 0.0
            imu_metrics['imu_vib_gz'] = 0.0

    return {
        'closest_t': closest_t,
        'min_dist_center': min_dist_center,
        'closest_clearance': closest_clearance,
        'avg_speed_wp2_wp3': avg_speed,
        'max_tracking_error': max_err,
        'mean_tracking_error': mean_err,
        'max_lateral_displacement': max_err,
        'path_spread_sdld': sdld,
        'achieved_impact_angle': achieved_angle,
        'impact_speed': impact_speed,
        'impact_accel': impact_accel,
        'before_impact_accel': before_impact_accel,
        'avg_dev_after': avg_dev_after,
        'max_dev_after': max_dev_after,
        'recovery_area': recovery_area,

        # 18 new IMU metrics
        'imu_peak_accel': imu_metrics['imu_peak_accel'],
        'imu_peak_accel_x': imu_metrics['imu_peak_accel_x'],
        'imu_peak_accel_y': imu_metrics['imu_peak_accel_y'],
        'imu_peak_accel_z': imu_metrics['imu_peak_accel_z'],
        'imu_peak_gyro': imu_metrics['imu_peak_gyro'],
        'imu_peak_gyro_x': imu_metrics['imu_peak_gyro_x'],
        'imu_peak_gyro_y': imu_metrics['imu_peak_gyro_y'],
        'imu_peak_gyro_z': imu_metrics['imu_peak_gyro_z'],
        'imu_accel_energy': imu_metrics['imu_accel_energy'],
        'imu_accel_energy_x': imu_metrics['imu_accel_energy_x'],
        'imu_accel_energy_y': imu_metrics['imu_accel_energy_y'],
        'imu_accel_energy_z': imu_metrics['imu_accel_energy_z'],
        'imu_gyro_energy': imu_metrics['imu_gyro_energy'],
        'imu_gyro_energy_x': imu_metrics['imu_gyro_energy_x'],
        'imu_gyro_energy_y': imu_metrics['imu_gyro_energy_y'],
        'imu_gyro_energy_z': imu_metrics['imu_gyro_energy_z'],
        'imu_accel_settling': imu_metrics['imu_accel_settling'],
        'imu_gyro_settling': imu_metrics['imu_gyro_settling'],
        'imu_vib_ax': imu_metrics.get('imu_vib_ax', 0.0),
        'imu_vib_ay': imu_metrics.get('imu_vib_ay', 0.0),
        'imu_vib_az': imu_metrics.get('imu_vib_az', 0.0),
        'imu_vib_gx': imu_metrics.get('imu_vib_gx', 0.0),
        'imu_vib_gy': imu_metrics.get('imu_vib_gy', 0.0),
        'imu_vib_gz': imu_metrics.get('imu_vib_gz', 0.0),
        
        # New IMU acceleration spread metrics
        'imu_ax_spread_impact': imu_metrics.get('imu_ax_spread_impact'),
        'imu_ay_spread_impact': imu_metrics.get('imu_ay_spread_impact'),
        'imu_az_spread_impact': imu_metrics.get('imu_az_spread_impact'),
        'imu_ax_spread_regular': imu_metrics.get('imu_ax_spread_regular'),
        'imu_ay_spread_regular': imu_metrics.get('imu_ay_spread_regular'),
        'imu_az_spread_regular': imu_metrics.get('imu_az_spread_regular')
    }
