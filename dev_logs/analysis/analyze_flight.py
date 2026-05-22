#!/usr/bin/env python3
import os
import sys
import glob
import time
import numpy as np
from mcap_ros2.reader import read_ros2_messages

def load_drone_name():
    # Try to load name from config, fallback to dorten_drone
    try:
        import json
        config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config", "drone_config.json")
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
                return config.get("drone_name", "dorten_drone")
    except Exception:
        pass
    return "dorten_drone"

def quaternion_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(script_dir))
    flights_dir = os.path.join(project_root, "dev_logs", "flights")
    
    # Check if a custom flight directory or name was provided
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if os.path.exists(target) and os.path.isdir(target):
            latest_folder = os.path.abspath(target)
        else:
            # Try to look it up in flights_dir
            potential_path = os.path.join(flights_dir, target)
            if os.path.exists(potential_path) and os.path.isdir(potential_path):
                latest_folder = potential_path
            else:
                print(f"\033[91m[ERROR] Provided flight path does not exist: {target}\033[0m")
                return
    else:
        # Find the latest descriptive flight folder
        flight_folders = glob.glob(os.path.join(flights_dir, "flight_*"))
        flight_folders = [d for d in flight_folders if os.path.isdir(d) and "px4_sd_logs" not in d]
        
        if not flight_folders:
            print(f"\033[91m[ERROR] No flight directories found in: {flights_dir}\033[0m")
            return
            
        latest_folder = max(flight_folders, key=os.path.getmtime)
        
    folder_name = os.path.basename(latest_folder)
    print(f"\n=============================================================")
    print(f"📊 FLIGHT RECORDER ANALYZER ACTIVE")
    print(f"=============================================================")
    print(f"📂 Folder: {latest_folder}")
    
    # Find .mcap file inside the folder
    mcap_files = glob.glob(os.path.join(latest_folder, "*.mcap"))
    if not mcap_files:
        print(f"\033[91m[ERROR] No .mcap file found inside: {latest_folder}\033[0m")
        return
        
    mcap_path = mcap_files[0]
    print(f"💾 Bag file: {os.path.basename(mcap_path)}")
    print(f"⏳ Reading messages... Please wait.")
    
    # 2. Read messages from the bag
    topic_data = {}
    msg_count = 0
    start_time = time.time()
    
    try:
        for msg in read_ros2_messages(mcap_path):
            topic = msg.channel.topic
            if topic not in topic_data:
                topic_data[topic] = []
            topic_data[topic].append(msg)
            msg_count += 1
    except Exception as e:
        print(f"\033[91m[ERROR] Failed to read bag: {e}\033[0m")
        return
        
    duration = time.time() - start_time
    print(f"✅ Loaded {msg_count} messages across {len(topic_data)} topics in {duration:.1f}s.\n")
    
    if msg_count == 0:
        print("\033[93m[WARN] Bag file is empty. Drone was probably killed instantly.\033[0m")
        return
        
    # Get bag start time
    bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
    def to_rel_time(ns):
        return (ns - bag_start_ns) * 1e-9
        
    # 3. Parse Topics
    poses_msgs = topic_data.get("/poses", [])
    battery_msgs = topic_data.get("/fmu/out/battery_status", [])
    setpoint_msgs = topic_data.get("/fmu/in/trajectory_setpoint", [])
    odom_msgs = topic_data.get("/fmu/out/vehicle_odometry", [])
    
    # --- MoCap Poses extraction ---
    drone_name_target = load_drone_name().lower()
    mocap_t, mocap_x, mocap_y, mocap_z = [], [], [], []
    
    unique_names = set()
    for m in poses_msgs:
        for p in m.ros_msg.poses:
            unique_names.add(p.name)
    if unique_names:
        print(f"📡 Unique rigid bodies found in /poses bag: {list(unique_names)}")
    
    for m in poses_msgs:
        t_rel = to_rel_time(m.log_time_ns)
        found_drone = False
        for p in m.ros_msg.poses:
            p_name = p.name.lower()
            if "column" in p_name or "origin" in p_name:
                continue
            # Match user drone name, or general keywords if custom
            if (drone_name_target in p_name) or any(k in p_name for k in ["jake_drone", "drone_frame", "jake", "drone", "arrow", "frame"]):
                mocap_t.append(t_rel)
                mocap_x.append(p.pose.position.x)
                mocap_y.append(p.pose.position.y)
                mocap_z.append(p.pose.position.z)
                found_drone = True
                break
        # fallback to first pose if not found but we have poses
        if not found_drone and len(m.ros_msg.poses) > 0:
            p = m.ros_msg.poses[0]
            mocap_t.append(t_rel)
            mocap_x.append(p.pose.position.x)
            mocap_y.append(p.pose.position.y)
            mocap_z.append(p.pose.position.z)

    mocap_t = np.array(mocap_t)
    mocap_x = np.array(mocap_x)
    mocap_y = np.array(mocap_y)
    mocap_z = np.array(mocap_z)
    
    # --- Battery status extraction ---
    bat_t, bat_v, bat_pct = [], [], []
    for m in battery_msgs:
        bat_t.append(to_rel_time(m.log_time_ns))
        bat_v.append(m.ros_msg.voltage_v)
        bat_pct.append(m.ros_msg.remaining * 100.0)
    bat_t = np.array(bat_t)
    bat_v = np.array(bat_v)
    bat_pct = np.array(bat_pct)
    
    # --- Trajectory setpoint extraction (Commanded in NED) ---
    sp_t, sp_x, sp_y, sp_z = [], [], [], []
    for m in setpoint_msgs:
        sp_t.append(to_rel_time(m.log_time_ns))
        # Map using the bridge's actual physical transform: X_enu = X_ned, Y_enu = -Y_ned, Z_enu = -Z_ned
        sp_x.append(m.ros_msg.position[0])  # X_ned -> X_enu
        sp_y.append(-m.ros_msg.position[1]) # -Y_ned -> Y_enu
        sp_z.append(-m.ros_msg.position[2]) # -Z_ned -> Z_enu
    sp_t = np.array(sp_t)
    sp_x = np.array(sp_x)
    sp_y = np.array(sp_y)
    sp_z = np.array(sp_z)
    
    # --- Fused Odometry extraction (Estimated in NED) ---
    odom_t, odom_x, odom_y, odom_z = [], [], [], []
    for m in odom_msgs:
        odom_t.append(to_rel_time(m.log_time_ns))
        # Map using the bridge's actual physical transform: X_enu = X_ned, Y_enu = -Y_ned, Z_enu = -Z_ned
        odom_x.append(m.ros_msg.position[0])
        odom_y.append(-m.ros_msg.position[1])
        odom_z.append(-m.ros_msg.position[2])
    odom_t = np.array(odom_t)
    odom_x = np.array(odom_x)
    odom_y = np.array(odom_y)
    odom_z = np.array(odom_z)
    
    # ────────────────────────────────────────────────────────
    # 1. Trajectory and Flight Envelope Report
    # ────────────────────────────────────────────────────────
    print("=" * 60)
    print("📊 1. MOCAP FLIGHT ENVELOPE SUMMARY")
    print("=" * 60)
    if len(mocap_t) > 0:
        flight_duration = mocap_t[-1] - mocap_t[0]
        print(f"  Flight Duration: {flight_duration:.1f}s (based on {len(mocap_t)} tracking samples)")
        print(f"  Takeoff Position: X: {mocap_x[0]:.3f}m | Y: {mocap_y[0]:.3f}m | Z: {mocap_z[0]:.3f}m")
        print(f"  Maximum Height reached: {np.max(mocap_z):.3f}m")
        print(f"  Flight Bounds:")
        print(f"    X (East/West)  : [{np.min(mocap_x):.3f}m to {np.max(mocap_x):.3f}m]")
        print(f"    Y (North/South): [{np.min(mocap_y):.3f}m to {np.max(mocap_y):.3f}m]")
        print(f"    Z (Height)     : [{np.min(mocap_z):.3f}m to {np.max(mocap_z):.3f}m]")
    else:
        print("  \033[91m[ERROR] No /poses matching drone found in the bag file!\033[0m")
        
    # ────────────────────────────────────────────────────────
    # 2. Battery Sag and Capacity Dissection
    # ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("🔋 2. BATTERY VOLTAGE SAG PROFILE")
    print("=" * 60)
    if len(bat_t) > 0:
        v_init = bat_v[0]
        v_min = np.min(bat_v)
        v_max = np.max(bat_v)
        v_final = bat_v[-1]
        pct_final = bat_pct[-1]
        
        sag_max = v_init - v_min
        print(f"  Initial Idle Voltage: {v_init:.2f}V")
        print(f"  Minimum Sag Voltage (Takeoff/Flight): {v_min:.2f}V  (Sag of -{sag_max:.2f}V under load)")
        print(f"  Final Landed Voltage: {v_final:.2f}V ({pct_final:.1f}%)")
        
        if v_min < 21.6:
            print("  Status: ⚠️ Low voltage warning triggered during load!")
        if v_min < 20.4:
            print("  Status: 🔴 Critical Battery Landing triggered!")
        else:
            print("  Status: ✅ Chemical health secure during motor loading.")
    else:
        print("  No battery telemetry captured in bag.")

    # ────────────────────────────────────────────────────────
    # 3. Dynamic Trajectory Tracking Analysis
    # ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("🎯 3. TRAJECTORY TRACKING PERFORMANCE")
    print("=" * 60)
    
    if len(sp_t) > 0 and len(mocap_t) > 0:
        tracking_errors = []
        sync_t = []
        # Interpolate MoCap positions to match Setpoint timestamps for precision correlation
        for i, t_sp in enumerate(sp_t):
            # Only track during the actual active mission flight window (when sp_z > 0.15m)
            if sp_z[i] < 0.15: continue
            
            # Find closest MoCap sample
            m_idx = np.searchsorted(mocap_t, t_sp)
            if m_idx >= len(mocap_t): continue
            
            # commnaded position in ENU
            cmd_pos = np.array([sp_x[i], sp_y[i], sp_z[i]])
            # actual position in MoCap ENU
            act_pos = np.array([mocap_x[m_idx], mocap_y[m_idx], mocap_z[m_idx]])
            
            error = np.linalg.norm(cmd_pos - act_pos)
            tracking_errors.append(error)
            sync_t.append(t_sp)
            
        tracking_errors = np.array(tracking_errors)
        if len(tracking_errors) > 0:
            print(f"  Active Flight Samples: {len(tracking_errors)}")
            print(f"  Mean Tracking Error   : {np.mean(tracking_errors)*100:.1f} cm")
            print(f"  Median Tracking Error : {np.median(tracking_errors)*100:.1f} cm")
            print(f"  Maximum Tracking Error: {np.max(tracking_errors)*100:.1f} cm")
            
            # Window tracking analysis
            print(f"\n  Tracking Error Timeline (relative to bag start):")
            print(f"    {'Time Range':<15} | {'Mean Err (cm)':>14} | {'Max Err (cm)':>14}")
            print(f"    {'-'*50}")
            sync_t = np.array(sync_t)
            for w in np.arange(0, sync_t[-1], 5.0):
                mask = (sync_t >= w) & (sync_t < w+5.0)
                if np.sum(mask) == 0: continue
                err_w = tracking_errors[mask]
                print(f"    {w:>5.1f}s - {w+5.0:<5.1f}s | {np.mean(err_w)*100:>12.1f} cm | {np.max(err_w)*100:>12.1f} cm")
        else:
            print("  Drone did not take off or active setpoints were not found.")
    else:
        print("  Unable to correlate tracking setpoints (Requires active mission flight).")

    # ────────────────────────────────────────────────────────
    # 4. Geofence Breach Forensic
    # ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("🚦 4. GEOFENCE BREACH FORENSIC TELEMETRY")
    print("=" * 60)
    
    # Find the exact moment when the geofence triggered land or when EKF2/Mocap went out
    if len(mocap_t) > 0:
        # Load geofence limits
        x_min, x_max = -1.25, 2.00
        y_min, y_max = -1.30, 1.50
        z_min, z_max = 0.08, 2.20
        
        breach_idx = -1
        for i in range(len(mocap_t)):
            x, y, z = mocap_x[i], mocap_y[i], mocap_z[i]
            if not (x_min <= x <= x_max) or not (y_min <= y <= y_max) or not (z_min <= z <= z_max):
                breach_idx = i
                break
                
        if breach_idx != -1:
            bx, by, bz = mocap_x[breach_idx], mocap_y[breach_idx], mocap_z[breach_idx]
            bt = mocap_t[breach_idx]
            print(f"  Geofence Breach Event: DETECTED at t = {bt:.2f}s")
            print(f"  Exact Coordinates at Breach: X: {bx:.3f}m | Y: {by:.3f}m | Z: {bz:.3f}m")
            
            # Print specific violation details
            print(f"  Violation Audit:")
            if bx < x_min: print(f"    🔴 X West-Wall Breach: {bx:.3f}m < limit {x_min:.2f}m")
            if bx > x_max: print(f"    🔴 X East-Wall Breach: {bx:.3f}m > limit {x_max:.2f}m")
            if by < y_min: print(f"    🔴 Y South-Wall Breach: {by:.3f}m < limit {y_min:.2f}m")
            if by > y_max: print(f"    🔴 Y North-Wall Breach: {by:.3f}m > limit {y_max:.2f}m")
            if bz < z_min: print(f"    🔴 Z Floor Breach: {bz:.3f}m < limit {z_min:.2f}m")
            if bz > z_max: print(f"    🔴 Z Ceiling Breach: {bz:.3f}m > limit {z_max:.2f}m")
            
            # Print immediate velocity vector at breach if possible
            if breach_idx > 0:
                dt = mocap_t[breach_idx] - mocap_t[breach_idx - 1]
                if dt > 0:
                    vx = (bx - mocap_x[breach_idx - 1]) / dt
                    vy = (by - mocap_y[breach_idx - 1]) / dt
                    vz = (bz - mocap_z[breach_idx - 1]) / dt
                    speed = np.sqrt(vx**2 + vy**2 + vz**2)
                    print(f"  Velocity Vector at Breach:")
                    print(f"    Vx: {vx:+.2f} m/s | Vy: {vy:+.2f} m/s | Vz: {vz:+.2f} m/s | Total Speed: {speed:.2f} m/s")
        else:
            print("  ✅ No geofence breach detected in raw MoCap log telemetry.")
    else:
        print("  MoCap logs unavailable to conduct breach forensic.")
        
    # ────────────────────────────────────────────────────────
    # 5. EKF2 vs MoCap Alignment check
    # ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("⚖️ 5. EKF2 ESTIMATE VS MOCAP RAW ALIGNMENT CHECK")
    print("=" * 60)
    if len(odom_t) > 0 and len(mocap_t) > 0:
        print(f"    {'Time':<6} | {'MoCap Pos (ENU)':<22} | {'EKF2 Pos (ENU)':<22} | {'Delta (m)':<8}")
        print(f"    {'-'*65}")
        for t_test in np.arange(0, odom_t[-1], 5.0):
            o_idx = np.searchsorted(odom_t, t_test)
            if o_idx >= len(odom_t): continue
            
            m_idx = np.searchsorted(mocap_t, t_test)
            if m_idx >= len(mocap_t): continue
            
            mx, my, mz = mocap_x[m_idx], mocap_y[m_idx], mocap_z[m_idx]
            ox, oy, oz = odom_x[o_idx], odom_y[o_idx], odom_z[o_idx]
            
            # Delta relative to takeoff positions
            dx = (mx - mocap_x[0]) - (ox - odom_x[0])
            dy = (my - mocap_y[0]) - (oy - odom_y[0])
            dz = (mz - mocap_z[0]) - (oz - odom_z[0])
            delta = np.sqrt(dx**2 + dy**2 + dz**2)
            
            print(f"    {t_test:>5.1f}s | ({mx:>6.2f}, {my:>6.2f}, {mz:>6.2f}) | ({ox:>6.2f}, {oy:>6.2f}, {oz:>6.2f}) | {delta:>6.2f}m")
    else:
        print("  Telemetry incomplete to perform alignment checks.")
        
    # ────────────────────────────────────────────────────────
    # 6. Sensor Latency & EKF2_EV_DELAY Cross-Correlation Calibration
    # ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("⏱️ 6. SENSOR LATENCY / EKF2_EV_DELAY DYNAMIC CALIBRATION")
    print("=" * 60)
    
    vis_msgs = topic_data.get("/fmu/in/vehicle_visual_odometry", [])
    
    if len(vis_msgs) > 10 and len(odom_msgs) > 10:
        # Extract visual yaw and times
        vis_t_list, vis_yaw_list = [], []
        for m in vis_msgs:
            t = to_rel_time(m.log_time_ns)
            yaw = quaternion_to_yaw(m.ros_msg.q)
            vis_t_list.append(t)
            vis_yaw_list.append(yaw)
            
        # Extract fused yaw and times
        odom_t_list, odom_yaw_list = [], []
        for m in odom_msgs:
            t = to_rel_time(m.log_time_ns)
            yaw = quaternion_to_yaw(m.ros_msg.q)
            odom_t_list.append(t)
            odom_yaw_list.append(yaw)
            
        vis_t_arr = np.array(vis_t_list)
        vis_yaw_arr = np.unwrap(np.array(vis_yaw_list))
        
        odom_t_arr = np.array(odom_t_list)
        odom_yaw_arr = np.unwrap(np.array(odom_yaw_list))
        
        # Interpolate onto a common 200Hz grid
        t_min = max(vis_t_arr[0], odom_t_arr[0]) + 1.0 # Skip first second
        t_max = min(vis_t_arr[-1], odom_t_arr[-1]) - 1.0 # Skip last second
        
        if t_max > t_min + 3.0: # Need at least 3 seconds of active data
            t_grid = np.arange(t_min, t_max, 0.005) # 5ms steps (200Hz)
            
            vis_yaw_interp = np.interp(t_grid, vis_t_arr, vis_yaw_arr)
            odom_yaw_interp = np.interp(t_grid, odom_t_arr, odom_yaw_arr)
            
            # Compute yaw velocity (differential) to focus on dynamic turns and eliminate static offsets
            vis_yaw_vel = np.diff(vis_yaw_interp) / 0.005
            odom_yaw_vel = np.diff(odom_yaw_interp) / 0.005
            
            # Keep only segments with meaningful rotation (yaw velocity > 0.05 rad/s)
            active_mask = (np.abs(vis_yaw_vel) > 0.05) | (np.abs(odom_yaw_vel) > 0.05)
            
            if np.sum(active_mask) > 100:
                # Compute cross-correlation for lags between -150ms and +150ms in 5ms increments
                lags_ms = np.arange(-150, 151, 5)
                corrs = []
                
                for lag_ms in lags_ms:
                    lag_steps = int(lag_ms / 5)
                    if lag_steps > 0:
                        v = vis_yaw_vel[lag_steps:]
                        o = odom_yaw_vel[:-lag_steps]
                    elif lag_steps < 0:
                        v = vis_yaw_vel[:lag_steps]
                        o = odom_yaw_vel[-lag_steps:]
                    else:
                        v = vis_yaw_vel
                        o = odom_yaw_vel
                        
                    # Calculate Pearson correlation coefficient
                    corr = np.corrcoef(v, o)[0, 1]
                    corrs.append(corr if not np.isnan(corr) else 0.0)
                    
                best_idx = np.argmax(corrs)
                best_lag = lags_ms[best_idx]
                best_corr = corrs[best_idx]
                
                print(f"  ⚡ Cross-Correlation Latency Analysis Successful!")
                print(f"  --------------------------------------------------")
                print(f"  Optimal Lag Detected: {best_lag:+.1f} ms  (Correlation: {best_corr:.3f})")
                print(f"  Recommended EKF2_EV_DELAY: {best_lag:.1f} ms")
                print(f"  --------------------------------------------------")
                
                # Print a small ascii-art graph of the correlation peaks
                print("  Correlation Profile:")
                for i, lag_ms in enumerate(lags_ms):
                    if lag_ms % 30 == 0: # Print every 30ms
                        bar_len = int((corrs[i] + 1.0) * 15) # Scale to 0-30 characters
                        star = "*" if i == best_idx else "|"
                        print(f"    {lag_ms:>4} ms: {'#' * bar_len}{star}")
            else:
                print("  Insufficient yaw dynamics (drone did not rotate enough) to isolate latency.")
        else:
            print("  Flight duration too short to conduct cross-correlation.")
    else:
        print("  Telemetry incomplete to perform latency checks.")
        
    print("=============================================================\n")

if __name__ == "__main__":
    main()
