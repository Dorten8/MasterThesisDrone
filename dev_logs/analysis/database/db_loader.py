import os
import glob
import time
import json
import pandas as pd
import numpy as np
from mcap_ros2.reader import read_ros2_messages

def load_drone_metadata(project_root):
    """Load configured primary tracker name from drone_config.json"""
    try:
        config_path = os.path.join(project_root, "config", "drone_config.json")
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
                for body in config.get("tracked_bodies", []):
                    if body.get("role") == "primary":
                        return body.get("name", "jake_drone_frame_01"), config
    except Exception as e:
        print(f"[WARN] Failed to load config/drone_config.json: {e}")
    return "jake_drone_frame_01", {}

def load_mcap(flight_path):
    """Finds and parses the first .mcap file inside flight_path (if directory), or parses the file directly."""
    # MCAP is a performant, indexed robotics log container superceding ROS 1 .bag.
    # We accept either a direct .mcap path or a flight directory and glob for the first
    # .mcap whose filename does NOT contain "-pass" — those are sliced sub-bags produced
    # by the segmenter, and we always want the full-flight raw log here.
    if os.path.isfile(flight_path):
        mcap_path = flight_path
    else:
        mcap_files = [f for f in glob.glob(os.path.join(flight_path, "*.mcap")) if "-pass" not in os.path.basename(f)]
        if not mcap_files:
            raise FileNotFoundError(f"No .mcap files discovered in: {flight_path}")
        mcap_path = mcap_files[0]

    # Streaming reader: read_ros2_messages() yields one deserialised ROS 2 message at a time.
    # We accumulate EVERY message into a topic-keyed dictionary — the entire flight is kept
    # in memory because downstream analytics (impact detection, IMU, EKF alignment) cross-
    # reference multiple topics and time ranges arbitrarily.
    topic_data = {}
    msg_count = 0

    for msg in read_ros2_messages(mcap_path):
        topic = msg.channel.topic
        if topic not in topic_data:
            topic_data[topic] = []
        topic_data[topic].append(msg)
        msg_count += 1

    if msg_count == 0:
        raise ValueError(f"No messages read from MCAP: {mcap_path}")

    # bag_start_ns: the earliest nanosecond log_time across ALL topics.  This becomes the
    # zero-reference for converting every message's absolute log_time_ns into relative
    # flight-seconds (t=0 at first recorded message), making time axes comparable across
    # flights regardless of when the logger was started.
    bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
    return topic_data, bag_start_ns

def build_dataframes(topic_data, drone_tracker_name, bag_start_ns):
    """Extracts ROS 2 topics into clean pandas DataFrames, returning a dictionary of dataframes."""
    def to_rel_time(ns):
        return (ns - bag_start_ns) * 1e-9

    poses_msgs = topic_data.get("/poses", [])
    battery_msgs = topic_data.get("/fmu/out/battery_status", [])
    setpoint_msgs = topic_data.get("/fmu/in/trajectory_setpoint", [])
    odom_msgs = topic_data.get("/fmu/out/vehicle_odometry", [])
    status_msgs = topic_data.get("/fmu/out/vehicle_status", []) or topic_data.get("/fmu/out/vehicle_status_v1", [])
    imu_msgs = topic_data.get("/fmu/out/sensor_combined", [])

    # ── MoCap pose extraction ──────────────────────────────────────────────
    # The /poses topic carries an array of named rigid-body poses from the
    # motion-capture system.  Each message MAY contain multiple bodies; we
    # filter down to the single body whose .name matches the configured
    # primary drone tracker (e.g. "jake_drone_frame_01"), taking only the
    # first matching pose per message (break after one hit).
    mocap_list = []
    for m in poses_msgs:
        t_rel = to_rel_time(m.log_time_ns)
        for p in m.ros_msg.poses:
            if p.name.lower() == drone_tracker_name.lower():
                mocap_list.append({
                    't': t_rel,
                    'x': p.pose.position.x,
                    'y': p.pose.position.y,
                    'z': p.pose.position.z
                })
                break

    df_mocap = pd.DataFrame(mocap_list)
    if df_mocap.empty:
        # Fallback keyword sweep — the configured tracker name was not found
        # in any /poses message, so we broaden to any body whose name contains
        # a known stem ("drone", "jake", "frame", "arrow").  Still one hit per
        # message to avoid duplicating the same time sample.
        mocap_list = []
        for m in poses_msgs:
            t_rel = to_rel_time(m.log_time_ns)
            if len(m.ros_msg.poses) > 0:
                for p in m.ros_msg.poses:
                    if any(k in p.name.lower() for k in ["drone", "jake", "frame", "arrow"]):
                        mocap_list.append({
                            't': t_rel,
                            'x': p.pose.position.x,
                            'y': p.pose.position.y,
                            'z': p.pose.position.z
                        })
                        break
        df_mocap = pd.DataFrame(mocap_list)

    if not df_mocap.empty:
        # Remove duplicate timestamps (camera frames may double-report) and
        # guarantee a monotonically increasing time index.
        df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    # ── Dynamic MoCap rate estimation ──────────────────────────────────────
    # Rather than assuming a fixed 240 Hz, we compute the empirical frame rate
    # from the inter-sample time deltas.  Using the median dt (not mean) makes
    # the estimate robust against occasional dropped frames or logging jitter.
    # The reciprocal 1 / median_dt yields the effective Hz, rounded to 0.1 Hz.
    mocap_rate = 240.0 # Default fallback
    if not df_mocap.empty and len(df_mocap) > 1:
        dts = df_mocap['t'].diff().dropna()
        median_dt = dts.median()
        if median_dt > 0:
            mocap_rate = float(round(1.0 / median_dt, 1))

    # ── Column / obstacle MoCap tracking ───────────────────────────────────
    # The obstacle column also has a rigid-body marker set; we extract its
    # pose from the same /poses topic by looking for body names containing
    # "column" (or the exact legacy name "jake_column_drone").  This gives us
    # the ground-truth position of the obstacle for clearance calculations.
    column_list = []
    for m in poses_msgs:
        t_rel = to_rel_time(m.log_time_ns)
        for p in m.ros_msg.poses:
            if "column" in p.name.lower() or p.name.lower() == "jake_column_drone":
                column_list.append({
                    't': t_rel,
                    'x': p.pose.position.x,
                    'y': p.pose.position.y,
                    'z': p.pose.position.z
                })
                break
    df_column = pd.DataFrame(column_list)
    if not df_column.empty:
        df_column = df_column.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    # ── Battery status ─────────────────────────────────────────────────────
    # PX4 publishes battery_status at ~1-2 Hz.  We extract the terminal
    # voltage (voltage_v) and the remaining capacity fraction.  The remaining
    # field is normalised [0, 1] so we scale by 100 to get a percentage.
    bat_list = []
    for m in battery_msgs:
        bat_list.append({
            't': to_rel_time(m.log_time_ns),
            'voltage': m.ros_msg.voltage_v,
            'remaining': m.ros_msg.remaining * 100.0
        })
    df_bat = pd.DataFrame(bat_list)

    # ── EKF odometry (vehicle_odometry) ────────────────────────────────────
    # PX4's EKF publishes position and velocity in the NED (North-East-Down)
    # local frame.  MoCap uses ENU (East-North-Up), the standard frame for
    # indoor robotics at the Flying Arena.  We apply a sign-flip coordinate
    # alignment ON READ so downstream code works entirely in ENU:
    #   x_ekf = position[0]        (North → X, unchanged)
    #   y_ekf = -position[1]       (East → Y, negated to align ENU East)
    #   z_ekf = -position[2]       (Down → Z, negated to align ENU Up)
    # Velocity is kept raw-NED here and aligned downstream wherever needed.
    odom_list = []
    for m in odom_msgs:
        odom_list.append({
            't': to_rel_time(m.log_time_ns),
            'x_ekf': m.ros_msg.position[0],
            'y_ekf': -m.ros_msg.position[1],
            'z_ekf': -m.ros_msg.position[2],
            # EKF velocity (raw NED — coordinate alignment applied downstream)
            'vx_ekf_raw': m.ros_msg.velocity[0],
            'vy_ekf_raw': m.ros_msg.velocity[1],
            'vz_ekf_raw': m.ros_msg.velocity[2]
        })
    df_odom = pd.DataFrame(odom_list)

    # ── Estimate EKF-to-MoCap coordinate alignment offset ──────────────────
    # The EKF local frame origin is set at arming position (GPS home), while
    # the MoCap ENU origin is a fixed corner of the Flying Arena.  We need the
    # rigid translation between the two frames so we can transform setpoint
    # commands (expressed in EKF-local) into MoCap ENU for spatial analysis.
    #
    # Method: pd.merge_asof joins EKF and MoCap rows on the nearest timestamp
    # (direction='nearest'), tolerating the different publish rates (~100 Hz
    # EKF vs ~240 Hz MoCap).  We then compute the median difference across the
    # full flight — the MEDIAN (not mean) is robust to occasional outliers from
    # brief tracking dropouts or EKF resets.
    #   transform_offset_x = median(x_ekf - x_mocap)  → EKF_x = MoCap_x + offset
    #   transform_offset_y = median(-y_ekf + y_mocap) → accounts for sign flip
    transform_offset_x = 0.0
    transform_offset_y = 0.0
    has_alignment = False

    if not df_mocap.empty and not df_odom.empty:
        # Merge EKF and MoCap on nearest time
        df_merged = pd.merge_asof(
            df_mocap[['t', 'x', 'y']].sort_values('t'),
            df_odom[['t', 'x_ekf', 'y_ekf']].sort_values('t'),
            on='t',
            direction='nearest'
        )
        if not df_merged.empty:
            # EKF_x = MoCap_x + offset_x  => offset_x = EKF_x - MoCap_x
            # EKF_y = -MoCap_y + offset_y => offset_y = EKF_y + MoCap_y
            transform_offset_x = (df_merged['x_ekf'] - df_merged['x']).median()
            transform_offset_y = (-df_merged['y_ekf'] + df_merged['y']).median()
            has_alignment = True

    # ── Setpoint (trajectory_setpoint) ─────────────────────────────────────
    # The flight director publishes commanded position targets in the EKF local
    # frame (NED, origin at arming location).  To plot these against MoCap
    # ground truth we must transform into the MoCap ENU frame by subtracting
    # the estimated alignment offset.
    #
    # With alignment:
    #   x_cmd = position[0] - offset_x      (North → X, shifted to ENU origin)
    #   y_cmd = -(position[1] - offset_y)   (East → Y, negated + shifted)
    #   z_cmd = -position[2]                (Down → Up, sign flip only)
    #
    # Without alignment (legacy fallback):
    #   x_cmd = position[1]                 (NED East  → ENU X)
    #   y_cmd = position[0]                 (NED North → ENU Y)
    sp_list = []
    for m in setpoint_msgs:
        if has_alignment:
            # Command conversion from flight director EKF local frame back to MoCap ENU frame:
            # x_cmd = msg.position[0] - offset_x
            # y_cmd = -(msg.position[1] - offset_y)
            x_cmd = m.ros_msg.position[0] - transform_offset_x
            y_cmd = -(m.ros_msg.position[1] - transform_offset_y)
        else:
            # Fallback for old/legacy bags without MoCap/Odom alignment
            x_cmd = m.ros_msg.position[1]  # NED East -> ENU X
            y_cmd = m.ros_msg.position[0]  # NED North -> ENU Y

        sp_list.append({
            't': to_rel_time(m.log_time_ns),
            'x_cmd': x_cmd,
            'y_cmd': y_cmd,
            'z_cmd': -m.ros_msg.position[2]  # NED Down -> ENU -Z (Up)
        })
    df_setpoint = pd.DataFrame(sp_list)

    # ── Dynamic waypoint extraction ────────────────────────────────────────
    # The setpoint stream is a continuous 100 Hz position command; waypoints
    # are not explicitly labeled.  We detect them by watching for CHANGES in
    # the commanded (x, y) target.  Three stages:
    #
    #   1. Round coordinates to 2 decimal places (cm precision) — this clusters
    #      noisy samples around the same target into a single "plateau".
    #   2. Walk the time series; whenever the rounded coordinate changes, we
    #      have a candidate waypoint transition.
    #   3. Filter candidates:
    #      - Skip the takeoff pad at (0, 0) with a 0.15 m tolerance.
    #      - Deduplicate against already-accepted waypoints using a 0.15 m
    #        Euclidean radius — if the candidate is within 15 cm of an existing
    #        waypoint, it is the same target re-visited.
    dynamic_waypoints = []
    if not df_setpoint.empty:
        # Round commanded coordinates to filter noise during transit transitions
        df_sp_active = df_setpoint.dropna(subset=['x_cmd', 'y_cmd']).copy()
        df_sp_active = df_sp_active[~np.isnan(df_sp_active['x_cmd']) & ~np.isnan(df_sp_active['y_cmd'])]

        # Rounding coordinates to 2 decimal places to cluster waypoint targets
        df_sp_active['xr'] = df_sp_active['x_cmd'].round(2)
        df_sp_active['yr'] = df_sp_active['y_cmd'].round(2)

        # Find distinct transition targets
        last_coord = None
        for _, row in df_sp_active.iterrows():
            coord = (row['xr'], row['yr'])
            if coord != last_coord:
                # Keep if it represents a significant waypoint target (not takeoff pad at origin 0,0)
                is_takeoff_pad = (abs(row['x_cmd']) < 0.15 and abs(row['y_cmd']) < 0.15)
                is_duplicate = False
                for _, wp in dynamic_waypoints:
                    dist = np.sqrt((row['x_cmd'] - wp[0])**2 + (row['y_cmd'] - wp[1])**2)
                    if dist < 0.15:
                        is_duplicate = True
                        break
                if not is_takeoff_pad and not is_duplicate:
                    wp_label = f"WP{len(dynamic_waypoints)+1}"
                    dynamic_waypoints.append((wp_label, (row['x_cmd'], row['y_cmd'])))
                last_coord = coord

    # ── Arming / disarming detection from vehicle_status ───────────────────
    # PX4's vehicle_status message carries arming_state as an integer:
    #   2 = ARMED  (motors can spin, flight controller active)
    #   1 = DISARMED
    # We scan the message stream in time order, capturing the FIRST transition
    # to each state.  The guards (arming_time is None, disarming_time is None)
    # ensure we only record the initial arm and the first subsequent disarm.
    # If no arm event is found (should not happen for flight logs), we default
    # to t=0 so downstream slicing does not break.
    arming_time = None
    disarming_time = None
    for m in status_msgs:
        t_evt = to_rel_time(m.log_time_ns)
        # arming_state: 2=ARMED, 1=DISARMED in PX4
        if m.ros_msg.arming_state == 2 and arming_time is None:
            arming_time = t_evt
        elif m.ros_msg.arming_state == 1 and arming_time is not None and disarming_time is None:
            disarming_time = t_evt

    if arming_time is None:
        arming_time = 0.0

    # ── IMU (sensor_combined) ──────────────────────────────────────────────
    # PX4's sensor_combined topic publishes fused accelerometer (m/s^2) and
    # gyroscope (rad/s) readings at the IMU rate (~800 Hz on this platform).
    # We extract the raw 3-axis signals and compute derived scalar magnitudes:
    #
    #   a_mag = sqrt(ax^2 + ay^2 + az^2)
    #     The total sensed acceleration vector magnitude.  At hover in steady
    #     flight this should be ~9.81 m/s^2 (1 g of gravity compensation).
    #
    #   g_mag = sqrt(gx^2 + gy^2 + gz^2)
    #     The total angular rate magnitude, useful for detecting rotational
    #     disturbances (e.g. collision-induced spinning).
    #
    #   a_deviation = |a_mag - 9.81|
    #     The absolute deviation from 1 g.  Spikes in a_deviation are the
    #     primary impact-detection signal — a collision momentarily adds a
    #     large non-gravitational acceleration component.
    imu_list = []
    for m in imu_msgs:
        imu_list.append({
            't': to_rel_time(m.log_time_ns),
            'ax': m.ros_msg.accelerometer_m_s2[0],
            'ay': m.ros_msg.accelerometer_m_s2[1],
            'az': m.ros_msg.accelerometer_m_s2[2],
            'gx': m.ros_msg.gyro_rad[0],
            'gy': m.ros_msg.gyro_rad[1],
            'gz': m.ros_msg.gyro_rad[2]
        })
    df_imu = pd.DataFrame(imu_list)
    if not df_imu.empty:
        df_imu['a_mag'] = np.sqrt(df_imu['ax']**2 + df_imu['ay']**2 + df_imu['az']**2)
        df_imu['g_mag'] = np.sqrt(df_imu['gx']**2 + df_imu['gy']**2 + df_imu['gz']**2)
        df_imu['a_deviation'] = np.abs(df_imu['a_mag'] - 9.81)

    return {
        'mocap': df_mocap,
        'column': df_column,
        'setpoint': df_setpoint,
        'battery': df_bat,
        'odom': df_odom,
        'imu': df_imu,
        'arming_time': arming_time,
        'disarming_time': disarming_time,
        'mocap_rate': mocap_rate,
        'dynamic_waypoints': dynamic_waypoints
    }
