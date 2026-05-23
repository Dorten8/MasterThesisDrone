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
    """Finds and parses the first .mcap file inside flight_path, returning a topic_data dict and bag_start_ns."""
    mcap_files = glob.glob(os.path.join(flight_path, "*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"No .mcap files discovered in: {flight_path}")
    mcap_path = mcap_files[0]
    
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
    status_msgs = topic_data.get("/fmu/out/vehicle_status", [])

    # MoCap
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
        # Fallback keyword sweep
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
        df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    # Battery
    bat_list = []
    for m in battery_msgs:
        bat_list.append({
            't': to_rel_time(m.log_time_ns),
            'voltage': m.ros_msg.voltage_v,
            'remaining': m.ros_msg.remaining * 100.0
        })
    df_bat = pd.DataFrame(bat_list)

    # Setpoint
    sp_list = []
    for m in setpoint_msgs:
        sp_list.append({
            't': to_rel_time(m.log_time_ns),
            'x_cmd': m.ros_msg.position[0],
            'y_cmd': -m.ros_msg.position[1],
            'z_cmd': -m.ros_msg.position[2]
        })
    df_setpoint = pd.DataFrame(sp_list)

    # Odom (EKF)
    odom_list = []
    for m in odom_msgs:
        odom_list.append({
            't': to_rel_time(m.log_time_ns),
            'x_ekf': m.ros_msg.position[0],
            'y_ekf': -m.ros_msg.position[1],
            'z_ekf': -m.ros_msg.position[2]
        })
    df_odom = pd.DataFrame(odom_list)

    # Find arming/disarming events from status_msgs
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

    return {
        'mocap': df_mocap,
        'setpoint': df_setpoint,
        'battery': df_bat,
        'odom': df_odom,
        'arming_time': arming_time,
        'disarming_time': disarming_time
    }
