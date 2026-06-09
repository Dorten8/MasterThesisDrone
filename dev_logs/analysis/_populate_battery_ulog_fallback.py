#!/usr/bin/env python3
"""ULog-based fallback: populate flights_battery_efficiency for flights without unsliced MCAPs.
   Uses pass MCAPs for MoCap takeoff/landing, .ulg for battery_status data.
"""
import sys, os, glob, re, datetime
sys.path.insert(0, os.path.expanduser('~/.local/lib/python3.10/site-packages'))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import pyulog
import pandas as pd
import numpy as np
import sqlite3

from dev_logs.analysis.database.db_manager import init_db, insert_or_replace_battery_efficiency
from dev_logs.analysis.database.db_loader import load_drone_metadata

FLIGHTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'flights'))
DB_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'experiments_summary.db'))

def _infer_condition(folder_name):
    name = folder_name.lower()
    if "rotating" in name: return "Rotating Cage"
    elif "fixed" in name: return "Fixed Cage"
    return "Unknown"

def get_battery_floats(df_bat, t_query):
    """Nearest voltage and remaining at t_query."""
    if df_bat is None or df_bat.empty: return None, None
    idx = np.searchsorted(df_bat['t'].values, t_query)
    idx = min(max(0, idx), len(df_bat) - 1)
    return float(df_bat['voltage'].iloc[idx]), float(df_bat['remaining'].iloc[idx])

def get_takeoff_landing_from_passes(folder_path):
    """Parse first and last pass MCAPs to detect takeoff_time and landing_time from MoCap Z."""
    from mcap_ros2.reader import read_ros2_messages
    from dev_logs.analysis.database.db_loader import build_dataframes

    drone_tracker_name, _ = load_drone_metadata(
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

    pass_files = sorted(glob.glob(os.path.join(folder_path, '*-pass*.mcap')))
    if len(pass_files) < 2:
        # Need at least first and last pass
        if not pass_files:
            return None, None
        first = pass_files[0]
        last = pass_files[0]
    else:
        first = pass_files[0]
        last = pass_files[-1]

    def get_z_range(mcap_path):
        topic_data = {}
        for msg in read_ros2_messages(mcap_path):
            t = msg.channel.topic
            if t not in topic_data: topic_data[t] = []
            topic_data[t].append(msg)
        if not topic_data: return None, None
        bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
        dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
        df_mocap = dfs['mocap']
        if df_mocap.empty: return None, None
        t_min = df_mocap['t'].min()
        t_max = df_mocap['t'].max()
        return df_mocap, t_min, t_max

    # From first pass: find takeoff (first Z > 0.15)
    result = get_z_range(first)
    if result[0] is None: return None, None
    df_first, _, _ = result
    mask = df_first['z'] > 0.15
    takeoff_time = df_first.loc[mask, 't'].iloc[0] if mask.any() else None

    # From last pass: find landing (last Z > 0.15)
    result = get_z_range(last)
    if result[0] is None: return None, None
    df_last, _, _ = result
    mask = df_last['z'] > 0.15
    landing_time = df_last.loc[mask, 't'].iloc[-1] if mask.any() else None

    return takeoff_time, landing_time

def main():
    init_db()

    # Find flights already in flights_summary but missing from flights_battery_efficiency
    db = sqlite3.connect(DB_PATH)
    cur = db.cursor()
    cur.execute("SELECT flight_name FROM flights_battery_efficiency")
    existing = {r[0] for r in cur.fetchall()}
    cur.execute("SELECT DISTINCT flight_name FROM flights_summary")
    all_summary = [r[0] for r in cur.fetchall()]
    db.close()

    base_folders = set()
    for f in all_summary:
        m = re.match(r'^(flight_\S+?) - Pass-', f)
        if m: base_folders.add(m.group(1))

    missing = sorted(base_folders - existing)
    print(f"Missing from battery table: {len(missing)} flights")

    inserted = 0
    skipped = 0
    errors = 0

    for folder_name in missing:
        folder_path = os.path.join(FLIGHTS_DIR, folder_name)
        if not os.path.isdir(folder_path):
            print(f"  ❌ Not found: {folder_name}")
            errors += 1
            continue

        # Check for .ulg file
        ulg_files = glob.glob(os.path.join(folder_path, '*.ulg'))
        if not ulg_files:
            print(f"  ❌ No .ulg: {folder_name}")
            errors += 1
            continue

        print(f"🔄 Processing: {folder_name}")
        try:
            # 1. Get takeoff/landing from pass MCAPs (MoCap Z > 0.15m)
            takeoff_time, landing_time = get_takeoff_landing_from_passes(folder_path)
            if takeoff_time is None or landing_time is None:
                print(f"  ⚠️  Cannot determine takeoff/landing from pass MCAPs")
                errors += 1
                continue

            # 2. Parse ULog for battery_status + vehicle_status
            ulog = pyulog.ULog(ulg_files[0], message_name_filter_list=['vehicle_status', 'battery_status'])

            ds_bat = ulog.get_dataset('battery_status')
            t_bat_us = ds_bat.data['timestamp']
            voltage = ds_bat.data['voltage_v']
            remaining = ds_bat.data['remaining']

            ds_stat = ulog.get_dataset('vehicle_status')
            t_stat_us = ds_stat.data['timestamp']
            arming_state = ds_stat.data['arming_state']

            # Find arming/disarming
            arming_time_us = None
            disarming_time_us = None
            for i in range(len(t_stat_us)):
                if arming_state[i] == 2 and arming_time_us is None:
                    arming_time_us = t_stat_us[i]
                elif arming_state[i] == 1 and arming_time_us is not None and disarming_time_us is None:
                    disarming_time_us = t_stat_us[i]

            if arming_time_us is None:
                print(f"  ⚠️  No arming detected in ULog")
                errors += 1
                continue
            if disarming_time_us is None:
                disarming_time_us = t_bat_us[-1]

            arming_time = (arming_time_us - t_bat_us[0]) / 1e6
            disarming_time = (disarming_time_us - t_bat_us[0]) / 1e6

            # Build battery DataFrame (relative time)
            t_rel = (t_bat_us - t_bat_us[0]) / 1e6
            df_bat = pd.DataFrame({
                't': t_rel,
                'voltage': voltage,
                'remaining': remaining * 100.0
            })

            total_armed_time = disarming_time - arming_time
            total_flying_time = landing_time - takeoff_time
            if total_flying_time <= 0:
                total_flying_time = total_armed_time - 4.0

            log_duration = t_rel[-1]

            # Extract battery metrics at milestones
            v_arm, rem_arm = get_battery_floats(df_bat, arming_time)
            v_takeoff, rem_takeoff = get_battery_floats(df_bat, takeoff_time)
            v_landing, rem_landing = get_battery_floats(df_bat, landing_time)
            v_disarm, rem_disarm = get_battery_floats(df_bat, disarming_time)

            # Flying phase stats
            flying_bat = df_bat[(df_bat['t'] >= takeoff_time) & (df_bat['t'] <= landing_time)]
            min_v = float(flying_bat['voltage'].min()) if not flying_bat.empty else None
            avg_v = float(flying_bat['voltage'].mean()) if not flying_bat.empty else None

            # Rates
            total_cap_consumed = max(0.0, (rem_arm or 0) - (rem_disarm or 0))
            total_v_dropped = max(0.0, (v_arm or 0) - (v_disarm or 0))

            armed_min = total_armed_time / 60.0
            flying_min = total_flying_time / 60.0

            v_drop_rate_armed = total_v_dropped / armed_min if armed_min > 0 else 0.0
            cap_drain_rate_armed = total_cap_consumed / armed_min if armed_min > 0 else 0.0

            v_drop_rate_flying = 0.0
            cap_drain_rate_flying = 0.0
            if flying_min > 0:
                if v_takeoff is not None and v_landing is not None:
                    v_drop_rate_flying = max(0.0, v_takeoff - v_landing) / flying_min
                if rem_takeoff is not None and rem_landing is not None:
                    cap_drain_rate_flying = max(0.0, rem_takeoff - rem_landing) / flying_min

            condition = _infer_condition(folder_name)
            metrics = {
                'log_duration': log_duration,
                'total_armed_time': total_armed_time,
                'total_flying_time': total_flying_time,
                'voltage_at_arm': v_arm, 'remaining_at_arm': rem_arm,
                'voltage_at_takeoff': v_takeoff, 'remaining_at_takeoff': rem_takeoff,
                'voltage_at_landing': v_landing, 'remaining_at_landing': rem_landing,
                'voltage_at_disarm': v_disarm, 'remaining_at_disarm': rem_disarm,
                'min_voltage_during_flight': min_v, 'avg_voltage_during_flight': avg_v,
                'voltage_drop_rate_armed': v_drop_rate_armed,
                'capacity_drain_rate_armed': cap_drain_rate_armed,
                'voltage_drop_rate_flying': v_drop_rate_flying,
                'capacity_drain_rate_flying': cap_drain_rate_flying,
                'total_capacity_consumed_pct': total_cap_consumed,
                'total_voltage_dropped': total_v_dropped,
            }

            insert_or_replace_battery_efficiency(folder_name, condition, metrics)
            inserted += 1

            print(f"  ✅ fly_time={total_flying_time:.0f}s cap_drain={cap_drain_rate_flying:.1f}%/min "
                  f"cap_cons={total_cap_consumed:.1f}% armed={total_armed_time:.0f}s")

        except Exception as e:
            print(f"  ❌ Error: {e}")
            import traceback; traceback.print_exc()
            errors += 1

    print(f"\n📦 Done: {inserted} inserted, {skipped} skipped, {errors} errors")


if __name__ == '__main__':
    main()
