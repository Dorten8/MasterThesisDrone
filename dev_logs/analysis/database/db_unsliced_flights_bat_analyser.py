import sys
import os
import glob
import re
import datetime
import gc
import numpy as np
import pandas as pd

# Self-healing path boilerplate for Python package discovery
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../" if "__file__" in locals() or "__file__" in globals() else "")))

from dev_logs.analysis.database.db_loader import load_drone_metadata
from dev_logs.analysis.database.db_manager import insert_or_replace_battery_efficiency, get_connection, init_db, is_approved_flight

def _infer_condition(folder_name):
    name = folder_name.lower()
    if "rotating" in name:
        return "Rotating Cage"
    elif "fixed" in name:
        return "Fixed Cage"
    return "Unknown"

def _stream_load_for_battery(mcap_path, drone_tracker_name):
    """Memory-safe streaming MCAP reader: only collects the 4 topics needed for
    battery analysis directly into lightweight lists, then builds DataFrames.

    A full unsliced MCAP (200MB+) can hold 1M+ messages across dozens of topics.
    Loading every message into Python objects triggers OOM on machines with <32GB RAM.
    By filtering to just battery/poses/status/odom and building DataFrames inline,
    peak memory stays under ~300MB even for the largest flights.
    """
    from mcap_ros2.reader import read_ros2_messages

    # ── Memory-saving technique: filter at the MCAP reader level ──────────
    # The unsliced MCAP contains 30+ ROS2 topics (camera feeds, IMU, GPS, etc.).
    # By passing `topics=NEEDED_TOPICS` to read_ros2_messages(), the deserializer
    # skips all other topics **before** creating Python objects. This avoids
    # allocating ~1-2 GB of short-lived message objects that would otherwise
    # be discarded, keeping peak memory under ~300 MB instead.
    NEEDED_TOPICS = [
        '/poses',                         # MoCap ground truth (OptiTrack)
        '/fmu/out/battery_status',        # Voltage + remaining capacity
        '/fmu/out/vehicle_status',        # Arming/disarming state transitions
        '/fmu/out/vehicle_odometry',      # EKF position estimate (NED frame)
    ]

    bag_start_ns = None
    # ── Inline list-of-dicts instead of appending to DataFrames ───────────
    # Building DataFrames via pd.DataFrame.append() in a loop is quadratic in
    # memory because each append copies the entire frame. Instead, we accumulate
    # lightweight dicts in plain Python lists (O(n) memory), then call
    # pd.DataFrame() once at the end — a single, efficient copy.
    pose_rows = []         # {t, x, y, z}
    bat_rows = []          # {t, voltage, remaining}
    odom_rows = []         # {t, x_ekf, y_ekf, z_ekf, vx_ekf_raw, vy_ekf_raw, vz_ekf_raw}
    arming_time = None     # First arming_state==2 timestamp, relative seconds
    disarming_time = None  # First arming_state==1 timestamp after arming

    # ── Single-pass streaming loop ────────────────────────────────────────
    # The generator yields each deserialized message one at a time. We never
    # hold more than a single frame's worth of data in memory simultaneously.
    for msg in read_ros2_messages(mcap_path, topics=NEEDED_TOPICS):
        if bag_start_ns is None:
            bag_start_ns = msg.log_time_ns  # Nanosecond epoch of the first message

        # Convert log_time to seconds relative to the bag start
        t_rel = (msg.log_time_ns - bag_start_ns) * 1e-9
        topic = msg.channel.topic

        if topic == '/poses':
            # The /poses topic carries an array of named rigid-body poses.
            # We only keep the pose whose name matches the drone tracker.
            for p in msg.ros_msg.poses:
                if p.name.lower() == drone_tracker_name.lower():
                    pose_rows.append({'t': t_rel, 'x': p.pose.position.x,
                                      'y': p.pose.position.y, 'z': p.pose.position.z})
                    break  # One tracker per message — stop scanning after first match
        elif topic == '/fmu/out/battery_status':
            # remaining is a fraction [0.0, 1.0]; multiply by 100 for percentage
            bat_rows.append({'t': t_rel, 'voltage': msg.ros_msg.voltage_v,
                             'remaining': msg.ros_msg.remaining * 100.0})
        elif topic == '/fmu/out/vehicle_status':
            # arming_state: 2 = ARMED, 1 = DISARMED (PX4 enum)
            # We capture the very first arm and the very first disarm after that
            if msg.ros_msg.arming_state == 2 and arming_time is None:
                arming_time = t_rel
            elif msg.ros_msg.arming_state == 1 and arming_time is not None and disarming_time is None:
                disarming_time = t_rel
        elif topic == '/fmu/out/vehicle_odometry':
            # PX4 odometry is in NED frame (x=north, y=east, z=down).
            # We negate Y and Z to align with the MoCap ENU convention
            # (x=forward, y=left, z=up) for consistent offset computation.
            odom_rows.append({'t': t_rel,
                              'x_ekf': msg.ros_msg.position[0],
                              'y_ekf': -msg.ros_msg.position[1],   # east -> -y (left)
                              'z_ekf': -msg.ros_msg.position[2],   # down -> -z (up)
                              'vx_ekf_raw': msg.ros_msg.velocity[0],
                              'vy_ekf_raw': msg.ros_msg.velocity[1],
                              'vz_ekf_raw': msg.ros_msg.velocity[2]})

    # Build DataFrames from the accumulated lists — single allocation each
    df_mocap = pd.DataFrame(pose_rows)
    if not df_mocap.empty:
        # MoCap can emit duplicate timestamps at high rates; keep the first
        df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    df_bat = pd.DataFrame(bat_rows)
    df_odom = pd.DataFrame(odom_rows)

    if arming_time is None:
        arming_time = 0.0  # Fallback: treat bag start as arming if no status message found

    # ── EKF-to-MoCap alignment offset via pd.merge_asof ───────────────────
    # The EKF and MoCap coordinate frames have a systematic offset (the drone
    # starts at different origin positions in each frame). We compute the
    # median X/Y difference between the two streams to align them.
    # pd.merge_asof() joins each MoCap row with the nearest-in-time EKF row,
    # tolerating the different publication rates (~100 Hz MoCap vs ~30 Hz EKF).
    # Using the median instead of mean makes the offset robust to outliers that
    # occur at the very start/end of the log where one stream may have stale data.
    # This is the identical alignment logic used by build_dataframes() in
    # db_loader.py for the sliced-pass pipeline.
    transform_offset_x = 0.0
    transform_offset_y = 0.0
    if not df_mocap.empty and not df_odom.empty:
        df_merged = pd.merge_asof(
            df_mocap[['t', 'x', 'y']].sort_values('t'),
            df_odom[['t', 'x_ekf', 'y_ekf']].sort_values('t'),
            on='t', direction='nearest'  # Snap each MoCap t to the closest EKF t
        )
        if not df_merged.empty:
            transform_offset_x = (df_merged['x_ekf'] - df_merged['x']).median()
            transform_offset_y = (-df_merged['y_ekf'] + df_merged['y']).median()

    return {
        'df_mocap': df_mocap,
        'df_bat': df_bat,
        'df_odom': df_odom,
        'arming_time': arming_time,
        'disarming_time': disarming_time,
        'transform_offset_x': transform_offset_x,
        'transform_offset_y': transform_offset_y,
        'bag_start_ns': bag_start_ns,
    }


def get_battery_floats(df_bat, t_query):
    """Helper to find nearest voltage and SOC % at t_query.

    Uses binary search (np.searchsorted) to locate the insertion point of
    t_query in the sorted battery timestamp array, then clamps to valid bounds.
    This is O(log n) per lookup instead of O(n) for a linear scan, which matters
    because we call this 4 times per flight (arm, takeoff, landing, disarm)
    across hundreds of flights.
    """
    if df_bat is None or df_bat.empty:
        return None, None
    # np.searchsorted returns the index where t_query would be inserted to
    # maintain sorted order — effectively the index of the first timestamp >= t_query.
    # The battery DataFrame is already sorted by 't', so this is a valid binary search.
    idx = np.searchsorted(df_bat['t'], t_query)
    # Clamp to [0, len-1] to handle queries before the first sample or after the last
    idx = min(max(0, idx), len(df_bat) - 1)
    return float(df_bat['voltage'].iloc[idx]), float(df_bat['remaining'].iloc[idx])

def main(force_recompute=False):
    """Scans all unsliced flights, extracts armed/flying battery metrics,
    caches to SQLite, and exports to CSV.
    """
    init_db()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
    flights_dir = os.path.join(project_root, "dev_logs", "flights")
    
    drone_tracker_name, _ = load_drone_metadata(project_root)
    
    # 1. Scan and collect raw unsliced flights
    # ── Unsliced MCAP discovery algorithm ─────────────────────────────────
    # Each flight_* folder may contain multiple MCAP files:
    #   - flight_2025-04-01.mcap             ← unsliced full flight
    #   - flight_2025-04-01.repaired.mcap    ← repaired version (preferred)
    #   - flight_2025-04-01-pass00.mcap      ← sliced pass (excluded by filter)
    # We discard any filename containing "-pass" to isolate the full-log file.
    # If a .repaired.mcap exists, it takes priority because the repair tool
    # has fixed truncated chunks that the raw .mcap may have.
    all_flights = []
    for folder in sorted(os.listdir(flights_dir)):
        # Only process experimental flight folders starting with flight_
        if not folder.startswith("flight_"):
            continue

        folder_path = os.path.join(flights_dir, folder)
        if not os.path.isdir(folder_path):
            continue

        mcap_files = glob.glob(os.path.join(folder_path, "*.mcap"))
        # Filter out pass slices: filename contains "-pass" string
        unsliced = [f for f in mcap_files if "-pass" not in os.path.basename(f)]
        if not unsliced:
            continue

        # Prefer repaired bag if present: repair tool fixes chunk truncation
        repaired = [f for f in unsliced if ".repaired.mcap" in f]
        mcap_path = repaired[0] if repaired else unsliced[0]

        # ── Gate: approved date cutoff (SSoT from db_manager) ──
        # is_approved_flight() reads the same whitelist of flight IDs that
        # db_manager.py uses for its own queries. This ensures the unsliced
        # analyser operates on exactly the same set of flights that the
        # main experiment database considers valid (same cutoff date, same
        # exclusion criteria). The db_manager is the Single Source of Truth.
        if not is_approved_flight(folder):
            continue

        condition = _infer_condition(folder)
        if condition == "Unknown":
            continue # Skip non-cage or miscellaneous test profiles

        all_flights.append({
            'folder': folder,
            'path': mcap_path,
            'condition': condition
        })
        
    print(f"🔍 Discovered {len(all_flights)} unsliced experimental MCAP flights.")
    
    # Check if already processed (to support idempotency)
    conn = get_connection()
    cursor = conn.cursor()
    cached_flights = set()
    try:
        cursor.execute("SELECT flight_name FROM flights_battery_efficiency")
        cached_flights = {row[0] for row in cursor.fetchall()}
    except Exception:
        pass
    finally:
        conn.close()
        
    inserted_count = 0
    skipped_count = 0
    error_count = 0
    
    for f in all_flights:
        flight_name = f['folder']
        mcap_path = f['path']
        condition = f['condition']
        
        if flight_name in cached_flights and not force_recompute:
            print(f"⏭️  Skipping (already cached): {flight_name}")
            skipped_count += 1
            continue
            
        print(f"🔄 Processing unsliced flight: {flight_name}")
        try:
            # 🧠 Memory-safe streaming load: only 4 topics, no full-message-object storage
            stream = _stream_load_for_battery(mcap_path, drone_tracker_name)

            df_mocap = stream['df_mocap']
            df_bat = stream['df_bat']
            df_odom = stream['df_odom']

            # 2. Determine Log boundaries
            max_t = 0.0
            if not df_bat.empty:
                max_t = max(max_t, df_bat['t'].max())
            if not df_mocap.empty:
                max_t = max(max_t, df_mocap['t'].max())
            if not df_odom.empty:
                max_t = max(max_t, df_odom['t'].max())
            log_duration = max_t if max_t > 0 else 60.0

            # 3. Determine Arm/Disarm relative times
            arming_time = stream.get('arming_time', 0.0)
            disarming_time = stream.get('disarming_time')
            if disarming_time is None:
                # Fallback to log end time if disarm message was truncated or not caught
                disarming_time = log_duration
                
            total_armed_time = disarming_time - arming_time
            if total_armed_time <= 0:
                total_armed_time = log_duration
                
            # 4. Determine Takeoff/Landing active flight times
            # ── Takeoff/landing detection via MoCap Z-height threshold ────
            # The drone starts on the ground (z ≈ 0). Takeoff is defined as
            # the first moment the MoCap Z coordinate exceeds 0.15 m after
            # arming. Landing is the last moment above 0.15 m before disarming.
            # The 0.15 m threshold is chosen to be well above OptiTrack noise
            # (~1-2 mm) while still catching the very beginning of ascent.
            takeoff_time = None
            landing_time = None
            if not df_mocap.empty:
                # Takeoff: first height crossing > 0.15m after arming
                takeoff_mask = (df_mocap['z'] > 0.15) & (df_mocap['t'] >= arming_time)
                if takeoff_mask.any():
                    takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0]
                    # Landing: last height crossing > 0.15m before disarming
                    landing_mask = (df_mocap['z'] > 0.15) & (df_mocap['t'] <= disarming_time)
                    if landing_mask.any():
                        landing_time = df_mocap.loc[landing_mask, 't'].iloc[-1]

            # ── Graceful fallbacks with ±2 s margins ─────────────────────
            # If MoCap data is unavailable for any reason (tracker occlusion,
            # short log), we fall back to arming_time + 2.0 s for takeoff and
            # disarming_time - 2.0 s for landing. The 2-second margin is a
            # conservative estimate of the typical pre-flight idle and
            # post-flight idle periods observed across the dataset.
            used_takeoff_fallback = False
            used_landing_fallback = False

            if takeoff_time is None:
                takeoff_time = arming_time + 2.0
                used_takeoff_fallback = True
            if landing_time is None:
                landing_time = disarming_time - 2.0
                used_landing_fallback = True
                
            total_flying_time = landing_time - takeoff_time
            if total_flying_time <= 0:
                total_flying_time = total_armed_time - 4.0
                
            if used_takeoff_fallback or used_landing_fallback:
                print(f"   ⚠️  Takeoff/Landing fallbacks triggered (MoCap height data unavailable or truncated).")
                
            # 5. Extract battery metrics at 4 flight-phase milestones
            # Each call uses np.searchsorted (binary search, O(log n)) to
            # find the battery sample nearest in time to the event:
            #   1. Arm     — pilot arms the drone, props start spinning
            #   2. Takeoff  — first Z > 0.15 m (or fallback)
            #   3. Landing  — last Z > 0.15 m (or fallback)
            #   4. Disarm   — pilot disarms, props stop
            v_arm, rem_arm = get_battery_floats(df_bat, arming_time)
            v_takeoff, rem_takeoff = get_battery_floats(df_bat, takeoff_time)
            v_landing, rem_landing = get_battery_floats(df_bat, landing_time)
            v_disarm, rem_disarm = get_battery_floats(df_bat, disarming_time)
            
            # Calculate sags & averages during active flying phase
            min_v = None
            avg_v = None
            if not df_bat.empty:
                flying_bat = df_bat[(df_bat['t'] >= takeoff_time) & (df_bat['t'] <= landing_time)]
                if not flying_bat.empty:
                    min_v = float(flying_bat['voltage'].min())
                    avg_v = float(flying_bat['voltage'].mean())
                    
            # Fallbacks if battery dataframe is empty or truncated
            if v_arm is None and not df_bat.empty:
                # Take overall log extremes if exact timestamps are out of bound
                v_arm = float(df_bat['voltage'].iloc[0])
                rem_arm = float(df_bat['remaining'].iloc[0])
                v_disarm = float(df_bat['voltage'].iloc[-1])
                rem_disarm = float(df_bat['remaining'].iloc[-1])
                
            # 6. Compute overall drops and efficiency rates
            total_cap_consumed = (rem_arm - rem_disarm) if (rem_arm is not None and rem_disarm is not None) else 0.0
            total_v_dropped = (v_arm - v_disarm) if (v_arm is not None and v_disarm is not None) else 0.0
            
            # ── Negative consumption prevention ──────────────────────────────
            # In rare edge cases, the nearest-timestamp search can return a
            # later battery sample that shows higher remaining capacity than
            # an earlier one (e.g., voltage recovery after landing, or
            # battery-status message dropout). This would produce a negative
            # consumption value. We clamp to zero with max(0.0, x) so that
            # rates and totals are physically meaningful.
            total_cap_consumed = max(0.0, total_cap_consumed)
            total_v_dropped = max(0.0, total_v_dropped)
            
            # Drop rates per minute: normalize by time spent in each window
            armed_min = total_armed_time / 60.0
            flying_min = total_flying_time / 60.0
            
            # ── Armed-window rates: from arm to disarm (includes idle on ground) ─
            v_drop_rate_armed = total_v_dropped / armed_min if armed_min > 0 else 0.0
            cap_drain_rate_armed = total_cap_consumed / armed_min if armed_min > 0 else 0.0
            
            # ── Flying-window rates: takeoff to landing (airborne only) ──────
            # These are a more precise measure of in-flight energy consumption,
            # excluding the variable idle time on the ground before/after flight.
            v_drop_rate_flying = 0.0
            cap_drain_rate_flying = 0.0
            if flying_min > 0:
                if v_takeoff is not None and v_landing is not None:
                    v_drop_rate_flying = max(0.0, v_takeoff - v_landing) / flying_min
                if rem_takeoff is not None and rem_landing is not None:
                    cap_drain_rate_flying = max(0.0, rem_takeoff - rem_landing) / flying_min
                    
            metrics = {
                'log_duration': log_duration,
                'total_armed_time': total_armed_time,
                'total_flying_time': total_flying_time,
                'voltage_at_arm': v_arm,
                'remaining_at_arm': rem_arm,
                'voltage_at_takeoff': v_takeoff,
                'remaining_at_takeoff': rem_takeoff,
                'voltage_at_landing': v_landing,
                'remaining_at_landing': rem_landing,
                'voltage_at_disarm': v_disarm,
                'remaining_at_disarm': rem_disarm,
                'min_voltage_during_flight': min_v,
                'avg_voltage_during_flight': avg_v,
                'voltage_drop_rate_armed': v_drop_rate_armed,
                'capacity_drain_rate_armed': cap_drain_rate_armed,
                'voltage_drop_rate_flying': v_drop_rate_flying,
                'capacity_drain_rate_flying': cap_drain_rate_flying,
                'total_capacity_consumed_pct': total_cap_consumed,
                'total_voltage_dropped': total_v_dropped
            }
            
            # Cache dynamically to SQLite
            insert_or_replace_battery_efficiency(flight_name, condition, metrics)
            inserted_count += 1
            
        except Exception as e:
            print(f"   ❌ Error processing {flight_name}: {e}")
            import traceback; traceback.print_exc()
            error_count += 1
        finally:
            # ── Explicit GC between flights ──────────────────────────────────
            # Each flight's DataFrames can transiently consume 200-300 MB.
            # Without explicit gc.collect(), Python's reference-cycle detector
            # may defer cleanup, causing memory to accumulate across flights.
            # Forcing collection keeps the peak working set under ~500 MB
            # even when processing 100+ flights sequentially.
            gc.collect()
            
    print(f"\n📦 SQLite Processing complete: {inserted_count} processed/inserted, {skipped_count} skipped, {error_count} errors.")
    
    # 7. Auto-export table to CSV file
    # ── CSV export: the entire battery_efficiency table is written as a flat
    # CSV so that downstream analysis (e.g., Jupyter notebooks, R scripts) can
    # consume it without a SQLite dependency. The CSV is regenerated from the
    # database on every run to guarantee consistency with the SQLite SSoT.
    csv_path = os.path.join(project_root, "dev_logs", "analysis", "database", "unsliced_battery_efficiency.csv")
    print(f"📁 Exporting fresh dataset to: {csv_path}")
    try:
        from dev_logs.analysis.database.db_manager import get_battery_efficiency_df
        df_eff = get_battery_efficiency_df()
        if not df_eff.empty:
            df_eff.to_csv(csv_path, index=False)
            print(f"✅ Successfully exported {len(df_eff)} flight battery logs to CSV!")
            
            # ── Quick comparative summary: group by experimental condition ──
            # Prints mean flying time, average voltage, capacity drain rate,
            # and voltage drop rate for Fixed Cage vs Rotating Cage, enabling
            # an immediate sanity check of the computed metrics.
            print("\n📊 --- QUICK COMPARATIVE SUMMARY STATISTICS ---")
            grouped = df_eff.groupby('condition')
            summary_stats = grouped.agg({
                'total_flying_time': 'mean',
                'avg_voltage_during_flight': 'mean',
                'capacity_drain_rate_flying': 'mean',
                'voltage_drop_rate_flying': 'mean'
            })
            print(summary_stats.to_string())
            print("-----------------------------------------------")
        else:
            print("⚠️  Database efficiency table is empty, skipping CSV export.")
    except Exception as e:
        print(f"❌ Failed to export CSV: {e}")

if __name__ == "__main__":
    force_recompute = "--force" in sys.argv
    main(force_recompute=force_recompute)
