import sys
import os
import glob
import re
import datetime
import numpy as np
import pandas as pd

# Self-healing path boilerplate for Python package discovery
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../" if "__file__" in locals() or "__file__" in globals() else "")))

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.database.db_manager import insert_or_replace_battery_efficiency, get_connection, init_db

def _infer_condition(folder_name):
    name = folder_name.lower()
    if "rotating" in name:
        return "Rotating Cage"
    elif "fixed" in name:
        return "Fixed Cage"
    return "Unknown"

def get_battery_floats(df_bat, t_query):
    """Helper to find nearest voltage and SOC % at t_query."""
    if df_bat is None or df_bat.empty:
        return None, None
    idx = np.searchsorted(df_bat['t'], t_query)
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
    all_flights = []
    for folder in sorted(os.listdir(flights_dir)):
        # Only process experimental flight folders starting with flight_
        if not folder.startswith("flight_"):
            continue
        
        folder_path = os.path.join(flights_dir, folder)
        if not os.path.isdir(folder_path):
            continue
            
        mcap_files = glob.glob(os.path.join(folder_path, "*.mcap"))
        # Filter out pass slices
        unsliced = [f for f in mcap_files if "-pass" not in os.path.basename(f)]
        if not unsliced:
            continue
            
        # Prefer repaired bag if present
        repaired = [f for f in unsliced if ".repaired.mcap" in f]
        mcap_path = repaired[0] if repaired else unsliced[0]
        
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
            # Load MCAP raw topics
            topic_data, bag_start_ns = load_mcap(mcap_path)
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
            
            df_mocap = dfs['mocap']
            df_bat = dfs['battery']
            df_odom = dfs['odom']
            df_imu = dfs.get('imu', pd.DataFrame())
            
            # 2. Determine Log boundaries
            max_t = 0.0
            if not df_bat.empty:
                max_t = max(max_t, df_bat['t'].max())
            if not df_mocap.empty:
                max_t = max(max_t, df_mocap['t'].max())
            if not df_odom.empty:
                max_t = max(max_t, df_odom['t'].max())
            if not df_imu.empty:
                max_t = max(max_t, df_imu['t'].max())
            log_duration = max_t if max_t > 0 else 60.0
            
            # 3. Determine Arm/Disarm relative times
            arming_time = dfs.get('arming_time', 0.0)
            disarming_time = dfs.get('disarming_time')
            if disarming_time is None:
                # Fallback to log end time if disarm message was truncated or not caught
                disarming_time = log_duration
                
            total_armed_time = disarming_time - arming_time
            if total_armed_time <= 0:
                total_armed_time = log_duration
                
            # 4. Determine Takeoff/Landing active flight times
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
            
            # Graceful fallbacks for takeoff/landing
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
                
            # 5. Extract battery metrics at milestones
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
            
            # Prevent negative consumption artifacts
            total_cap_consumed = max(0.0, total_cap_consumed)
            total_v_dropped = max(0.0, total_v_dropped)
            
            # Drop rates per minute
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
            
    print(f"\n📦 SQLite Processing complete: {inserted_count} processed/inserted, {skipped_count} skipped, {error_count} errors.")
    
    # 7. Auto-export table to CSV file
    csv_path = os.path.join(project_root, "dev_logs", "analysis", "unsliced_battery_efficiency.csv")
    print(f"📁 Exporting fresh dataset to: {csv_path}")
    try:
        from dev_logs.analysis.database.db_manager import get_battery_efficiency_df
        df_eff = get_battery_efficiency_df()
        if not df_eff.empty:
            df_eff.to_csv(csv_path, index=False)
            print(f"✅ Successfully exported {len(df_eff)} flight battery logs to CSV!")
            
            # Print quick comparative summary statistics to verify
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
