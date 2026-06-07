import os
import sys
import pickle
import sqlite3
import pandas as pd
import numpy as np

# Load db_loader and db_pipeline logic
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
sys.path.insert(0, project_root)

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes

from mcap_ros2.reader import read_ros2_messages

def extract_and_cache_imu():
    db_path = os.path.join(project_root, "dev_logs", "analysis", "experiments_summary.db")
    cache_path = os.path.join(current_dir, "imu_cache.pkl")
    
    if os.path.exists(cache_path):
        print(f"IMU cache already exists at {cache_path}. Delete it to rebuild.")
        return
        
    print(f"Connecting to database {db_path}...")
    conn = sqlite3.connect(db_path)
    df_flights = pd.read_sql_query("SELECT * FROM flights_summary WHERE impact_detected = 1", conn)
    conn.close()
    
    print(f"Found {len(df_flights)} flights with impact. Processing...")
    
    drone_tracker_name, _ = load_drone_metadata(project_root)
    flights_dir = os.path.join(project_root, "dev_logs", "flights")
    
    cached_data = []
    
    for idx, row in df_flights.iterrows():
        flight_name = row['flight_name']
        condition = row['condition']
        
        # Determine paths
        parts = flight_name.split(' - ')
        folder_name = parts[0]
        pass_name = parts[1]
        pass_idx = int(pass_name.split('-')[1])
        
        folder_path = os.path.join(flights_dir, folder_name)
        import glob
        mcap_candidates = glob.glob(os.path.join(folder_path, f"*-pass{pass_idx:02d}.mcap"))
        if not mcap_candidates:
            print(f"MCAP missing for {flight_name}, skipping.")
            continue
        pass_path = mcap_candidates[0]
            
        try:
            topic_data, bag_start_ns = load_mcap(pass_path)
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
            
            df_mocap  = dfs['mocap']
            df_setpoint = dfs.get('setpoint', pd.DataFrame())
            df_imu    = dfs.get('imu', pd.DataFrame())
            df_column = dfs.get('column', pd.DataFrame())
            arming_time = dfs['arming_time']
            dynamic_waypoints = dfs.get('dynamic_waypoints', [])

            if df_imu.empty or df_mocap.empty:
                print(f"Skipping {flight_name} (Missing MoCap or IMU)")
                continue

            takeoff_mask = df_mocap['z'] > 0.15
            takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0

            col_x, col_y = (df_column['x'].head(50).mean(), df_column['y'].head(50).mean()) \
                if (not df_column.empty) else (0.408, 0.358)

            from dev_logs.analysis.kinematics.kin_calculator import find_waypoint_events, compute_velocity
            df_mocap = compute_velocity(df_mocap, resample=True)
            
            wp_events_list = find_waypoint_events(
                df_mocap, df_setpoint, takeoff_time, label=pass_name,
                column_x=col_x, column_y=col_y,
                column_radius=0.045, cage_radius=0.179,
                return_all=True, dynamic_waypoints=dynamic_waypoints
            )

            if not wp_events_list:
                print(f"Skipping {flight_name} (No WP events)")
                continue
                
            wp_events = wp_events_list[0]
            impact_t = wp_events.get('WP2')
            if impact_t is None:
                continue
                
            # Keep data from impact - 2.0 to impact + 2.0 to ensure we capture the peak
            t_rel_impact = df_imu['t'] - impact_t
            search_mask = (t_rel_impact >= -2.0) & (t_rel_impact <= 2.0)
            df_search = df_imu[search_mask].copy()
            
            if df_search.empty:
                print(f"Skipping {flight_name} (No IMU data near impact)")
                continue
                
            # Align the collision timeline to MoCap closest approach (WP2)
            true_impact_t = impact_t
            
            # Recalculate relative time based on the true high-frequency shock
            df_search['t_rel'] = df_search['t'] - true_impact_t
            
            # Crop to the final desired display window [-1.0, 2.0]
            mask_final = (df_search['t_rel'] >= -1.0) & (df_search['t_rel'] <= 2.0)
            df_crop = df_search[mask_final].copy()
            
            cached_data.append({
                'flight_name': flight_name,
                'condition': condition,
                't_rel': df_crop['t_rel'].values,
                'a_deviation': df_crop['a_deviation'].values,
                'g_mag': df_crop['g_mag'].values
            })
            
            if (idx + 1) % 10 == 0:
                print(f"Processed {idx + 1}/{len(df_flights)} flights...")
                
        except Exception as e:
            print(f"Error processing {flight_name}: {e}")
            
    with open(cache_path, 'wb') as f:
        pickle.dump(cached_data, f)
        
    print(f"Saved {len(cached_data)} IMU traces to {cache_path}")

if __name__ == "__main__":
    extract_and_cache_imu()
