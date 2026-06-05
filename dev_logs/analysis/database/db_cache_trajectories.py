import os
import sys
import glob
import re
import pickle
import pandas as pd
import numpy as np

# Resolve project root dynamically
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
sys.path.append(project_root)

from dev_logs.analysis.database.db_manager import get_database_df
from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes

def main():
    print("⏳ Initializing Trajectory Cache Builder...")
    
    # 1. Load drone metadata and paths
    drone_tracker_name, system_config = load_drone_metadata(project_root)
    flights_dir = os.path.join(project_root, "dev_logs", "flights")
    cache_path = os.path.join(current_dir, "trajectory_cache.pkl")
    
    # Obstacle default config
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    col_x_default = 0.408
    col_y_default = 0.358
    
    # 2. Get all flights from DB
    df_all = get_database_df()
    df_impacts = df_all.query("impact_detected == 1").copy()
    print(f"🔍 Found {len(df_impacts)} impacted flights in the SQLite database.")
    
    cache_flights = []
    
    # 3. Process each flight
    for idx, row in df_impacts.iterrows():
        flight_name = row["flight_name"]
        condition = row["condition"]
        
        m_pass = re.search(r'^(.*?) - Pass-(\d+)$', flight_name)
        if not m_pass:
            print(f"⚠️  [SKIP] Flight name does not match expected pass pattern: {flight_name}")
            continue
            
        base_folder = m_pass.group(1)
        pass_idx = int(m_pass.group(2))
        
        # Locate pass MCAP file
        dir_path = os.path.join(flights_dir, base_folder)
        if not os.path.exists(dir_path):
            dir_path = os.path.join(project_root, base_folder)
            
        pattern = os.path.join(dir_path, f"*-pass{pass_idx:02d}.mcap")
        matches = glob.glob(pattern)
        if not matches:
            print(f"❌  [NOT FOUND] Pass file not found for: {flight_name} at {pattern}")
            continue
            
        mcap_path = matches[0]
        print(f"📦 [{idx+1}/{len(df_impacts)}] Processing {flight_name}...")
        
        try:
            # Load and parse MCAP
            topic_data, bag_start_ns = load_mcap(mcap_path)
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
            
            df_mocap = dfs['mocap']
            df_column = dfs.get('column', pd.DataFrame())
            
            if df_mocap.empty:
                print(f"⚠️  [EMPTY] No MoCap data in {flight_name}")
                continue
                
            # Extract column position
            if df_column is not None and not df_column.empty:
                col_x = float(df_column['x'].head(50).mean())
                col_y = float(df_column['y'].head(50).mean())
            else:
                col_x = col_x_default
                col_y = col_y_default
                
            # Store essential coordinates
            cache_flights.append({
                "flight_name": flight_name,
                "condition": condition,
                "x": df_mocap["x"].to_numpy(),
                "y": df_mocap["y"].to_numpy(),
                "z": df_mocap["z"].to_numpy(),
                "t": df_mocap["t"].to_numpy(),
                "col_x": col_x,
                "col_y": col_y
            })
        except Exception as e:
            print(f"💥 [ERROR] Failed to process {flight_name}: {e}")
            
    # 4. Save cache file
    print(f"💾 Saving {len(cache_flights)} cached trajectories to {cache_path}...")
    with open(cache_path, "wb") as f:
        pickle.dump(cache_flights, f, protocol=pickle.HIGHEST_PROTOCOL)
        
    print("✅ Trajectory Cache completed successfully!")

if __name__ == "__main__":
    main()
