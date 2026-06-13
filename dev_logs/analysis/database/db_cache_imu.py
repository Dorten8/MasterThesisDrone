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
    """Build a pickle cache of IMU traces aligned to the moment of column impact.

    Why cache? The experiments_summary database contains 179 impact flights.
    Each flight's MCAP file is 20-40 MB of raw ROS2 messages. Re-loading and
    re-parsing 179 MCAPs to extract IMU traces would take several minutes every
    time a notebook regenerates aggregated plots. This cache stores only the
    aligned arrays (t_rel, a_deviation, g_mag) — three small NumPy arrays per
    flight — making reloading nearly instantaneous (~0.2 s vs ~180 s).
    """
    db_path = os.path.join(project_root, "dev_logs", "analysis", "experiments_summary.db")
    cache_path = os.path.join(current_dir, "imu_cache.pkl")
    
    if os.path.exists(cache_path):
        print(f"IMU cache already exists at {cache_path}. Delete it to rebuild.")
        return
        
    print(f"Connecting to database {db_path}...")
    conn = sqlite3.connect(db_path)
    # Query only flights marked with impact_detected=1 in the summary table.
    # This filters to the subset of flights where the drone actually struck
    # the column — the only flights for which IMU shock analysis is meaningful.
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
            # ── Load MCAP and build DataFrames ─────────────────────────────
            # load_mcap() deserialises all ROS2 topics into a dict of lists,
            # then build_dataframes() pivots them into typed DataFrames and
            # computes derived columns (velocity, deviation, etc.).
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
            # ── Impact time alignment ───────────────────────────────────────
            # 'Column Impact' is the MoCap-calculated time of closest approach
            # to the column centre. Falls back to 'WP2' (waypoint-2 crossing)
            # if Column Impact was not detected (e.g., the drone passed wide).
            impact_t = wp_events.get('Column Impact') or wp_events.get('WP2')
            if impact_t is None:
                continue
                
            # ── Two-stage crop for IMU window ──────────────────────────────
            # Stage 1: wide [impact-2.0, impact+2.0] s — generous 4 s window
            # ensures the full acceleration transient is captured even if the
            # waypoint detector's impact time is off by a few hundred ms.
            # Stage 2 (below): tight [-1.0, 2.0] s — final display window.
            t_rel_impact = df_imu['t'] - impact_t
            search_mask = (t_rel_impact >= -2.0) & (t_rel_impact <= 2.0)
            df_search = df_imu[search_mask].copy()
            
            if df_search.empty:
                print(f"Skipping {flight_name} (No IMU data near impact)")
                continue
                
            # ── Dynamically find the actual acceleration peak ───────────────────
            # The MoCap-based 'Column Impact' time (speed peak near closest approach)
            # is a good first estimate, but the actual physical shock can be offset
            # by several ms due to the speed-peak search window. To guarantee every
            # flight's IMU trace is perfectly aligned at the moment of collision,
            # we find the absolute maximum of |a_deviation| in a tight window
            # around the MoCap estimate and re-anchor to that peak.
            #
            # Tight window: [-0.5, +0.5]s around the MoCap impact_t. This is wide
            # enough to catch any realistic offset but narrow enough to avoid picking
            # up spurious noise events from outside the collision window.
            peak_window_mask = (
                (df_search['a_deviation'].notna())
                & (df_search['t'] >= impact_t - 0.5)
                & (df_search['t'] <= impact_t + 0.5)
            )
            df_peak = df_search.loc[peak_window_mask]
            if not df_peak.empty:
                idx_max = df_peak['a_deviation'].idxmax()
                true_impact_t = df_peak.loc[idx_max, 't']
            else:
                true_impact_t = impact_t  # fallback to MoCap estimate

            # Recalculate relative time: t_rel = 0 at the true impact peak.
            # This is the time axis used in all aggregated IMU plots so that
            # every flight's shock signature is perfectly temporally aligned
            # and the median aggregation produces a sharp spike at t=0.
            df_search['t_rel'] = df_search['t'] - true_impact_t
            
            # Stage 2: final display crop to [-1.0, 2.0] s.
            # -1.0 s pre-impact captures the pre-collision baseline.
            # +2.0 s post-impact captures the full deceleration + recovery.
            mask_final = (df_search['t_rel'] >= -1.0) & (df_search['t_rel'] <= 2.0)
            df_crop = df_search[mask_final].copy()
            
            # ── Store only NumPy arrays, not DataFrames ───────────────────
            # .values extracts the underlying NumPy array, which pickles far
            # more compactly than a DataFrame (no index, column names, or
            # metadata overhead). Three arrays per flight × 179 flights ≈
            # a few MB total vs potentially 100+ MB with full DataFrames.
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
            
    # ── Serialise all aligned traces to a single pickle file ────────────
    # A single pickle of ~179 dicts with three small NumPy arrays each loads
    # in ~0.2 s. This is the entire point of the cache: downstream plotting
    # code calls pickle.load() instead of re-running this 180 s pipeline.
    with open(cache_path, 'wb') as f:
        pickle.dump(cached_data, f)
        
    print(f"Saved {len(cached_data)} IMU traces to {cache_path}")

if __name__ == "__main__":
    extract_and_cache_imu()
