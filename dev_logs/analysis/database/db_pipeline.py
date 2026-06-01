import os
import sys
import numpy as np
import pandas as pd
from IPython.display import display, HTML

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.kinematics.kin_calculator import compute_velocity, find_waypoint_events, build_events_log, calculate_metrics
from dev_logs.analysis.kinematics.kin_plot_trajectory import plot_trajectory
from dev_logs.analysis.kinematics.kin_plot_kinematics import plot_velocity_profile, plot_battery_sag, plot_imu_dynamics, plot_imu_xyz_components, plot_tangential_accel
from dev_logs.analysis.kinematics.kin_plot_statistics import plot_angle_boxplots
from dev_logs.analysis.database.db_manager import insert_or_replace_flight, get_database_summary_markdown, is_already_cached


def run(label, angle_deg, column_x=0.408, column_y=0.358, 
        flights_rotating_cage=None, flights_fixed_cage=None, 
        representative_rotating_cage=0, representative_fixed_cage=0, 
        project_root=None, **kwargs):
    """Orchestrates the loading, analysis, and visualization pipeline for one angle.
    Processes both rotating cage and fixed cage passes, prints event tables, and draws
    representative plots and comparative statistics boxplots.
    """
    # Map old aliases if provided for robustness
    if flights_rotating_cage is None:
        flights_rotating_cage = kwargs.get('flights_cage', None)
    if flights_fixed_cage is None:
        flights_fixed_cage = kwargs.get('flights_no_cage', None)

    if flights_rotating_cage is None:
        flights_rotating_cage = []
    if flights_fixed_cage is None:
        flights_fixed_cage = []

    representative_rotating_cage = kwargs.get('representative_cage', representative_rotating_cage)
    representative_fixed_cage = kwargs.get('representative_no_cage', representative_fixed_cage)

    # 1. Resolve project root and flights path dynamically
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if project_root is None:
        project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

    flights_dir = os.path.join(project_root, "dev_logs", "flights")

    # Load SSoT drone configurations
    drone_tracker_name, system_config = load_drone_metadata(project_root)
    primary_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"), {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    
    cage_diameter = primary_body.get("cage_diameter_m", 0.358)
    column_diameter = obstacle_body.get("diameter_m", 0.09)
    cage_radius = cage_diameter / 2.0
    column_radius = column_diameter / 2.0

    print(f"🎬 Starting Analysis Pipeline for: {label} ({angle_deg}°)")
    print(f"🔧 Loaded SSoT Configs: Tracker='{drone_tracker_name}', Cage D={cage_diameter*100:.1f}cm, Column D={column_diameter*100:.1f}cm\n")

    # Helper to process a list of flight folders
    def process_flights(flight_folder_names, condition_label, force_plot=False):
        metrics_list = []
        details_list = []
        
        import glob
        import re

        pass_files_to_process = []
        for f_name in flight_folder_names:
            m_pass = re.search(r'^(.*?) - Pass-(\d+)$', f_name)
            if m_pass:
                base_folder = m_pass.group(1)
                pass_idx = int(m_pass.group(2))
                
                dir_path = os.path.join(flights_dir, base_folder)
                if not os.path.exists(dir_path):
                    dir_path = os.path.join(project_root, base_folder)
                
                pattern = os.path.join(dir_path, f"*-pass{pass_idx:02d}.mcap")
                matches = glob.glob(pattern)
                if matches:
                    pass_files_to_process.append((base_folder, pass_idx, matches[0]))
                else:
                    print(f"[WARN] Pass file not found for: {f_name}")
            else:
                base_folder = f_name
                dir_path = os.path.join(flights_dir, base_folder)
                if not os.path.exists(dir_path):
                    dir_path = os.path.join(project_root, base_folder)
                
                pattern = os.path.join(dir_path, "*-pass*.mcap")
                matches = sorted(glob.glob(pattern))
                if matches:
                    for m_path in matches:
                        m = re.search(r'-pass(\d+)\.mcap$', os.path.basename(m_path))
                        if m:
                            p_idx = int(m.group(1))
                            pass_files_to_process.append((base_folder, p_idx, m_path))
                else:
                    print(f"[WARN] No pass files found in directory: {f_name}")

        for base_folder, pass_idx, pass_path in pass_files_to_process:
            try:
                # Load and process MCAP
                topic_data, bag_start_ns = load_mcap(pass_path)
                dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
                
                df_mocap = dfs['mocap']
                df_setpoint = dfs.get('setpoint', pd.DataFrame())
                df_bat = dfs['battery']
                df_imu = dfs.get('imu', pd.DataFrame())
                df_column = dfs.get('column', pd.DataFrame())
                arming_time = dfs['arming_time']
                disarming_time = dfs['disarming_time']
                mocap_rate = dfs.get('mocap_rate', 240.0)
                dynamic_waypoints = dfs.get('dynamic_waypoints', [])
                
                # Compute both raw and smoothed velocity derivative signals
                df_mocap_raw = compute_velocity(df_mocap.copy(), resample=False)
                df_mocap = compute_velocity(df_mocap, resample=True)
                
                # Determine column position for this flight dynamically
                if df_column is not None and not df_column.empty:
                    col_x_flight = df_column['x'].head(50).mean()
                    col_y_flight = df_column['y'].head(50).mean()
                    print(f"📍 [{base_folder}] Detected Live Column Position: ({col_x_flight:.3f}, {col_y_flight:.3f})")
                else:
                    col_x_flight = column_x
                    col_y_flight = column_y
                    print(f"📍 [{base_folder}] Column not found in poses. Falling back to default: ({col_x_flight:.3f}, {col_y_flight:.3f})")
                
                # Detect Takeoff Trigger (using MoCap Z height crossing 0.15m)
                takeoff_mask = df_mocap['z'] > 0.15 if not df_mocap.empty else pd.Series(dtype=bool)
                takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0
                
                # Detect waypoint events with dynamic column Y coordinate and impact detection (get ALL passes)
                wp_events_list = find_waypoint_events(
                    df_mocap, df_setpoint, takeoff_time, label=f"{base_folder} - Pass-{pass_idx:02d}", 
                    column_x=col_x_flight, column_y=col_y_flight, 
                    column_radius=column_radius, cage_radius=cage_radius,
                    return_all=True, dynamic_waypoints=dynamic_waypoints
                )
                
                if not wp_events_list:
                    wp_events_list = [{}]
 
                # --- Resolve declared sweep_speed from the mission class ---
                sweep_speed = None
                try:
                    from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                    _m = detect_mission_class(df_setpoint, label=base_folder)
                    sweep_speed = _m.sweep_speed  # e.g. 0.3 m/s
                except Exception:
                    pass  # Not critical — falls back to None in the DB
 
                for p_idx, wp_events in enumerate(wp_events_list):
                    pass_name = f"{base_folder} - Pass-{pass_idx:02d}"
 
                    # Only write a complete pass (WP1 + WP2 + WP3 all detected)
                    has_full_pass = ('WP1' in wp_events or 'WP2' in wp_events) and 'WP3' in wp_events
                    if not has_full_pass:
                        print(f"⚠️  Incomplete pass detected ({pass_name}), skipping DB insert.")
                        continue

                    print(f"🔄 Segmented implicit pass: {pass_name}")

                    # Calculate physical metrics with dynamic column coordinates
                    metrics = calculate_metrics(df_mocap, wp_events, col_x_flight, col_y_flight, column_radius, cage_radius, df_imu=df_imu)
                    closest_clearance = metrics.get('closest_clearance')
                    metrics['impact_detected'] = 1 if (closest_clearance is not None and closest_clearance < 0.0) else 0

                    # Read declared nominal waypoints from mission SSoT
                    try:
                        from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                        _m_dyn = detect_mission_class(df_setpoint, label=base_folder)
                        nom_sp = (_m_dyn.exp_sp[0], _m_dyn.exp_sp[1], _m_dyn.exp_sp[2])
                        nom_ep = (_m_dyn.exp_ep[0], _m_dyn.exp_ep[1], _m_dyn.exp_ep[2])
                    except Exception:
                        is_45 = base_folder and "45" in base_folder.lower()
                        x_lane = 0.248 if is_45 else 0.186
                        nom_sp = (x_lane, 0.950, 0.500)
                        nom_ep = (x_lane, -1.200, 0.500)
                    metrics['nom_sp'] = nom_sp
                    metrics['nom_ep'] = nom_ep

                    # Compute actual physical start/end points using MoCap trajectory at wp_events timestamps
                    def _get_mocap_coords(df, t):
                        if df.empty or t is None or np.isnan(t):
                            return (None, None, None)
                        idx = (df['t'] - t).abs().idxmin()
                        return (float(df.loc[idx, 'x']), float(df.loc[idx, 'y']), float(df.loc[idx, 'z']))

                    metrics['act_sp'] = _get_mocap_coords(df_mocap, wp_events.get('WP2') or wp_events.get('WP1'))
                    metrics['act_ep'] = _get_mocap_coords(df_mocap, wp_events.get('WP3'))

                    # Attach declared sweep speed to metrics
                    metrics['sweep_speed'] = sweep_speed

                    # Battery % at Exp. Start-point (WP1 / WP2, whichever is earliest available)
                    t_exp_start = wp_events.get('WP1') or wp_events.get('WP2')
                    if t_exp_start is not None and not df_bat.empty:
                        from dev_logs.analysis.kinematics.kin_calculator import query_battery
                        bat_pct_str, _ = query_battery(df_bat, t_exp_start)
                        try:
                            metrics['battery_at_start'] = float(bat_pct_str.replace('%', ''))
                        except (ValueError, AttributeError):
                            metrics['battery_at_start'] = None
                    else:
                        metrics['battery_at_start'] = None

                    # Skip if already cached in the database (idempotent runs, force-recompute if missing schema columns)
                    if is_already_cached(pass_name, check_columns=['impact_detected', 'nom_sp_x', 'before_impact_accel', 'imu_peak_accel', 'imu_vib_ay']):
                        print(f"⏭️  '{pass_name}' already in database, skipping insert.")
                    else:
                        insert_or_replace_flight(pass_name, condition_label, metrics)

                    # Generate and save all 5 high-fidelity plots to the individual flight capsule directory
                    flight_dir = os.path.dirname(pass_path)
                    
                    if metrics.get('impact_detected', 0) == 1 or force_plot:
                        pass_prefix = f"pass{pass_idx:02d}_"
                        
                        # 1. Trajectory Top-Down 2D Spatial Plot
                        traj_capsule_path = os.path.join(flight_dir, f"{pass_prefix}trajectory_top_down.png")
                        plot_trajectory(df_mocap, wp_events, col_x_flight, col_y_flight, 
                                        cage_diameter, column_diameter, output_path=traj_capsule_path, flight_name=pass_name,
                                        dynamic_waypoints=dynamic_waypoints, df_column=df_column,
                                        df_setpoint=df_setpoint, show_plot=False)
                        
                        # Build segment events list for velocity/kinetic plots
                        events_log = build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=metrics.get('achieved_impact_angle'))
                        
                        # 2. Kinetic Profile (3-Panel stack: Velocity, Tangential Accel, MoCap Rate)
                        kinetic_capsule_path = os.path.join(flight_dir, f"{pass_prefix}kinetic_profile.png")
                        plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, 
                                              label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                              mocap_rate=mocap_rate, condition=condition_label, output_path=kinetic_capsule_path, show_plot=False)
                        
                        # 2b. Raw Unsmoothed Kinetic Profile
                        kinetic_raw_capsule_path = os.path.join(flight_dir, f"{pass_prefix}kinetic_profile_raw.png")
                        plot_velocity_profile(df_mocap_raw, wp_events, arming_time, takeoff_time, disarming_time, events_log, 
                                              label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                              mocap_rate=mocap_rate, condition=condition_label, output_path=kinetic_raw_capsule_path, show_plot=False)
                        
                        # 3. Battery Voltage Sag Profile
                        battery_capsule_path = os.path.join(flight_dir, f"{pass_prefix}battery_sag.png")
                        plot_battery_sag(df_bat, takeoff_time, wp_events, arming_time, label=condition_label, flight_name=pass_name,
                                         achieved_angle=metrics.get('achieved_impact_angle'), output_path=battery_capsule_path, show_plot=False)
                        
                        # 4. Physical IMU Dynamics Plot (Acceleration & Gyro deviation/surge)
                        imu_dyn_capsule_path = os.path.join(flight_dir, f"{pass_prefix}imu_dynamics.png")
                        plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                          label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                          output_path=imu_dyn_capsule_path, show_plot=False)
                        
                        # 5. Raw IMU XYZ Components Plot
                        imu_xyz_capsule_path = os.path.join(flight_dir, f"{pass_prefix}imu_xyz_components.png")
                        plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                                label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                                output_path=imu_xyz_capsule_path, show_plot=False)
                    else:
                        print(f"⏭️  No collision detected in {pass_name}, skipping plot generation.")

                    metrics_list.append(metrics)
                    details_list.append({
                        'flight_name': pass_name,
                        'df_mocap': df_mocap,
                        'df_mocap_raw': df_mocap_raw,
                        'df_bat': df_bat,
                        'df_imu': df_imu,
                        'df_column': df_column,
                        'df_setpoint': df_setpoint,
                        'col_x_flight': col_x_flight,
                        'col_y_flight': col_y_flight,
                        'arming_time': arming_time,
                        'takeoff_time': takeoff_time,
                        'disarming_time': disarming_time,
                        'wp_events': wp_events,
                        'achieved_impact_angle': metrics['achieved_impact_angle'],
                        'mocap_rate': mocap_rate,
                        'dynamic_waypoints': dynamic_waypoints
                    })
            except Exception as e:
                print(f"[ERROR] Failed processing flight {f_name}: {e}")
                import traceback; traceback.print_exc()

        return metrics_list, details_list

    force_plot = kwargs.get('force_plot', False)

    # Process all flights
    print("⏳ Processing 'Rotating Cage' passes...")
    metrics_rotating, details_rotating = process_flights(flights_rotating_cage, "Rotating Cage", force_plot=force_plot)
    print(f"✅ Successfully processed {len(metrics_rotating)} passes.\n")

    print("⏳ Processing 'Fixed Cage' passes...")
    metrics_fixed, details_fixed = process_flights(flights_fixed_cage, "Fixed Cage", force_plot=force_plot)
    print(f"✅ Successfully processed {len(metrics_fixed)} passes.\n")

    # Render representative rotating cage flight
    if details_rotating:
        rep_idx = min(max(0, representative_rotating_cage), len(details_rotating) - 1)
        rep = details_rotating[rep_idx]
        rep_metrics = metrics_rotating[rep_idx]
        
        print(f"📈 --- Representative Pass: ROTATING CAGE ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (ROTATING CAGE)")
        display(HTML(df_events_table.to_html(index=False)))
        
        if rep_metrics.get('achieved_impact_angle') is not None:
            impact_str = f"💥 Column Impact (Achieved Angle: {rep_metrics['achieved_impact_angle']:.1f}°)"
            if rep_metrics.get('impact_speed') is not None:
                impact_str += f" - Speed at impact: {rep_metrics['impact_speed']:.3f} m/s"
            if rep_metrics.get('impact_accel') is not None:
                impact_str += f" - Acceleration at impact: {rep_metrics['impact_accel']:.3f} m/s²"
            print(impact_str)
            
        print("-" * 80)
        
        # Render trajectory plot using the flight's exact dynamic column coordinates
        clean_label = label.lower().replace(' ', '_').replace('°', 'deg')
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"trajectory_{clean_label}_rotating_cage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'],
                        dynamic_waypoints=rep['dynamic_waypoints'], df_column=rep['df_column'],
                        df_setpoint=rep.get('df_setpoint'))
        
        # Render raw velocity profile
        plot_velocity_profile(rep['df_mocap_raw'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Raw MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Rotating Cage (Raw)")
        
        # Render splined velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Splined MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Rotating Cage (Splined)")
        
        # Render tangential acceleration / deceleration profile
        plot_tangential_accel(rep['df_mocap'], rep['wp_events'], rep['arming_time'],
                              rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], rep['wp_events'], rep['arming_time'], label="Rotating Cage", flight_name=rep['flight_name'],
                         achieved_angle=rep_metrics.get('achieved_impact_angle'))
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage", flight_name=rep['flight_name'],
                          achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage", flight_name=rep['flight_name'],
                                achieved_angle=rep_metrics.get('achieved_impact_angle'))
        print("\n" + "="*80 + "\n")
  
    # Render representative fixed cage flight
    if details_fixed:
        rep_idx = min(max(0, representative_fixed_cage), len(details_fixed) - 1)
        rep = details_fixed[rep_idx]
        rep_metrics = metrics_fixed[rep_idx]
        
        print(f"📈 --- Representative Pass: FIXED CAGE ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (FIXED CAGE)")
        display(HTML(df_events_table.to_html(index=False)))
        
        if rep_metrics.get('achieved_impact_angle') is not None:
            impact_str = f"💥 Column Impact (Achieved Angle: {rep_metrics['achieved_impact_angle']:.1f}°)"
            if rep_metrics.get('impact_speed') is not None:
                impact_str += f" - Speed at impact: {rep_metrics['impact_speed']:.3f} m/s"
            if rep_metrics.get('impact_accel') is not None:
                impact_str += f" - Acceleration at impact: {rep_metrics['impact_accel']:.3f} m/s²"
            print(impact_str)
            
        print("-" * 80)
        
        # Render trajectory plot using the flight's exact dynamic column coordinates
        clean_label = label.lower().replace(' ', '_').replace('°', 'deg')
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"trajectory_{clean_label}_fixed_cage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'],
                        dynamic_waypoints=rep['dynamic_waypoints'], df_column=rep['df_column'],
                        df_setpoint=rep.get('df_setpoint'))
        
        # Render raw velocity profile
        plot_velocity_profile(rep['df_mocap_raw'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Raw MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Fixed Cage (Raw)")
        
        # Render splined velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Splined MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Fixed Cage (Splined)")
        
        # Render tangential acceleration / deceleration profile
        plot_tangential_accel(rep['df_mocap'], rep['wp_events'], rep['arming_time'],
                              rep['takeoff_time'], rep['disarming_time'], events, label="Fixed Cage", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], rep['wp_events'], rep['arming_time'], label="Fixed Cage", flight_name=rep['flight_name'],
                         achieved_angle=rep_metrics.get('achieved_impact_angle'))
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Fixed Cage", flight_name=rep['flight_name'],
                          achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Fixed Cage", flight_name=rep['flight_name'],
                                achieved_angle=rep_metrics.get('achieved_impact_angle'))
        print("\n" + "="*80 + "\n")

    # Render side-by-side comparative boxplots
    if metrics_rotating or metrics_fixed:
        plot_angle_boxplots(metrics_rotating, metrics_fixed, label=label)

    # Output consolidated thesis experiments summary from SQLite database
    print("\n" + "="*80)
    print("📚 CONSOLIDATED MASTER THESIS COLLISION EXPERIMENTS DATABASE SUMMARY")
    print("="*80)
    print(get_database_summary_markdown())
    print("="*80 + "\n")

    return {
        'label': label,
        'angle_deg': angle_deg,
        'metrics_rotating_cage': metrics_rotating,
        'metrics_fixed_cage': metrics_fixed
    }


# ─────────────────────────────────────────────────────────────────────────────
# Standalone DB population mode
# Run as:  python3 -m dev_logs.analysis.database.db_pipeline
#
# Scans dev_logs/flights/ for *-passXX.mcap files (only approved flights),
# runs analysis on each, and populates / updates the SQLite database.
# Already-cached passes are skipped (idempotent).
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import re as _re
    import glob as _glob
    import argparse
    import datetime

    # Dynamically resolve today's date formatted as YYYYMMDD-0000
    _today_cutoff = datetime.date.today().strftime("%Y%m%d-0000")

    parser = argparse.ArgumentParser(description="Standalone DB population pipeline")
    parser.add_argument('--cutoff', '-c', type=str, default="20260524-1904",
                        help="Skipping any flight directories recorded before this timestamp in YYYYMMDD-HHMM format (default: 20260524-1904)")
    parser.add_argument('--today', '-t', action='store_true',
                        help=f"Shortcut to process only today's flights (sets cutoff to {_today_cutoff})")
    parser.add_argument('--force-plot', '-f', action='store_true',
                        help="Force plot generation even for passes without detected collisions")
    
    args = parser.parse_args()

    APPROVED_CUTOFF = _today_cutoff if args.today else args.cutoff

    _current_dir = os.path.dirname(os.path.abspath(__file__))
    _project_root = os.path.abspath(os.path.join(_current_dir, "..", "..", ".."))
    _flights_dir  = os.path.join(_project_root, "dev_logs", "flights")

    drone_tracker_name, system_config = load_drone_metadata(_project_root)
    primary_body  = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"),  {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    cage_diameter   = primary_body.get("cage_diameter_m", 0.358)
    column_diameter = obstacle_body.get("diameter_m", 0.09)
    cage_radius   = cage_diameter / 2.0
    column_radius = column_diameter / 2.0

    def _infer_condition(folder_name):
        name = folder_name.lower()
        if "rotating" in name:
            return "Rotating Cage"
        return "Fixed Cage"

    def _flight_ts(folder_name):
        m = _re.search(r'flight_(\d{8}-\d{4})', folder_name)
        return m.group(1) if m else None

    # Collect all approved pass files
    all_pass_files = []
    for folder in sorted(os.listdir(_flights_dir)):
        ts = _flight_ts(folder)
        if ts is None or ts < APPROVED_CUTOFF:
            continue
        folder_path = os.path.join(_flights_dir, folder)
        passes = sorted(_glob.glob(os.path.join(folder_path, "*-pass*.mcap")))
        for p in passes:
            all_pass_files.append((folder, folder_path, p))

    if not all_pass_files:
        print("⚠️  No pass files found. Run mcap_segmenter.py first.")
        sys.exit(0)

    print(f"\n🗄️  DB Population Mode — found {len(all_pass_files)} pass file(s) across approved flights.\n")

    try:
        sys.path.insert(0, _project_root)
        from drone_control.missions.exp_collision_75deg import ExpCollision75Deg as _M
        sweep_speed_declared = _M().sweep_speed
    except Exception:
        sweep_speed_declared = None

    populated, skipped, errors = 0, 0, 0

    for folder_name, folder_path, pass_path in all_pass_files:
        pass_basename = os.path.basename(pass_path)
        # pass_name matches the DB primary key: "<folder_name> - Pass-XX"
        m = _re.search(r'-pass(\d+)\.mcap$', pass_basename)
        if not m:
            continue
        pass_idx  = int(m.group(1))
        pass_name = f"{folder_name} - Pass-{pass_idx:02d}"

        if is_already_cached(pass_name, check_columns=['impact_detected', 'nom_sp_x', 'before_impact_accel', 'imu_peak_accel', 'imu_vib_ay']):
            print(f"⏭️  Skipping (already cached): {pass_name}")
            skipped += 1
            continue

        print(f"\n🔄 Processing: {pass_name}")
        try:
            # Load the self-contained pass MCAP
            from mcap_ros2.reader import read_ros2_messages as _read_msgs
            topic_data = {}
            for msg in _read_msgs(pass_path):
                t = msg.channel.topic
                if t not in topic_data:
                    topic_data[t] = []
                topic_data[t].append(msg)

            if not topic_data:
                print(f"   ⚠️  Empty MCAP, skipping.")
                errors += 1
                continue

            bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)

            df_mocap  = dfs['mocap']
            df_setpoint = dfs.get('setpoint', pd.DataFrame())
            df_bat    = dfs['battery']
            df_imu    = dfs.get('imu', pd.DataFrame())
            df_column = dfs.get('column', pd.DataFrame())
            arming_time       = dfs['arming_time']
            disarming_time    = dfs.get('disarming_time', arming_time + 10.0)
            mocap_rate        = dfs.get('mocap_rate', 240.0)
            dynamic_waypoints = dfs.get('dynamic_waypoints', [])

            if df_mocap.empty:
                print(f"   ⚠️  No MoCap data, skipping.")
                errors += 1
                continue

            df_mocap_raw = compute_velocity(df_mocap.copy(), resample=False)
            df_mocap = compute_velocity(df_mocap, resample=True)

            takeoff_mask = df_mocap['z'] > 0.15
            takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0

            col_x, col_y = (df_column['x'].head(50).mean(), df_column['y'].head(50).mean()) \
                if (df_column is not None and not df_column.empty) else (0.408, 0.358)

            wp_events_list = find_waypoint_events(
                df_mocap, df_setpoint, takeoff_time, label=pass_name,
                column_x=col_x, column_y=col_y,
                column_radius=column_radius, cage_radius=cage_radius,
                return_all=True, dynamic_waypoints=dynamic_waypoints
            )

            if not wp_events_list:
                print(f"   ⚠️  No waypoint events found, skipping.")
                errors += 1
                continue

            wp_events = wp_events_list[0]  # Pass file contains exactly one pass

            metrics = calculate_metrics(df_mocap, wp_events, col_x, col_y, column_radius, cage_radius, df_imu=df_imu)
            closest_clearance = metrics.get('closest_clearance')
            metrics['impact_detected'] = 1 if (closest_clearance is not None and closest_clearance < 0.0) else 0

            # Read declared nominal waypoints from mission SSoT
            _m = None
            try:
                from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                _m = detect_mission_class(df_setpoint, label=pass_name)
                nom_sp = (_m.exp_sp[0], _m.exp_sp[1], _m.exp_sp[2])
                nom_ep = (_m.exp_ep[0], _m.exp_ep[1], _m.exp_ep[2])
            except Exception:
                is_45 = pass_name and "45" in pass_name.lower()
                x_lane = 0.248 if is_45 else 0.186
                nom_sp = (x_lane, 0.950, 0.500)
                nom_ep = (x_lane, -1.200, 0.500)
            metrics['nom_sp'] = nom_sp
            metrics['nom_ep'] = nom_ep

            # Compute actual physical start/end points using MoCap trajectory at wp_events timestamps
            def _get_mocap_coords(df, t):
                if df.empty or t is None or np.isnan(t):
                    return (None, None, None)
                idx = (df['t'] - t).abs().idxmin()
                return (float(df.loc[idx, 'x']), float(df.loc[idx, 'y']), float(df.loc[idx, 'z']))

            metrics['act_sp'] = _get_mocap_coords(df_mocap, wp_events.get('WP2') or wp_events.get('WP1'))
            metrics['act_ep'] = _get_mocap_coords(df_mocap, wp_events.get('WP3'))

            metrics['sweep_speed'] = _m.sweep_speed if _m is not None else sweep_speed_declared

            # Battery % at Exp. Start-point
            t_exp_start = wp_events.get('WP1') or wp_events.get('WP2')
            if t_exp_start is not None and not df_bat.empty:
                from dev_logs.analysis.kinematics.kin_calculator import query_battery as _qb
                bat_pct_str, _ = _qb(df_bat, t_exp_start)
                try:
                    metrics['battery_at_start'] = float(bat_pct_str.replace('%', ''))
                except (ValueError, AttributeError):
                    metrics['battery_at_start'] = None
            else:
                metrics['battery_at_start'] = None

            condition = _infer_condition(folder_name)
            insert_or_replace_flight(pass_name, condition, metrics)

            # Generate and save all 5 high-fidelity plots to the individual flight capsule directory
            if metrics.get('impact_detected', 0) == 1 or args.force_plot:
                pass_prefix = f"pass{pass_idx:02d}_"
                
                # 1. Trajectory Top-Down 2D Spatial Plot
                traj_capsule_path = os.path.join(folder_path, f"{pass_prefix}trajectory_top_down.png")
                plot_trajectory(df_mocap, wp_events, col_x, col_y, 
                                cage_diameter, column_diameter, output_path=traj_capsule_path, flight_name=pass_name,
                                dynamic_waypoints=dynamic_waypoints, df_column=df_column,
                                df_setpoint=df_setpoint, show_plot=False)
                
                # Build segment events list for velocity/kinetic plots
                events_log = build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=metrics.get('achieved_impact_angle'))
                
                # 2. Kinetic Profile (3-Panel stack: Velocity, Tangential Accel, MoCap Rate)
                kinetic_capsule_path = os.path.join(folder_path, f"{pass_prefix}kinetic_profile.png")
                plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, 
                                      label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                      mocap_rate=mocap_rate, condition=condition, output_path=kinetic_capsule_path, show_plot=False)
                
                # 2b. Raw Unsmoothed Kinetic Profile
                kinetic_raw_capsule_path = os.path.join(folder_path, f"{pass_prefix}kinetic_profile_raw.png")
                plot_velocity_profile(df_mocap_raw, wp_events, arming_time, takeoff_time, disarming_time, events_log, 
                                      label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                      mocap_rate=mocap_rate, condition=condition, output_path=kinetic_raw_capsule_path, show_plot=False)
                
                # 3. Battery Voltage Sag Profile
                battery_capsule_path = os.path.join(folder_path, f"{pass_prefix}battery_sag.png")
                plot_battery_sag(df_bat, takeoff_time, wp_events, arming_time, label=condition, flight_name=pass_name,
                                 achieved_angle=metrics.get('achieved_impact_angle'), output_path=battery_capsule_path, show_plot=False)
                
                # 4. Physical IMU Dynamics Plot (Acceleration & Gyro deviation/surge)
                imu_dyn_capsule_path = os.path.join(folder_path, f"{pass_prefix}imu_dynamics.png")
                plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                  label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                  output_path=imu_dyn_capsule_path, show_plot=False)
                
                # 5. Raw IMU XYZ Components Plot
                imu_xyz_capsule_path = os.path.join(folder_path, f"{pass_prefix}imu_xyz_components.png")
                plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                        label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                        output_path=imu_xyz_capsule_path, show_plot=False)
            else:
                print(f"   ⏭️  No collision detected in {pass_name}, skipping plot generation.")

            populated += 1

        except Exception as e:
            print(f"   ❌ Error: {e}")
            import traceback; traceback.print_exc()
            errors += 1

    print(f"\n{'='*80}")
    print(f"✅ DB population complete: {populated} inserted, {skipped} skipped, {errors} errors.")
    print(f"{'='*80}\n")
    print(get_database_summary_markdown())

