import os
import sys
import numpy as np
import pandas as pd
from IPython.display import display, HTML

from .exa_loader import load_drone_metadata, load_mcap, build_dataframes
from .exa_kinematics import compute_velocity, find_waypoint_events, build_events_log, calculate_metrics
from .exa_plot_trajectory import plot_trajectory
from .exa_plot_kinematics import plot_velocity_profile, plot_battery_sag, plot_imu_dynamics, plot_imu_xyz_components, plot_tangential_accel
from .exa_plot_statistics import plot_angle_boxplots

def run(label, angle_deg, column_x=0.408, column_y=0.358, 
        flights_cage=None, flights_no_cage=None, 
        representative_cage=0, representative_no_cage=0, 
        project_root=None):
    """Orchestrates the loading, analysis, and visualization pipeline for one angle.
    Processes both cage and no-cage passes, prints event tables, and draws
    representative plots and comparative statistics boxplots.
    """
    if flights_cage is None:
        flights_cage = []
    if flights_no_cage is None:
        flights_no_cage = []

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
    def process_flights(flight_folder_names):
        metrics_list = []
        details_list = []
        
        for idx, f_name in enumerate(flight_folder_names):
            flight_path = os.path.join(flights_dir, f_name)
            if not os.path.exists(flight_path):
                # Retry relative to workspace directly
                flight_path = os.path.join(project_root, f_name)
                
            if not os.path.exists(flight_path):
                print(f"[WARN] Flight directory not found: {f_name}, skipping.")
                continue

            try:
                # Load and process MCAP
                topic_data, bag_start_ns = load_mcap(flight_path)
                dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
                
                df_mocap = dfs['mocap']
                df_bat = dfs['battery']
                df_imu = dfs.get('imu', pd.DataFrame())
                df_column = dfs.get('column', pd.DataFrame())
                arming_time = dfs['arming_time']
                disarming_time = dfs['disarming_time']
                
                # Compute velocity derivative signals
                df_mocap = compute_velocity(df_mocap)
                
                # Determine column position for this flight dynamically
                if df_column is not None and not df_column.empty:
                    col_x_flight = df_column['x'].head(50).mean()
                    col_y_flight = df_column['y'].head(50).mean()
                    print(f"📍 [{f_name}] Detected Live Column Position: ({col_x_flight:.3f}, {col_y_flight:.3f})")
                else:
                    col_x_flight = column_x
                    col_y_flight = column_y
                    print(f"📍 [{f_name}] Column not found in poses. Falling back to default: ({col_x_flight:.3f}, {col_y_flight:.3f})")
                
                # Detect Takeoff Trigger (using MoCap Z height crossing 0.15m)
                takeoff_mask = df_mocap['z'] > 0.15 if not df_mocap.empty else pd.Series(dtype=bool)
                takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0
                
                # Detect waypoint events with dynamic column Y coordinate and impact detection (get ALL passes)
                wp_events_list = find_waypoint_events(
                    df_mocap, takeoff_time, label=f_name, 
                    column_x=col_x_flight, column_y=col_y_flight, 
                    column_radius=column_radius, cage_radius=cage_radius,
                    return_all=True
                )
                
                if not wp_events_list:
                    wp_events_list = [{}]
                
                for p_idx, wp_events in enumerate(wp_events_list):
                    pass_name = f"{f_name} - Pass-{p_idx+1:02d}"
                    print(f"🔄 Segmented implicit pass: {pass_name}")
                    
                    # Calculate physical metrics with dynamic column coordinates
                    metrics = calculate_metrics(df_mocap, wp_events, col_x_flight, col_y_flight, column_radius, cage_radius)
                    
                    metrics_list.append(metrics)
                    details_list.append({
                        'flight_name': pass_name,
                        'df_mocap': df_mocap,
                        'df_bat': df_bat,
                        'df_imu': df_imu,
                        'col_x_flight': col_x_flight,
                        'col_y_flight': col_y_flight,
                        'arming_time': arming_time,
                        'takeoff_time': takeoff_time,
                        'disarming_time': disarming_time,
                        'wp_events': wp_events,
                        'achieved_impact_angle': metrics['achieved_impact_angle']
                    })
            except Exception as e:
                print(f"[ERROR] Failed processing flight {f_name}: {e}")
                
        return metrics_list, details_list

    # Process all flights
    print("⏳ Processing 'Rotating Cage (Collision)' passes...")
    metrics_cage, details_cage = process_flights(flights_cage)
    print(f"✅ Successfully processed {len(metrics_cage)} passes.\n")

    print("⏳ Processing 'Rotating Cage (Safe Sweep)' passes...")
    metrics_nocage, details_nocage = process_flights(flights_no_cage)
    print(f"✅ Successfully processed {len(metrics_nocage)} passes.\n")

    # Render representative with cage flight
    if details_cage:
        rep_idx = min(max(0, representative_cage), len(details_cage) - 1)
        rep = details_cage[rep_idx]
        rep_metrics = metrics_cage[rep_idx]
        
        print(f"📈 --- Representative Pass: ROTATING CAGE COLLISION ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (WITH CAGE)")
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
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", f"trajectory_{label.lower().replace(' ', '_')}_cage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'])
        
        # Render velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Collision)", flight_name=rep['flight_name'])
        
        # Render tangential acceleration / deceleration profile
        plot_tangential_accel(rep['df_mocap'], rep['wp_events'], rep['arming_time'],
                              rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Collision)", flight_name=rep['flight_name'])
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], label="Rotating Cage (Collision)", flight_name=rep['flight_name'])
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Collision)", flight_name=rep['flight_name'])
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Collision)", flight_name=rep['flight_name'])
        print("\n" + "="*80 + "\n")
 
    # Render representative no cage flight
    if details_nocage:
        rep_idx = min(max(0, representative_no_cage), len(details_nocage) - 1)
        rep = details_nocage[rep_idx]
        rep_metrics = metrics_nocage[rep_idx]
        
        print(f"📈 --- Representative Pass: ROTATING CAGE SAFE SWEEP ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (WITHOUT CAGE)")
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
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", f"trajectory_{label.lower().replace(' ', '_')}_nocage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'])
        
        # Render velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Safe Sweep)", flight_name=rep['flight_name'])
        
        # Render tangential acceleration / deceleration profile
        plot_tangential_accel(rep['df_mocap'], rep['wp_events'], rep['arming_time'],
                              rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Safe Sweep)", flight_name=rep['flight_name'])
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], label="Rotating Cage (Safe Sweep)", flight_name=rep['flight_name'])
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Safe Sweep)", flight_name=rep['flight_name'])
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage (Safe Sweep)", flight_name=rep['flight_name'])
        print("\n" + "="*80 + "\n")

    # Render side-by-side comparative boxplots
    if metrics_cage or metrics_nocage:
        plot_angle_boxplots(metrics_cage, metrics_nocage, label=label)

    return {
        'label': label,
        'angle_deg': angle_deg,
        'metrics_cage': metrics_cage,
        'metrics_no_cage': metrics_nocage
    }
