#!/usr/bin/env python3
import os
import sys
import csv
import json

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(script_dir))
    
    config_path = os.path.join(project_root, "config", "drone_config.json")
    csv_dir = os.path.join(project_root, "dev_logs", "paths")
    csv_path = os.path.join(csv_dir, "ghost_path_latest.csv")
    
    if not os.path.exists(csv_path):
        print(f"❌ Error: Latest boundary CSV not found at: {csv_path}")
        print("Please run 'python3 dev_logs/ghost_flight.py' first to record the boundary!")
        sys.exit(1)
        
    print(f"🔍 Reading trajectory boundary data from: {csv_path}")
    
    xs, ys, zs = [], [], []
    
    try:
        with open(csv_path, mode='r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # NED x is North (ENU y), NED y is East (ENU x)
                xs.append(float(row['y'])) # East is ENU X
                ys.append(float(row['x'])) # North is ENU Y
                # NED z is Down (negative is UP in ENU)
                zs.append(-float(row['z'])) # Up is ENU Z
    except Exception as e:
        print(f"❌ Error reading CSV: {e}")
        sys.exit(1)
        
    if not xs:
        print("❌ Error: No coordinate points found in CSV file!")
        sys.exit(1)
        
    # Calculate physical limits
    phys_x_min, phys_x_max = min(xs), max(xs)
    phys_y_min, phys_y_max = min(ys), max(ys)
    phys_z_min, phys_z_max = min(zs), max(zs)
    
    # 5cm Safety Buffer requested by the user
    margin = 0.05
    
    geo_x_min = phys_x_min + margin
    geo_x_max = phys_x_max - margin
    geo_y_min = phys_y_min + margin
    geo_y_max = phys_y_max - margin
    geo_z_min = phys_z_min + margin # Floor margin
    geo_z_max = phys_z_max - margin # Ceiling margin
    
    print("\n=== CALIBRATED PHYSICAL ROOM BOUNDS ===")
    print(f"   X (East) : {phys_x_min:6.3f}m  to  {phys_x_max:6.3f}m (Span: {phys_x_max-phys_x_min:.2f}m)")
    print(f"   Y (North): {phys_y_min:6.3f}m  to  {phys_y_max:6.3f}m (Span: {phys_y_max-phys_y_min:.2f}m)")
    print(f"   Z (Height): {phys_z_min:6.3f}m  to  {phys_z_max:6.3f}m (Span: {phys_z_max-phys_z_min:.2f}m)")
    
    print(f"\n=== GEOFENCE LIMITS (With {margin*100:.0f}cm Buffer) ===")
    print(f"   X Bounds: {geo_x_min:6.3f}m  to  {geo_x_max:6.3f}m")
    print(f"   Y Bounds: {geo_y_min:6.3f}m  to  {geo_y_max:6.3f}m")
    print(f"   Z Bounds: {geo_z_min:6.3f}m  to  {geo_z_max:6.3f}m")
    
    # Load and update drone_config.json
    if not os.path.exists(config_path):
        print(f"❌ Error: Config file not found at {config_path}")
        sys.exit(1)
        
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"❌ Error parsing drone_config.json: {e}")
        sys.exit(1)
        
    # Inject calibrated bounds
    config["mocap_geofence"] = {
        "x_min": round(geo_x_min, 4),
        "x_max": round(geo_x_max, 4),
        "y_min": round(geo_y_min, 4),
        "y_max": round(geo_y_max, 4),
        "z_min": round(geo_z_min, 4),
        "z_max": round(geo_z_max, 4),
        "safety_margin": margin
    }
    
    try:
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"\n💾 Calibrated bounds successfully saved directly to config/drone_config.json!")
    except Exception as e:
        print(f"❌ Error saving to config/drone_config.json: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
