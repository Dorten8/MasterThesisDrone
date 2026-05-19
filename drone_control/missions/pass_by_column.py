import os
import json
from .base_mission import BaseMission

class PassByColumn(BaseMission):
    MISSION_NAME = "Pass By Column (Absolute)"
    MISSION_DESCRIPTION = "Transits to a starting point, then flies a straight-line passing safely by the registered obstacle column."
    
    # Enforce safety geofence checking
    ENFORCE_GEOFENCE = True

    def __init__(self, target_x=0.0, start_y=-1.0, end_y=1.2, target_z=0.522, speed=0.2, hold_time=2.0, face_forward=True):
        super().__init__()
        self.target_x = target_x
        self.start_y = start_y
        self.end_y = end_y
        self.target_z = target_z
        self.speed = speed
        self.hold_time = hold_time
        self.face_forward = face_forward

    def on_start(self, initial_mocap_pos):
        x0 = initial_mocap_pos.x
        y0 = initial_mocap_pos.y
        z0 = initial_mocap_pos.z
        
        # Load physical metadata from drone_config.json for dynamic clearance display
        cage_diameter = 0.358  # Default fallback
        column_diameter = 0.09  # Default fallback
        
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.abspath(os.path.join(script_dir, "../../config/drone_config.json"))
            with open(config_path, 'r') as f:
                config = json.load(f)
                for body in config.get("tracked_bodies", []):
                    if body.get("name") == "jake_drone_frame_01":
                        cage_diameter = body.get("cage_diameter_m", cage_diameter)
                    elif body.get("name") == "jake_column_drone":
                        column_diameter = body.get("diameter_m", column_diameter)
        except Exception as e:
            print(f"[MISSION] [WARN] Could not parse config metadata: {e}")

        cage_radius = cage_diameter / 2.0
        column_radius = column_diameter / 2.0
        
        # Column X is hardcoded at 0.41 for this absolute test
        column_x = 0.41
        horizontal_dist = abs(column_x - self.target_x)
        clearance = horizontal_dist - cage_radius - column_radius

        # Waypoints format: (Absolute X, Absolute Y, Absolute Z, Face Forward Bool, Speed m/s, Pause At Waypoint Bool)
        # s is our speed limit
        s = self.speed
        
        self.waypoints = [
            (x0, y0, self.target_z, False, s, True),                                   # WP0: Hover at current position (INTERRUPT)
            (self.target_x, self.start_y, self.target_z, self.face_forward, s, False), # WP1: Transit smoothly to Point A (Start)
            (self.target_x, self.start_y, self.target_z, False, s, True),              # WP2: Hold/Align at Point A (INTERRUPT - USER DOUBLE-CHECK)
            (self.target_x, self.end_y, self.target_z, self.face_forward, s, False),   # WP3: Execute straight-line Pass-By to Point B
            (self.target_x, self.end_y, self.target_z, False, s, True)                 # WP4: Final Hold at Point B (INTERRUPT)
        ]
        
        print(f"\n=============================================")
        print(f"🎯 MISSION: PASS BY COLUMN ACTIVE")
        print(f"=============================================")
        print(f"  🏁 Takeoff Coordinates : X: {x0:.3f}m | Y: {y0:.3f}m | Z: {z0:.3f}m")
        print(f"  📍 Transit Start Point A: X: {self.target_x:.3f}m | Y: {self.start_y:.3f}m | Z: {self.target_z:.3f}m")
        print(f"  📍 Passing End Point B  : X: {self.target_x:.3f}m | Y: {self.end_y:.3f}m | Z: {self.target_z:.3f}m")
        print(f"  ⚡ Explicit Speed Limit : {s:.2f} m/s")
        print(f"  -------------------------------------------")
        print(f"  🛠️ SAFETY METRICS PRE-CALCULATION:")
        print(f"    - Drone Cage Radius : {cage_radius*100.0:.1f} cm")
        print(f"    - Column Radius     : {column_radius*100.0:.1f} cm")
        print(f"    - Trajectory Offset : {horizontal_dist*100.0:.1f} cm")
        print(f"    - 💨 Air Clearance   : \033[92m{clearance*100.0:.1f} cm\033[0m")
        print(f"=============================================")
