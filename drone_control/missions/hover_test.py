from .base_mission import BaseMission

class HoverTest(BaseMission):
    MISSION_NAME = "Hover Test"
    MISSION_DESCRIPTION = "Takes off and hovers 0.5m above the starting MoCap position."
    
    ENFORCE_GEOFENCE = True

    def __init__(self, relative_z_hover=0.5):
        super().__init__()
        self.relative_z_hover = relative_z_hover

    def on_start(self, initial_mocap_pos):
        x0 = initial_mocap_pos.x
        y0 = initial_mocap_pos.y
        h = initial_mocap_pos.z + self.relative_z_hover
        
        # Waypoints format: (Absolute X, Absolute Y, Absolute Z, Face Forward Bool, Speed m/s, Pause At Waypoint Bool)
        self.waypoints = [
            (x0, y0, h, False, 0.2, True) # 0.2 m/s explicit takeoff speed, pause indefinitely to hover
        ]
        print(f"\n[MISSION] HoverTest started! Target absolute altitude: {h:.2f}m (MoCap Z).")

