from .base_mission import BaseMission

class RectangleFlightRelativePos(BaseMission):
    MISSION_NAME = "Rectangle Flight (Relative)"
    MISSION_DESCRIPTION = "Flies a rectangle pattern using relative offsets from the takeoff position."
    
    # We want geofence checking ON for this standard mission
    ENFORCE_GEOFENCE = True

    def __init__(self, relative_pos_x=0.8, relative_pos_y=0.8, relative_pos_z_hover=0.5, hold_time=2.0, face_forward=True):
        super().__init__()
        self.relative_pos_x = relative_pos_x
        self.relative_pos_y = relative_pos_y
        self.relative_pos_z_hover = relative_pos_z_hover
        self.hold_time = hold_time
        
        # This boolean tells the FlightDirector whether to auto-yaw towards the next waypoint
        self.face_forward = face_forward

    def on_start(self, initial_mocap_pos):
        x0 = initial_mocap_pos.x
        y0 = initial_mocap_pos.y
        h = initial_mocap_pos.z + self.relative_pos_z_hover

        # Waypoints format: (Absolute X, Absolute Y, Absolute Z, Face Forward Bool, Speed m/s, Pause At Waypoint Bool)
        # We pass self.face_forward for moving segments, but False for hovering/landing to hold orientation
        s = 0.2 # 0.2 m/s explicit speed limit
        self.waypoints = [
            (x0, y0, h, False, s, True),                                                # WP0: Hover at start (INTERRUPT)
            (x0 + self.relative_pos_x, y0, h, self.face_forward, s, False),             # WP1: Forward X
            (x0 + self.relative_pos_x, y0 + self.relative_pos_y, h, self.face_forward, s, False), # WP2: Left Y
            (x0, y0 + self.relative_pos_y, h, self.face_forward, s, False),             # WP3: Backward X
            (x0, y0, h, self.face_forward, s, False),                                   # WP4: Return Origin
            (x0, y0, h, False, s, True)                                                 # WP5: Final Hover (INTERRUPT)
        ]
        
        print(f"\n[MISSION] RectangleFlightRelativePos Initialized!")
        print(f" - Relative Size: {self.relative_pos_x}m x {self.relative_pos_y}m")
        print(f" - Target Altitude: {h:.2f}m absolute MoCap Z")
        print(f" - Face Forward: {self.face_forward}")
