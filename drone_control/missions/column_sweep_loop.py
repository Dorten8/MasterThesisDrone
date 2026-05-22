import os
import json
from .base_mission import BaseMission

class ColumnSweepLoop(BaseMission):
    MISSION_NAME = "Column Sweep Loop (Hardcoded)"
    MISSION_DESCRIPTION = "Take off, transit to WP1, sweep WP1->WP2->WP3->WP4, then pause and repeat."
    
    # Enforce safety geofence checking
    ENFORCE_GEOFENCE = True

    def __init__(self, target_z=0.5, transit_speed=0.3, sweep_speed=0.4, climb_speed=0.4, face_forward=True):
        super().__init__()
        # Explicit absolute MoCap coordinates (ENU Poses)
        # Shifted Y from 1.350 to 1.200 to clear the 1.50m geofence with drone's physical 17.9cm cage radius
        self.wp1 = (0.000, 1.200, target_z)
        self.wp2 = (0.100, 1.200, target_z)
        self.wp3 = (0.100, -1.200, target_z)
        self.wp4 = (0.000, -1.200, target_z)
        
        self.target_z = target_z
        self.transit_speed = transit_speed
        self.sweep_speed = sweep_speed
        self.climb_speed = climb_speed
        self.face_forward = face_forward

    def on_start(self, initial_mocap_pos):
        x0 = initial_mocap_pos.x
        y0 = initial_mocap_pos.y
        z0 = initial_mocap_pos.z
        
        cs = self.climb_speed
        ts = self.transit_speed
        ss = self.sweep_speed
        ff = self.face_forward
        
        # Waypoints format: (Absolute X, Absolute Y, Absolute Z, Face Forward Bool, Speed m/s, Pause At Waypoint Bool)
        self.waypoints = [
            (x0, y0, self.target_z, False, cs, True),                 # WP0: Takeoff and wait
            (self.wp1[0], self.wp1[1], self.wp1[2], ff, ts, True),    # WP1: Transit to Start, wait
            (self.wp2[0], self.wp2[1], self.wp2[2], ff, ss, False),   # WP2: Sweep step 1
            (self.wp3[0], self.wp3[1], self.wp3[2], ff, ss, False),   # WP3: Sweep step 2 (Long leg)
            (self.wp4[0], self.wp4[1], self.wp4[2], ff, ss, True)     # WP4: Sweep step 3, wait before looping
        ]
        
        print(f"\n=============================================")
        print(f"🎯 MISSION: HARDCODED COLUMN SWEEP LOOP")
        print(f"=============================================")
        print(f"  🏁 Takeoff Pad : X: {x0:.3f}m | Y: {y0:.3f}m | Z: {z0:.3f}m")
        print(f"  -------------------------------------------")
        print(f"  📍 WP1: X={self.wp1[0]:.3f}, Y={self.wp1[1]:.3f}, Z={self.wp1[2]:.3f} (PAUSE)")
        print(f"  📍 WP2: X={self.wp2[0]:.3f}, Y={self.wp2[1]:.3f}, Z={self.wp2[2]:.3f}")
        print(f"  📍 WP3: X={self.wp3[0]:.3f}, Y={self.wp3[1]:.3f}, Z={self.wp3[2]:.3f}")
        print(f"  📍 WP4: X={self.wp4[0]:.3f}, Y={self.wp4[1]:.3f}, Z={self.wp4[2]:.3f} (PAUSE & LOOP)")
        print(f"  -------------------------------------------")
        print(f"  ⚡ Climb speed  : {cs:.2f} m/s")
        print(f"  ⚡ Transit speed: {ts:.2f} m/s")
        print(f"  ⚡ Sweep speed  : {ss:.2f} m/s")
        print(f"=============================================")

    def get_next_setpoint(self, current_mocap_pos, dt):
        if self.current_wp_idx >= len(self.waypoints):
            print("\n[MISSION] Loop completed! Restarting from Waypoint 1...")
            # Reset to Waypoint 1 (index 1 in the list, so we bypass takeoff WP0)
            self.current_wp_idx = 1
            self.wp_entered_time = None
            self.has_paused_at_current_wp = False

        return super().get_next_setpoint(current_mocap_pos, dt)
