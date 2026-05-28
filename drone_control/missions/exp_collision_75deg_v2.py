import os
import json
from .base_mission import BaseMission

# ANSI colour codes for terminal output
_BLUE  = "\033[94m"
_RESET = "\033[0m"

class ExpCollision75DegV2(BaseMission):
    MISSION_NAME = "75° Column Collision Loop v2"
    MISSION_DESCRIPTION = (
        "Transit to WP_stage (U-turn), arrive at WP1 heading south, "
        "sweep at 75° angle (WP1→WP2) with shortened staging distance (10cm) "
        "giving the drone extra runway to stabilize, recover to WP3, auto-loop."
    )

    # Enforce safety geofence checking
    ENFORCE_GEOFENCE = True

    def __init__(self, target_z=0.5, transit_speed=0.3, sweep_speed=0.3, climb_speed=0.4, face_forward=True):
        super().__init__()
        # ── Geometry ─────────────────────────────────────────────────────────────
        # Column at (0.408, 0.358). Sweep lane: X=0.186.
        # WP_stage: pass-through U-turn point north of the gate.
        #
        # WP1 (exp_sp): sweep gate/pause. Moved to Y=1.100 to make distance to wp_stage
        #   exactly 10cm, giving the drone an extra 15cm of runway to stabilize.
        # ─────────────────────────────────────────────────────────────────────────
        self.wp_stage = (0.186, 1.200, target_z)  # U-turn staging  (pass-through, no stop)
        self.exp_sp   = (0.186, 1.100, target_z)  # Sweep gate      (PAUSE, 5cm, arrives heading south)
        self.exp_ep   = (0.186, -1.200, target_z) # Sweep end       (post-column exit)
        self.wp3      = (0.000, 0.300, target_z)  # Recovery        (auto-loops to WP_stage)

        self.target_z      = target_z
        self.transit_speed = transit_speed
        self.sweep_speed   = sweep_speed
        self.climb_speed   = climb_speed
        self.face_forward  = face_forward

    def on_start(self, initial_mocap_pos):
        x0 = initial_mocap_pos.x
        y0 = initial_mocap_pos.y
        z0 = initial_mocap_pos.z

        cs = self.climb_speed
        ts = self.transit_speed
        ss = self.sweep_speed
        ff = self.face_forward

        # Waypoint format: (x, y, z, face_forward, speed_m/s, pause_at_waypoint, arrival_threshold_m)
        self.waypoints = [
            (x0, y0, self.target_z, False, cs, True,  0.10),  # WP0:     Takeoff climb & hold
            (*self.wp_stage, ff, ts, False, 0.15),             # WP_stage: U-turn pass-through, 15cm
            (*self.exp_sp,      ff, ts, True,  0.05),             # WP1:      Gate, PAUSE, 5cm tight
            (*self.exp_ep,      ff, ss, False, 0.15),             # WP2:      75° sweep, 15cm
            (*self.wp3,      ff, ts, False, 0.15),             # WP3:      Recovery, auto-loop, 15cm
        ]

        g  = self.wp_stage
        w1 = self.exp_sp
        w2 = self.exp_ep
        w3 = self.wp3
        print(f"\n=============================================")
        print(f"🎯 MISSION: 75° COLUMN COLLISION LOOP V2")
        print(f"=============================================")
        print(f"  🏁 Takeoff Pad : X={x0:.3f}m | Y={y0:.3f}m | Z={z0:.3f}m")
        print(f"  -------------------------------------------")
        print(f"  📍 WP0 (Takeoff)      : X={x0:.3f},  Y={y0:.3f},  Z={self.target_z:.3f}  [ONE-TIME CLIMB · PAUSE]")
        print(f"  📍 WP_stage (U-turn)  : X={g[0]:.3f},  Y={g[1]:.3f},  Z={g[2]:.3f}  [PASS-THROUGH · 15cm]")
        print(f"  📍 WP1 (Gate)         : X={w1[0]:.3f}, Y={w1[1]:.3f}, Z={w1[2]:.3f}  [PAUSE · 5cm]")
        print(f"  📍 WP2 (Sweep)        : X={w2[0]:.3f}, Y={w2[1]:.3f}, Z={w2[2]:.3f}  [75° IMPACT · 15cm]")
        print(f"  📍 WP3 (Recovery)     : X={w3[0]:.3f}, Y={w3[1]:.3f}, Z={w3[2]:.3f}  [AUTO-LOOP · 15cm]")
        print(f"  -------------------------------------------")
        print(f"  ⚡ Climb speed  : {cs:.2f} m/s")
        print(f"  ⚡ Transit speed: {ts:.2f} m/s")
        print(f"  ⚡ Sweep speed  : {ss:.2f} m/s")
        print(f"=============================================")

    def get_next_setpoint(self, current_mocap_pos, dt):
        if self.current_wp_idx >= len(self.waypoints):
            self.loop_counter += 1
            print(f"\n[MISSION] Loop {self.loop_counter} completed! Restarting from WP_stage...")
            self.current_wp_idx = 1       # Index 1 = WP_stage — skips WP0 takeoff on all subsequent loops
            self.wp_entered_time = None
            self.has_paused_at_current_wp = False

        return super().get_next_setpoint(current_mocap_pos, dt)
