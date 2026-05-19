import math
import time

class BaseMission:
    """
    Base class for all flight missions.
    Missions define WHAT to do (providing absolute waypoints), 
    while the Flight Director handles HOW to do it (transitions, distances, and yaw).
    """
    # Metadata for dynamic loading by FlightDirector
    MISSION_NAME = "Unnamed Mission"
    MISSION_DESCRIPTION = "No description provided."
    
    # Whether to enforce FlightDirector geofence checks pre-flight
    ENFORCE_GEOFENCE = True

    def __init__(self):
        self.is_finished = False
        # Waypoints: (x, y, z, face_forward_boolean, speed_mps, pause_at_waypoint_boolean)
        self.waypoints = [] 
        self.current_wp_idx = 0
        self.wp_entered_time = None
        self.hold_time = 0.0 # Default hold time at each waypoint
        
        self.is_paused = False
        self.has_paused_at_current_wp = False

    def on_start(self, initial_mocap_pos):
        """
        Called when the mission transitions from TAKEOFF to MISSION state.
        :param initial_mocap_pos: The absolute MoCap coordinate [x, y, z] when takeoff began.
        """
        pass

    def get_all_absolute_waypoints(self):
        """
        Returns the list of generated absolute waypoints.
        Used by the Flight Director for pre-flight Geofence validation.
        """
        return self.waypoints

    def get_next_setpoint(self, current_mocap_pos, dt):
        """
        Called at 10Hz by the Flight Director.
        Handles checking distance thresholds and hold times.
        :param current_mocap_pos: The current absolute MoCap coordinate.
        :param dt: Time elapsed since the last tick.
        :return: A tuple (x, y, z, face_forward) representing the target active waypoint.
        """
        if self.current_wp_idx >= len(self.waypoints):
            if not self.is_finished:
                print("\n[MISSION] All waypoints completed! Setting mission finished...")
                self.is_finished = True
            return self.waypoints[-1] if self.waypoints else None

        target = self.waypoints[self.current_wp_idx]
        # target can be either 4, 5, or 6 elements long (backwards compatibility)
        target_x, target_y, target_z = target[0], target[1], target[2]
        pause_at_waypoint = target[5] if len(target) >= 6 else False

        # Calculate Euclidean distance to active waypoint
        dx = current_mocap_pos.x - target_x
        dy = current_mocap_pos.y - target_y
        dz = current_mocap_pos.z - target_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Transition Logic
        if dist < 0.15:  # 15 cm threshold
            # 1. Check for defined mission interrupts
            if pause_at_waypoint and not self.has_paused_at_current_wp:
                self.is_paused = True
                self.has_paused_at_current_wp = True
                print(f"\n\033[93m[MISSION] INTERRUPT POINT REACHED at WP {self.current_wp_idx}.\033[0m")
                print("[SYSTEM] Drone is holding current position.")
                print(" -> Press [ENTER] to Resume.")
                print(" -> Press [ L ] to Land.")
                return target

            # 2. Handle hold timers
            now = time.time()
            if self.wp_entered_time is None:
                self.wp_entered_time = now
            
            elapsed = now - self.wp_entered_time
            if elapsed >= self.hold_time:
                # Advance waypoint!
                if self.hold_time > 0:
                    print(f"\n[MISSION] Reached Waypoint {self.current_wp_idx}! Holding for {elapsed:.1f}s. Moving to next WP...")
                else:
                    print(f"\n[MISSION] Reached Waypoint {self.current_wp_idx}! Moving to next WP...")
                
                self.current_wp_idx += 1
                self.wp_entered_time = None
                self.has_paused_at_current_wp = False # Reset pause flag for next waypoint
        else:
            # Reset timer if we drift out of threshold
            self.wp_entered_time = None

        return target

    def toggle_pause(self):
        """
        Toggles the mission pause state. Called when the user presses [ENTER].
        """
        self.is_paused = not self.is_paused
        if self.is_paused:
            print(f"\n\033[93m[MISSION] INTERRUPT: Mission Paused at WP {self.current_wp_idx}.\033[0m")
            print("[SYSTEM] Drone is holding current position.")
            print(" -> Press [ENTER] to Resume.")
            print(" -> Press [ L ] to Land.")
        else:
            print(f"\n\033[92m[MISSION] RESUMED: Continuing to WP {self.current_wp_idx}.\033[0m")
