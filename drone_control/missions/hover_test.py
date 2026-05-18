from .base_mission import BaseMission

class HoverTest(BaseMission):
    """
    A simple mission that commands the drone to take off to an absolute 
    MoCap altitude of 1.0 meters and hold position.
    """
    def __init__(self):
        super().__init__()
        self.target_z = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

    def on_start(self, initial_mocap_pos):
        self.origin_x = initial_mocap_pos.x
        self.origin_y = initial_mocap_pos.y
        # MoCap ENU Z is positive UP. We take off 0.5 meters relative to the start position!
        self.target_z = initial_mocap_pos.z + 0.5
        print(f"\n[MISSION] HoverTest started! Target absolute altitude: {self.target_z:.2f}m (MoCap Z).")

    def get_next_setpoint(self, current_mocap_pos, dt):
        # We simply hold the origin X/Y and the new target Z.
        # The Flight Director will apply the Transform Layer automatically.
        return (self.origin_x, self.origin_y, self.target_z, 0.0)

    def on_proceed(self):
        print("\n[MISSION] HoverTest has no further phases. Landing...")
        self.is_finished = True
