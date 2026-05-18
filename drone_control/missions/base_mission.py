import math

class BaseMission:
    """
    Base class for all flight missions.
    Missions define WHAT to do, while the Flight Director handles HOW to do it.
    """
    def __init__(self):
        self.is_finished = False

    def on_start(self, initial_mocap_pos):
        """
        Called when the mission transitions from TAKEOFF to MISSION state.
        :param initial_mocap_pos: The absolute MoCap coordinate [x, y, z] when takeoff began.
        """
        pass

    def get_next_setpoint(self, current_mocap_pos, dt):
        """
        Called at 10Hz by the Flight Director.
        :param current_mocap_pos: The current absolute MoCap coordinate.
        :param dt: Time elapsed since the last tick.
        :return: A tuple (x, y, z, yaw) representing the target absolute MoCap coordinate.
        """
        raise NotImplementedError("Missions must implement get_next_setpoint()")

    def on_proceed(self):
        """
        Called when the user presses [ENTER] during the mission.
        Can be used to advance the mission state machine.
        """
        pass
