"""
MAVLink Command Formatter - Builds and sends MAVLink messages
"""
import threading
from pymavlink import mavutil

class FlightState:
    """Tracks arming state and flight mode."""
    def __init__(self):
        self.is_armed = False
        self.lock = threading.Lock()
    
    def set_armed(self, armed):
        with self.lock:
            self.is_armed = armed
    
    def get_armed(self):
        with self.lock:
            return self.is_armed
        

# ============================================================================
# MAVLink COMMAND HELPERS
# ============================================================================

def arm_disarm(mav, target_system, arm=True):
    """
    Send MAVLink COMMAND_LONG to arm or disarm the drone.
    
    HOW IT WORKS (phonetically):
    - COMMAND_LONG is MAVLink's way to send commands (like arm/disarm, takeoff, etc.)
    - Command ID 400 = MAV_CMD_COMPONENT_ARM_DISARM
    - Param1 = 1 means ARM, 0 means DISARM
    - Other params are 0 (unused for this command)
    
    This is the PROPER way to arm a drone programmatically.
    """
    cmd_id = 400  # MAV_CMD_COMPONENT_ARM_DISARM
    param1 = 1 if arm else 0
    
    mav.mav.command_long_send(
        target_system,
        1,  # target_component (1 = autopilot)
        cmd_id,
        0,  # confirmation (0 = first attempt)
        param1,  # arm (1) or disarm (0)
        0,  # force disarm (0 = normal disarm)
        0, 0, 0, 0, 0  # unused params
    )
    
    action = "ARMING" if arm else "DISARMING"
    print(f"[CMD] {action} drone...")


def send_manual_control(mav, target_system, roll, pitch, throttle, yaw):
    """
    Send MAVLink MANUAL_CONTROL message to the drone.
    
    HOW IT WORKS (phonetically):
    - MANUAL_CONTROL is the standard way to send joystick input to PX4
    - All values are normalized to int16 range: -1000 (min) to +1000 (max)
    - PX4 interprets them as:
      * pitch: forward/back on left stick (negative = forward)
      * roll: left/right on left stick (negative = left)
      * throttle: 0 to 1000 (0 = idle, 1000 = full power)
      * yaw: left/right rotation (negative = counter-clockwise)
    - buttons (0): bitmask for arm/mode changes (0 = no buttons pressed)
    
    This message MUST be sent continuously (20 Hz recommended) or PX4 will
    treat the input as "lost" and trigger failsafe behavior.
    """
    mav.mav.manual_control_send(
        target_system,
        pitch,
        roll,
        throttle,
        yaw,
        0  # buttons - not used in this script
    )

