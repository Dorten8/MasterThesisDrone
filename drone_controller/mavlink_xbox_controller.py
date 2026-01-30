"""
Drone Joystick Controller - Non-blocking, thread-based manual control

This script reads an Xbox/PS4 controller connected to /dev/input/js0 and sends
MAVLink MANUAL_CONTROL messages to PX4 at a fixed 20 Hz rate (50ms intervals).

Key features:
- Non-blocking joystick reads (doesn't freeze waiting for stick input)
- Separate arm/disarm via button (press Y to toggle arm state)
- Throttle safety: throttle only increases if RIGHT BUMPER (RB) is held
- Smooth 20 Hz message transmission (not dependent on joystick events)
- Clear visual feedback and printed logs
- Proper connection handshake with heartbeat

Author: Jakub Sejkora (with improvements for reliability)
"""

import os
import struct
import time
import threading
import fcntl
import sys
from pymavlink import mavutil

# ============================================================================
# CONFIGURATION - Adjust these to match your joystick layout
# ============================================================================

JS_DEVICE = "/dev/input/js0"  # Xbox controller input device
SERIAL_PORT = "/dev/serial0"  # Pi to FC connection
BAUD_RATE = 921600            # INAV/PX4 default baud rate

# Axis mappings (check with 'jstest /dev/input/js0' if unsure)
AX_ROLL = 0    # Left stick horizontal (left = -1000, right = +1000)
AX_PITCH = 1   # Left stick vertical (forward = -1000, back = +1000)
AX_YAW = 3     # Right stick horizontal (left = -1000, right = +1000)
AX_THROTTLE = 4  # Right trigger (not pressed = -1000, fully pressed = +1000)

# Button mappings (check with jstest if unsure)
BTN_ARM_DISARM = 3   # Y button (square on PS4)
BTN_THROTTLE_ENABLE = 5  # RB - Right Bumper (must be held to allow throttle increase)
BTN_EMERGENCY_DISARM = 2  # X button (hard emergency disarm)

# Message rate
SEND_RATE_HZ = 20
SEND_INTERVAL_S = 1.0 / SEND_RATE_HZ  # 0.05 seconds = 50 milliseconds

# ============================================================================
# GLOBAL STATE - Thread-safe joystick and flight state
# ============================================================================

class JoystickState:
    """Holds current joystick axis and button states."""
    def __init__(self):
        self.axes = [0] * 8      # 8 analog axes (normalized -1000 to +1000)
        self.buttons = [0] * 16  # 16 buttons (0 or 1)
        self.lock = threading.Lock()
    
    def update_axis(self, axis_num, raw_value):
        """Store axis value (raw int16 from /dev/input/js0)."""
        with self.lock:
            if axis_num < len(self.axes):
                # Normalize int16 [-32767..32767] to [-1000..1000]
                self.axes[axis_num] = int(raw_value / 32.767)
    
    def update_button(self, btn_num, pressed):
        """Store button state (0 or 1)."""
        with self.lock:
            if btn_num < len(self.buttons):
                self.buttons[btn_num] = 1 if pressed else 0
    
    def get_axes(self):
        """Return a copy of current axis values (thread-safe)."""
        with self.lock:
            return self.axes.copy()
    
    def get_buttons(self):
        """Return a copy of current button states (thread-safe)."""
        with self.lock:
            return self.buttons.copy()


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


# Global objects
joystick_state = JoystickState()
flight_state = FlightState()


# ============================================================================
# JOYSTICK READER THREAD - Non-blocking input polling
# ============================================================================

def read_joystick_events():
    """
    Thread function: reads joystick events in non-blocking mode.
    
    HOW IT WORKS (phonetically):
    - Opens the joystick device file (/dev/input/js0)
    - Sets it to non-blocking mode using fcntl (so read() doesn't freeze)
    - Reads 8-byte events in a loop
    - If no event is waiting, loop continues without blocking
    - Each event is either an axis motion (type 0x02) or button press/release (type 0x01)
    - Updates the shared JoystickState object (thread-safe via locks)
    """
    try:
        with open(JS_DEVICE, 'rb') as f:
            # fcntl.ioctl sets the device to non-blocking:
            # O_NONBLOCK flag = file descriptor won't block on read()
            fcntl.fcntl(f, fcntl.F_SETFL, os.O_NONBLOCK)
            print(f"[JS] Joystick opened in non-blocking mode: {JS_DEVICE}")
            
            while True:
                try:
                    # Each joystick event is exactly 8 bytes:
                    # - Bytes 0-3: timestamp (uint32) - when event happened
                    # - Bytes 4-5: value (int16) - axis position or button state
                    # - Byte 6: type (uint8) - 0x01 = button, 0x02 = axis, 0x80 = initial
                    # - Byte 7: number (uint8) - which axis/button (0-15)
                    ev = f.read(8)
                    
                    if len(ev) < 8:
                        # No data available right now (non-blocking mode returns 0 bytes)
                        time.sleep(0.001)  # Short sleep to avoid busy-loop
                        continue
                    
                    # Unpack the 8-byte event
                    t, value, etype, num = struct.unpack("IhBB", ev)
                    
                    # Separate the event type and initial flag
                    # etype & 0x02 = axis motion event (stick moved)
                    # etype & 0x01 = button event (button pressed/released)
                    # etype & 0x80 = initial state (ignore at startup)
                    
                    is_initial = bool(etype & 0x80)
                    is_axis = bool(etype & 0x02)
                    is_button = bool(etype & 0x01)
                    
                    if is_initial:
                        # Skip initial state events at startup
                        continue
                    
                    if is_axis:
                        joystick_state.update_axis(num, value)
                    elif is_button:
                        joystick_state.update_button(num, value)
                
                except (BlockingIOError, IOError):
                    # No data available; this is normal in non-blocking mode
                    time.sleep(0.001)
    
    except FileNotFoundError:
        print(f"[ERROR] Joystick device not found: {JS_DEVICE}")
        print("        Run 'ls -l /dev/input/js*' to check available devices")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Joystick reader thread crashed: {e}")
        sys.exit(1)


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


# ============================================================================
# MAIN CONTROL LOOP - Runs at fixed 20 Hz
# ============================================================================

def control_loop(mav, target_system):
    """
    Main flight control loop: runs at 20 Hz (every 50ms).
    
    HOW IT WORKS (phonetically):
    1. Read current joystick state (thread-safe)
    2. Check button states to determine arming and throttle safety
    3. Convert axis values to MAVLink MANUAL_CONTROL command
    4. Apply throttle safety: only allow positive throttle if RB is held
    5. Send the command to PX4
    6. Sleep exactly 50ms, then repeat
    
    This ensures PX4 always receives a fresh input command, so it never thinks
    the joystick is "lost" between stick movements.
    """
    
    print("[MAIN] Starting control loop at 20 Hz (50ms per cycle)...")
    print("[CTRL] Y button = arm/disarm")
    print("[CTRL] X button = emergency disarm")
    print("[CTRL] RB (right bumper) = throttle enable (must hold to increase throttle)")
    print("[CTRL] Ready for flight!")
    print()
    
    last_arm_state = False  # Track previous arm button state to detect edge (press)
    last_send_time = time.time()
    
    while True:
        loop_start = time.time()
        
        # --- 1. Read joystick state (thread-safe) ---
        axes = joystick_state.get_axes()
        buttons = joystick_state.get_buttons()
        
        # --- 2. Check arm/disarm button (Y) ---
        # Only trigger on press edge (button transitions from 0 to 1)
        arm_btn = buttons[BTN_ARM_DISARM]
        if arm_btn and not last_arm_state:
            # Button just pressed
            new_armed = not flight_state.get_armed()
            flight_state.set_armed(new_armed)
            arm_disarm(mav, target_system, new_armed)
        last_arm_state = arm_btn
        
        # --- 3. Check emergency disarm button (X) ---
        if buttons[BTN_EMERGENCY_DISARM]:
            flight_state.set_armed(False)
            print("[EMERGENCY] DISARM pressed!")
            arm_disarm(mav, target_system, False)
        
        # --- 4. Extract raw axis values ---
        roll = axes[AX_ROLL]
        pitch = axes[AX_PITCH]
        yaw = axes[AX_YAW]
        throttle_raw = axes[AX_THROTTLE]
        
        # --- 5. Apply throttle safety ---
        # Throttle safety: ONLY increase throttle if RB (right bumper) is held
        # If RB is NOT held, force throttle to 0 (idle)
        # This prevents accidental throttle spikes if you bump the stick
        throttle_enable = buttons[BTN_THROTTLE_ENABLE]
        
        if not throttle_enable:
            # RB not held: force throttle to 0 (idle/safe)
            throttle = 0
        else:
            # RB held: map throttle from [-1000..1000] to [0..1000]
            # The trigger on Xbox goes from -1000 (unpressed) to +1000 (fully pressed)
            # We want: -1000 or 0 (unpressed) -> 0 (idle)
            #          +1000 (pressed) -> 1000 (full power)
            throttle = max(0, int((throttle_raw + 1000) / 2000 * 1000))
        
        # --- 6. Clamp all values to valid range [-1000..1000] for safety ---
        def clamp(v):
            return max(-1000, min(1000, int(v)))
        
        roll = clamp(roll)
        pitch = clamp(pitch)
        yaw = clamp(yaw)
        throttle = clamp(throttle)
        
        # --- 7. Send MANUAL_CONTROL to PX4 ---
        send_manual_control(mav, target_system, roll, pitch, throttle, yaw)
        
        # --- 8. Print status (optional, every 10 cycles = 0.5 sec) ---
        if int(time.time() * 10) % 10 == 0:
            armed_str = "ARMED" if flight_state.get_armed() else "DISARMED"
            thr_enable_str = "ON" if throttle_enable else "OFF"
            print(f"[{armed_str}] R={roll:5d} P={pitch:5d} Y={yaw:5d} THR={throttle:4d} (enable={thr_enable_str})")
        
        # --- 9. Maintain 20 Hz rate ---
        # Sleep until the next 50ms boundary
        elapsed = time.time() - loop_start
        sleep_time = max(0, SEND_INTERVAL_S - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main():
    """
    Main entry point:
    1. Connect to flight controller via serial
    2. Wait for heartbeat (proves FC is responding)
    3. Start joystick reader thread
    4. Run control loop
    """
    
    print("=" * 70)
    print("SpinFly Drone Joystick Controller")
    print("=" * 70)
    print(f"Joystick device:  {JS_DEVICE}")
    print(f"Serial port:      {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Send rate:        {SEND_RATE_HZ} Hz ({SEND_INTERVAL_S*1000:.1f}ms)")
    print()
    
    # --- Connect to flight controller ---
    print("[INIT] Connecting to flight controller...")
    try:
        mav = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {SERIAL_PORT}: {e}")
        print("        Check: sudo ls -l /dev/serial0")
        sys.exit(1)
    
    # --- Wait for heartbeat ---
    print("[INIT] Waiting for heartbeat from flight controller...")
    if not mav.wait_heartbeat(timeout=10):
        print("[ERROR] No heartbeat received! Is the FC powered and connected?")
        sys.exit(1)
    
    print(f"[OK]   Heartbeat received!")
    print(f"       System ID: {mav.target_system}, Component ID: {mav.target_component}")
    
    target_system = mav.target_system
    
    # --- Start joystick reader thread (daemon) ---
    print("[INIT] Starting joystick reader thread...")
    js_thread = threading.Thread(target=read_joystick_events, daemon=True)
    js_thread.start()
    
    time.sleep(1)  # Give thread time to open joystick
    
    # --- Run main control loop ---
    try:
        control_loop(mav, target_system)
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Ctrl+C detected, disarming and exiting...")
        flight_state.set_armed(False)
        arm_disarm(mav, target_system, False)
        time.sleep(0.5)
        mav.close()
        print("[OK] Closed.")


if __name__ == "__main__":
    main()