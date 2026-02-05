import time
import threading
import sys
from pymavlink import mavutil
from xbox_controller_reader import JoystickState, read_joystick_events
from mavlink_command_formatter import FlightState, arm_disarm, send_manual_control

# ============================================================================
# CONFIGURATION
# ============================================================================

JS_DEVICE = "/dev/input/js0"  # Xbox controller input device
SERIAL_PORT = "/dev/ttyAMA0"  # Pi to FC connection
BAUD_RATE = 921600            # PX4 default baud rate for ROS2, used here also for Mavlink for simplicity

THROTTLE_RATE = 400  # Max throttle change per second (to prevent sudden spikes)
EXPO_CURVE = 0.3    # Exponential curve factor for stick sensitivity (0 = linear, 1 = full expo)

# Axis mappings (check with 'jstest /dev/input/js0' if unsure)
AX_ROLL = 2    # right stick horizontal (left = -1000, right = +1000)
AX_PITCH = 3   # right stick vertical (forward = -1000, back = +1000)
AX_YAW = 0     # left stick horizontal (left = -1000, right = +1000)
AX_THROTTLE = 1  # left stick vertical (forward = -1000, back = +1000)

# Button mappings (check with jstest if unsure)
BTN_ARM_DISARM = 0   # A button
BTN_EMERGENCY_DISARM = 2  # X button (hard emergency disarm)

# Message rate
SEND_RATE_HZ = 20
SEND_INTERVAL_S = 1.0 / SEND_RATE_HZ  # 0.05 seconds = 50 milliseconds

# ============================================================================
# GLOBAL STATE - Thread-safe joystick and flight state
# ============================================================================
joystick_state = JoystickState()
flight_state = FlightState()

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
    arm_button_press_time = None
    current_throttle = 0  # Starting with throttle at 0 for safety
    
    while True:
        loop_start = time.time()
        
# --- 1. Read joystick state (thread-safe) ---
        axes = joystick_state.get_axes()
        buttons = joystick_state.get_buttons()
        
        # --- 2. Arm/disarm logic ---
        arm_btn = buttons[0]  # A button
        
        if arm_btn:
            if arm_button_press_time is None:
                arm_button_press_time = time.time()  # Start counting
            elif time.time() - arm_button_press_time >= 3.0:
                if not flight_state.get_armed():
                    arm_disarm(mav, target_system, True)
                    flight_state.set_armed(True)
                    print("[ARMED] (held A 3 seconds)")
                arm_button_press_time = None  # Reset after arm
        else:
            arm_button_press_time = None  # Reset when A released
        
        # --- 3. Disarm any button (A/B/X/Y = buttons 0,1,2,3) ---
        if any(buttons[i] for i in [1, 2, 3]):
            if flight_state.get_armed():
                arm_disarm(mav, target_system, False)
                flight_state.set_armed(False)
                print("[DISARMED]")
        
        # --- 4. Extract raw axis values ---
        roll = axes[AX_ROLL]
        pitch = axes[AX_PITCH]
        yaw = axes[AX_YAW]
        throttle_input = axes[AX_THROTTLE]
        
        #Throttle smoothing (PX4 spring loaded throttle will return to hold alltitude at 0)
        #stick centered = no throttle change
        throttle_change_per_sec = (throttle_input / 1000.0) * THROTTLE_RATE
        throttle_change_this_loop = throttle_change_per_sec * SEND_INTERVAL_S
        current_throttle += throttle_change_this_loop
        current_throttle = max(0, min(1000, current_throttle))
        throttle = int(current_throttle)
  
        # Expo curve (standard QCG-style_)
        def apply_expo(v, expo=EXPO_CURVE):
            #QCG-style exponential curve
            abs_v = abs(v)
            sign = 1 if v > 0 else -1
            expo_v = sign * (abs_v ** (1 + expo*3)) / (1000 ** (expo*3)) 
            return max(-1000, min(1000, int(expo_v)))       
    
        roll = apply_expo(roll)
        pitch = apply_expo(pitch)
        yaw = apply_expo(yaw)
        throttle = apply_expo(throttle)
        
        # --- 7. Send MANUAL_CONTROL to PX4 ---
        send_manual_control(mav, target_system, roll, pitch, throttle, yaw)
        
        
        # --- 8. Print status (optional, every 10 cycles = 0.5 sec) ---
        if int(time.time() * 10) % 10 == 0:
            armed_str = "ARMED" if flight_state.get_armed() else "DISARMED"
            print(f"[{armed_str}] R={roll:5d} P={pitch:5d} Y={yaw:5d} THR={throttle:4d}")

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
    print("Drone Joystick Controller")
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
        armed_str = "ARMED" if flight_state.get_armed() else "DISARMED"
        print(f"[{armed_str}] R={roll:5d} P={pitch:5d} Y={yaw:5d} THR={throttle:4d}")
    
    print(f"[OK]   Heartbeat received!")
    print(f"       System ID: {mav.target_system}, Component ID: {mav.target_component}")
    
    target_system = mav.target_system
    
    # --- Start joystick reader thread (daemon) ---
    print("[INIT] Starting joystick reader thread...")
    js_thread = threading.Thread(target=read_joystick_events, args=(joystick_state,), daemon=True)
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