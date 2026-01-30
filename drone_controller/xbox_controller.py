import time
import threading
import sys
from pymavlink import mavutil
from xbox_controller_reader import JoystickState, read_joystick_events
from mavlink_command_formatter import FlightState, arm_disarm, send_manual_control, FlightState

# ============================================================================
# CONFIGURATION
# ============================================================================

JS_DEVICE = "/dev/input/js0"  # Xbox controller input device
SERIAL_PORT = "/dev/serial0"  # Pi to FC connection
BAUD_RATE = 921600            # PX4 default baud rate for ROS2, used here also for Mavlink for simplicity

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