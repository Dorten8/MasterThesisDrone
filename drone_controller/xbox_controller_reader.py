import os
import struct
import time
import threading
import fcntl
import sys

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