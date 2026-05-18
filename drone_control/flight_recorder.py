import os
import sys
import subprocess
import signal
import time
import threading

class FlightRecorder:
    """
    Handles launching, managing, and cleanly stopping the ROS 2 bag recording subprocess.
    """
    def __init__(self, script_path=None):
        if script_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # record_flight_bag.sh is in dev_logs/ which is one level up from drone_control/
            self.script_path = os.path.join(os.path.dirname(script_dir), "dev_logs", "record_flight_bag.sh")
        else:
            self.script_path = script_path
        self.recording_proc = None
        self.delayed_thread = None

    def start_recording(self):
        if self.recording_proc is not None:
            print("[RECORDER] Recording already active.")
            return

        try:
            self.recording_proc = subprocess.Popen(
                [self.script_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            print("\n[SYSTEM] Automatic Flight Recording Started (MCAP Bag)!")
        except Exception as e:
            print(f"[RECORDER] [ERROR] Failed to start bag recording: {e}")

    def stop_recording(self):
        if self.recording_proc is not None:
            print("\n[SYSTEM] Stopping automatic flight recording cleanly...")
            try:
                os.killpg(os.getpgid(self.recording_proc.pid), signal.SIGINT)
                self.recording_proc.wait(timeout=5.0)
                print("[SYSTEM] Recording stopped successfully.")
            except Exception as e:
                print(f"[RECORDER] [WARN] Failed to cleanly stop recording: {e}")
                try:
                    os.killpg(os.getpgid(self.recording_proc.pid), signal.SIGKILL)
                except:
                    pass
            self.recording_proc = None

    def _delayed_stop(self, delay_seconds):
        time.sleep(delay_seconds)
        self.stop_recording()

    def stop_recording_delayed(self, delay_seconds=3.0):
        """ Spawns a background thread to wait before stopping the recording """
        if self.recording_proc is None:
            return
            
        print(f"\n[SYSTEM] Triggering delayed recording stop (waiting {delay_seconds}s to capture descent/impact)...")
        self.delayed_thread = threading.Thread(target=self._delayed_stop, args=(delay_seconds,), daemon=True)
        self.delayed_thread.start()
