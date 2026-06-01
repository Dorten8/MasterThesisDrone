# Networking:
## Finding out
#### local IP (Linux)
hostname -I
#### network discovery (Linux)
sudo nmap -sn 192.168.74.0/23

## MAC Adresses
### Pi5 drone
eth0 (Ethernet): 88:a2:9e:65:43:b4
wlan0 (WiFi): 88:a2:9e:65:43:b5
docker0 (Docker bridge): 36:b5:a7:3c:10:02

### Optitrack Windows PC server
Wifi 70:CD:0D:B1:67:C3

### Ubuntu Laptop
wifi c8:94:02:5b:f0:d9

# ROS2
### Pc settup
run this
```bash
bash -lc 'source /opt/ros/humble/setup.bash && rviz2'
# or
rviz2b
```

# SSHFS
~/pi_drone_sshfs

### Make sure the target folder exists first
mkdir -p ~/pi_drone_sshfs
### Connect the Pi's workspace to your laptop
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs -o auto_cache,reconnect,follow_symlinks
### Combined command (with cache disabled to prevent SQLite/metadata page corruption over SSHFS)
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs -o cache=no,cache_timeout=0,cache_stat_timeout=0,cache_dir_timeout=0,cache_link_timeout=0,reconnect,follow_symlinks
### unmount
umount ~/pi_drone_sshfs
### The -u is unmount, -z is 'lazy' (cleans up the mess even if busy)
fusermount -u -z ~/pi_drone_sshfs

# IDE Troubleshooting
## Antigravity Webview Service Worker Crash (InvalidStateError)
If you get `Unable to open 'experiments_analysis.ipynb'` with the error:
`Could not initialize webview: Error: Could not register service worker: InvalidStateError: Failed to register a ServiceWorker: The document is in an invalid state.`

This is an Chromium-webview cache desynchronization bug inside the IDE. Run this single block in your terminal to force-reset:
```fish
# 1. Force-kill all running and helper processes
pkill -f -9 antigravity; pkill -f -9 vscodium; pkill -f -9 code

# 2. Delete active lock files
rm -f ~/.config/Antigravity/code.lock
rm -f ~/.config/Antigravity/SingletonLock
rm -f ~/.config/Antigravity/SingletonSocket

# 3. Wipe the corrupted Service Worker and Webview caches
rm -rf ~/.config/Antigravity/'Service Worker'
rm -rf ~/.config/Antigravity/Cache
rm -rf ~/.config/Antigravity/'Code Cache'
```
Reopen Antigravity and the notebook will render perfectly!

## MAVLINK-ROUTER
Running it with specific config (not default in root)
mavlink-routerd -c /home/ws/config/mavlink-router/main.conf

# Git backup workflow for flight data
## Goal
Keep using `git add .` as the normal habit, while still backing up the important flight MCAP files under `dev_logs/flights/`.

## What changed
- The flight archive folder stays ignored by default.
- Two file patterns are allowed through the ignore rules:
  - `*_0.mcap` flight bags
  - `*-pass*.mcap` pass slices
- Sidecar files like `metadata.yaml`, `temp_recovered.mcap`, and `.corrupt` files remain ignored.

## How `git add .` behaves now
From the repo root, `git add .` will still stage normal code changes, and it will also stage the flight MCAP files that match the allowed patterns.

It will not stage:
- ignored sidecar files in the flight folders
- submodule changes under `src/PX4-Autopilot` and `src/px4_ros_com`

## Recommended workflow
### Option A: keep the normal habit
```bash
cd /home/ws
git add .
git status
git commit -m "your short message"
git push
```

### Option B: use the backup helper when you only want flight MCAPs
```bash
cd /home/ws
./dev_logs/stage_flight_backups.sh /home/ws/dev_logs/flights 100M
git commit -m "backup flight mcap files"
git push
```

## Rule of thumb
Use `git add .` when you want your normal repo changes plus the allowed flight bags.
Use the helper when you only want to stage the flight MCAP backups and skip everything else.

## Quick backup command
```bash
cd /home/ws && git add . && git commit -m "backup flight data" && git push
```

## PX4 <-> ROS 2 (micro-ROS agent) workflow

### 1) Build workspace 
no longer needed (implemented in container image)

### 2) Run micro-ROS agent (Terminal A)
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600 -v6
```

### 3) Observe PX4 topics (Terminal B)
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
ros2 topic list --no-daemon --include-hidden-topics | grep -E '^/fmu/|^/rt/fmu/'
```

### 4) Switch back to MAVLink-router mode
Only one process can own `/dev/ttyAMA0` at a time.
```bash
# stop micro-ROS agent (Ctrl+C in Terminal A), then:
mavlink-routerd -c /home/ws/config/mavlink-router/main.conf
```

### 5) PX4 parameter baseline for ROS2 over TELEM2
```text
UXRCE_DDS_CFG=102 (TELEM2)
SER_TEL2_BAUD=921600
MAV_0_CONFIG=0   # disable MAVLink on same port while debugging ROS2 link
```

If agent terminal shows only `running...` and no DDS/client lines:
- ensure nothing else uses `/dev/ttyAMA0`
- reboot FC once more from QGC (PX4 uXRCE client sometimes starts only after reboot)

### ROS2 FMU topics:
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
ros2 topic list --no-daemon --include-hidden-topics | grep -E '^/fmu/|^/rt/fmu/'
```

```text
# /fmu/in ROS2 -> PX4
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/arming_check_reply
/fmu/in/aux_global_position
/fmu/in/config_control_setpoints
/fmu/in/config_overrides_request
/fmu/in/distance_sensor
/fmu/in/goto_setpoint
/fmu/in/manual_control_input
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/unregister_ext_component
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_command_mode_executor
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_visual_odometry

# /fmu/out PX4 -> ROS2
/fmu/out/airspeed_validated
/fmu/out/arming_check_request
/fmu/out/battery_status
/fmu/out/collision_constraints
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/home_position
/fmu/out/manual_control_setpoint
/fmu/out/message_format_response
/fmu/out/mode_completed
/fmu/out/position_setpoint_triplet
/fmu/out/register_ext_component_reply
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status_v1
/fmu/out/vtol_vehicle_status
```

### PX4-Autopilot

#### Regular run
keeping it lightweight for docker build
```bash
bash /home/ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh
cd /home/ws/src/PX4-Autopilot/
make px4_sitl
```

#### If `gz_x500` target is missing
If `make px4_sitl gz_x500` fails with `ninja: error: unknown target 'gz_x500'`, clean and rebuild:
```bash
cd /home/ws/src/PX4-Autopilot/
make distclean
make px4_sitl gz_x500
```
Headless mode (no GUI):
```bash
cd /home/ws/src/PX4-Autopilot/
HEADLESS=1 make px4_sitl gz_x500
```

#### Option: Add to post-create.sh (if needed for Docker build)
```bash
bash -lc '
  source /opt/ros/humble/setup.bash
  cd /home/ws
  rosdep update
  rosdep install --from-paths /home/ws/src --ignore-src -r -y
  sudo chown -R $(whoami) /home/ws/
  colcon build --symlink-install
  bash /home/ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh
  cd /home/ws/src/PX4-Autopilot/
  make px4_sitl
'
```

#### Checking if serial port is free!
```bash
sudo fuser -v /dev/ttyAMA0
# if it is owned by something, like mavlink-routerd:
sudo systemctl stop mavlink-routerd
```

### Running Micro-XRCE-DDS-Agent
```bash
MicroXRCEAgent udp4 -p 8888 #runs it on default SITH port

# if connected to the Flight Controller!
# CFG parameter on the FC needs to be set to the coonected TELEM2 port!
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
# if it does not work reboot the FC!
#not this one!!!
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600 -v6
# find topics:
ros2 topic list | grep /fmu

### Run in another terminal to check real-time status
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort

```

### motion_capture_tracking


# How to run (Updated 2026-05-10)

## Automated Startup (Recommended)
Simply run the startup script which safely handles the Agent, MoCap, Bridge, and Foxglove:
```bash
./startup-sequence.sh
```
*(To shut down gracefully without zombie processes, use `./shutdown-sequence.sh`)*

## Manual Startup (For debugging)
1. Run XRCE-Agent on Pi5 -> should have continuous print out
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
2. Run MOCAP 
```bash
ros2 run motion_capture_tracking motion_capture_tracking_node --ros-args -p type:=optitrack -p hostname:=192.168.74.9 #or other IP of the MOTIVE server
```
  2.1 You should now see tracking topics in `ros2 topic list`

3. Run MOCAP PX4 Bridge to adjust the XYZ frame
```bash
# Note: drone_name must match the OptiTrack topic, e.g., rigid_body_7
ros2 run mocap_px4_bridge mocap_px4_bridge --ros-args -p mocap_topic:=/poses -p drone_name:=rigid_body_7 -p px4_topic:=/fmu/in/vehicle_visual_odometry
```

4. Run Ros2 topic echo to see if it is being updated
```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry
```
  Check the topic metadata:
```bash
ros2 topic info -v /fmu/in/vehicle_visual_odometry
```

5. Run Foxglove
```bash
# on Pi (Handled by startup-sequence.sh if using automated method)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0

# on pc 
foxglove-studio
```

6. Run Rviz
```fish
# on Pi (using Fish Shell)
set -x ROS_DOMAIN_ID 0
set -x ROS_LOCALHOST_ONLY 0
set -x RMW_IMPLEMENTATION rmw_fastrtps_cpp
bash -lc 'source /opt/ros/humble/setup.bash && rviz2'
```

To slice a new flight (after landing on the Pi):
# 1. After landing — slice passes from raw bag:
python3 -m dev_logs.analysis.database.db_mcap_event_segmenter

# 2. Re-run analysis — new passes auto-populate DB, old ones are skipped:
# Standard/Full historical database rebuild:
python3 -m dev_logs.analysis.database.db_pipeline

# Incremental/Today-only update (highly optimized, scans only today's folders):
python3 -m dev_logs.analysis.database.db_pipeline --today  # or -t

# Custom cutoff (skip directories before specific YYYYMMDD-HHMM timestamp):
python3 -m dev_logs.analysis.database.db_pipeline --cutoff 20260530-0000  # or -c

## Python Path Resiliency & CLI Best Practices

### 🔍 The "Why": Direct Script Invocation vs. Module Execution
When running Python scripts inside nested folders (like `dev_logs/analysis/database/`):
* **Direct Script Run (`python3 path/to/script.py`):** Python automatically adds only the directory containing `script.py` to `sys.path`. It has no knowledge of parent folders, so `from dev_logs.analysis...` imports will raise a `ModuleNotFoundError`.
* **Module Execution (`python3 -m dev_logs.analysis.database.script`):** By executing with the `-m` (module) flag from the repository root, Python automatically prepends the root directory (`/home/dorten/pi_drone_sshfs`) to `sys.path`, resolving all top-level package imports flawlessly.

# Reset and run database pipeline
```bash
rm -f dev_logs/analysis/experiments_summary.db
PYTHONPATH=. python3 dev_logs/analysis/database/db_pipeline.py
```


### 🛡️ The "How": Universal Self-Healing Path Boilerplate
To make any Python script runnable from **anywhere** (both via direct script invocation and module execution), paste this standard, self-healing one-liner at the absolute top of the file before any package imports:

```python
import sys, os; sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../" if "__file__" in locals() or "__file__" in globals() else "")))
```

#### How it works:
1. `os.path.dirname(__file__)` finds the script's own folder (e.g. `dev_logs/analysis/database/`).
2. `os.path.join(..., "../../../")` traverses 3 directory levels up to dynamically resolve the absolute path of the repository root (`pi_drone_sshfs`).
3. `sys.path.insert(0, ...)` prepends the root folder to Python's search path, making `dev_logs` instantly importable.
