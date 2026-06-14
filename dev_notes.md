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

---

# Full Rebuild from Scratch

Use this when setting up a new Pi 5 (or replacing the SD card), or after a `git clean -xdf` or full Docker rebuild. It covers the entire chain: host OS → container → workspace → firmware → experiment pipeline.

## 0. Prerequisites

- **Pi 5 (8 GB)** with freshly flashed **Ubuntu 24.04 Desktop** (use Raspberry Pi Imager)
- Set hostname: `pi5drone`
- Enable SSH **and** Wi-Fi during Imager setup
- This repo cloned onto the Pi:
  ```bash
  cd ~
  git clone git@github.com:Dorten8/MasterThesisDrone.git ws
  ```

> **Imager caveat:** despite ticking "Enable SSH", the SSH daemon is NOT started on first boot. Fix below.

## 1. Pi 5 Host OS

Run these once after the initial flash, **before** opening the container:

```bash
# ── SSH fix (Imager lies) ──
sudo apt install openssh-server
sudo systemctl enable --now ssh

# ── mDNS so you can reach pi5drone.local ──
sudo apt install avahi-daemon
sudo systemctl enable --now avahi-daemon
# Also install on your workstation: avahi-daemon libnss-mdns

# ── Docker (full suite) ──
sudo apt install docker.io docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo apt remove containerd          # ⚠️ kills the conflicting stub
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker                       # or log out & back in

# ── UART for Pixhawk serial ──
sudo nano /boot/firmware/config.txt
# Append:  enable_uart=1
# Reboot, then verify:
ls -l /dev/ttyAMA0
cat /boot/firmware/cmdline.txt      # should show console=tty1 (not serial)

# ── Python venv (host tooling only) ──
sudo apt install python3.12-venv
python3 -m venv ~/ros2_env

# ── SSH key for Git ──
ssh-keygen -t ed25519 -C "pi5drone"
cat ~/.ssh/id_ed25519.pub          # add to GitHub
```

**Verify:**
```bash
ping pi5drone.local    # from workstation → responds
ssh dorten@pi5drone.local
docker ps              # daemon running
ls -l /dev/ttyAMA0     # UART exists
```

## 2. VS Code Remote-SSH + Dev Container

On your **workstation** (not Pi):

1. Install **Remote-SSH** and **Dev Containers** VS Code extensions
2. `Ctrl+Shift+P` → Remote-SSH: Connect to Host → `dorten@pi5drone.local`
3. Open `/home/dorten/ws`
4. `Ctrl+Shift+P` → Dev Containers: Reopen in Container

**First build:** ~30–50 min on Pi 5. Docker caches layers afterward.

The container build (`Dockerfile`) does all of this automatically:
- Installs ROS 2 Humble base layers
- Builds **Micro XRCE-DDS Agent** from source (`src/Micro-XRCE-DDS-Agent/`)
- Builds **MAVLink Router** from source (`mavlink-router/`)
- Installs Python deps (meson, pymavlink, mcap-ros2-support, pandas, scipy, matplotlib, etc.)

## 3. ROS 2 Workspace Build (colcon)

Triggered automatically by `postCreateCommand`, but you can re-run manually:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**What gets built:**
| Package | Source | Type |
|---|---|---|
| `mocap_px4_bridge` | `src/mocap_px4_bridge/` | C++ (ENU→NED bridge) |
| `motion_capture_tracking` | `src/mocap/` | C++ (OptiTrack NatNet client) |
| `px4_msgs` | `src/px4_msgs/` | ROS 2 msg definitions |
| `drone_control` | `drone_control/` | Python nodes (flight_director, ghost_flight, flight_recorder) |

**Troubleshooting:**
```bash
# Missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean rebuild
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-clean-first

# Source the new build
source install/setup.bash
```

## 4. PX4 Firmware Build (SITL only — optional)

Only needed if you modify PX4 internals or want Gazebo simulation:

```bash
cd /home/ws/src/PX4-Autopilot/
bash Tools/setup/ubuntu.sh
make px4_sitl                            # headless simulation
# or
HEADLESS=1 make px4_sitl gz_x500         # Gazebo headless
```

The real flight controller runs the official **PX4 v1.16.1rc** binary (flashed via QGroundControl). The SITL build is for testing only.

## 5. Validate the Container

```bash
# Inside the dev container:
lsb_release -a          # → Ubuntu 22.04 LTS
ros2 --version          # → ROS 2 Humble
echo $ROS_DISTRO        # → humble
which MicroXRCEAgent    # → /usr/local/bin/MicroXRCEAgent
which mavlink-routerd   # → /usr/bin/mavlink-routerd
colcon --version        # → colcon ...
```

## 6. Startup Sequence (on the real drone)

Used every flight session. Handles the full pipeline: time sync → uXRCE-DDS Agent → MoCap → Bridge → Foxglove.

```bash
cd /home/ws
./startup-sequence.sh
```

Shut down gracefully (no zombie processes):
```bash
./shutdown-sequence.sh
```

**Manual step-by-step** (for debugging — three terminals):

```bash
# Terminal A: uXRCE-DDS Agent
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# Terminal B: MoCap → Bridge
ros2 run motion_capture_tracking motion_capture_tracking_node \
  --ros-args -p type:=optitrack -p hostname:=192.168.74.3
ros2 run mocap_px4_bridge mocap_px4_bridge \
  --ros-args -p mocap_topic:=/poses -p drone_name:=rigid_body_7 \
  -p px4_topic:=/fmu/in/vehicle_visual_odometry

# Terminal C: Verify
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort
```

## 7. Run a Collision Experiment

```bash
# 1. Startup (handles Agent + MoCap + Bridge)
./startup-sequence.sh

# 2. Launch the flight director and recorder
ros2 run drone_control flight_director.py &
ros2 run drone_control flight_recorder.py &

# 3. The column_sweep_loop.py mission runs autonomously:
#    WP1 (0.000, 1.200) → WP2 (0.100, 1.200) → WP3 (0.100, -1.200) → WP4 (0.000, -1.200) → loop
```

## 8. Slice Flights + Rebuild Database (after flying)

```bash
# Step 1 — slice raw MCAP bags into individual passes:
python3 -m dev_logs.analysis.database.db_mcap_event_segmenter

# Step 2 — ingest passes into the SQLite DB:
# Full rebuild (scans all directories):
rm -f dev_logs/analysis/experiments_summary.db
python3 -m dev_logs.analysis.database.db_pipeline

# Incremental (today's folders only, much faster):
python3 -m dev_logs.analysis.database.db_pipeline --today

# Custom cutoff (skip dirs before a specific timestamp):
python3 -m dev_logs.analysis.database.db_pipeline --cutoff 20260601-0000
```

## 9. Rebuild the Thesis PDF

```bash
cd /home/dorten/MasterThesisDrone/thesis
rm -f main.out main.log main.aux main.bbl main.blg main.toc \
      main.lof main.lot main.bcf main.run.xml main.fdb_latexmk main.fls
latexmk -pdf main.tex
```

Requires a full LaTeX distribution (`texlive-full`) on the host (not in the container). On Ubuntu 24.04:
```bash
sudo apt install texlive-full latexmk biber
```

## 10. Quick-Reference Scripts

| Script | When to run |
|---|---|
| `startup-sequence.sh` | Every flight session |
| `shutdown-sequence.sh` | After flying (cleanup) |
| `.devcontainer/post-create.sh` | First container build (auto) |
| `.devcontainer/post-start.sh` | Every container start (auto) |
| `dev_logs/stage_flight_backups.sh` | Before `git push` to include MCAPs |

## 11. Environment Variables (must be set)

These are baked into the container by `devcontainer.json` + `post-start.sh`:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SSOT_CONFIG_PATH=/home/ws/config
```

If running raw ROS 2 commands outside the container, set them manually or source the env:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
```
