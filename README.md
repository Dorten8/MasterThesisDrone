# Intro

ROS 2 Humble development environment for autonomous PX4-based drone, built on Docker with Ubuntu 22.04 base image for Gazebo Classic compatibility. 
### Bibliography
AI -> read newest Master Thesis_bibliography in the root of this repository

## Development Environment

- **Base Image:** Ubuntu 22.04 (Jammy)
- **ROS Distribution:** ROS 2 Humble Hawksbill
- **Simulator:** Gazebo Classic (requires Ubuntu 22.04 for full compatibility)

- **Hardware:** Raspberry Pi 5 (Ubuntu 24.04 host)
- **Flight Controller:** Pixhawk 6C
- **Power module** Holybro PM02
- **Optical Flow sensor:** Matek 3901 Optical Flow
- **ESC** Tekko32 F4 4in1 50A ESC(AM32) 

### Resources

- [PX4 Autopilot Documentation](https://docs.px4.io/)

## Prerequisites

### On Development Computer
- Docker
- VS Code with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- SSH client

### On Raspberry Pi 5 (Host OS)
The following must be configured on the Ubuntu 24.04 host system (not in container):

# How to
## PX4 based flight conntroller (Pixhawk 6C in my case)
apart from well documented settup of the flight controller (FC) a correct middleware settup is crucial. This middlaware is used for communication of the FC and either a companion computer (mounted on the drone) or ground control station (GCS). [PX4 is developed with asynchronous publish/subscribe uORB messaging in mind, which well translates to topics on ROS2 with uXRCE-DDS protocol. For control via Mavlink (for example to issue lower level commands such as throttle, pitch, yaw and roll) the FC has to have these parameters settup:](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi)
```bash
# Mavlink setup
MAV_0_CONFIG = TELEM2
MAV_1_CONFIG = 0 (Disabled)
UXRCE_DDS_CFG = 0 (Disabled)
SER_TEL2_BAUD = 57600  # Standard MAVLink baud rate
                       # NOTE: Using 921600 for simplicity when planning to
                       # switch to ROS2 later (avoids baud rate reconfiguration)

# ROS2 uXRCE-DDS setup
MAV_0_CONFIG = TELEM2  # Can keep MAV_0 enabled for simultaneous MAVLink telemetry
MAV_1_CONFIG = 0 (Disabled)
UXRCE_DDS_CFG = 102 (TELEM2)
SER_TEL2_BAUD = 921600  # Required for ROS2 DDS bridge
```

## Pi5 host OS(UBUNTU 24.04 used in this case)
#### 1. Install Required Packages
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Docker
sudo apt install -y docker.io
sudo usermod -aG docker $USER

# Install Avahi for mDNS hostname resolution
sudo apt install -y avahi-daemon avahi-utils

# Install SSH server
sudo apt install -y openssh-server
```

#### 2. Configure Hostname for Network Discovery
```bash
# Set hostname (makes Pi discoverable as pi5drone.local)
sudo hostnamectl set-hostname pi5drone

# Restart Avahi
sudo systemctl restart avahi-daemon

# Verify mDNS is working
avahi-browse -a -t
```

#### 3. Configure SSH Server
```bash
# Enable and start SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Optional: Configure SSH for key-only authentication
sudo nano /etc/ssh/sshd_config
# Set: PasswordAuthentication no (after adding your public key)

# Restart SSH service
sudo systemctl restart ssh
```

#### 4. Add Your Development Computer's SSH Key
```bash
# On your development computer, generate key if needed
ssh-keygen -t ed25519 -C "your-email@example.com"

# Copy public key to Pi
ssh-copy-id dorten@pi5drone.local

# Test connection
ssh dorten@pi5drone.local
```

#### UART pinout assignment 
```bash
### frees pins from login console to be used as UART
sudo nano /boot/firmware/config.txt
### scroll to end and add:
enable_uart=1
# reboot and check if it worked
cat /boot/firmware/cmdline.txt
    # if:
    console=tty1 # then it worked
```

#### Finding the UART Device File

##### 

```bash
# Check for the convenient symlink first
ls -l /dev/serial0
# if exists use </dev/serial0> in the code (older Pies)
# else: 
ls -l /dev/ttyAMA* #(the actual hardware UART on Pi5)
# look for </dev/ttyAMA0>, if found all good
```

***

##### Why two names?

- **`/dev/ttyAMA0`** = the actual hardware device (always exists if UART is enabled)
- **`/dev/serial0`** = a convenient alias/symlink (created on some systems, not others)


## Setup Instructions in Docker container
The UART configuration below is done on the **Raspberry Pi host OS**, not inside the container:
- UART is exposed by the host Linux kernel, and `/boot/firmware/config.txt` is a host boot-time file, so container changes cannot configure hardware UART.
- Practical implication: editing this from inside Docker will not persist hardware boot configuration for the Pi.
```bash
sudo nano /boot/firmware/config.txt
### add this line at the end (if not present)
enable_uart=1
```


### 1. Clone Repository

**On Raspberry Pi 5:**
```bash
cd /home
git clone git@github.com:Dorten8/MasterThesisDrone.git ws
cd ws
```

### 2. Configure SSH for Container

The container is configured to use the host's SSH keys via read-only mount. Ensure you have SSH keys on the Pi host:

```bash
# Generate SSH key on Pi (if not already done)
ssh-keygen -t ed25519 -C "your-email@example.com"

# Add public key to GitHub
cat ~/.ssh/id_ed25519.pub
# Copy output and add to: https://github.com/settings/keys
```

**How it works:**
- `.devcontainer/devcontainer.json` mounts host's `~/.ssh` into container
- Container has `openssh-client` installed via Dockerfile
- Git operations inside container use host's SSH credentials

### 3. Open in VS Code

**From your development computer:**
```bash
# Connect to Pi via SSH
ssh dorten@pi5drone.local

# Or use VS Code Remote-SSH extension:
# 1. Install Remote-SSH extension
# 2. Press F1 → "Remote-SSH: Connect to Host"
# 3. Enter: dorten@pi5drone.local
# 4. Open folder: /home/ws
```

**When VS Code opens the workspace:**
1. VS Code detects `.devcontainer` configuration
2. Click "Reopen in Container" when prompted
3. Container builds automatically (first time takes ~5-10 minutes)
4. Development environment ready

### 4. Verify Setup

Inside the container:
```bash
# Check ROS 2 installation
ros2 --version

# Check SSH access to GitHub
ssh -T git@github.com

# Check Git configuration
git status
```

### 4.1 Verify PX4 <-> ROS 2 XRCE-DDS Link (SITL)

These checks confirm PX4 SITL can publish to ROS 2 and accept commands back.

#### A) PX4 -> ROS 2 (read path)
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

ros2 topic list | grep -E '^/fmu/|^/rt/fmu/'
ros2 interface show px4_msgs/msg/VehicleStatus
ros2 topic hz /fmu/out/vehicle_status
ros2 topic echo /fmu/out/vehicle_status
```

#### B) ROS 2 -> PX4 (write path)
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: 0, param1: 1.0, param2: 0.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
```
In the PX4 shell, confirm an ACK arrives:
```bash
listener vehicle_command_ack
```
Note: If `result: 1` appears, the command was rejected (common when not ready to arm), but the round-trip link still works.

#### C) XRCE status and time sync (sanity)
```bash
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

ros2 topic echo /fmu/out/timesync_status
ros2 topic hz /fmu/out/sensor_combined
```

### 5. Getting Started (Docker) for `mocap_px4_bridge`

Inside the container, use this repository as ROS 2 workspace root (`/home/ws`), and place ROS packages in `/home/ws/src`:

```bash
cd /home/ws/src

git clone https://github.com/SaxionMechatronics/mocap.git
git clone https://github.com/SaxionMechatronics/mocap_px4_bridge.git
git clone https://github.com/PX4/px4_msgs.git

cd /home/ws
source /opt/ros/humble/setup.bash
rosdep update
# `rosdep init` is already handled in the Docker image; `rosdep update` should run as the current user to keep cache files in that user's home.
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /home/ws/install/setup.bash

ros2 launch mocap_px4_bridge run.launch.py
```

Notes:
- This adapts the upstream `mocap_px4_bridge` Getting Started flow to this Docker workspace layout.
- Keep all ROS 2 packages in `/home/ws/src` so `rosdep` and `colcon` can resolve and build correctly.
- Clone/build inside the container terminal so dependency resolution and builds run against the ROS 2 Humble Docker environment, not the host OS.

## Architecture Notes

### Why Ubuntu 22.04 in Container on Ubuntu 24.04 Host?
- **ROS 2 Humble** officially supports Ubuntu 22.04 (Jammy)
- **Gazebo Classic** requires Ubuntu 22.04 dependencies
- Docker provides isolation, allowing older Ubuntu in container while host runs newer version
- Ensures reproducible environment across different development machines

### Why Docker Container on Pi 5?
- **Portability:** Identical environment on any machine
- **Isolation:** Development dependencies don't affect host system
- **Version Control:** Container configuration tracked in Git
- **Reproducibility:** Anyone can replicate exact setup

## Troubleshooting

### Cannot connect to Pi via hostname
```bash
# On Pi, check Avahi status
sudo systemctl status avahi-daemon

# On development computer, check mDNS resolution
ping pi5drone.local
```

### SSH connection refused
```bash
# On Pi, check SSH service
sudo systemctl status ssh

# Check firewallT
sudo ufw status
```

### Git push fails with "Permission denied"
```bash
# Verify SSH key is on GitHub
ssh -T git@github.com

# Should respond: "Hi Dorten8! You've successfully authenticated..."
```

### Container won't build
```bash
# Check Docker service
sudo systemctl status docker

# Clean Docker cache
docker system prune -a
```


### 5. Getting Started (Docker) for `mocap_px4_bridge`
...

## Motion Capture (Mocap) Integration with OptiTrack

### Overview

The drone uses motion capture data from an OptiTrack system to provide accurate position and orientation estimates. The `mocap_px4_bridge` package bridges mocap data from the OptiTrack client into ROS 2 topics and forwards it to the PX4 autopilot via the uXRCE-DDS middleware.

### Architecture

```
OptiTrack System
       ↓
   mocap package (OptiTrack client)
       ↓
  ROS 2 Topics (/mocap/pose, etc.)
       ↓
mocap_px4_bridge (converts to px4_msgs)
       ↓
uXRCE-DDS Bridge
       ↓
PX4 Autopilot (EKF2 estimator fusion)
```

### How It Works

1. **mocap package**: 
   - Reads pose data from OptiTrack hardware/software
   - Publishes stamped pose and marker data as ROS 2 topics
   - Handles coordinate frame transformations and sensor synchronization

2. **mocap_px4_bridge**:
   - Subscribes to mocap ROS 2 topics
   - Converts mocap pose estimates to PX4 vehicle odometry messages (`px4_msgs/VehicleOdometry`)
   - Publishes on uXRCE-DDS bridge for PX4 to consume

3. **PX4 Autopilot**:
   - Receives mocap odometry via uXRCE-DDS
   - Fuses with onboard sensors (IMU, barometer, optical flow) in EKF2 estimator
   - Uses fused state for autonomous flight control

### Dependencies

The mocap integration relies on three key packages managed as Git submodules:

1. **mocap** (`Dorten8/mocap`)
   - **Source:** Forked from [SaxionMechatronics/mocap](https://github.com/SaxionMechatronics/mocap)
   - **Reason for fork:** The original repository supports both Vicon and OptiTrack motion capture systems. Since this project uses **OptiTrack exclusively**, the Vicon SDK was removed to reduce build complexity, eliminate unused dependencies, and streamline compilation. The modified version focuses only on OptiTrack support.
   - **Key modifications:** Disabled Vicon SDK build configuration, removed Vicon client headers and source files

2. **mocap_px4_bridge** (`SaxionMechatronics/mocap_px4_bridge`)
   - **Source:** Upstream from [SaxionMechatronics/mocap_px4_bridge](https://github.com/SaxionMechatronics/mocap_px4_bridge)
   - **Purpose:** Bridges ROS 2 mocap topics to PX4 vehicle odometry messages
   - **No modifications:** Used as-is from upstream

3. **px4_msgs** (`SaxionMechatronics/px4_msgs`)
   - **Source:** Upstream from [PX4/px4_msgs](https://github.com/PX4/px4_msgs) via Saxion
   - **Purpose:** Provides PX4 message definitions for ROS 2
   - **No modifications:** Used as-is from upstream

### Building

The packages are automatically cloned and built as part of the normal colcon workflow:

```bash
cd /home/ws
colcon build --symlink-install
```

# RVIZ simple start
### Goal
Show a visible object in **RViz on the laptop** that is **published from the Pi** over ROS 2 (Humble) using the simplest reliable setup: **static TF + Marker**.

Assumptions:
- Pi IP: `192.168.74.5`
- Laptop and Pi are on the same Wi‑Fi.
- Both use `ROS_DOMAIN_ID=0` and `rmw_fastrtps_cpp`.

---
## Pi (inside the devcontainer)

### Terminal A — publish TF (`map -> base_link`)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

### Terminal B — publish a red cube marker on `/visualization_marker`
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 5 /visualization_marker visualization_msgs/msg/Marker "{
  header: {frame_id: 'map'},
  ns: 'demo',
  id: 1,
  type: 1,
  action: 0,
  pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}},
  scale: {x: 0.5, y: 0.5, z: 0.5},
  color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}
}"
```

Leave both running.

---

## Laptop

### Terminal — set env + start RViz
(fish shell version)
```fish
set -x ROS_DOMAIN_ID 0
set -x ROS_LOCALHOST_ONLY 0
set -x RMW_IMPLEMENTATION rmw_fastrtps_cpp
bash -lc 'source /opt/ros/humble/setup.bash && rviz2'
```

### RViz steps
1. Set **Fixed Frame** = `map`
2. **Add** → **Marker**
3. Set **Topic** = `/visualization_marker`

You should see the red cube.

(Optional quick check from laptop terminal)
```fish
bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /visualization_marker --once'
```
# Micro-XRCE-DDS Agent
I needed to re-implment this agent as per the offical guide to PX4 with ROS2
### Basic work check
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 
#prints topic_created....etc

#in another terminal on the companion computer:
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort
# prints one of the topics -> in real



## Thesis Finish Bootstrap (Draft)

1. Lock the ROS 2 <-> PX4 link (SITL + FC) using the XRCE-DDS checks above.
2. Bring up the FC in XRCE-DDS mode on TELEM2 at 921600 and confirm `/fmu/out/*` topics.
3. Ingest OptiTrack mocap and convert ENU/FLU -> NED/FRD once, immediately after read.
4. Feed the transformed pose into `mocap_px4_bridge` -> `/fmu/in/vehicle_visual_odometry`.
5. Validate fusion with stationary tests (move up/down, yaw) against `vehicle_odometry` or `vehicle_local_position`.
6. Run simple ROS 2 commands: takeoff, hover, land.
7. Run a simple end-to-end test setup from A -> B.
8. Introduce obstacles.
9. Design and run tests that collide with the obstacle.
10. Review data in Foxglove, plot signals, and check hypothesized correlations.
11. Write the empirical part of the thesis.
12. Write the remaining thesis sections.