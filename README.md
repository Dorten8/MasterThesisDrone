# RATFLY — Deflective Advantage: Confined-Space Drone Operations

[![PX4](https://img.shields.io/badge/PX4-v1.16.1rc-orange)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20(container)-E95420)](https://ubuntu.com/)

Reproduction companion for the Master's thesis *"The Deflective Advantage: Reducing Impact Shock and Vibration in Confined-Space Drone Operations"* by Jakub Sejkora (ITU Copenhagen).

This repository contains the complete hardware BOM, software stack, and data analysis pipeline for an autonomous quadcopter with a freely rotating protective cage, designed to survive and exploit physical collisions in signal-denied indoor environments.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Repository Structure](#repository-structure)
- [Hardware Bill of Materials](#hardware-bill-of-materials)
- [3D CAD Model (OnShape)](#3d-cad-model-onshape)
- [How the Drone Flies](#how-the-drone-flies)
- [Network Reference](#network-reference)
- [Pi 5 Host Setup](#pi-5-host-setup)
- [Container Setup (ROS 2 Humble)](#container-setup-ros-2-humble)
- [PX4 Integration](#px4-integration)
- [Startup & Experiment Pipeline](#startup--experiment-pipeline)
- [SSHFS Workflow](#sshfs-workflow)
- [Data Analysis Pipeline](#data-analysis-pipeline)
- [🚨 Critical Gotchas](#-critical-gotchas)
- [Full Rebuild from Scratch](#full-rebuild-from-scratch)
- [Troubleshooting](#troubleshooting)

---

## Architecture Overview

The system is split across **four computational layers** that communicate over separate channels:

```
┌──────────────────────────────────────────────────────────────────────┐
│                   WORKSTATION (laptop, your desk)                    │
│  ┌─────────┐  ┌───────────┐  ┌─────────────┐  ┌──────────────────┐  │
│  │ VS Code │  │ Foxglove  │  │ Rviz2       │  │ QGroundControl   │  │
│  │ Remote  │  │ Studio    │  │ (visualise) │  │ (PX4 params,     │  │
│  │ SSH+Dev │  │ (telemetry│  │             │  │  firmware flash) │  │
│  │ Cont.   │  │  viz)     │  │             │  │                  │  │
│  └────┬────┘  └─────┬─────┘  └──────┬──────┘  └────────┬─────────┘  │
│       │Wi-Fi        │Wi-Fi          │Wi-Fi             │Wi-Fi       │
└───────┼─────────────┼───────────────┼──────────────────┼────────────┘
        │             │               │                  │
┌───────┼─────────────┼───────────────┼──────────────────┼────────────┐
│   ┌───┘             │               │                  │            │
│   │     Pi 5 DRONE (Raspberry Pi 5 — Ubuntu 24.04 Host)            │
│   │                                                                 │
│   │  ┌─────────────── DOCKER CONTAINER (Ubuntu 22.04 + ROS 2 Humble)│
│   │  │  ┌──────────────────────────────────────────────────────┐   │
│   │  │  │  HIGH-LEVEL CONTROL (Python nodes)                   │   │
│   │  │  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ │   │
│   │  │  │  │flight_      │ │flight_       │ │flight_       │ │   │
│   │  │  │  │director.py  │ │missions.py   │ │recorder.py   │ │   │
│   │  │  │  └──────┬───────┘ └──────────────┘ └──────┬───────┘ │   │
│   │  │  │         │ publishes setpoints              │ logs to │   │
│   │  │  │         ▼                                  ▼ MCAP    │   │
│   │  │  │  ┌──────────────────────────────────────────────┐    │   │
│   │  │  │  │  ROS 2 NODE GRAPH (pub/sub + services)       │    │   │
│   │  │  │  │  Topics: /fmu/in/trajectory_setpoint,        │    │   │
│   │  │  │  │          /fmu/out/vehicle_odometry, ...      │    │   │
│   │  │  │  └──────────┬───────────────────────────┬───────┘    │   │
│   │  │  │             │                           │            │   │
│   │  │  │  ┌──────────▼──────────┐  ┌─────────────▼────────┐  │   │
│   │  │  │  │ mocap_px4_bridge    │  │ motion_capture_      │  │   │
│   │  │  │  │ (ENU → NED coord    │  │ tracking_node        │  │   │
│   │  │  │  │  transform)         │  │ (OptiTrack NatNet    │  │   │
│   │  │  │  └──────────┬──────────┘  │  client)             │  │   │
│   │  │  │             │             └──────────┬───────────┘  │   │
│   │  │  │             │ Wi-Fi multicast         │             │   │
│   │  │  │             │ (from MoCap server)     │             │   │
│   │  │  └─────────────┼─────────────────────────┼─────────────┘   │
│   │  │                │                         │                 │
│   │  │  ┌─────────────▼─────────────────────────▼─────────────┐   │
│   │  │  │  uXRCE-DDS AGENT (MicroXRCEAgent)                   │   │
│   │  │  │  bridges ROS 2 ↔ PX4 uORB over UART serial          │   │
│   │  │  └─────────────┬───────────────────────────────────────┘   │
│   │  │                │ /dev/ttyAMA0 (921600 baud)                │
│   │  └────────────────┼───────────────────────────────────────────┘
│   │                   │
│   │  ┌────────────────▼──────────────────────────────────────────┐
│   │  │  PIXHAWK 6C FLIGHT CONTROLLER (PX4 v1.16.1rc firmware)   │
│   │  │  ┌────────┐  ┌──────────┐  ┌───────────┐  ┌───────────┐  │
│   │  │  │EKF2    │  │Attitude  │  │Position  │  │uXRCE-DDS │  │
│   │  │  │Estimator│◄─│& Rate   │◄─│Controller│  │Client     │  │
│   │  │  │(IMU+   │  │Control   │  │(PID)     │  │(publishes │  │
│   │  │  │ MoCap) │  │          │  │          │  │ uORB as   │  │
│   │  │  └────────┘  └──────────┘  └───────────┘  │ ROS 2)   │  │
│   │  │                                            └───────────┘  │
│   │  └────────────────────┬───────────────────────────────────────┘
│   │                       │ PWM signals (50 Hz)
│   │  ┌────────────────────▼──────────────────────────────────────┐
│   │  │  ESCs (Tekko32 F4 4-in-1, AM32 firmware)                 │
│   │  │  + Motors (EMAX ECOII 2004 1600KV × 4) + GF4023 props    │
│   │  └───────────────────────────────────────────────────────────┘
│   │
│   └─────────────────────────────────────────────────────────────────┘
```

**Key data paths:**

| Path | Medium | Rate | Content |
|------|--------|------|---------|
| MoCap server ↔ Pi 5 | Wi-Fi (NatNet) | 120 Hz | Pose + yaw (ENU frame) |
| Pi 5 ↔ Pixhawk 6C | UART (TELEM2) | 921600 baud | uXRCE-DDS serialised ROS 2 topics |
| Pixhawk 6C → ESCs | PWM | 50 Hz | Motor throttle commands |
| Workstation ↔ Pi 5 | Wi-Fi (SSH) | — | Remote development + telemetry (Foxglove WS) |

---

## Repository Structure

```
.
├── .devcontainer/          # Docker + VS Code devcontainer config
│   ├── Dockerfile          # Ubuntu 22.04 + ROS 2 Humble + tools
│   ├── devcontainer.json   # --net=host, UART passthrough, X11
│   ├── post-create.sh      # First-build setup (colcon, deps)
│   └── post-start.sh       # Every-start env vars
├── config/                 # PX4 parameters, drone_config.json,
│                           #   mavlink-router config
├── drone_control/          # Flight control Python nodes
│   ├── flight_director.py  # Primary mission executor (state machine)
│   ├── flight_missions.py  # Discrete flight routines (collision sweeps)
│   ├── ghost_flight.py     # Alternative trajectory generation
│   └── flight_recorder.py  # Telemetry logger (MCAP output, one file per loop)
├── dev_logs/
│   ├── flights/            # Raw .mcap telemetry files
│   ├── analysis/           # Python analysis pipeline + Jupyter notebooks
│   │   ├── database/       # SQLite ingestion, event segmenter
│   │   ├── eda/            # EDA notebooks (angle inference, etc.)
│   │   └── models/         # Random Forest, Huber regression, feature importance
│   └── session_journals/   # Daily development log
├── src/                    # ROS 2 packages (submodules)
│   ├── mocap/              # OptiTrack NatNet client (forked)
│   ├── mocap_px4_bridge/   # ENU→NED coordinate frame transformer (C++)
│   └── px4_msgs/           # PX4 uORB → ROS 2 message definitions
├── startup-sequence.sh     # One-command flight startup
├── shutdown-sequence.sh    # Graceful cleanup (no zombies)
└── thesis/                 # LaTeX source for the thesis document
```

---

## Hardware Bill of Materials

| Component | Model | Purpose |
|---|---|---|
| Flight Controller | Holybro Pixhawk 6C | PX4 autopilot, dual IMU (ICM-42688-P + BMI088) |
| Companion Computer | Raspberry Pi 5 (8 GB) | High-level autonomy, ROS 2, MoCap ingestion |
| Frame | 4-inch wheelbase (custom) | Indoor flight compliance |
| Motors | EMAX ECOII 2004 1600KV (×4) | 860 g thrust each at 6S |
| ESCs | Tekko32 F4 4-in-1 50 A (AM32) | Motor control |
| Propellers | Gemfan GF4023 (4-inch, ×4) | Matched to motor specs |
| Battery | 6S LiPo 22.2 V nominal | High voltage → lower current draw |
| Cage (Fixed) | PETG-printed, 35.8 cm diameter | Rigid collision protection |
| Cage (Rotating) | PETG-printed, bearing-mounted | Freely rotating shell |
| Power Module | Holybro PM02 | Regulated 5 V for FC |

**Design rationale:** The 2004 stator (20 mm diameter × 4 mm height) provides high torque at mid-throttle, operating at ~45–50 % hover throttle for optimal efficiency. The 1600 KV motors on 6S produce 24 000–28 000 RPM under load, well within the 4-inch propeller's efficient range. Full TWR: ~3.8:1.

### Coordinate Frames

The system operates across three coordinate frame conventions. The figure below shows the two primary ground-fixed frames used in this work:

![Coordinate frame conventions (NED vs ENU)](thesis/Figures/coordinate_frames_NEDvsENU.png)

*NED (North-East-Down) and ENU (East-North-Up) conventions. PX4 uses NED internally; the MoCap system streams in ENU. The `mocap_px4_bridge` node performs the static rotation:  X_NED = Y_ENU, Y_NED = X_ENU, Z_NED = −Z_ENU.*

---

## 3D CAD Model (OnShape)

The complete drone assembly CAD model is available on OnShape:

🔗 **[OnShape CAD Document](https://cad.onshape.com/documents/7cbebd1596e6b9c6c98f0e3f/w/5e6908f6b6eb3e1d2b683876/e/866a8eb8a37ae0706272cbd1?renderMode=0&uiState=6a4263881140b20f69c49cb9)**

This document contains the frame plates, bearing assembly, protective cage (35.8 cm diameter), and all mounted components. It served as the primary mechanical reference throughout the design, manufacturing, and assembly phases of the thesis.

---

## How the Drone Flies

### The Full Loop (Conceptual)

1. **MoCap broadcasts poses** at 120 Hz over Wi-Fi (NatNet) from the OptiTrack system.
2. **`motion_capture_tracking_node`** receives poses and publishes them as `/poses` (ENU).
3. **`mocap_px4_bridge`** transforms ENU → NED and publishes to `/fmu/in/vehicle_visual_odometry`.
4. **PX4 EKF2** fuses external vision with onboard IMU (250 Hz) → full state estimate.
5. **`flight_director.py`** publishes `TrajectorySetpoint` commands to `/fmu/in/trajectory_setpoint`.
6. **PX4 position/attitude controller** computes motor commands → ESCs → props → thrust.
7. **`flight_recorder.py`** logs every ROS 2 topic to segmented `.mcap` files.

All of this runs inside a Docker container on the Pi 5. The flight director implements a state machine that drives the autonomous collision loop:

![Flight director state machine](thesis/Figures/flight_director_state_machine.pdf)

*State machine: DISARMED → TAKEOFF (WP0) → WP_stage (U-turn) → Exp. Start-point (WP1) → Exp. End-point (WP2) → Recovery (WP3) → auto-loop or LAND. Failsafe transitions to EMERGENCY from any state.*

### The Collision Experiment

The experimental flight pattern is a closed-loop sweep that repeatedly impacts a column at a controlled angle:

![Experiment geometry](thesis/Figures/experiment_geometry.pdf)

*Top-down layout: the drone flies a sweep leg that intersects the column at a chosen incidence angle (45° or 75°), then recovers on the far side.*

![Waypoint state machine](thesis/Figures/waypoint_state_machine.pdf)

*The autonomous waypoint loop with labelled waypoint positions and auto-loop-back.*

### Startup Sequence

```bash
# One command — starts everything
./startup-sequence.sh
```

This runs:
1. NTP time synchronisation (or MoCap fallback)
2. `MicroXRCEAgent` serial bridge (/dev/ttyAMA0, 921600 baud)
3. `motion_capture_tracking_node` (OptiTrack receiver)
4. `mocap_px4_bridge` (ENU → NED transformation)
5. `flight_director.py` + `flight_recorder.py`
6. (Optional) Foxglove bridge for telemetry visualisation

### The Experiment Pipeline

![Experiment pipeline](thesis/Figures/experiments_pipeline.pdf)

*Complete data flow from hardware through the ROS~2 software stack to telemetry storage.*

---

## Network Reference

### Finding the Drone on the Network

The Pi 5 lives at `pi5drone.local` via mDNS. If that doesn't resolve:

```bash
# Find the Pi 5's IP (scan your subnet — adjust CIDR as needed)
sudo nmap -sn 192.168.74.0/23

# Or from the Pi itself:
hostname -I
```

### MAC Addresses

| Device | Interface | MAC |
|---|---|---|
| Pi 5 drone | Ethernet (eth0) | `88:a2:9e:65:43:b4` |
| Pi 5 drone | Wi-Fi (wlan0) | `88:a2:9e:65:43:b5` |
| Pi 5 drone | Docker bridge (docker0) | `36:b5:a7:3c:10:02` |
| OptiTrack PC | Wi-Fi | `70:cd:0d:b1:67:c3` |
| Ubuntu laptop | Wi-Fi | `c8:94:02:5b:f0:d9` |

### SSHFS Workflow

Mount the Pi's workspace to your laptop for direct file access without SCP:

```bash
# Make sure the target folder exists first
mkdir -p ~/pi_drone_sshfs

# Basic mount
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs \
  -o auto_cache,reconnect,follow_symlinks

# Safer mount (disables cache to prevent SQLite/metadata page corruption over SSHFS)
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs \
  -o cache=no,cache_timeout=0,cache_stat_timeout=0,cache_dir_timeout=0,\
     cache_link_timeout=0,reconnect,follow_symlinks

# Unmount
fusermount -u -z ~/pi_drone_sshfs
```

---

## Pi 5 Host Setup

### 1. Flash OS

Flash **Ubuntu 24.04 Desktop** using Raspberry Pi Imager:
- Hostname: `pi5drone`
- SSH enabled (Services tab)
- Wi-Fi configured for local network
- Locale: Europe/Copenhagen

### 2. Post-Flash Fixes

```bash
# SSH server is NOT auto-started despite Imager setting
sudo apt install openssh-server
sudo systemctl enable --now ssh

# mDNS hostname resolution
sudo apt install avahi-daemon
# (also install on workstation: avahi-daemon libnss-mdns)
sudo systemctl enable --now avahi-daemon

# Test: ping pi5drone.local from workstation
```

### 3. Docker

```bash
sudo apt install docker.io docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# ⚠️ Remove conflicting package first:
sudo apt remove containerd

# Rootless Docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

### 4. UART (for Pixhawk serial)

```bash
sudo nano /boot/firmware/config.txt
# Add at end:
enable_uart=1

# Reboot and verify:
cat /boot/firmware/cmdline.txt
# Expected: console=tty1 (not serial)
ls -l /dev/ttyAMA0
```

### 5. Python Venv (for host-side tooling)

```bash
sudo apt install python3.12-venv
python3 -m venv ~/ros2_env
```

### 6. SSH Key for Git

```bash
ssh-keygen -t ed25519 -C "pi5drone"
cat ~/.ssh/id_ed25519.pub   # add to GitHub
```

---

## Container Setup (ROS 2 Humble)

Ubuntu 24.04 (Pi 5 host) is not directly compatible with ROS 2 Humble binary packages. Containerisation provides an Ubuntu 22.04 environment with native ROS 2 support.

```bash
mkdir -p ~/ws/.devcontainer
cd ~/ws
git clone git@github.com:Dorten8/MasterThesisDrone.git .
```

The `.devcontainer/` directory contains:
- **`Dockerfile`** — Ubuntu 22.04 + ROS 2 Humble base, `openssh-client`, non-root user, passwordless sudo
- **`devcontainer.json`** — `--net=host`, `--pid=host`, `--privileged` (UART access), X11 forwarding, SSH key mount

Open in VS Code:
1. Install Remote-SSH and Dev Containers extensions
2. `Ctrl+Shift+P` → Remote-SSH: Connect to Host → `dorten@pi5drone.local`
3. Open `/home/ws`
4. "Reopen in Container" (first build: ~30–50 min on Pi 5)

Verify:
```bash
lsb_release -a          # Ubuntu 22.04 LTS
ros2 --version          # ROS 2 Humble
echo $ROS_DISTRO        # humble
```

### Environment Variables (baked into the container)

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SSOT_CONFIG_PATH=/home/ws/config
```

---

## PX4 Integration

### Pixhawk ↔ Pi Wiring

| Pixhawk TELEM2 | Pi GPIO | Function |
|---|---|---|
| TX (pin 2) | GPIO 15 / RXD (pin 10) | FC → Pi |
| RX (pin 3) | GPIO 14 / TXD (pin 8) | Pi → FC |
| GND (pin 6) | Ground (pin 6) | Common ground |

### PX4 Parameters (set via QGroundControl)

| Parameter | Value | Reason |
|---|---|---|
| `MAV_0_CONFIG` | TELEM2 | Primary MAVLink (debug only) |
| `UXRCE_DDS_CFG` | 102 (TELEM2) | uXRCE-DDS client on serial |
| `SER_TEL2_BAUD` | 921600 | Baud for uXRCE-DDS link |
| `EKF2_MAG_TYPE` | 5 (None) | Disable magnetometer indoors |
| `EKF2_EV_DELAY` | ~20–60 ms | Calibrate via cross-correlation (see *Critical Gotchas*) |

### Launch

```bash
# Terminal 1: uXRCE-DDS agent
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# Terminal 2: Verify topics
ros2 topic list
ros2 topic echo /fmu/out/vehicle_odometry

# Terminal 3: Startup sequence
./startup-sequence.sh
```

### uXRCE-DDS vs MAVLink Mode

Only one process can own `/dev/ttyAMA0`:

```bash
# uXRCE-DDS mode (flight operations)
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# Switch to MAVLink-router mode (debug/ground station)
mavlink-routerd -c /home/ws/config/mavlink-router/main.conf
```

### ROS 2 ↔ PX4 Topic Map

```
/fmu/in ROS2 → PX4            /fmu/out PX4 → ROS2
──────────────────────         ──────────────────────
/fmu/in/trajectory_setpoint    /fmu/out/vehicle_odometry
/fmu/in/vehicle_command        /fmu/out/vehicle_attitude
/fmu/in/offboard_control_mode  /fmu/out/vehicle_local_position
/fmu/in/vehicle_visual_odometry /fmu/out/battery_status
/fmu/in/actuator_motors        /fmu/out/sensor_combined
... (40+ topics total)         ... (40+ topics total)
```

---

## Startup & Experiment Pipeline

### One-Command Startup

```bash
./startup-sequence.sh
```

Shut down gracefully:
```bash
./shutdown-sequence.sh
```

### Manual Startup (for debugging)

```bash
# Terminal A: uXRCE-DDS Agent
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# Terminal B: MoCap receiver
ros2 run motion_capture_tracking motion_capture_tracking_node \
  --ros-args -p type:=optitrack -p hostname:=192.168.74.3

# Terminal C: Coordinate frame bridge
ros2 run mocap_px4_bridge mocap_px4_bridge \
  --ros-args -p mocap_topic:=/poses -p drone_name:=rigid_body_7 \
  -p px4_topic:=/fmu/in/vehicle_visual_odometry

# Terminal D: Verify
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort

# Terminal E: Flight director + recorder
ros2 run drone_control flight_director.py &
ros2 run drone_control flight_recorder.py &
```

### Run a Collision Experiment

```bash
# 1. Full startup
./startup-sequence.sh

# 2. The column_sweep_loop mission runs autonomously:
#    WP1 (0.000, 1.200) → WP2 (0.100, 1.200) →
#    WP3 (0.100, -1.200) → WP4 (0.000, -1.200) → loop
```

### Slice Flights + Rebuild Database (after flying)

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

---

## SSHFS Workflow

```bash
# Mount the Pi's workspace to your laptop
mkdir -p ~/pi_drone_sshfs

# Connect (no-cache mode — prevents SQLite corruption over SSHFS)
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs \
  -o cache=no,cache_timeout=0,cache_stat_timeout=0,cache_dir_timeout=0,\
     cache_link_timeout=0,reconnect,follow_symlinks

# Unmount
fusermount -u -z ~/pi_drone_sshfs
```

---

## Data Analysis Pipeline

The analysis pipeline lives in `dev_logs/analysis/`:

```
dev_logs/analysis/
├── experiments_analysis.ipynb     # Primary analysis notebook
├── experiments_kinematics.ipynb   # Kinematic decomposition
├── eda/eda_angle_prediction.py    # Feature correlation, parallel coordinates
├── models/rf_angle_prediction.py  # Random Forest + Huber regression
├── summary_plots.py               # All thesis figures
├── database/
│   ├── db_mcap_event_segmenter.py # Splits raw MCAP → individual collision passes
│   ├── db_pipeline.py             # Ingests passes → SQLite DB
│   ├── exa_loader.py             # MCAP → NumPy ingestion
│   ├── exa_pipeline.py           # Savitzky-Golay filtering + event detection
│   ├── exa_kinematics.py         # Perpendicular error, closest approach
│   └── kin_calculator.py         # Collision clearance, RMSLD metrics
└── experiments_summary.db        # SQLite (auto-generated)
```

**Key metrics:**
- **Impact angle**: Measured from velocity vector at collision
- **Closest clearance**: `min_dist_center − column_radius − cage_radius`
- **Path spread (RMSLD)**: `sqrt(mean(perpendicular_distances²))` × 1000 (mm)
- **SIAE**: Spatial integral of absolute error (recovery area in m²)

### Python Path Resiliency

Always run pipeline modules from the repository root:

```bash
# ✅ Correct — resolves top-level imports
python3 -m dev_logs.analysis.database.db_pipeline

# ❌ Wrong — ModuleNotFoundError
python3 dev_logs/analysis/database/db_pipeline.py
```

### Git Backup Workflow for Flight Data

```bash
# Option A: normal habit (stages code + allowed flight MCAPs)
git add .
git commit -m "short message"
git push

# Option B: stage only flight MCAPs
./dev_logs/stage_flight_backups.sh /home/ws/dev_logs/flights 100M
git commit -m "backup flight mcap files"
git push
```

### Rebuild the Thesis PDF

```bash
cd /home/dorten/MasterThesisDrone/thesis
rm -f main.out main.log main.aux main.bbl main.blg main.toc \
      main.lof main.lot main.bcf main.run.xml main.fdb_latexmk main.fls
latexmk -pdf main.tex
```

Requires `texlive-full` on the host (not in the container):
```bash
sudo apt install texlive-full latexmk biber
```

---

## 🚨 Critical Gotchas

These are hard-won lessons from the development process. Ignore them at your peril.

### 1. EKF2 Magnetometer Must Be Disabled Indoors

Set `EKF2_MAG_TYPE = 5` (None). Indoor magnetic fields (building steel, power cabling, lab equipment) fight the MoCap yaw data. The EKF2 will oscillate between magnetic north and the OptiTrack yaw, producing yaw drift that destabilises the controller.

### 2. Velocity Feedforward Requires Explicit Heartbeat Flag

The offboard heartbeat message must set `hb.velocity = True`. Without it, PX4 completely ignores the velocity vector in `TrajectorySetpoint`. The drone will then rely on a laggy P-only position controller, producing jerky velocity spikes and waypoint overshoot.

```python
hb.position = True
hb.velocity = True   # ← This is the critical line
```

### 3. Geofence Boundaries Must Account for Cage Radius

The PETG cage has a 17.9 cm radius (35.8 cm diameter). If you place a waypoint at coordinate limit `1.350 m`, the cage edge will be at `1.350 + 0.179 = 1.529 m` — breaching a 1.50 m geofence and triggering motor shutdown. **Always subtract the cage radius** from the geofence boundary to find the true waypoint limit.

### 4. Battery Voltage Sag → 3-Minute Effective Flight Time

Under high-torque load, a 6S battery drops over 1 V instantly. Measured consumption rate is ~20 % per minute, yielding a hard limit of **3 minutes** per battery pack. Set a failsafe to land/disarm at 40 % battery to avoid cell damage. The voltage sag also introduces control latency — the FC commands 70 % throttle, but the actual motor RPM corresponds to a lower voltage than expected.

### 5. NatNet Streaming Up-Axis: Z-Axis (Not Y-Axis)

Motive's internal viewport is Y-Up, but the ROS 2 `mocap_px4_bridge` expects Z-Up (ENU). In Motive: **Settings → Streaming → NatNet → Up Axis = Z-Axis**. If set to Y-Axis, the bridge will misinterpret Y-as-altitude, crashing the EKF2.

### 6. The 90-Degree Cross-Coupling Trap

The `mocap_px4_bridge` C++ node hardcodes:
```
X_ned = X_mocap
Y_ned = -Y_mocap
```
Your Python offboard controller must apply *exactly* this convention. A different rotation matrix will cause the drone to fly figure-eights or spiral sideways.

### 7. QoS Durability: PX4 Subscribes with TRANSIENT_LOCAL

When publishing to `/fmu/in/vehicle_command` or `/fmu/in/offboard_control_mode`, use `durability = TRANSIENT_LOCAL`. If you publish with `VOLATILE`, PX4 never sees the commands.

Similarly, when subscribing to `/poses` (from `motion_capture_tracking`), use `VOLATILE` — otherwise you get stale cached poses.

### 8. EKF2_EV_DELAY Calibration via Cross-Correlation

The MoCap pipeline (camera exposure + NatNet network + bridge processing) introduces 20–60 ms of latency. If `EKF2_EV_DELAY = 0`, the EKF2 fuses vision data at the wrong timestamp, breaking gyro-to-vision alignment.

**Calibration method:** Cross-correlate the onboard gyroscope yaw rate against the delayed visual odometry yaw rate from a flight recording. The time offset maximising the correlation is your true `EKF2_EV_DELAY` in milliseconds.

### 9. Checking if the Serial Port Is Free

```bash
sudo fuser -v /dev/ttyAMA0
# if owned by mavlink-routerd:
sudo systemctl stop mavlink-routerd
```

### 10. IDE Service Worker Crash

If notebook rendering fails with `InvalidStateError: Failed to register a ServiceWorker`:

```bash
pkill -f -9 antigravity; pkill -f -9 vscodium; pkill -f -9 code
rm -f ~/.config/Antigravity/code.lock
rm -f ~/.config/Antigravity/SingletonLock
rm -f ~/.config/Antigravity/SingletonSocket
rm -rf ~/.config/Antigravity/'Service Worker'
rm -rf ~/.config/Antigravity/Cache
rm -rf ~/.config/Antigravity/'Code Cache'
```

---

## Quick-Reference Scripts

| Script | When to run |
|---|---|
| `startup-sequence.sh` | Every flight session |
| `shutdown-sequence.sh` | After flying (cleanup) |
| `.devcontainer/post-create.sh` | First container build (auto) |
| `.devcontainer/post-start.sh` | Every container start (auto) |
| `dev_logs/stage_flight_backups.sh` | Before `git push` to include MCAPs |

---

## Full Rebuild from Scratch

### 0. Prerequisites

- **Pi 5 (8 GB)** with freshly flashed **Ubuntu 24.04 Desktop** (Raspberry Pi Imager)
- Set hostname: `pi5drone`
- Enable SSH **and** Wi-Fi during Imager setup
- This repo cloned onto the Pi:
  ```bash
  cd ~
  git clone git@github.com:Dorten8/MasterThesisDrone.git ws
  ```

> **Imager caveat:** despite ticking "Enable SSH", the SSH daemon is NOT started on first boot. Fix below.

### 1. Pi 5 Host OS

```bash
# ── SSH fix (Imager lies) ──
sudo apt install openssh-server
sudo systemctl enable --now ssh

# ── mDNS ──
sudo apt install avahi-daemon
sudo systemctl enable --now avahi-daemon
# Also install on workstation: avahi-daemon libnss-mdns

# ── Docker ──
sudo apt install docker.io docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo apt remove containerd
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# ── UART ──
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
# Reboot, then verify:
ls -l /dev/ttyAMA0
cat /boot/firmware/cmdline.txt   # should show console=tty1

# ── Python venv ──
sudo apt install python3.12-venv
python3 -m venv ~/ros2_env

# ── SSH key ──
ssh-keygen -t ed25519 -C "pi5drone"
cat ~/.ssh/id_ed25519.pub
```

**Verify:**
```bash
ping pi5drone.local    # from workstation → responds
ssh dorten@pi5drone.local
docker ps              # daemon running
ls -l /dev/ttyAMA0     # UART exists
```

### 2. VS Code Remote-SSH + Dev Container

On your **workstation** (not Pi):

1. Install **Remote-SSH** and **Dev Containers** VS Code extensions
2. `Ctrl+Shift+P` → Remote-SSH: Connect to Host → `dorten@pi5drone.local`
3. Open `/home/dorten/ws`
4. `Ctrl+Shift+P` → Dev Containers: Reopen in Container

**First build:** ~30–50 min on Pi 5. Docker caches layers afterward.

### 3. ROS 2 Workspace Build (colcon)

Triggered automatically by `postCreateCommand`, but can re-run:

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
| `drone_control` | `drone_control/` | Python nodes |

### 4. Validate the Container

```bash
lsb_release -a          # → Ubuntu 22.04 LTS
ros2 --version          # → ROS 2 Humble
echo $ROS_DISTRO        # → humble
which MicroXRCEAgent    # → /usr/local/bin/MicroXRCEAgent
which mavlink-routerd   # → /usr/bin/mavlink-routerd
```

---

## Troubleshooting

| Issue | Cause | Fix |
|---|---|---|
| SSH connection fails | SSH daemon not running | `sudo systemctl status ssh; sudo systemctl enable --now ssh` |
| `pi5drone.local` unreachable | Avahi not running | `sudo systemctl enable --now avahi-daemon` (on both Pi and workstation) |
| Docker build very slow | First build on ARM64 | Normal; subsequent builds are cached |
| ROS 2 commands not found | Environment not sourced | Verify `postCreateCommand` in devcontainer.json |
| uXRCE-DDS won't connect | UART wiring or baud mismatch | Check `enable_uart=1` in config.txt; verify `/dev/ttyAMA0` exists; confirm 921600 baud matches between Agent and PX4 parameter |
| ROS 2 topics empty | uXRCE-DDS client not running on FC | Verify `UXRCE_DDS_CFG = 102` in PX4 parameters |
| MoCap tracking drops during cage collision | Occlusion of reflective markers | Normal — EKF2 bridges via IMU propagation (see thesis §4.1.3) |
