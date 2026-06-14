# RATFLY — Deflective Advantage: Confined-Space Drone Operations

[![PX4](https://img.shields.io/badge/PX4-v1.16.1rc-orange)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20(container)-E95420)](https://ubuntu.com/)

Reproduction companion for the Master's thesis *"The Deflective Advantage: Reducing Impact Shock and Vibration in Confined-Space Drone Operations"* by Jakub Sejkora (ITU Copenhagen).

This repository contains the complete hardware BOM, software stack, and data analysis pipeline for an autonomous quadcopter with a freely rotating protective cage, designed to survive and exploit physical collisions in signal-denied indoor environments.

---

## Repository Structure

```
.
├── .devcontainer/          # Docker + VS Code devcontainer config
├── config/                 # PX4 parameters, drone_config.json
├── drone_control/          # Flight control Python nodes
│   ├── flight_director.py  # Primary mission executor
│   ├── flight_missions.py  # Discrete flight routines (collision sweeps)
│   ├── ghost_flight.py     # Alternative trajectory generation
│   └── flight_recorder.py  # Telemetry logger (MCAP output)
├── dev_logs/
│   ├── flights/            # Raw .mcap telemetry files
│   ├── analysis/           # Python analysis pipeline + Jupyter notebooks
│   └── session_journals/   # Daily development log
├── src/                    # ROS 2 packages (submodules)
│   ├── mocap/              # OptiTrack client (forked)
│   ├── mocap_px4_bridge/   # ENU→NED coordinate transformer
│   └── px4_msgs/           # PX4 message definitions
└── thesis/                 # LaTeX source for the thesis document
```

---

## Hardware Bill of Materials

| Component | Model | Purpose |
|---|---|---|
| Flight Controller | Holybro Pixhawk 6C | PX4 autopilot, dual IMU |
| Companion Computer | Raspberry Pi 5 (8 GB) | High-level autonomy, ROS 2 |
| Frame | 4-inch wheelbase (custom) | Indoor flight compliance |
| Motors | EMAX ECOII 2004 1600KV (×4) | 860 g thrust each at 6S |
| ESCs | Tekko32 F4 4-in-1 50 A (AM32) | Motor control |
| Propellers | Gemfan GF4023 (4-inch, ×4) | Matched to motor specs |
| Battery | 6S LiPo 22.2 V nominal | High voltage → lower current draw |
| Cage (Fixed) | PETG-printed, 35.8 cm diameter | Rigid collision protection |
| Cage (Rotating) | PETG-printed, bearing-mounted | Freely rotating shell |
| Optical Flow | Matek 3901-L | Supplemental EKF2 velocity |
| Power Module | Holybro PM02 | Regulated 5 V for FC |

**Design rationale:** The 2004 stator (20 mm diameter × 4 mm height) provides high torque at mid-throttle, operating at ~45–50 % hover throttle for optimal efficiency. The 1600 KV motors on 6S produce 24 000–28 000 RPM under load, well within the 4-inch propeller's efficient range. Full TWR: ~3.8:1.

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

---

## Experiment Pipeline

### Startup Sequence

The `startup-sequence.sh` script automates the full pipeline:

1. NTP time synchronisation (or MOCAP fallback)
2. `MicroXRCEAgent` (serial, 921600 baud)
3. `motion_capture_tracking_node` (OptiTrack receiver)
4. `mocap_px4_bridge` (ENU → NED)
5. `flight_director.py` + `flight_recorder.py`

### Collision Sweep Loop

The `column_sweep_loop.py` mission flies a fixed-cycle waypoint pattern:
- **WP1**: `(0.000, 1.200, 0.500)` — pause
- **WP2**: `(0.100, 1.200, 0.500)` — transition
- **WP3**: `(0.100, −1.200, 0.500)` — collection sweep
- **WP4**: `(0.000, −1.200, 0.500)` — pause, loop back to WP1

The loop runs autonomously. At WP1 and WP4 the drone pauses, waiting for an operator prompt before continuing. Collision targets are placed at the column position with adjustable incidence angles (45°, 75°).

### Telemetry

All ROS 2 topics are logged to segmented `.mcap` files (one per loop iteration) in `dev_logs/flights/`. The `flight_recorder.py` node synchronises MoCap poses, PX4 odometry, and actuator feedback at native publish rates (~100 Hz).

---

## Data Analysis Pipeline

The analysis pipeline lives in `dev_logs/analysis/`:

```
experiments_analysis.ipynb   # Primary analysis notebook
experiments_kinematics.ipynb # Kinematic decomposition
exa_loader.py               # MCAP → NumPy ingestion
exa_pipeline.py             # Savitzky-Golay filtering + event detection
exa_kinematics.py           # Perpendicular error, closest approach
kin_calculator.py           # Collision clearance, SDLD metrics
```

**Key metrics:**
- **Impact angle**: Measured from velocity vector at collision
- **Closest clearance**: `min_dist_center − column_radius − cage_radius`
- **Path spread (SDLD)**: `np.std(perpendicular_distances) × 1000` (mm)
- **SIAE**: Spatial integral of absolute error (recovery area)

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
| MoCap tracking drops during cage collision | Occlusion of reflective markers | Normal — EKF2 bridges via IMU propagation (see Data Processing in thesis) |
