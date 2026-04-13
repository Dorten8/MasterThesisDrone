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