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

# SSHFS
~/pi_drone_sshfs

### Make sure the target folder exists first
mkdir -p ~/pi_drone_sshfs
### Connect the Pi's workspace to your laptop
sshfs dorten@dorten-pi5drone.local:/home/dorten/ws ~/pi_drone_sshfs -o auto_cache,reconnect,follow_symlinks
### unmount
umount ~/pi_drone_sshfs
### The -u is unmount, -z is 'lazy' (cleans up the mess even if busy)
fusermount -u -z ~/pi_drone_sshfs

## MAVLINK-ROUTER
Running it with specific config (not default in root)
mavlink-routerd -c /home/ws/config/mavlink-router/main.conf

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

The daemon is not running
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