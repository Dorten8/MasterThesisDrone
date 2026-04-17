# MAC addresses:
eth0 (Ethernet): 88:a2:9e:65:43:b4

wlan0 (WiFi): 88:a2:9e:65:43:b5

docker0 (Docker bridge): 36:b5:a7:3c:10:02

## SSHFS
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

### 0) Prerequisite in this workspace
- Keep only one agent repo: `src/micro-ROS-Agent`
- Do not use `src/Micro-XRCE-DDS-Agent` for ROS colcon builds (it is not a ROS package)

### 1) Build workspace (inside dev container)
```bash
cd /home/ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /home/ws/install/setup.bash
```

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
