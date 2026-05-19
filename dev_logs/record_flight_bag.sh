#!/bin/bash
# Record ROS 2 topics on Pi for post-flight analysis
# Usage: ./record_flight_bag.sh <output_dir>
# Example: ./record_flight_bag.sh ~/flight_bags/test_flight_2026_05_07

set -e

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

# Set network environment
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Get script directory
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_OUTPUT_DIR="$SCRIPT_DIR/flights"

# Output directory
OUTPUT_DIR="${1:-$DEFAULT_OUTPUT_DIR}"
MISSION_NAME="${2:-unknown_mission}"

# Create timestamp for bag file (YYYYMMDD-HHMM format)
TIMESTAMP=$(date +%Y%m%d-%H%M)
BAG_FILE="$OUTPUT_DIR/flight_${TIMESTAMP}_${MISSION_NAME}"

echo "=== ROS 2 Bag Recording Started ==="
echo "Output: $BAG_FILE"
echo ""
echo "Topics being recorded:"
echo "  - /fmu/in/vehicle_visual_odometry   (Mocap pose sent to PX4)"
echo "  - /fmu/out/vehicle_odometry         (PX4 fused odometry - World Frame)"
echo "  - /fmu/out/vehicle_local_position   (PX4 local position - NED)"
echo "  - /fmu/out/vehicle_attitude         (PX4 attitude/orientation)"
echo "  - /fmu/out/sensor_combined           (Raw/Filtered IMU data)"
echo "  - /fmu/out/actuator_outputs         (Motor/ESC signals)"
echo "  - /fmu/out/battery_status           (Voltage and Current)"
echo "  - /fmu/out/vehicle_status           (Arming/Flight Mode state)"
echo "  - /fmu/out/timesync_status          (Pi-to-PX4 clock sync quality)"
echo "  - /fmu/in/trajectory_setpoint       (Target commands from Offboard scripts)"
echo "  - /poses                            (Raw Mocap data from OptiTrack)"
echo ""
echo "Recording... Press Ctrl+C to stop."
echo ""

# Function to cleanly forward signals to the ros2 bag process
cleanup() {
  echo "Received stop signal. Cleanly stopping ros2 bag record..."
  kill -INT "$BAG_PID" 2>/dev/null
  wait "$BAG_PID"
  echo "=== Recording Complete (Cleanly Stopped) ==="
  exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup INT TERM

mkdir -p "$OUTPUT_DIR"

ros2 bag record \
  -s mcap \
  -o "$BAG_FILE" \
  /fmu/in/vehicle_visual_odometry \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /fmu/out/sensor_combined \
  /fmu/out/actuator_outputs \
  /fmu/out/battery_status \
  /fmu/out/vehicle_status \
  /fmu/out/timesync_status \
  /fmu/in/trajectory_setpoint \
  /poses &

BAG_PID=$!

# Wait for the background process to finish or be interrupted
wait "$BAG_PID"

echo ""
echo "=== Recording Complete ==="
echo "Bag file saved to: $BAG_FILE"
echo ""
echo "To replay and visualize:"
echo "  ros2 bag play $BAG_FILE"
echo "  (in another terminal: rviz2 -d /home/ws/config/rviz/drone_odometry.rviz)"
