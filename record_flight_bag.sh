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

# Output directory
OUTPUT_DIR="${1:-.}"

# Create timestamp for bag file
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILE="$OUTPUT_DIR/flight_${TIMESTAMP}"

echo "=== ROS 2 Bag Recording Started ==="
echo "Output: $BAG_FILE"
echo ""
echo "Topics being recorded:"
echo "  - /fmu/in/vehicle_visual_odometry   (mocap pose sent to PX4)"
echo "  - /fmu/out/vehicle_odometry         (PX4 fused odometry)"
echo "  - /fmu/out/vehicle_local_position   (PX4 local position)"
echo "  - /fmu/out/vehicle_attitude         (PX4 attitude)"
echo "  - /poses                            (raw mocap poses)"
echo ""
echo "Recording... Press Ctrl+C to stop."
echo ""

mkdir -p "$OUTPUT_DIR"

ros2 bag record \
  -o "$BAG_FILE" \
  /fmu/in/vehicle_visual_odometry \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /poses

echo ""
echo "=== Recording Complete ==="
echo "Bag file saved to: $BAG_FILE"
echo ""
echo "To replay and visualize:"
echo "  ros2 bag play $BAG_FILE"
echo "  (in another terminal: rviz2 -d /home/ws/config/rviz/drone_odometry.rviz)"
