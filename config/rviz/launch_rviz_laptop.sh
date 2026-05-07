#!/bin/bash
# Launch RViz on laptop for real-time drone odometry visualization
# Usage: ./launch_rviz_laptop.sh

set -e

# Set ROS 2 environment for network communication with Pi
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Starting RViz for drone odometry visualization ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""
echo "Waiting for ROS 2 to initialize..."
sleep 2

# Get the path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Path to RViz config (relative to this script)
RVIZ_CONFIG="$SCRIPT_DIR/config/rviz/drone_odometry.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "ERROR: RViz config not found at: $RVIZ_CONFIG"
    exit 1
fi

echo "Using RViz config: $RVIZ_CONFIG"
echo ""
echo "=== RViz starting... ==="
echo "The odometry display shows:"
echo "  - Green axes: drone frame orientation (FRD - Forward-Right-Down)"
echo "  - Purple arrows: velocity vector"
echo "  - Trail: trajectory history (max 1000 points)"
echo ""
echo "Topics visualized: /fmu/in/vehicle_visual_odometry"
echo ""

source /opt/ros/humble/setup.bash
rviz2 -d "$RVIZ_CONFIG"
