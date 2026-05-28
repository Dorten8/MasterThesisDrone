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
echo "  - /fmu/in/actuator_motors           (Commanded actuator motor inputs)"
echo "  - /poses                            (Raw Mocap data from OptiTrack)"
echo "  - /tf                               (Coordinate transform tree - live 3D visualization)"
echo "  - /tf_static                        (Static coordinate transforms)"
echo "  - /rosout                           (ROS 2 logs for state events)"
echo "  - /flight_director/active_waypoint  (Flight Director semantic state events)"
echo ""
echo "Recording... Press Ctrl+C to stop."
echo ""

# Function to cleanly forward signals to the ros2 bag process and post-process the bag
cleanup() {
  echo "Received stop signal. Cleanly stopping ros2 bag record..."
  kill -INT "$BAG_PID" 2>/dev/null
  wait "$BAG_PID"
  echo "=== Recording Stopped ==="
  
  # Resolve dynamic path to mcap binary based on architecture
  ARCH=$(uname -m)
  PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
  
  if [ "$ARCH" = "x86_64" ]; then
    MCAP_BIN="$PROJECT_ROOT/dev_logs/analysis/bin/mcap_amd64"
  elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    MCAP_BIN="$PROJECT_ROOT/dev_logs/analysis/bin/mcap_arm64"
  else
    MCAP_BIN="mcap"
  fi
  
  RAW_BAG="$BAG_FILE/$(basename "$BAG_FILE")_0.mcap"
  TEMP_RECOVERED="$BAG_FILE/temp_recovered.mcap"
  
  if [ -f "$RAW_BAG" ]; then
    echo ""
    echo "=============================================================="
    echo "⚡ POST-FLIGHT BAG OPTIMIZATION ACTIVE"
    echo "=============================================================="
    echo "  System Architecture: $ARCH"
    echo "  Optimizing: $RAW_BAG"
    echo "  Applying: Chunking, Full Indexing, and Zstd Compression"
    echo "  Processing... Please wait."
    
    # Try using the pre-compiled binary first, falling back to global 'mcap' command
    if [ -x "$MCAP_BIN" ]; then
      RUN_CMD="$MCAP_BIN"
    else
      RUN_CMD="mcap"
    fi
    
    if "$RUN_CMD" recover "$RAW_BAG" -o "$TEMP_RECOVERED" >/dev/null 2>&1; then
      mv "$TEMP_RECOVERED" "$RAW_BAG"
      echo "  Status: ✅ Success! File is fully indexed and compressed."
    else
      echo "  Status: ⚠️ Optimization failed. File left in high-throughput raw format."
      rm -f "$TEMP_RECOVERED"
    fi
    echo "=============================================================="
    echo ""
  fi

  echo "=== Recording Complete ==="
  exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup INT TERM

mkdir -p "$OUTPUT_DIR"

# Record in high-throughput mode (fastwrite) to keep CPU overhead at absolute zero during flight.
# Chunking, indexing, and compression are automatically handled after the flight in the cleanup handler.
ros2 bag record \
  -s mcap \
  --storage-preset-profile fastwrite \
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
  /fmu/in/actuator_motors \
  /poses \
  /rosout \
  /tf \
  /tf_static \
  /flight_director/active_waypoint &

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
