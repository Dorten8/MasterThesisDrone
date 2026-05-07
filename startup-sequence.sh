#!/bin/bash

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
TIMEOUT=15

# Arrays to track results
declare -a RESULTS
declare -a TIMINGS
declare -a PIDS

# Cleanup function
cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
}
trap cleanup EXIT

# Kill existing processes by pattern
kill_if_running() {
    local name="$1"
    local pattern="$2"

    if pgrep -f "$pattern" >/dev/null; then
        echo -e "${YELLOW}Found existing $name. Killing...${NC}"
        pkill -f "$pattern"
        sleep 1
        if pgrep -f "$pattern" >/dev/null; then
            echo -e "${YELLOW}Force killing $name...${NC}"
            pkill -9 -f "$pattern"
        fi
    fi
}

# Function to print header
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}ROS2 Drone System Startup${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# Function to run command in background and monitor output
run_with_monitor() {
    local name="$1"
    local command="$2"
    local search_pattern="$3"
    local success_message="$4"
    local kill_pattern="$5"

    kill_if_running "$name" "$kill_pattern"

    echo -e "${YELLOW}[$(date +%H:%M:%S)]${NC} Starting: ${BLUE}$name${NC}"
    local start_time=$(date +%s)

    local temp_file=$(mktemp)

    {
        source /opt/ros/humble/setup.bash
        source /home/ws/install/setup.bash
        eval "$command"
    } > "$temp_file" 2>&1 &
    
    local cmd_pid=$!
    PIDS+=("$cmd_pid")

    local found=false
    local elapsed=0

    while [ $elapsed -lt $TIMEOUT ]; do
        if grep -q "$search_pattern" "$temp_file" 2>/dev/null; then
            found=true
            break
        fi

        if ! kill -0 "$cmd_pid" 2>/dev/null; then
            break
        fi

        sleep 0.5
        elapsed=$(( $(date +%s) - start_time ))
    done

    local end_time=$(date +%s)
    local duration=$(( end_time - start_time ))

    if [ "$found" = true ]; then
        echo -e "${GREEN}✓ $success_message${NC} (${duration}s)\n"
        RESULTS+=("${GREEN}✓${NC} $name")
        TIMINGS+=("$name: ${duration}s")
        kill "$cmd_pid" 2>/dev/null
        wait "$cmd_pid" 2>/dev/null
        rm -f "$temp_file"
        return 0
    else
        echo -e "${RED}✗ $name - TIMEOUT (exceeded ${TIMEOUT}s)${NC}"
        echo -e "${RED}Last output:${NC}"
        tail -5 "$temp_file"
        echo ""
        RESULTS+=("${RED}✗${NC} $name (timeout)")
        TIMINGS+=("$name: timeout")
        kill "$cmd_pid" 2>/dev/null
        wait "$cmd_pid" 2>/dev/null
        rm -f "$temp_file"
        return 1
    fi
}

# Print startup header
print_header

SUCCESS_COUNT=0
TOTAL_STEPS=5

# Step 1: MicroXRCEAgent (critical)
if run_with_monitor \
    "MicroXRCEAgent" \
    "MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600" \
    "participant created" \
    "MicroXRCEAgent Running" \
    "MicroXRCEAgent serial --dev /dev/ttyAMA0"; then
    ((SUCCESS_COUNT++))
else
    echo -e "${RED}⚠ Stopping sequence - MicroXRCEAgent is critical${NC}\n"
    exit 1
fi

# Step 2: motion_capture_tracking_node
if run_with_monitor \
    "motion_capture_tracking_node" \
    "ros2 run motion_capture_tracking motion_capture_tracking_node --ros-args -p type:=optitrack -p hostname:=192.168.74.9" \
    "Joined multicast group" \
    "motion_capture_tracking_node Running" \
    "motion_capture_tracking_node"; then
    ((SUCCESS_COUNT++))
fi

# Step 3: mocap_px4_bridge
if run_with_monitor \
    "mocap_px4_bridge" \
    "ros2 run mocap_px4_bridge mocap_px4_bridge --ros-args -p mocap_topic:=/poses -p drone_name:=Puck -p px4_topic:=/fmu/in/vehicle_visual_odometry" \
    "Sent to PX4 as:" \
    "mocap_px4_bridge Running" \
    "mocap_px4_bridge"; then
    ((SUCCESS_COUNT++))
fi

# Step 4: Topic Echo Verification
if run_with_monitor \
    "Topic Echo (/fmu/in/vehicle_visual_odometry)" \
    "timeout 5 ros2 topic echo /fmu/in/vehicle_visual_odometry | head -10" \
    "pose:" \
    "/fmu/in/vehicle_visual_odometry Published!" \
    "ros2 topic echo /fmu/in/vehicle_visual_odometry"; then
    ((SUCCESS_COUNT++))
fi

# Step 5: Foxglove Bridge
if run_with_monitor \
    "Foxglove Bridge" \
    "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0" \
    "publishing connection graph" \
    "Foxglove connection running!" \
    "foxglove_bridge_launch.xml"; then
    ((SUCCESS_COUNT++))
fi

# Summary
echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}Startup Summary${NC}"
echo -e "${BLUE}========================================${NC}\n"

for result in "${RESULTS[@]}"; do
    echo -e "$result"
done

echo -e "\n${YELLOW}Timing Details:${NC}"
for timing in "${TIMINGS[@]}"; do
    echo -e "  $timing"
done

echo -e "\n${BLUE}Success Rate: $SUCCESS_COUNT/$TOTAL_STEPS${NC}\n"

if [ $SUCCESS_COUNT -eq $TOTAL_STEPS ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ All systems online!${NC}"
    echo -e "${GREEN}========================================${NC}\n"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ Startup incomplete - $(($TOTAL_STEPS - $SUCCESS_COUNT)) component(s) failed${NC}"
    echo -e "${RED}========================================${NC}\n"
    exit 1
fi