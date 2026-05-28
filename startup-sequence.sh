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

# Cleanup function (if the script itself is killed)
cleanup() {
    for pid in "${PIDS[@]}"; do
        # SIGINT (Ctrl+C equivalent) for graceful shutdown
        kill -2 "$pid" 2>/dev/null
    done
}
trap cleanup EXIT

# Kill existing processes by pattern gracefully
kill_if_running() {
    local name="$1"
    local pattern="$2"

    if pgrep -f "$pattern" >/dev/null; then
        echo -e "${YELLOW}Found existing $name. Gently shutting down (SIGINT)...${NC}"
        pkill -INT -f "$pattern"
        sleep 2
        if pgrep -f "$pattern" >/dev/null; then
            echo -e "${YELLOW}Process stubborn. Sending SIGTERM...${NC}"
            pkill -TERM -f "$pattern"
            sleep 1
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
        # We don't kill it here! We let it run in the background.
        rm -f "$temp_file"
        return 0
    else
        echo -e "${RED}✗ $name - TIMEOUT (exceeded ${TIMEOUT}s)${NC}"
        echo -e "${RED}Last output:${NC}"
        tail -5 "$temp_file"
        echo ""
        RESULTS+=("${RED}✗${NC} $name (timeout)")
        TIMINGS+=("$name: timeout")
        kill -INT "$cmd_pid" 2>/dev/null
        wait "$cmd_pid" 2>/dev/null
        rm -f "$temp_file"
        return 1
    fi
}

# Print startup header
print_header

# Clean the ROS2 daemon to prevent stale XML-RPC faults before starting
echo -e "${YELLOW}Resetting ROS 2 daemon to clear stale caches...${NC}"
ros2 daemon stop
ros2 daemon start

SUCCESS_COUNT=0
TOTAL_STEPS=5

# Step 0: Time Synchronization
# We do this first so the Pi has the correct wall-clock date before
# the Agent starts and pushes it to the PX4.
echo -e "${YELLOW}[$(date +%H:%M:%S)]${NC} Starting: ${BLUE}Time Synchronization${NC}"
# Read Mocap server IP from drone_config.json for NTP fallback
MOCAP_NTP=$(python3 -c "import json; print(json.load(open('/home/ws/config/drone_config.json'))['optitrack_server_ip'])" 2>/dev/null)
# Try Internet first, then Mocap PC as fallback
if sudo ntpdate -u pool.ntp.org >/dev/null 2>&1 || sudo ntpdate -u "$MOCAP_NTP" >/dev/null 2>&1; then
    echo -e "${GREEN}✓ System time synchronized: $(date)${NC}"
    ((SUCCESS_COUNT++))
    RESULTS+=("${GREEN}✓${NC} Time Synchronization")
    TIMINGS+=("Time Sync: 2s")
else
    echo -e "${RED}✗ Time Sync failed (No internet or Mocap Server NTP)${NC}"
    RESULTS+=("${RED}✗${NC} Time Synchronization")
    TIMINGS+=("Time Sync: timeout")
fi

# Step 1: MicroXRCEAgent (critical)
# We must never kill this if it's running, because the PX4 client won't recover.
if pgrep -f "MicroXRCEAgent serial" >/dev/null; then
    echo -e "${GREEN}✓ MicroXRCEAgent is already running. Leaving it alone!${NC}"
    ((SUCCESS_COUNT++))
    RESULTS+=("${GREEN}✓${NC} MicroXRCEAgent (kept alive)")
    TIMINGS+=("MicroXRCEAgent: 0s")
else
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
fi

# Step 2: motion_capture_tracking_node
# Read Mocap server IP from drone_config.json (SSoT)
MOCAP_SERVER=$(python3 -c "import json; print(json.load(open('/home/ws/config/drone_config.json'))['optitrack_server_ip'])" 2>/dev/null)
PI_IP=$(python3 -c "import json; print(json.load(open('/home/ws/config/drone_config.json'))['pi_ip'])" 2>/dev/null)
if [ -z "$MOCAP_SERVER" ]; then
    MOCAP_SERVER="192.168.74.3" # Fallback
    echo -e "${YELLOW}Could not read Mocap server IP from config. Defaulting to: $MOCAP_SERVER${NC}"
else
    echo -e "${GREEN}Loaded Mocap server IP '$MOCAP_SERVER' from drone_config.json${NC}"
fi
if [ -z "$PI_IP" ]; then
    PI_IP="192.168.74.8" # Fallback
    echo -e "${YELLOW}Could not read Pi IP from config. Defaulting to: $PI_IP${NC}"
else
    echo -e "${GREEN}Loaded Pi IP '$PI_IP' from drone_config.json${NC}"
fi

if run_with_monitor \
    "motion_capture_tracking_node" \
    "ros2 run motion_capture_tracking motion_capture_tracking_node --ros-args -p type:=optitrack -p hostname:=$MOCAP_SERVER -p interface_ip:=$PI_IP" \
    "logClouds\|Joined multicast" \
    "motion_capture_tracking_node Running" \
    "motion_capture_tracking_node"; then
    ((SUCCESS_COUNT++))
fi

# Step 3: mocap_px4_bridge
# Dynamically read the primary drone name from drone_config.json
DRONE_NAME=$(python3 -c "import json; print(next((item['name'] for item in json.load(open('/home/ws/config/drone_config.json'))['tracked_bodies'] if item['role'] == 'primary'), ''))" 2>/dev/null)
if [ -z "$DRONE_NAME" ]; then
    DRONE_NAME="Arrow" # Fallback if python parsing fails
    echo -e "${YELLOW}Could not read primary drone from config. Defaulting to: $DRONE_NAME${NC}"
else
    echo -e "${GREEN}Loaded drone name '$DRONE_NAME' from drone_config.json!${NC}"
fi

# Updated to wait for 'px4_topic:' instead of 'Sent to PX4 as:' so it succeeds even if no MoCap data is flowing yet
if run_with_monitor \
    "mocap_px4_bridge" \
    "ros2 run mocap_px4_bridge mocap_px4_bridge --ros-args -p mocap_topic:=/poses -p drone_name:=$DRONE_NAME -p px4_topic:=/fmu/in/vehicle_visual_odometry" \
    "px4_topic" \
    "mocap_px4_bridge Running" \
    "mocap_px4_bridge"; then
    ((SUCCESS_COUNT++))
fi

# Step 3b: Diagnostic - Check MicroXRCEAgent connection and MoCap data
# MicroXRCEAgent health: /fmu/out/* topics should exist
# MoCap health: /poses topic should have data
echo -e "${YELLOW}[$(date +%H:%M:%S)]${NC} Checking: ${BLUE}System Health${NC}"

# Wait for topics to register
sleep 2

FMU_TOPICS=$(ros2 topic list 2>/dev/null | grep "^/fmu/out/" | wc -l)
POSES_TOPIC=$(ros2 topic list 2>/dev/null | grep -c "^/poses$")

echo ""
if [ "$FMU_TOPICS" -gt 0 ]; then
    echo -e "${GREEN}✓ MicroXRCEAgent is connected to FC (${FMU_TOPICS} /fmu/out/* topics)${NC}"
    RESULTS+=("${GREEN}✓${NC} MicroXRCEAgent ↔ FC Connection")
else
    echo -e "${RED}✗ MicroXRCEAgent NOT connected to FC (no /fmu/out/* topics)${NC}"
    echo -e "${RED}  → Power cycle the drone and flight controller${NC}"
    RESULTS+=("${RED}✗${NC} MicroXRCEAgent ↔ FC Connection (stale)")
fi

if [ "$POSES_TOPIC" -eq 1 ]; then
    if timeout 5s ros2 topic echo --qos-reliability best_effort -n 1 /poses >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Motion capture tracking is publishing fresh /poses messages${NC}"
        RESULTS+=("${GREEN}✓${NC} Motion Capture System")
    else
        echo -e "${RED}✗ /poses exists but no fresh mocap message arrived within 5s${NC}"
        echo -e "${RED}  → If Motive was restarted, restart motion_capture_tracking_node too${NC}"
        RESULTS+=("${RED}✗${NC} Motion Capture System (stale)")
    fi
else
    echo -e "${RED}✗ Motion capture tracking not working (no /poses topic)${NC}"
    RESULTS+=("${RED}✗${NC} Motion Capture System")
fi
echo ""

# Step 4: Foxglove Bridge
# Updated to wait for 'Advertising new channel' or similar
if run_with_monitor \
    "Foxglove Bridge" \
    "ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0" \
    "Advertising new channel\|Listening on\|foxglove_bridge" \
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
    # To keep background processes alive when script ends, we clear the trap
    trap - EXIT
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}✗ Startup incomplete - $(($TOTAL_STEPS - $SUCCESS_COUNT)) component(s) failed${NC}"
    echo -e "${RED}========================================${NC}\n"
    exit 1
fi