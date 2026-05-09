#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Drone System Shutdown${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Function to gracefully kill by pattern
graceful_kill() {
    local name="$1"
    local pattern="$2"

    if pgrep -f "$pattern" >/dev/null; then
        echo -e "${YELLOW}Shutting down $name (SIGINT)...${NC}"
        pkill -INT -f "$pattern"
        
        # Wait up to 3 seconds for it to exit
        local elapsed=0
        while pgrep -f "$pattern" >/dev/null && [ $elapsed -lt 6 ]; do
            sleep 0.5
            elapsed=$((elapsed + 1))
        done
        
        if pgrep -f "$pattern" >/dev/null; then
            echo -e "${RED}$name is stubborn. Terminating (SIGTERM)...${NC}"
            pkill -TERM -f "$pattern"
            sleep 1
        else
            echo -e "${GREEN}✓ $name stopped gracefully.${NC}"
        fi
    else
        echo -e "${GREEN}✓ $name is already stopped.${NC}"
    fi
}

graceful_kill "mocap_px4_bridge" "mocap_px4_bridge"
graceful_kill "Foxglove Bridge" "foxglove_bridge_launch.xml"
graceful_kill "motion_capture_tracking_node" "motion_capture_tracking_node"

echo -e "\n${YELLOW}Resetting ROS 2 daemon cache to ensure a clean slate...${NC}"
source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Shutdown Complete!${NC}"
echo -e "${GREEN}========================================${NC}\n"
