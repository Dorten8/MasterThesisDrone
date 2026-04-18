#!/bin/bash

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source /home/ws/install/setup.bash" ~/.bashrc; then
  echo "[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash" >> ~/.bashrc
fi

if ! grep -q "export ROS_DOMAIN_ID=0" ~/.bashrc; then
  echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
fi

if ! grep -q "export ROS_LOCALHOST_ONLY=0" ~/.bashrc; then
  echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
fi

if ! grep -q "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ~/.bashrc; then
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
fi

if ! grep -q "export SSOT_CONFIG_PATH" ~/.bashrc; then
  echo "export SSOT_CONFIG_PATH=/home/ws/config" >> ~/.bashrc
fi

source ~/.bashrc
echo "✅ ROS 2 environment loaded! (.devcontainer/post-start.sh executed)"
