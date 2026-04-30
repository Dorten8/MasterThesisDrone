#!/bin/bash
set -e

echo "🤖 Initializing ROS 2 workspace..."

bash -lc '
  source /opt/ros/humble/setup.bash
  cd /home/ws
  rosdep update
  rosdep install --from-paths /home/ws/src --ignore-src -r -y
  sudo chown -R $(whoami) /home/ws/
  colcon build --symlink-install
  pip install -e /home/ws/src/object-tracking-
  /home/ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh
  cd /home/ws/src/PX4-Autopilot/
  make px4_sitl/
  source /home/ws/install/setup.bash
'



echo "✅ Workspace ready! (.devcontainer/post-create.sh executed)"