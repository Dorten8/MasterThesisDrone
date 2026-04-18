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
'

echo "✅ Workspace ready! (.devcontainer/post-create.sh executed)"