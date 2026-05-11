#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 bag convert -i /home/dorten/pi_drone_sshfs/flight_20260511_082944/ -o /home/dorten/pi_drone_sshfs/flight_20260511_082944/mcap_convert_options.yaml
