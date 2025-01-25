#!/bin/bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /root/igvc_autonav_2025/bot_ws/install/setup.bash

# Run the desired command
exec "$@"
