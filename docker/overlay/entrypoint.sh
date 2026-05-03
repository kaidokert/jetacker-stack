#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source overlay workspace (patched robot_localization)
source /overlay_ws/install/setup.bash

# Execute the main command
exec "$@"
