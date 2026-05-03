# Overlay workspace with patched robot_localization
# Fixes deadlock bug when using use_sim_time:=true
#
# The bug: robot_localization 3.8.3 calls wait_until_started() before the
# executor starts spinning, causing a deadlock when waiting for /clock messages.
#
# The fix: Remove wait_until_started() blocking call from ros_filter.cpp and
# navsat_transform.cpp. The node will receive clock messages once rclcpp::spin() starts.

ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-gazebo:latest

ENV ROS_DISTRO=${ROS_DISTRO}

# Install colcon build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy overlay workspace source
COPY workdir/overlay_ws/src /overlay_ws/src

WORKDIR /overlay_ws

# Build overlay workspace
# This will compile the patched robot_localization on top of the ROS base
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install"

# Update entrypoint to source overlay workspace
COPY docker/overlay/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
