ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-gazebo:latest

ENV ROS_DISTRO=${ROS_DISTRO}

# Install Nav2 stack
# Using navigation2 metapackage for complete stack + common plugins
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*
