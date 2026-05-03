# Debug container with apt search tools
ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-base:latest

ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-scenario-execution-interfaces \
    ros-${ROS_DISTRO}-pal-statistics-msgs \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-foxglove-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-foxglove-bridge-dbgsym \
    ros-${ROS_DISTRO}-foxglove-msgs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
