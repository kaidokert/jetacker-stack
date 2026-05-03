# Debug container with apt search tools
ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-base:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install debug tools and keep apt cache for searching
RUN apt-get update && apt-get install -y \
    apt-file \
    file \
    nano \
    less \
    dpkg-dev \
    apt-rdepends \
    && apt-file update

# Build dependencies for robot_localization (for building overlay workspace)
RUN apt-get install -y \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-message-filters \
    libgeographiclib-dev \
    ccache \
    libboost-dev

# Don't clean apt lists - we want to keep them for searching
# RUN apt-get update is already done above

# Install Python tools for bag analysis
RUN apt-get install -y python3-pip \
    && pip3 install --break-system-packages mcap mcap-ros2-support

# Add keyboard teleop for manual robot control
RUN apt-get install -y \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

CMD ["/bin/bash"]
