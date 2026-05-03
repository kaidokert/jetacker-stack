# X11 Server with VNC/noVNC
# Provides a persistent display server for Gazebo containers
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Xvfb, TigerVNC, window manager, and net-tools
RUN apt-get update && apt-get install -y \
    xvfb \
    tigervnc-standalone-server \
    tigervnc-common \
    openbox \
    git \
    python3-numpy \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install noVNC and websockify
RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC && \
    git clone https://github.com/novnc/websockify /opt/noVNC/utils/websockify && \
    ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Copy startup script
COPY docker/x11-server/start-x11.sh /start-x11.sh
RUN chmod +x /start-x11.sh

# VNC port and noVNC web port
EXPOSE 5900 6080

CMD ["/start-x11.sh"]
