#!/bin/bash
set -e

# Get resolution from environment or use default
RESOLUTION=${VNC_RESOLUTION:-1920x1080}

echo "Starting X11 Server Environment with TigerVNC..."
echo "Resolution: ${RESOLUTION}"

# Clean up any existing locks (important for restarts)
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99

# Start TigerVNC Xvnc server (X server with built-in VNC)
# Xvnc has better OpenGL/expose event handling than Xvfb + x11vnc
echo "Starting Xvnc (TigerVNC) on :99..."
Xvnc :99 \
  -geometry ${RESOLUTION} \
  -depth 24 \
  -rfbport 5900 \
  -SecurityTypes None \
  -AlwaysShared \
  -AcceptSetDesktopSize \
  -SendCutText \
  -AcceptCutText \
  &

export DISPLAY=:99

# Wait for X server to start
sleep 2

# No window manager - testing if openbox is the problem
echo "Running without window manager..."

# Start noVNC websockify proxy
echo "Starting noVNC..."
/opt/noVNC/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &

echo "-------------------------------------------------------"
echo "X11 Server Ready"
echo "DISPLAY=:99"
echo "noVNC web interface available at http://localhost:6080"
echo "-------------------------------------------------------"

# Keep container alive
wait
