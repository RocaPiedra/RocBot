#!/bin/bash
# RocBot micro-ROS Setup Script
# Installs dependencies and prepares the micro-ROS agent environment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================="
echo " RocBot micro-ROS Setup"
echo "========================================="

# Check for Docker
if command -v docker &> /dev/null; then
    echo "[✓] Docker found."
    echo "Pulling micro-ROS Agent image (humble)..."
    docker pull microros/micro-ros-agent:humble
    echo "[✓] Image pulled successfully."
    echo ""
    echo "To start the agent:"
    echo "  docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6"
else
    echo "[✗] Docker not found."
    echo "Please install Docker or build the agent from source:"
    echo "  mkdir -p ~/microros_ws/src && cd ~/microros_ws"
    echo "  git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent"
    echo "  rosdep update && rosdep install --from-paths src --ignore-src -y"
    echo "  colcon build"
    exit 1
fi

# Check for ESP-IDF / PlatformIO
if command -v pio &> /dev/null; then
    echo "[✓] PlatformIO found."
    echo "To flash firmware: cd $PROJECT_DIR && pio run -e microros -t upload"
else
    echo "[!] PlatformIO not found. Install it to flash the ESP32."
fi

echo ""
echo "Setup complete!"
echo "Run 'python app.py' to start the RocBot Tuner."
