#!/bin/bash
# RocBot Flash & Agent Launcher
# Usage: ./flash_and_agent.sh [environment]
#   environment: microros (default), hello_microros, debug

set -e

ENV=${1:-microros}
PORT=${2:-/dev/ttyUSB0}
AGENT_NAME="rocbot_microros_agent"

echo "========================================="
echo " RocBot Flash & Agent Launcher"
echo " Environment: $ENV | Port: $PORT"
echo "========================================="

# 1. Stop any running agent
echo "[1/4] Stopping any running agent..."
docker stop "$AGENT_NAME" 2>/dev/null || true
sleep 1

# 2. Flash firmware
echo "[2/4] Flashing firmware ($ENV)..."
platformio run -e "$ENV" -t upload

# 3. Wait for ESP32 to boot
echo "[3/4] Waiting for ESP32 to boot..."
sleep 2

# 4. Start agent
echo "[4/4] Starting micro-ROS agent..."
echo ""
echo "Agent is running. Press Ctrl+C to stop."
echo "========================================="

docker run -it --rm --name "$AGENT_NAME" \
  -v /dev:/dev \
  --privileged \
  --net=host \
  microros/micro-ros-agent:humble \
  serial --dev "$PORT" -b 115200 -v6

echo ""
echo "Agent stopped."
