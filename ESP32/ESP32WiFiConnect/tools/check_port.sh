#!/bin/bash
# RocBot Port Diagnostic Script
# Usage: ./check_port.sh [port]

PORT=${1:-/dev/ttyUSB0}

echo "========================================="
echo " RocBot Port Diagnostic: $PORT"
echo "========================================="

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "[✗] Port $PORT does not exist."
    echo "    Check your USB cable and connection."
    exit 1
fi

echo "[✓] Port exists."

# Check permissions
if [ ! -w "$PORT" ]; then
    echo "[✗] Permission denied. Run: sudo usermod -a -G dialout $USER"
    echo "    (Log out and log back in to apply)"
else
    echo "[✓] Permissions OK."
fi

# Check for processes holding the port
echo ""
echo "Processes using $PORT:"
USERS=$(sudo fuser "$PORT" 2>/dev/null)
if [ -z "$USERS" ]; then
    echo "    None. Port is free."
else
    echo "    $USERS"
    echo "    Kill them with: sudo fuser -k $PORT"
fi

# Check for Docker containers
echo ""
echo "Docker containers potentially using serial:"
docker ps --filter "volume=/dev" --format "{{.Names}} ({{.Image}})" 2>/dev/null || echo "    None found."

echo ""
echo "========================================="
echo " Next Steps:"
echo " 1. Kill processes: sudo fuser -k $PORT"
echo " 2. Stop Docker: docker stop \$(docker ps -q)"
echo " 3. Flash: pio run -e microros -t upload"
echo "========================================="
