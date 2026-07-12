#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

PORT=${1:-9090}

echo "Starting ROS Bridge WebSocket Server on port $PORT (WS_ROOT=$WS_ROOT)..."
echo "Press Ctrl+C to stop."
roslaunch rosbridge_server rosbridge_websocket.launch port:=$PORT
