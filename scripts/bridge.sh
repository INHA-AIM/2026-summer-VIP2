#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

PORT=${1:-9090}

# Ensure a master exists (idempotent if already running)
if ! rostopic list >/dev/null 2>&1; then
  echo "Starting roscore..."
  roscore >/tmp/roscore.log 2>&1 &
  sleep 2
fi

echo "=========================================="
echo " ROS Bridge for MORAI"
echo "=========================================="
echo " WS_ROOT = $WS_ROOT"
echo " Listen  = 0.0.0.0:${PORT}"
echo " MORAI Bridge IP/PORT = 127.0.0.1 / ${PORT}"
echo " (Docker must publish ${PORT}:${PORT})"
echo " Press Ctrl+C to stop."
echo "=========================================="

# address:=0.0.0.0 → 컨테이너 밖(Windows localhost)에서 접속 가능
roslaunch rosbridge_server rosbridge_websocket.launch \
  port:=${PORT} \
  address:=0.0.0.0
