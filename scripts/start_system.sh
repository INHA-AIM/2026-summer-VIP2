#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

echo "=========================================="
echo "  AIM Autonomous Vehicle System Startup"
echo "=========================================="

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$WS_ROOT"

echo -e "${YELLOW}[1/5] Cleaning ROS environment...${NC}"
"$WS_ROOT/scripts/cleanup_ros.sh" > /dev/null 2>&1 || true
sleep 1

echo -e "${YELLOW}[2/5] Starting ROS Bridge...${NC}"
"$WS_ROOT/scripts/bridge.sh" > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 5

if ps -p $BRIDGE_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ ROS Bridge started (PID: $BRIDGE_PID)${NC}"
else
    echo -e "${RED}✗ ROS Bridge failed to start${NC}"
    tail -20 /tmp/bridge.log
    exit 1
fi

echo -e "${YELLOW}[3/5] Starting Camera...${NC}"
"$WS_ROOT/scripts/camera.sh" > /tmp/camera.log 2>&1 &
CAMERA_PID=$!
sleep 3

if ps -p $CAMERA_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Camera started (PID: $CAMERA_PID)${NC}"
else
    echo -e "${RED}✗ Camera failed to start (see /tmp/camera.log)${NC}"
fi

echo -e "${YELLOW}[4/5] Starting LiDAR...${NC}"
"$WS_ROOT/scripts/lidar.sh" > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
sleep 3

if ps -p $LIDAR_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ LiDAR started (PID: $LIDAR_PID)${NC}"
else
    echo -e "${RED}✗ LiDAR failed to start (see /tmp/lidar.log)${NC}"
fi

echo -e "${YELLOW}[5/5] Done${NC}"
echo ""
echo -e "${BLUE}Next: in another terminal${NC}"
echo "  cd \"$WS_ROOT\""
echo "  source scripts/ws_env.sh"
echo "  source set_mode.sh va   # or static / fusion / original"
echo "  ./scripts/lattice_planner.sh"
echo ""
echo -e "${BLUE}Optional gRPC ctrl bridge:${NC}"
echo "  ./scripts/grpc_bridge.sh"
echo ""
echo -e "${BLUE}Stop:${NC} ./scripts/cleanup_ros.sh"
