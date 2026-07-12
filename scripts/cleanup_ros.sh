#!/bin/bash
echo "=========================================="
echo "  ROS Environment Cleanup & Reset"
echo "=========================================="

echo "[1/3] Killing all ROS processes..."
pkill -9 -f "rosmaster" 2>/dev/null || true
pkill -9 -f "roscore" 2>/dev/null || true
pkill -9 -f "rosbridge" 2>/dev/null || true
pkill -9 -f "roslaunch" 2>/dev/null || true
pkill -9 -f "rospy" 2>/dev/null || true
pkill -9 -f "lattice_test_node" 2>/dev/null || true
pkill -9 -f "ros_ctrl_to_grpc" 2>/dev/null || true
sleep 2

echo "[2/3] Cleaning ROS environment variables..."
unset ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_PACKAGE_PATH

echo "[3/3] Cleaning temporary ROS files..."
rm -rf /tmp/rosmaster* 2>/dev/null
rm -rf /tmp/ros_* 2>/dev/null
rm -rf /tmp/setup.sh* 2>/dev/null
rm -rf ~/.ros/log/* 2>/dev/null
mkdir -p ~/.ros/log

echo "✓ Cleanup completed"
echo "Next: ./scripts/bridge.sh  (or ./scripts/start_system.sh)"
