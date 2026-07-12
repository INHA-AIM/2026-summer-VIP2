#!/bin/bash
set -e
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_ROOT"

sudo apt-get update
sudo apt-get install -y ros-noetic-tf2-sensor-msgs ros-noetic-rosbridge-server

pip3 install ultralytics pandas matplotlib numpy

if [ ! -e src/CMakeLists.txt ] && [ -f /opt/ros/noetic/share/catkin/cmake/toplevel.cmake ]; then
  ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
fi

echo "Init done. Next: ./build.sh"
