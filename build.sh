#!/bin/bash
set -e
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_ROOT"

if [ ! -e src/CMakeLists.txt ]; then
  if [ -f /opt/ros/noetic/share/catkin/cmake/toplevel.cmake ]; then
    ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
    echo "Created src/CMakeLists.txt -> catkin toplevel.cmake"
  else
    echo "ERROR: ROS Noetic not found. Install ROS or run inside docker/ first."
    exit 1
  fi
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source devel/setup.bash
fi

catkin_make
