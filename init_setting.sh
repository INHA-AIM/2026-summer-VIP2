#!/bin/bash
# 의존성 설치. Ubuntu 20.04 + ROS Noetic 또는 vip2 Docker 컨테이너 안에서 실행.
set -e
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_ROOT"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ERROR: ROS Noetic 이 없습니다."
  echo "  - Ubuntu 22.04/24.04/26.04 호스트에는 ros-noetic-* apt 패키지가 없습니다."
  echo "  - Docker를 사용하세요: docs/setup.md 의 'Docker (권장)' 절"
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

if command -v sudo >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=
fi

$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-rosbridge-server \
  ros-noetic-pcl-ros \
  python3-pip

pip3 install --upgrade "pip<25" "wheel" "setuptools<70"
pip3 install -r "$WS_ROOT/requirements-python.txt"

if [ ! -e src/CMakeLists.txt ] && [ -f /opt/ros/noetic/share/catkin/cmake/toplevel.cmake ]; then
  ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
fi

echo "Init done. Next: ./build.sh"
