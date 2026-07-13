#!/bin/bash

# RViz 시각화 — external visualization_2_rviz.rviz 사용
# Docker Desktop + WSLg: LIBGL_ALWAYS_INDIRECT=1 이면 GLXContext 실패하는 경우가 많음

echo "Starting RViz with visualization_2_rviz.rviz..."

if [ -z "$DISPLAY" ]; then
    # WSLg 기본 디스플레이
    export DISPLAY=:0
    echo "Warning: DISPLAY was unset; using DISPLAY=:0 (WSLg)"
fi

export QT_X11_NO_MITSHM=1

# 간접 GL은 Docker Desktop에서 RViz GLX를 깨뜨리는 경우가 많아 기본 OFF.
# VcXsrv 등으로 간접 렌더가 필요할 때만: LIBGL_ALWAYS_INDIRECT=1 ./scripts/rviz_visualization.sh
if [ "${LIBGL_ALWAYS_INDIRECT:-}" = "1" ]; then
    echo "LIBGL_ALWAYS_INDIRECT=1 (user override)"
else
    unset LIBGL_ALWAYS_INDIRECT
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/visualization_2_rviz.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "Error: RViz config file not found: $RVIZ_CONFIG"
    exit 1
fi

echo "DISPLAY=$DISPLAY"
echo "LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT:-<unset>}"
echo "Running RViz..."

rviz -d "$RVIZ_CONFIG"
