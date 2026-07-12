#!/bin/bash
# Resolve catkin workspace root (= repo root) and optionally source devel.
# Usage: source "$(dirname "$0")/ws_env.sh"   OR   source scripts/ws_env.sh

_WS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -d "$_WS_ENV_DIR/../src/main" ]; then
  export WS_ROOT="$(cd "$_WS_ENV_DIR/.." && pwd)"
elif [ -d "$_WS_ENV_DIR/src/main" ]; then
  export WS_ROOT="$_WS_ENV_DIR"
else
  export WS_ROOT="$(cd "$_WS_ENV_DIR/.." && pwd)"
fi

cd "$WS_ROOT" || return 1 2>/dev/null || exit 1

if [ -f "$WS_ROOT/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_ROOT/devel/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
