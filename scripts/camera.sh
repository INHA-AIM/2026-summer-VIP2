#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

roslaunch Camera camera.launch
