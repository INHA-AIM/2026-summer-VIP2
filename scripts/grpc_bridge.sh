#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

# ROS /ctrl_cmd_0 -> MORAI gRPC ControlVehicle
export PYTHONPATH="$WS_ROOT/grpc_inha_univ/src:${PYTHONPATH:-}"
python3 "$WS_ROOT/grpc_inha_univ/src/ros_ctrl_to_grpc_bridge.py" "$@"
