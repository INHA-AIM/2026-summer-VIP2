#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

# Must run from WS_ROOT so relative paths like src/main/config/... resolve
cd "$WS_ROOT"
rosrun PlanningControl lattice_test_node
