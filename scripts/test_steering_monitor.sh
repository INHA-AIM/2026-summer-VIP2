#!/bin/bash
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

mkdir -p "$WS_ROOT/logs"
echo "Steering monitor logs -> $WS_ROOT/logs/steering_continuity.csv"
echo "Run lattice planner; SteeringMonitor writes CSV automatically."
echo "  source set_mode.sh va"
echo "  ./scripts/lattice_planner.sh"
