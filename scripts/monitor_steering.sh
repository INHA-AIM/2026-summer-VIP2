#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

mkdir -p "$WS_ROOT/logs"
echo "Steering logs: $WS_ROOT/logs/steering_continuity.csv"
echo "Visualizer script is optional; CSV is written by PlanningControl."

if [ -f "$WS_ROOT/scripts/steering_visualizer.py" ]; then
  python3 "$WS_ROOT/scripts/steering_visualizer.py"
else
  echo "No steering_visualizer.py — planner already logs to logs/ when running."
fi
