#!/bin/bash
set -e
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws_env.sh"

cd "$WS_ROOT/src/main/scripts"
python3 driving_mode_ui.py
