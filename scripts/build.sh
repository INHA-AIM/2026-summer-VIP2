#!/bin/bash
# Convenience wrapper — same as repo-root ./build.sh
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../build.sh"
