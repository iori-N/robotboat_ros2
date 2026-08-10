#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

# look_ahead_control requires pyproj from venv
source ~/minix_ws/.venv/bin/activate
export PYTHONPATH=~/minix_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}

exec ros2 launch look_ahead_control look_ahead_following.launch.py
