#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

# lg580p_gnss_driver requires pyproj from venv
source ~/minix_ws/.venv/bin/activate
export PYTHONPATH=~/minix_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}

exec ros2 launch lg580p_gnss_driver lg580p_gnss.launch.py
