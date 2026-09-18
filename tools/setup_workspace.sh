#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: tools/setup_workspace.sh [--no-build] [--skip-apt]

Set up this ROS2 workspace:
  - install required apt packages
  - import source packages with vcstool, including git submodules
  - create .venv with access to ROS2/system Python packages
  - install pip-managed Python packages
  - build the workspace with colcon

Options:
  --no-build   Skip colcon build
  --skip-apt   Skip apt update/install
  -h, --help   Show this help

Environment:
  ROS_DISTRO  ROS distribution name (default: jazzy)
USAGE
}

RUN_BUILD=1
RUN_APT=1

for arg in "$@"; do
  case "$arg" in
    --no-build)
      RUN_BUILD=0
      ;;
    --skip-apt)
      RUN_APT=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
VENV_DIR="${WS_DIR}/.venv"

cd "$WS_DIR"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  echo "Install ROS2 ${ROS_DISTRO} before running this script." >&2
  exit 1
fi

if [ ! -f "${WS_DIR}/minix_ws.repos" ]; then
  echo "minix_ws.repos not found in ${WS_DIR}" >&2
  exit 1
fi

if [ "$RUN_APT" -eq 1 ]; then
  sudo apt update
  sudo apt install -y \
    python3-venv \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-serial \
    "ros-${ROS_DISTRO}-tf-transformations"
fi

mkdir -p "${WS_DIR}/src"
vcs import --recursive "${WS_DIR}/src" < "${WS_DIR}/minix_ws.repos"

if [ -d "${WS_DIR}/src/mavlink_ros2_bridge/.git" ]; then
  git -C "${WS_DIR}/src/mavlink_ros2_bridge" submodule update --init --recursive
fi

# ROS-generated setup scripts may read optional variables without first
# checking whether they are defined, so temporarily disable nounset.
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u

python3 -m venv --system-site-packages "$VENV_DIR"
source "${VENV_DIR}/bin/activate"

python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install pyproj

if [ "$RUN_BUILD" -eq 1 ]; then
  colcon build --symlink-install --packages-select bme_common_msgs
  set +u
  # shellcheck source=/dev/null
  source "${WS_DIR}/install/setup.bash"
  set -u
  colcon build --symlink-install
fi

echo "Workspace setup complete: ${WS_DIR}"
