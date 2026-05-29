#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_TIMEOUT="${ROS_TIMEOUT:-5s}"

SERVICES=(
  "ros2-ublox|/rtk_rover_node /moving_base_node|/gnss/solution"
  "ros2-ntrip|/ntrip_client_node|/ntrip_rtcm"
  "ros2-mavlink|/mavlink_bridge_node|/mav/modes"
  "ros2-look-ahead|/look_ahead_following|/auto_log /rc_pwm /cmd_vel"
  "ros2-maestro|/maestro_driver_node|"
)

ROS_READY=0
if [ -f "${WS_DIR}/deploy/env.sh" ] && [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WS_DIR}/deploy/env.sh" >/dev/null 2>&1 && ROS_READY=1
  set -u
fi

NODE_LIST=""
TOPIC_LIST=""
if [ "$ROS_READY" -eq 1 ]; then
  NODE_LIST="$(timeout "$ROS_TIMEOUT" ros2 node list 2>/dev/null || true)"
  TOPIC_LIST="$(timeout "$ROS_TIMEOUT" ros2 topic list 2>/dev/null || true)"
fi

contains_line() {
  local list="$1"
  local expected="$2"
  printf '%s\n' "$list" | grep -Fxq "$expected"
}

check_items() {
  local list="$1"
  local items="$2"

  if [ -z "$items" ]; then
    echo "N/A"
    return
  fi

  local item
  for item in $items; do
    if ! contains_line "$list" "$item"; then
      echo "MISS"
      return
    fi
  done

  echo "OK"
}

printf '%-16s %-8s %-8s %-8s %-8s %s\n' \
  "SERVICE" "SYSTEMD" "NODE" "TOPIC" "RESTARTS" "RESULT"

for entry in "${SERVICES[@]}"; do
  IFS='|' read -r service nodes topics <<< "$entry"
  unit="${service}.service"

  if systemctl is-active --quiet "$unit" 2>/dev/null; then
    systemd_status="OK"
  elif systemctl is-failed --quiet "$unit" 2>/dev/null; then
    systemd_status="FAIL"
  else
    systemd_status="WARN"
  fi

  restarts="$(systemctl show "$unit" --property=NRestarts --value 2>/dev/null || true)"
  if [ -z "$restarts" ]; then
    restarts="-"
  fi

  if [ "$ROS_READY" -eq 1 ]; then
    node_status="$(check_items "$NODE_LIST" "$nodes")"
    topic_status="$(check_items "$TOPIC_LIST" "$topics")"
  else
    node_status="SKIP"
    topic_status="SKIP"
  fi

  result="OK"
  if [ "$systemd_status" = "FAIL" ] || [ "$node_status" = "MISS" ]; then
    result="FAIL"
  elif [ "$systemd_status" = "WARN" ] || [ "$topic_status" = "MISS" ] || [ "$node_status" = "SKIP" ]; then
    result="WARN"
  fi

  printf '%-16s %-8s %-8s %-8s %-8s %s\n' \
    "$service" "$systemd_status" "$node_status" "$topic_status" "$restarts" "$result"
done

if [ "$ROS_READY" -ne 1 ]; then
  echo
  echo "WARN: ROS environment was not loaded. Build the workspace first, then retry."
fi
