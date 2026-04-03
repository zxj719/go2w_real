#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/unitree/ros_ws"
UNITREE_SETUP="/home/unitree/unitree_ros2/setup.sh"
XT16_BIN="/unitree/module/unitree_slam/bin/xt16_driver"
DEFAULT_PRESET="zt_0"
NAV2_WAIT_TIMEOUT_SEC=120

safe_source() {
  local source_file="$1"
  if [[ ! -f "${source_file}" ]]; then
    echo "[start_nav_headless] source file not found: ${source_file}" >&2
    exit 1
  fi

  # ROS setup scripts access optional vars directly, so temporarily disable
  # nounset while sourcing them and then restore strict mode.
  set +u
  # shellcheck disable=SC1090
  source "${source_file}"
  set -u
}

print_help() {
  cat <<'EOF'
Usage: start_navigation_headless.sh [zt_0|test_1]

Starts the full headless navigation stack:
  1. xt16_driver
  2. ros2 launch go2w_real slam_rf2o.launch.py use_rviz:=false
  3. ros2 run go2w_real navigation_executor.py

Map presets:
  zt_0    Default. Uses go2w_map_waypoints.yaml + /home/unitree/ros_ws/src/map/zt_0
  test_1  Uses go2w_waypoints.yaml + /home/unitree/ros_ws/src/map/test_1
EOF
}

PRESET="${1:-$DEFAULT_PRESET}"
if [[ "${PRESET}" == "-h" || "${PRESET}" == "--help" ]]; then
  print_help
  exit 0
fi

if [[ $# -gt 1 ]]; then
  echo "[start_nav_headless] too many arguments" >&2
  print_help >&2
  exit 1
fi

case "${PRESET}" in
  zt_0)
    WAYPOINT_FILE="${WS_DIR}/src/go2w_real/config/go2w_map_waypoints.yaml"
    SLAM_MAP_PREFIX="${WS_DIR}/src/map/zt_0"
    ;;
  test_1)
    WAYPOINT_FILE="${WS_DIR}/src/go2w_real/config/go2w_waypoints.yaml"
    SLAM_MAP_PREFIX="${WS_DIR}/src/map/test_1"
    ;;
  *)
    echo "[start_nav_headless] unknown map preset: ${PRESET}" >&2
    print_help >&2
    exit 1
    ;;
esac

for required_file in \
  "${WAYPOINT_FILE}" \
  "${SLAM_MAP_PREFIX}.data" \
  "${SLAM_MAP_PREFIX}.posegraph" \
  "${XT16_BIN}" \
  "${WS_DIR}/install/setup.bash"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "[start_nav_headless] missing required file: ${required_file}" >&2
    exit 1
  fi
done

safe_source /opt/ros/foxy/setup.bash
if [[ -f "${UNITREE_SETUP}" ]]; then
  safe_source "${UNITREE_SETUP}"
fi
safe_source "${WS_DIR}/install/setup.bash"

LOG_DIR="${WS_DIR}/log/start_navigation_headless_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

XT16_PID=""
LAUNCH_PID=""
EXECUTOR_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "${EXECUTOR_PID}" ]] && kill -0 "${EXECUTOR_PID}" 2>/dev/null; then
    kill "${EXECUTOR_PID}" 2>/dev/null || true
    wait "${EXECUTOR_PID}" 2>/dev/null || true
  fi

  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if [[ -n "${XT16_PID}" ]] && kill -0 "${XT16_PID}" 2>/dev/null; then
    kill "${XT16_PID}" 2>/dev/null || true
    wait "${XT16_PID}" 2>/dev/null || true
  fi

  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

echo "[start_nav_headless] preset: ${PRESET}"
echo "[start_nav_headless] waypoint file: ${WAYPOINT_FILE}"
echo "[start_nav_headless] slam map prefix: ${SLAM_MAP_PREFIX}"
echo "[start_nav_headless] logs: ${LOG_DIR}"

if pgrep -f "${XT16_BIN}" >/dev/null 2>&1; then
  echo "[start_nav_headless] xt16_driver already running, reusing existing process"
else
  "${XT16_BIN}" >"${LOG_DIR}/xt16_driver.log" 2>&1 &
  XT16_PID=$!
  echo "[start_nav_headless] started xt16_driver (pid=${XT16_PID})"
  sleep 2
  if ! kill -0 "${XT16_PID}" 2>/dev/null; then
    echo "[start_nav_headless] xt16_driver exited early, check ${LOG_DIR}/xt16_driver.log" >&2
    exit 1
  fi
fi

ros2 launch go2w_real slam_rf2o.launch.py \
  use_rviz:=false \
  slam_map_file:="${SLAM_MAP_PREFIX}" >"${LOG_DIR}/slam_rf2o.log" 2>&1 &
LAUNCH_PID=$!
echo "[start_nav_headless] started slam_rf2o.launch.py (pid=${LAUNCH_PID})"

echo "[start_nav_headless] waiting for Nav2 bt_navigator node..."
deadline=$((SECONDS + NAV2_WAIT_TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[start_nav_headless] slam_rf2o.launch.py exited early, check ${LOG_DIR}/slam_rf2o.log" >&2
    exit 1
  fi

  if ros2 node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
    break
  fi

  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
  echo "[start_nav_headless] Nav2 did not become ready within ${NAV2_WAIT_TIMEOUT_SEC}s" >&2
  exit 1
fi

ros2 run go2w_real navigation_executor.py \
  --waypoint-file "${WAYPOINT_FILE}" >"${LOG_DIR}/navigation_executor.log" 2>&1 &
EXECUTOR_PID=$!
echo "[start_nav_headless] started navigation_executor.py (pid=${EXECUTOR_PID})"
echo "[start_nav_headless] stack is ready. Press Ctrl+C to stop all managed processes."

wait -n "${LAUNCH_PID}" "${EXECUTOR_PID}" ${XT16_PID:+"${XT16_PID}"}
echo "[start_nav_headless] a managed process exited, stopping the stack" >&2
exit 1
