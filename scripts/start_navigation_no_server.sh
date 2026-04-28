#!/usr/bin/env bash
# start_navigation_no_server.sh — bring up the GO2W navigation stack without
# the external WebSocket server executor.
#
# Responsibilities:
#   1. xt16_driver
#   2. ros2 launch go2w_real slam_rf2o.launch.py (slam_mode=localization, no RViz)
#   3. wait for Nav2 bt_navigator
#
# Navigation goals are dispatched separately by the operator_console
# launch_manager (which invokes navigate_to_waypoint.py per POI selection).
#
# Accepts the same preset/map/network overrides as start_navigation_headless.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="/home/unitree/ros_ws"
UNITREE_SETUP="/home/unitree/unitree_ros2/setup.sh"
XT16_BIN="${GO2W_XT16_BIN:-${SCRIPT_DIR}/xt16_driver_timefix.sh}"
XT16_PROCESS_MATCHER="${GO2W_XT16_PROCESS_MATCHER:-${XT16_BIN}|/unitree/module/unitree_slam/bin/xt16_driver_go2w_timefix|/unitree/module/unitree_slam/bin/xt16_driver}"
DEFAULT_PRESET="zt_0"
DEFAULT_NETWORK_INTERFACE="eth0"
NAV2_WAIT_TIMEOUT_SEC=120

safe_source() {
  local source_file="$1"
  if [[ ! -f "${source_file}" ]]; then
    echo "[start_nav_no_server] source file not found: ${source_file}" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${source_file}"
  set -u
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_sec="${2:-5}"
  local deadline=$((SECONDS + timeout_sec))
  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done
  return 0
}

stop_pid_if_running() {
  local pid="$1"
  local label="${2:-process}"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  echo "[start_nav_no_server] stopping ${label} (pid=${pid})"
  kill "${pid}" 2>/dev/null || true
  if ! wait_for_pid_exit "${pid}" 5; then
    echo "[start_nav_no_server] ${label} did not exit after SIGTERM, sending SIGKILL (pid=${pid})" >&2
    kill -9 "${pid}" 2>/dev/null || true
    wait_for_pid_exit "${pid}" 2 || true
  fi
}

print_help() {
  cat <<'EOF'
Usage: start_navigation_no_server.sh [zt_0|test_1] [options]

Starts the GO2W navigation stack headlessly, WITHOUT launching
navigation_executor.py (no outbound WebSocket server connection).

Steps:
  1. xt16_driver
  2. ros2 launch go2w_real slam_rf2o.launch.py
       slam_mode:=localization use_odom_fusion:=true use_rviz:=false ...
  3. wait for Nav2 bt_navigator

Map presets:
  zt_0    Default. /home/unitree/ros_ws/src/go2w_real/map/zt_0
  test_1  /home/unitree/ros_ws/src/go2w_real/map/14_1

Options:
  --preset NAME            Preset name: zt_0 / test_1
  --slam-map-file PREFIX   Serialized slam_toolbox map prefix
  --network-interface IF   Interface passed to slam_rf2o.launch.py
  -h, --help               Show this help

Environment overrides:
  GO2W_NAV_PRESET
  GO2W_SLAM_MAP_FILE
  GO2W_NETWORK_INTERFACE
EOF
}

apply_preset_defaults() {
  local preset_name="$1"
  local preset_slam_map_prefix=""
  case "${preset_name}" in
    zt_0)
      preset_slam_map_prefix="${WS_DIR}/src/go2w_real/map/zt_0"
      ;;
    test_1)
      preset_slam_map_prefix="${WS_DIR}/src/go2w_real/map/14_1"
      ;;
    *)
      echo "[start_nav_no_server] unknown map preset: ${preset_name}" >&2
      print_help >&2
      exit 1
      ;;
  esac
  if [[ -z "${SLAM_MAP_PREFIX}" ]]; then
    SLAM_MAP_PREFIX="${preset_slam_map_prefix}"
  fi
}

PRESET="${GO2W_NAV_PRESET:-$DEFAULT_PRESET}"
SLAM_MAP_PREFIX="${GO2W_SLAM_MAP_FILE:-}"
NETWORK_INTERFACE="${GO2W_NETWORK_INTERFACE:-$DEFAULT_NETWORK_INTERFACE}"
POSITIONAL_PRESET=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      print_help
      exit 0
      ;;
    --preset)
      PRESET="${2:-}"
      shift 2
      ;;
    --preset=*)
      PRESET="${1#*=}"
      shift
      ;;
    --slam-map-file)
      SLAM_MAP_PREFIX="${2:-}"
      shift 2
      ;;
    --slam-map-file=*)
      SLAM_MAP_PREFIX="${1#*=}"
      shift
      ;;
    --network-interface)
      NETWORK_INTERFACE="${2:-}"
      shift 2
      ;;
    --network-interface=*)
      NETWORK_INTERFACE="${1#*=}"
      shift
      ;;
    -*)
      echo "[start_nav_no_server] unknown option: $1" >&2
      print_help >&2
      exit 1
      ;;
    *)
      if [[ -n "${POSITIONAL_PRESET}" ]]; then
        echo "[start_nav_no_server] unexpected extra positional argument: $1" >&2
        print_help >&2
        exit 1
      fi
      POSITIONAL_PRESET="$1"
      shift
      ;;
  esac
done

if [[ -n "${POSITIONAL_PRESET}" ]]; then
  PRESET="${POSITIONAL_PRESET}"
fi

apply_preset_defaults "${PRESET}"

if [[ -z "${NETWORK_INTERFACE}" || -z "${SLAM_MAP_PREFIX}" ]]; then
  echo "[start_nav_no_server] missing required runtime parameter after preset/override resolution" >&2
  print_help >&2
  exit 1
fi

for required_file in \
  "${SLAM_MAP_PREFIX}.data" \
  "${SLAM_MAP_PREFIX}.posegraph" \
  "${XT16_BIN}" \
  "${WS_DIR}/install/setup.bash"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "[start_nav_no_server] missing required file: ${required_file}" >&2
    exit 1
  fi
done

safe_source /opt/ros/foxy/setup.bash
if [[ -f "${UNITREE_SETUP}" ]]; then
  safe_source "${UNITREE_SETUP}"
fi
safe_source "${WS_DIR}/install/setup.bash"

LOG_DIR="${WS_DIR}/log/start_navigation_no_server_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

XT16_PID=""
LAUNCH_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT HUP INT QUIT TERM
  stop_pid_if_running "${LAUNCH_PID}" "slam_rf2o.launch.py"
  stop_pid_if_running "${XT16_PID}" "xt16_driver"
  exit "${exit_code}"
}

trap cleanup EXIT HUP INT QUIT TERM

echo "[start_nav_no_server] preset: ${PRESET}"
echo "[start_nav_no_server] slam map prefix: ${SLAM_MAP_PREFIX}"
echo "[start_nav_no_server] network interface: ${NETWORK_INTERFACE}"
echo "[start_nav_no_server] logs: ${LOG_DIR}"

if pgrep -f "${XT16_PROCESS_MATCHER}" >/dev/null 2>&1; then
  echo "[start_nav_no_server] xt16_driver already running, reusing existing process"
else
  "${XT16_BIN}" >"${LOG_DIR}/xt16_driver.log" 2>&1 &
  XT16_PID=$!
  echo "[start_nav_no_server] started xt16_driver (pid=${XT16_PID})"
  sleep 2
  if ! kill -0 "${XT16_PID}" 2>/dev/null; then
    echo "[start_nav_no_server] xt16_driver exited early, check ${LOG_DIR}/xt16_driver.log" >&2
    exit 1
  fi
fi

LAUNCH_CMD=(
  ros2 launch go2w_real slam_rf2o.launch.py
  slam_mode:=localization
  use_odom_fusion:=true
  use_rviz:=false
  network_interface:="${NETWORK_INTERFACE}"
  slam_map_file:="${SLAM_MAP_PREFIX}"
)
"${LAUNCH_CMD[@]}" >"${LOG_DIR}/slam_rf2o.log" 2>&1 &
LAUNCH_PID=$!
echo "[start_nav_no_server] started slam_rf2o.launch.py (pid=${LAUNCH_PID})"

echo "[start_nav_no_server] waiting for Nav2 bt_navigator node..."
deadline=$((SECONDS + NAV2_WAIT_TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[start_nav_no_server] slam_rf2o.launch.py exited early, check ${LOG_DIR}/slam_rf2o.log" >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
  echo "[start_nav_no_server] Nav2 did not become ready within ${NAV2_WAIT_TIMEOUT_SEC}s" >&2
  exit 1
fi

echo "[start_nav_no_server] stack is ready. Press Ctrl+C to stop all managed processes."
echo "[start_nav_no_server] dispatch POI goals via operator_console (launch_manager REST + navigate_to_waypoint.py)."

wait -n "${LAUNCH_PID}" ${XT16_PID:+"${XT16_PID}"}
echo "[start_nav_no_server] a managed process exited, stopping the stack" >&2
exit 1
