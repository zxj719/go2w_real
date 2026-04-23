#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/unitree/ros_ws"
UNITREE_SETUP="/home/unitree/unitree_ros2/setup.sh"
XT16_BIN="/unitree/module/unitree_slam/bin/xt16_driver"
EXECUTOR_BIN="${WS_DIR}/install/go2w_real/lib/go2w_real/navigation_executor.py"
EXECUTOR_LOCK_FILE="/tmp/go2w_navigation_executor.lock"
DEFAULT_PRESET="zt_0"
DEFAULT_SERVER_URI="ws://192.168.123.186:8100/ws/navigation/executor"
DEFAULT_NETWORK_INTERFACE="eth0"
DEFAULT_SHOW_EXECUTOR_LOGS="1"
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

  echo "[start_nav_headless] stopping ${label} (pid=${pid})"
  kill "${pid}" 2>/dev/null || true

  if ! wait_for_pid_exit "${pid}" 5; then
    echo "[start_nav_headless] ${label} did not exit after SIGTERM, sending SIGKILL (pid=${pid})" >&2
    kill -9 "${pid}" 2>/dev/null || true
    wait_for_pid_exit "${pid}" 2 || true
  fi
}

cleanup_executor_lock() {
  local lock_pid=""

  if [[ ! -f "${EXECUTOR_LOCK_FILE}" ]]; then
    return 0
  fi

  lock_pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
  if [[ -z "${lock_pid}" ]] || ! kill -0 "${lock_pid}" 2>/dev/null; then
    rm -f "${EXECUTOR_LOCK_FILE}"
  fi
}

stop_existing_executors() {
  local executor_pid=""
  local found_existing=0

  cleanup_executor_lock

  while read -r executor_pid; do
    [[ -z "${executor_pid}" ]] && continue
    found_existing=1
    stop_pid_if_running "${executor_pid}" "pre-existing navigation_executor"
  done < <(pgrep -f "${EXECUTOR_BIN}" 2>/dev/null || true)

  if [[ -f "${EXECUTOR_LOCK_FILE}" ]]; then
    executor_pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${executor_pid}" ]]; then
      if (( found_existing == 0 )); then
        echo "[start_nav_headless] found executor lock from pid=${executor_pid}, cleaning it up"
      fi
      stop_pid_if_running "${executor_pid}" "locked navigation_executor"
    fi
  fi

  cleanup_executor_lock

  if pgrep -f "${EXECUTOR_BIN}" >/dev/null 2>&1; then
    echo "[start_nav_headless] failed to stop existing navigation_executor.py instances" >&2
    pgrep -fa "${EXECUTOR_BIN}" >&2 || true
    exit 1
  fi
}

print_help() {
  cat <<'EOF'
Usage: start_navigation_headless.sh [zt_0|test_1] [options]

Starts the full headless navigation stack:
  1. xt16_driver
  2. ros2 launch go2w_real slam_rf2o.launch.py slam_mode:=localization use_odom_fusion:=true use_rviz:=false ...
  3. ros2 run go2w_real navigation_executor.py ...

Map presets:
  zt_0    Default. Uses go2w_map_waypoints.yaml + /home/unitree/ros_ws/src/map/zt_0
  test_1  Uses go2w_waypoints.yaml + /home/unitree/ros_ws/src/map/test_1

Options:
  --preset NAME            Preset name: zt_0 / test_1
  --slam-map-file PREFIX   Serialized slam_toolbox map prefix
  --waypoint-file PATH     Waypoint YAML file used by navigation_executor.py
  --server-uri URI         WebSocket server URI for navigation_executor.py
                           default: ws://192.168.123.186:8100/ws/navigation/executor
  --network-interface IF   Interface passed to slam_rf2o.launch.py
  --server-timeout SEC     Passed to navigation_executor.py
  --heartbeat-interval SEC Passed to navigation_executor.py
  --reconnect-delay SEC    Passed to navigation_executor.py
  --nav-event-timeout SEC  Passed to navigation_executor.py
  --show-executor-logs 0|1 Stream navigation_executor.log to the current terminal, default: 1
  -h, --help               Show this help

Environment overrides:
  GO2W_NAV_PRESET
  GO2W_SLAM_MAP_FILE
  GO2W_WAYPOINT_FILE
  GO2W_SERVER_URI
  GO2W_NETWORK_INTERFACE
  GO2W_EXECUTOR_SERVER_TIMEOUT
  GO2W_EXECUTOR_HEARTBEAT_INTERVAL
  GO2W_EXECUTOR_RECONNECT_DELAY
  GO2W_EXECUTOR_NAV_EVENT_TIMEOUT
  GO2W_SHOW_EXECUTOR_LOGS
EOF
}

apply_preset_defaults() {
  local preset_name="$1"
  local preset_waypoint_file=""
  local preset_slam_map_prefix=""

  case "${preset_name}" in
    zt_0)
      preset_waypoint_file="${WS_DIR}/src/go2w_real/config/go2w_map_waypoints.yaml"
      preset_slam_map_prefix="${WS_DIR}/src/map/zt_0"
      ;;
    test_1)
      preset_waypoint_file="${WS_DIR}/src/go2w_real/config/go2w_waypoints.yaml"
      preset_slam_map_prefix="${WS_DIR}/src/map/test_1"
      ;;
    *)
      echo "[start_nav_headless] unknown map preset: ${preset_name}" >&2
      print_help >&2
      exit 1
      ;;
  esac

  if [[ -z "${WAYPOINT_FILE}" ]]; then
    WAYPOINT_FILE="${preset_waypoint_file}"
  fi
  if [[ -z "${SLAM_MAP_PREFIX}" ]]; then
    SLAM_MAP_PREFIX="${preset_slam_map_prefix}"
  fi
}

PRESET="${GO2W_NAV_PRESET:-$DEFAULT_PRESET}"
WAYPOINT_FILE="${GO2W_WAYPOINT_FILE:-}"
SLAM_MAP_PREFIX="${GO2W_SLAM_MAP_FILE:-}"
SERVER_URI="${GO2W_SERVER_URI:-$DEFAULT_SERVER_URI}"
NETWORK_INTERFACE="${GO2W_NETWORK_INTERFACE:-$DEFAULT_NETWORK_INTERFACE}"
EXECUTOR_SERVER_TIMEOUT="${GO2W_EXECUTOR_SERVER_TIMEOUT:-}"
EXECUTOR_HEARTBEAT_INTERVAL="${GO2W_EXECUTOR_HEARTBEAT_INTERVAL:-}"
EXECUTOR_RECONNECT_DELAY="${GO2W_EXECUTOR_RECONNECT_DELAY:-}"
EXECUTOR_NAV_EVENT_TIMEOUT="${GO2W_EXECUTOR_NAV_EVENT_TIMEOUT:-}"
SHOW_EXECUTOR_LOGS="${GO2W_SHOW_EXECUTOR_LOGS:-$DEFAULT_SHOW_EXECUTOR_LOGS}"
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
    --waypoint-file)
      WAYPOINT_FILE="${2:-}"
      shift 2
      ;;
    --waypoint-file=*)
      WAYPOINT_FILE="${1#*=}"
      shift
      ;;
    --server-uri)
      SERVER_URI="${2:-}"
      shift 2
      ;;
    --server-uri=*)
      SERVER_URI="${1#*=}"
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
    --server-timeout)
      EXECUTOR_SERVER_TIMEOUT="${2:-}"
      shift 2
      ;;
    --server-timeout=*)
      EXECUTOR_SERVER_TIMEOUT="${1#*=}"
      shift
      ;;
    --heartbeat-interval)
      EXECUTOR_HEARTBEAT_INTERVAL="${2:-}"
      shift 2
      ;;
    --heartbeat-interval=*)
      EXECUTOR_HEARTBEAT_INTERVAL="${1#*=}"
      shift
      ;;
    --reconnect-delay)
      EXECUTOR_RECONNECT_DELAY="${2:-}"
      shift 2
      ;;
    --reconnect-delay=*)
      EXECUTOR_RECONNECT_DELAY="${1#*=}"
      shift
      ;;
    --nav-event-timeout)
      EXECUTOR_NAV_EVENT_TIMEOUT="${2:-}"
      shift 2
      ;;
    --nav-event-timeout=*)
      EXECUTOR_NAV_EVENT_TIMEOUT="${1#*=}"
      shift
      ;;
    --show-executor-logs)
      SHOW_EXECUTOR_LOGS="${2:-}"
      shift 2
      ;;
    --show-executor-logs=*)
      SHOW_EXECUTOR_LOGS="${1#*=}"
      shift
      ;;
    -*)
      echo "[start_nav_headless] unknown option: $1" >&2
      print_help >&2
      exit 1
      ;;
    *)
      if [[ -n "${POSITIONAL_PRESET}" ]]; then
        echo "[start_nav_headless] unexpected extra positional argument: $1" >&2
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

if [[ -z "${NETWORK_INTERFACE}" ]]; then
  echo "[start_nav_headless] missing required network interface after override resolution" >&2
  print_help >&2
  exit 1
fi

if [[ -z "${WAYPOINT_FILE}" || -z "${SLAM_MAP_PREFIX}" || -z "${SERVER_URI}" || -z "${NETWORK_INTERFACE}" ]]; then
  echo "[start_nav_headless] missing required runtime parameter after preset/override resolution" >&2
  print_help >&2
  exit 1
fi

for required_file in \
  "${WAYPOINT_FILE}" \
  "${SLAM_MAP_PREFIX}.data" \
  "${SLAM_MAP_PREFIX}.posegraph" \
  "${XT16_BIN}" \
  "${EXECUTOR_BIN}" \
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
LOG_TAIL_PID=""

cleanup() {
  local exit_code=$?
  local lock_pid=""
  trap - EXIT HUP INT QUIT TERM

  stop_pid_if_running "${EXECUTOR_PID}" "navigation_executor"
  stop_pid_if_running "${LOG_TAIL_PID}" "executor log tail"

  if [[ -f "${EXECUTOR_LOCK_FILE}" ]]; then
    lock_pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${lock_pid}" ]] && kill -0 "${lock_pid}" 2>/dev/null; then
      stop_pid_if_running "${lock_pid}" "locked navigation_executor"
    fi
  fi

  cleanup_executor_lock

  stop_pid_if_running "${LAUNCH_PID}" "slam_rf2o.launch.py"

  stop_pid_if_running "${XT16_PID}" "xt16_driver"

  exit "${exit_code}"
}

trap cleanup EXIT HUP INT QUIT TERM

echo "[start_nav_headless] preset: ${PRESET}"
echo "[start_nav_headless] waypoint file: ${WAYPOINT_FILE}"
echo "[start_nav_headless] slam map prefix: ${SLAM_MAP_PREFIX}"
echo "[start_nav_headless] server uri: ${SERVER_URI}"
echo "[start_nav_headless] network interface: ${NETWORK_INTERFACE}"
echo "[start_nav_headless] show executor logs: ${SHOW_EXECUTOR_LOGS}"
echo "[start_nav_headless] logs: ${LOG_DIR}"

stop_existing_executors

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

EXECUTOR_CMD=(
  "${EXECUTOR_BIN}"
  --waypoint-file "${WAYPOINT_FILE}"
  --server-uri "${SERVER_URI}"
)

if [[ -n "${EXECUTOR_SERVER_TIMEOUT}" ]]; then
  EXECUTOR_CMD+=(--server-timeout "${EXECUTOR_SERVER_TIMEOUT}")
fi
if [[ -n "${EXECUTOR_HEARTBEAT_INTERVAL}" ]]; then
  EXECUTOR_CMD+=(--heartbeat-interval "${EXECUTOR_HEARTBEAT_INTERVAL}")
fi
if [[ -n "${EXECUTOR_RECONNECT_DELAY}" ]]; then
  EXECUTOR_CMD+=(--reconnect-delay "${EXECUTOR_RECONNECT_DELAY}")
fi
if [[ -n "${EXECUTOR_NAV_EVENT_TIMEOUT}" ]]; then
  EXECUTOR_CMD+=(--nav-event-timeout "${EXECUTOR_NAV_EVENT_TIMEOUT}")
fi

"${EXECUTOR_CMD[@]}" >"${LOG_DIR}/navigation_executor.log" 2>&1 &
EXECUTOR_PID=$!
echo "[start_nav_headless] started navigation_executor.py (pid=${EXECUTOR_PID})"

if [[ "${SHOW_EXECUTOR_LOGS}" == "1" ]]; then
  tail -n +1 -F "${LOG_DIR}/navigation_executor.log" &
  LOG_TAIL_PID=$!
  echo "[start_nav_headless] streaming navigation_executor.log to this terminal (pid=${LOG_TAIL_PID})"
fi

echo "[start_nav_headless] stack is ready. Press Ctrl+C to stop all managed processes."

wait -n "${LAUNCH_PID}" "${EXECUTOR_PID}" ${XT16_PID:+"${XT16_PID}"}
echo "[start_nav_headless] a managed process exited, stopping the stack" >&2
exit 1
