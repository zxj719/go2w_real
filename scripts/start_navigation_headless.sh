#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_workspace_dir() {
  local candidate="${SCRIPT_DIR}"

  while [[ "${candidate}" != "/" ]]; do
    if [[ -d "${candidate}/src/go2w_real" ]]; then
      echo "${candidate}"
      return 0
    fi
    candidate="$(dirname "${candidate}")"
  done

  echo "/home/unitree/ros_ws"
}

WS_DIR="${GO2W_WS_DIR:-$(resolve_workspace_dir)}"
PACKAGE_SRC_DIR="${WS_DIR}/src/go2w_real"
ROS_SETUP_FILE="${GO2W_ROS_SETUP_FILE:-/opt/ros/foxy/setup.bash}"
INSTALL_SETUP_FILE="${GO2W_INSTALL_SETUP_FILE:-${WS_DIR}/install/setup.bash}"
UNITREE_SETUP="${GO2W_UNITREE_SETUP_FILE:-/home/unitree/unitree_ros2/setup.sh}"
XT16_BIN="${GO2W_XT16_BIN:-${SCRIPT_DIR}/xt16_driver_timefix.sh}"
EXECUTOR_BIN="${GO2W_EXECUTOR_BIN:-${SCRIPT_DIR}/navigation_executor.py}"
CONFIG_HELPER_BIN="${GO2W_CONFIG_HELPER_BIN:-${SCRIPT_DIR}/navigation_headless_config.py}"
BAG_TOPICS_HELPER="${GO2W_BAG_TOPICS_HELPER:-${SCRIPT_DIR}/navigation_debug_bag_topics.sh}"
EXECUTOR_LOCK_FILE="/tmp/go2w_navigation_executor.lock"
HEADLESS_CONFIG_FILE="${GO2W_HEADLESS_CONFIG_FILE:-${WS_DIR}/src/go2w_real/config/navigation_headless.yaml}"
NAV2_PARAMS_FILE="${GO2W_NAV2_PARAMS_FILE:-${PACKAGE_SRC_DIR}/config/nav2_params_foxy.yaml}"
SLAM_LOCALIZATION_PARAMS_FILE="${GO2W_SLAM_LOCALIZATION_PARAMS_FILE:-${PACKAGE_SRC_DIR}/config/slam_params.yaml}"
ROS2_BIN="${GO2W_ROS2_BIN:-ros2}"
HEADLESS_PROFILE="${GO2W_HEADLESS_PROFILE:-}"
NAV2_WAIT_TIMEOUT_SEC=120
STOP_TERM_TIMEOUT_SEC=5
STOP_KILL_TIMEOUT_SEC=2

if [[ ! -f "${BAG_TOPICS_HELPER}" ]]; then
  echo "[start_nav_headless] missing bag topics helper: ${BAG_TOPICS_HELPER}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${BAG_TOPICS_HELPER}"

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

wait_for_process_group_exit() {
  local process_group_id="$1"
  local timeout_sec="${2:-5}"
  local deadline=$((SECONDS + timeout_sec))

  while kill -0 -- "-${process_group_id}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done

  return 0
}

get_process_group_id() {
  local pid="$1"
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]'
}

stop_pid_if_running() {
  local pid="$1"
  local label="${2:-process}"
  local signal_name="${3:-TERM}"
  local term_timeout_sec="${4:-$STOP_TERM_TIMEOUT_SEC}"
  local kill_timeout_sec="${5:-$STOP_KILL_TIMEOUT_SEC}"
  local use_process_group="${6:-0}"
  local process_group_id="${pid}"

  if [[ "${use_process_group}" == "1" ]]; then
    if [[ -z "${process_group_id}" ]] || ! kill -0 -- "-${process_group_id}" 2>/dev/null; then
      return 0
    fi
  else
    if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
  fi

  if [[ "${use_process_group}" == "1" ]]; then
    echo "[start_nav_headless] stopping ${label} (pgid=${process_group_id}, signal=${signal_name})"
    kill "-${signal_name}" -- "-${process_group_id}" 2>/dev/null || kill "-${signal_name}" "${pid}" 2>/dev/null || true
    if ! wait_for_process_group_exit "${process_group_id}" "${term_timeout_sec}"; then
      echo "[start_nav_headless] ${label} did not exit after SIG${signal_name}, sending SIGKILL (pgid=${process_group_id})" >&2
      kill -9 -- "-${process_group_id}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
      wait_for_process_group_exit "${process_group_id}" "${kill_timeout_sec}" || true
    fi
  else
    echo "[start_nav_headless] stopping ${label} (pid=${pid}, signal=${signal_name})"
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
    if ! wait_for_pid_exit "${pid}" "${term_timeout_sec}"; then
      echo "[start_nav_headless] ${label} did not exit after SIG${signal_name}, sending SIGKILL (pid=${pid})" >&2
      kill -9 "${pid}" 2>/dev/null || true
      wait_for_pid_exit "${pid}" "${kill_timeout_sec}" || true
    fi
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

stop_matching_processes() {
  local matcher=""
  local pid=""
  local pgid=""
  local -A stopped_process_groups=()

  cleanup_executor_lock

  for matcher in "${HEADLESS_CLEANUP_MATCHERS[@]}"; do
    while read -r pid; do
      [[ -z "${pid}" ]] && continue
      pgid="$(get_process_group_id "${pid}")"
      if [[ -n "${pgid}" ]]; then
        if [[ -n "${stopped_process_groups["${pgid}"]:-}" ]]; then
          continue
        fi
        stopped_process_groups["${pgid}"]=1
        stop_pid_if_running "${pgid}" "cleanup matcher '${matcher}'" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" 1
      else
        stop_pid_if_running "${pid}" "cleanup matcher '${matcher}'"
      fi
    done < <(pgrep -f "${matcher}" 2>/dev/null || true)
  done

  if [[ -f "${EXECUTOR_LOCK_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      pgid="$(get_process_group_id "${pid}")"
      if [[ -n "${pgid}" ]]; then
        stop_pid_if_running "${pgid}" "locked navigation_executor" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" 1
      else
        stop_pid_if_running "${pid}" "locked navigation_executor"
      fi
    fi
  fi

  cleanup_executor_lock
}

print_help() {
  cat <<'EOF'
Usage: start_navigation_headless.sh [options]

Starts the full headless navigation stack from a unified YAML config:
  1. cleanup stale managed processes
  2. xt16_driver
  3. ros2 launch go2w_real slam_rf2o.launch.py ...
  4. wait for bt_navigator
  5. navigation_executor.py

Options:
  --config PATH     Unified headless config YAML
  --profile NAME    Headless config profile name, default: config default_profile
  -h, --help        Show this help

Default config profiles:
  zt_0    go2w_map_waypoints.yaml + map/zt_0
  14_1    go2w_waypoints.yaml + map/14_1

Environment overrides:
  GO2W_HEADLESS_CONFIG_FILE
  GO2W_HEADLESS_PROFILE
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      print_help
      exit 0
      ;;
    --config)
      HEADLESS_CONFIG_FILE="${2:-}"
      shift 2
      ;;
    --config=*)
      HEADLESS_CONFIG_FILE="${1#*=}"
      shift
      ;;
    --profile)
      HEADLESS_PROFILE="${2:-}"
      shift 2
      ;;
    --profile=*)
      HEADLESS_PROFILE="${1#*=}"
      shift
      ;;
    *)
      echo "[start_nav_headless] unknown option: $1" >&2
      print_help >&2
      exit 1
      ;;
  esac
done

CONFIG_RESOLVE_CMD=(
  python3 "${CONFIG_HELPER_BIN}"
  --config "${HEADLESS_CONFIG_FILE}"
  --format shell
)
if [[ -n "${HEADLESS_PROFILE}" ]]; then
  CONFIG_RESOLVE_CMD+=(--profile "${HEADLESS_PROFILE}")
fi

eval "$("${CONFIG_RESOLVE_CMD[@]}")"

WAYPOINT_FILE="${HEADLESS_WAYPOINT_FILE}"
SLAM_MAP_PREFIX="${HEADLESS_SLAM_MAP_FILE}"
SERVER_URI="${HEADLESS_SERVER_URI}"
NETWORK_INTERFACE="${HEADLESS_NETWORK_INTERFACE}"
SHOW_EXECUTOR_LOGS="${HEADLESS_SHOW_LOGS}"
RECORD_BAG="${HEADLESS_RECORD_BAG}"
NAV2_WAIT_TIMEOUT_SEC="${HEADLESS_NAV2_WAIT_TIMEOUT_SEC}"
STOP_TERM_TIMEOUT_SEC="${HEADLESS_TERM_TIMEOUT_SEC}"
STOP_KILL_TIMEOUT_SEC="${HEADLESS_KILL_TIMEOUT_SEC}"

for required_file in \
  "${HEADLESS_CONFIG_PATH}" \
  "${CONFIG_HELPER_BIN}" \
  "${BAG_TOPICS_HELPER}" \
  "${WAYPOINT_FILE}" \
  "${SLAM_MAP_PREFIX}.data" \
  "${SLAM_MAP_PREFIX}.posegraph" \
  "${NAV2_PARAMS_FILE}" \
  "${SLAM_LOCALIZATION_PARAMS_FILE}" \
  "${XT16_BIN}" \
  "${EXECUTOR_BIN}" \
  "${ROS_SETUP_FILE}" \
  "${INSTALL_SETUP_FILE}"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "[start_nav_headless] missing required file: ${required_file}" >&2
    exit 1
  fi
done

safe_source "${ROS_SETUP_FILE}"
if [[ -f "${UNITREE_SETUP}" ]]; then
  safe_source "${UNITREE_SETUP}"
fi
safe_source "${INSTALL_SETUP_FILE}"

LOG_DIR="${WS_DIR}/log/start_navigation_headless_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

XT16_PID=""
LAUNCH_PID=""
EXECUTOR_PID=""
LOG_TAIL_PID=""
BAG_PID=""
XT16_USE_PROCESS_GROUP="0"
LAUNCH_USE_PROCESS_GROUP="0"
EXECUTOR_USE_PROCESS_GROUP="0"
BAG_USE_PROCESS_GROUP="0"
BAG_OUTPUT_DIR="${LOG_DIR}/rosbag_navigation_debug"
BAG_LOG_FILE="${LOG_DIR}/rosbag_record.log"

cleanup() {
  local exit_code=$?
  local lock_pid=""
  trap - EXIT HUP INT QUIT TERM

  stop_pid_if_running "${EXECUTOR_PID}" "navigation_executor" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${EXECUTOR_USE_PROCESS_GROUP}"
  stop_pid_if_running "${LOG_TAIL_PID}" "executor log tail"
  stop_pid_if_running "${BAG_PID}" "rosbag recorder" INT "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${BAG_USE_PROCESS_GROUP}"

  if [[ -f "${EXECUTOR_LOCK_FILE}" ]]; then
    lock_pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${lock_pid}" ]] && kill -0 "${lock_pid}" 2>/dev/null; then
      stop_pid_if_running "${lock_pid}" "locked navigation_executor" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${EXECUTOR_USE_PROCESS_GROUP}"
    fi
  fi

  cleanup_executor_lock

  stop_pid_if_running "${LAUNCH_PID}" "slam_rf2o.launch.py" INT "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${LAUNCH_USE_PROCESS_GROUP}"

  stop_pid_if_running "${XT16_PID}" "xt16_driver" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${XT16_USE_PROCESS_GROUP}"

  exit "${exit_code}"
}

trap cleanup EXIT HUP INT QUIT TERM

echo "[start_nav_headless] profile: ${HEADLESS_PROFILE}"
echo "[start_nav_headless] waypoint file: ${WAYPOINT_FILE}"
echo "[start_nav_headless] slam map prefix: ${SLAM_MAP_PREFIX}"
echo "[start_nav_headless] nav2 params file: ${NAV2_PARAMS_FILE}"
echo "[start_nav_headless] slam localization params file: ${SLAM_LOCALIZATION_PARAMS_FILE}"
echo "[start_nav_headless] server uri: ${SERVER_URI}"
echo "[start_nav_headless] network interface: ${NETWORK_INTERFACE}"
echo "[start_nav_headless] show executor logs: ${SHOW_EXECUTOR_LOGS}"
echo "[start_nav_headless] record bag: ${RECORD_BAG}"
echo "[start_nav_headless] logs: ${LOG_DIR}"

stop_matching_processes

if command -v setsid >/dev/null 2>&1; then
  setsid "${XT16_BIN}" >"${LOG_DIR}/xt16_driver.log" 2>&1 &
  XT16_USE_PROCESS_GROUP="1"
else
  "${XT16_BIN}" >"${LOG_DIR}/xt16_driver.log" 2>&1 &
fi
XT16_PID=$!
echo "[start_nav_headless] started xt16_driver (pid=${XT16_PID})"
sleep 2
if ! kill -0 "${XT16_PID}" 2>/dev/null; then
  echo "[start_nav_headless] xt16_driver exited early, check ${LOG_DIR}/xt16_driver.log" >&2
  exit 1
fi

LAUNCH_CMD=(
  "${ROS2_BIN}" launch go2w_real slam_rf2o.launch.py
  headless_config_file:="${HEADLESS_CONFIG_PATH}"
  headless_profile:="${HEADLESS_PROFILE}"
  slam_mode:="${HEADLESS_SLAM_MODE}"
  use_odom_fusion:=$([[ "${HEADLESS_USE_ODOM_FUSION}" == "1" ]] && echo true || echo false)
  use_rviz:=$([[ "${HEADLESS_USE_RVIZ}" == "1" ]] && echo true || echo false)
  network_interface:="${NETWORK_INTERFACE}"
  slam_map_file:="${SLAM_MAP_PREFIX}"
  nav2_params_file:="${NAV2_PARAMS_FILE}"
  slam_localization_params_file:="${SLAM_LOCALIZATION_PARAMS_FILE}"
)
if command -v setsid >/dev/null 2>&1; then
  setsid "${LAUNCH_CMD[@]}" >"${LOG_DIR}/slam_rf2o.log" 2>&1 &
  LAUNCH_USE_PROCESS_GROUP="1"
else
  "${LAUNCH_CMD[@]}" >"${LOG_DIR}/slam_rf2o.log" 2>&1 &
fi
LAUNCH_PID=$!
echo "[start_nav_headless] started slam_rf2o.launch.py (pid=${LAUNCH_PID})"

echo "[start_nav_headless] waiting for Nav2 bt_navigator node..."
deadline=$((SECONDS + NAV2_WAIT_TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[start_nav_headless] slam_rf2o.launch.py exited early, check ${LOG_DIR}/slam_rf2o.log" >&2
    exit 1
  fi

  if "${ROS2_BIN}" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
    break
  fi

  sleep 1
done

if ! "${ROS2_BIN}" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
  echo "[start_nav_headless] Nav2 did not become ready within ${NAV2_WAIT_TIMEOUT_SEC}s" >&2
  exit 1
fi

if [[ "${RECORD_BAG}" == "1" ]]; then
  printf '%s\n' "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" > "${LOG_DIR}/rosbag_topics.txt"
  if command -v setsid >/dev/null 2>&1; then
    setsid "${ROS2_BIN}" bag record \
      --output "${BAG_OUTPUT_DIR}" \
      --compression-mode file \
      --compression-format zstd \
      --include-hidden-topics \
      "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" >"${BAG_LOG_FILE}" 2>&1 &
    BAG_USE_PROCESS_GROUP="1"
  else
    "${ROS2_BIN}" bag record \
      --output "${BAG_OUTPUT_DIR}" \
      --compression-mode file \
      --compression-format zstd \
      --include-hidden-topics \
      "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" >"${BAG_LOG_FILE}" 2>&1 &
  fi
  BAG_PID=$!
  echo "[start_nav_headless] started rosbag recorder (pid=${BAG_PID})"
  sleep 2
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "[start_nav_headless] rosbag recorder exited early, check ${BAG_LOG_FILE}" >&2
    exit 1
  fi
fi

EXECUTOR_CMD=(
  "${EXECUTOR_BIN}"
  --config "${HEADLESS_CONFIG_PATH}"
  --waypoint-file "${WAYPOINT_FILE}"
  --server-uri "${SERVER_URI}"
)
if [[ -n "${HEADLESS_PROFILE}" ]]; then
  EXECUTOR_CMD+=(--profile "${HEADLESS_PROFILE}")
fi
EXECUTOR_CMD+=(--server-timeout "${HEADLESS_SERVER_TIMEOUT}")
EXECUTOR_CMD+=(--heartbeat-interval "${HEADLESS_HEARTBEAT_INTERVAL}")
EXECUTOR_CMD+=(--reconnect-delay "${HEADLESS_RECONNECT_DELAY}")
EXECUTOR_CMD+=(--nav-event-timeout "${HEADLESS_NAV_EVENT_TIMEOUT}")

if command -v setsid >/dev/null 2>&1; then
  setsid "${EXECUTOR_CMD[@]}" >"${LOG_DIR}/navigation_executor.log" 2>&1 &
  EXECUTOR_USE_PROCESS_GROUP="1"
else
  "${EXECUTOR_CMD[@]}" >"${LOG_DIR}/navigation_executor.log" 2>&1 &
fi
EXECUTOR_PID=$!
echo "[start_nav_headless] started navigation_executor.py (pid=${EXECUTOR_PID})"

if [[ "${SHOW_EXECUTOR_LOGS}" == "1" ]]; then
  tail -n +1 -F "${LOG_DIR}/navigation_executor.log" &
  LOG_TAIL_PID=$!
  echo "[start_nav_headless] streaming navigation_executor.log to this terminal (pid=${LOG_TAIL_PID})"
fi

echo "[start_nav_headless] stack is ready. Press Ctrl+C to stop all managed processes."

wait -n "${LAUNCH_PID}" "${EXECUTOR_PID}" ${XT16_PID:+"${XT16_PID}"} ${BAG_PID:+"${BAG_PID}"}
echo "[start_nav_headless] a managed process exited, stopping the stack" >&2
exit 1
