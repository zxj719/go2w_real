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

safe_source() {
  local source_file="$1"
  if [[ ! -f "${source_file}" ]]; then
    echo "[navigate_debug_bag] source file not found: ${source_file}" >&2
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

stop_pid_if_running() {
  local pid="$1"
  local label="${2:-process}"
  local signal_name="${3:-TERM}"
  local term_timeout_sec="${4:-5}"
  local kill_timeout_sec="${5:-2}"
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

  echo "[navigate_debug_bag] stopping ${label} (pid=${pid}, signal=${signal_name})"
  if [[ "${use_process_group}" == "1" ]]; then
    kill "-${signal_name}" -- "-${process_group_id}" 2>/dev/null || kill "-${signal_name}" "${pid}" 2>/dev/null || true
  else
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
  fi

  if [[ "${use_process_group}" == "1" ]]; then
    if ! wait_for_process_group_exit "${process_group_id}" "${term_timeout_sec}"; then
      echo "[navigate_debug_bag] ${label} did not exit cleanly, sending SIGKILL (pgid=${process_group_id})" >&2
      kill -9 -- "-${process_group_id}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
      wait_for_process_group_exit "${process_group_id}" "${kill_timeout_sec}" || true
    fi
  elif ! wait_for_pid_exit "${pid}" "${term_timeout_sec}"; then
    echo "[navigate_debug_bag] ${label} did not exit cleanly, sending SIGKILL (pid=${pid})" >&2
    kill -9 "${pid}" 2>/dev/null || true
    wait_for_pid_exit "${pid}" "${kill_timeout_sec}" || true
  fi
}

print_help() {
  cat <<'EOF'
Usage: navigate_to_waypoint_with_debug_bag.sh [wrapper options] [-- navigate_to_waypoint args]

Default workflow:
  1. start xt16_driver
  2. source ROS / Unitree / workspace environments
  3. launch slam_rf2o localization with an existing map
  4. start rosbag recording
  5. run navigate_to_waypoint.py

Starts rosbag recording immediately before navigate_to_waypoint.py
using the shared navigation debug topic set.
Stops recording automatically when navigation exits.

Wrapper options:
  --no-bringup                 Skip xt16_driver + slam_rf2o bringup and only do bag + navigate
  --profile NAME              Navigation preset: zt_0 / 14_1, default: zt_0
  --waypoint-file PATH        Waypoint YAML for navigate_to_waypoint.py
  --slam-map-file PATH        Map prefix for slam_rf2o localization
  --slam-mode MODE            slam_rf2o slam_mode value, default: localization
  --nav-ready-timeout SEC     Wait timeout for bt_navigator readiness, default: 120
  --tf-ready-timeout SEC      Wait timeout for TF map <- base readiness, default: 30
  --xt16-warmup-sec SEC       Warmup delay after starting xt16_driver, default: 2
  --skip-nav-ready-wait       Do not wait for bt_navigator before starting rosbag + navigate
  --skip-tf-ready-wait        Do not wait for TF map <- base before starting rosbag + navigate
  --map-frame FRAME           TF target frame for readiness wait; forwarded to navigate_to_waypoint.py
  --base-frame FRAME          TF source frame for readiness wait; forwarded to navigate_to_waypoint.py

Useful examples:
  navigate_to_waypoint_with_debug_bag.sh --all-waypoints
  navigate_to_waypoint_with_debug_bag.sh --waypoint-sequence POI_019,POI_020
  navigate_to_waypoint_with_debug_bag.sh --no-bringup --waypoint POI_019

Environment overrides:
  GO2W_WS_DIR
  GO2W_DEBUG_BAG_OUTPUT_DIR
  GO2W_NAVIGATE_BIN
  GO2W_ROS2_BIN
  GO2W_XT16_BIN
  GO2W_NAV_PROFILE
  GO2W_WAYPOINT_FILE
  GO2W_SLAM_MAP_FILE
  GO2W_SLAM_MODE
  GO2W_SKIP_BRINGUP
  GO2W_SKIP_NAV_READY_WAIT
  GO2W_SKIP_TF_READY_WAIT
  GO2W_NAV_READY_TIMEOUT_SEC
  GO2W_TF_READY_TIMEOUT_SEC
  GO2W_XT16_WARMUP_SEC
  GO2W_MAP_FRAME
  GO2W_BASE_FRAME
  GO2W_WAIT_FOR_TF_BIN
  GO2W_ROS_SETUP_FILE
  GO2W_INSTALL_SETUP_FILE
  GO2W_UNITREE_SETUP_FILE
EOF
}

WS_DIR="${GO2W_WS_DIR:-$(resolve_workspace_dir)}"
PACKAGE_SRC_DIR="${WS_DIR}/src/go2w_real"
ROS_SETUP_FILE="${GO2W_ROS_SETUP_FILE:-/opt/ros/foxy/setup.bash}"
INSTALL_SETUP_FILE="${GO2W_INSTALL_SETUP_FILE:-${WS_DIR}/install/setup.bash}"
UNITREE_SETUP_FILE="${GO2W_UNITREE_SETUP_FILE-/home/unitree/unitree_ros2/setup.sh}"
NAVIGATE_BIN="${GO2W_NAVIGATE_BIN:-${PACKAGE_SRC_DIR}/scripts/navigate_to_waypoint.py}"
ROS2_BIN="${GO2W_ROS2_BIN:-ros2}"
XT16_BIN="${GO2W_XT16_BIN:-/unitree/module/unitree_slam/bin/xt16_driver}"
NAV_PROFILE="${GO2W_NAV_PROFILE:-zt_0}"
WAYPOINT_FILE="${GO2W_WAYPOINT_FILE:-}"
SLAM_MAP_FILE="${GO2W_SLAM_MAP_FILE:-}"
SLAM_MODE="${GO2W_SLAM_MODE:-localization}"
START_BRINGUP="1"
WAIT_FOR_NAV_READY="1"
WAIT_FOR_TF_READY="1"
NAV_READY_TIMEOUT_SEC="${GO2W_NAV_READY_TIMEOUT_SEC:-120}"
TF_READY_TIMEOUT_SEC="${GO2W_TF_READY_TIMEOUT_SEC:-30.0}"
XT16_WARMUP_SEC="${GO2W_XT16_WARMUP_SEC:-2}"
TF_TARGET_FRAME="${GO2W_MAP_FRAME:-map}"
TF_SOURCE_FRAME="${GO2W_BASE_FRAME:-base}"
WAIT_FOR_TF_BIN="${GO2W_WAIT_FOR_TF_BIN:-${PACKAGE_SRC_DIR}/scripts/wait_for_transform.py}"
BAG_TOPICS_HELPER="${SCRIPT_DIR}/navigation_debug_bag_topics.sh"
OUTPUT_ROOT="${GO2W_DEBUG_BAG_OUTPUT_DIR:-${WS_DIR}/log/navigate_to_waypoint_debug_$(date +%Y%m%d_%H%M%S)}"
BAG_OUTPUT_DIR="${OUTPUT_ROOT}/bag"
BAG_LOG_FILE="${OUTPUT_ROOT}/rosbag_record.log"
XT16_LOG_FILE="${OUTPUT_ROOT}/xt16_driver.log"
LAUNCH_LOG_FILE="${OUTPUT_ROOT}/slam_rf2o.log"
TF_WAIT_LOG_FILE="${OUTPUT_ROOT}/wait_for_tf.log"
NAVIGATE_ARGS=()
BAG_PID=""
NAVIGATE_PID=""
XT16_PID=""
LAUNCH_PID=""
BAG_USE_PROCESS_GROUP="0"
XT16_USE_PROCESS_GROUP="0"
LAUNCH_USE_PROCESS_GROUP="0"

apply_navigation_profile_defaults() {
  local profile_name="$1"
  local profile_waypoint_file=""
  local profile_slam_map_file=""

  case "${profile_name}" in
    zt_0)
      profile_waypoint_file="${PACKAGE_SRC_DIR}/config/go2w_map_waypoints.yaml"
      profile_slam_map_file="${PACKAGE_SRC_DIR}/map/zt_0"
      ;;
    14_1)
      profile_waypoint_file="${PACKAGE_SRC_DIR}/config/go2w_waypoints.yaml"
      profile_slam_map_file="${PACKAGE_SRC_DIR}/map/14_1"
      ;;
    *)
      echo "[navigate_debug_bag] unknown navigation profile: ${profile_name}" >&2
      print_help >&2
      exit 1
      ;;
  esac

  if [[ -z "${WAYPOINT_FILE}" ]]; then
    WAYPOINT_FILE="${profile_waypoint_file}"
  fi
  if [[ -z "${SLAM_MAP_FILE}" ]]; then
    SLAM_MAP_FILE="${profile_slam_map_file}"
  fi
}

if [[ "${GO2W_SKIP_BRINGUP:-0}" == "1" ]]; then
  START_BRINGUP="0"
fi
if [[ "${GO2W_SKIP_NAV_READY_WAIT:-0}" == "1" ]]; then
  WAIT_FOR_NAV_READY="0"
fi
if [[ "${GO2W_SKIP_TF_READY_WAIT:-0}" == "1" ]]; then
  WAIT_FOR_TF_READY="0"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      print_help
      exit 0
      ;;
    --no-bringup)
      START_BRINGUP="0"
      shift
      ;;
    --profile)
      NAV_PROFILE="${2:-}"
      if [[ -z "${NAV_PROFILE}" ]]; then
        echo "[navigate_debug_bag] --profile requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --profile=*)
      NAV_PROFILE="${1#*=}"
      shift
      ;;
    --waypoint-file)
      WAYPOINT_FILE="${2:-}"
      if [[ -z "${WAYPOINT_FILE}" ]]; then
        echo "[navigate_debug_bag] --waypoint-file requires a value" >&2
        exit 1
      fi
      NAVIGATE_ARGS+=("$1" "$2")
      shift 2
      ;;
    --waypoint-file=*)
      WAYPOINT_FILE="${1#*=}"
      NAVIGATE_ARGS+=("$1")
      shift
      ;;
    --slam-map-file)
      SLAM_MAP_FILE="${2:-}"
      if [[ -z "${SLAM_MAP_FILE}" ]]; then
        echo "[navigate_debug_bag] --slam-map-file requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --slam-map-file=*)
      SLAM_MAP_FILE="${1#*=}"
      shift
      ;;
    --slam-mode)
      SLAM_MODE="${2:-}"
      if [[ -z "${SLAM_MODE}" ]]; then
        echo "[navigate_debug_bag] --slam-mode requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --slam-mode=*)
      SLAM_MODE="${1#*=}"
      shift
      ;;
    --nav-ready-timeout)
      NAV_READY_TIMEOUT_SEC="${2:-}"
      if [[ -z "${NAV_READY_TIMEOUT_SEC}" ]]; then
        echo "[navigate_debug_bag] --nav-ready-timeout requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --nav-ready-timeout=*)
      NAV_READY_TIMEOUT_SEC="${1#*=}"
      shift
      ;;
    --xt16-warmup-sec)
      XT16_WARMUP_SEC="${2:-}"
      if [[ -z "${XT16_WARMUP_SEC}" ]]; then
        echo "[navigate_debug_bag] --xt16-warmup-sec requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --xt16-warmup-sec=*)
      XT16_WARMUP_SEC="${1#*=}"
      shift
      ;;
    --skip-nav-ready-wait)
      WAIT_FOR_NAV_READY="0"
      shift
      ;;
    --tf-ready-timeout)
      TF_READY_TIMEOUT_SEC="${2:-}"
      if [[ -z "${TF_READY_TIMEOUT_SEC}" ]]; then
        echo "[navigate_debug_bag] --tf-ready-timeout requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --tf-ready-timeout=*)
      TF_READY_TIMEOUT_SEC="${1#*=}"
      shift
      ;;
    --skip-tf-ready-wait)
      WAIT_FOR_TF_READY="0"
      shift
      ;;
    --map-frame)
      TF_TARGET_FRAME="${2:-}"
      if [[ -z "${TF_TARGET_FRAME}" ]]; then
        echo "[navigate_debug_bag] --map-frame requires a value" >&2
        exit 1
      fi
      NAVIGATE_ARGS+=("$1" "$2")
      shift 2
      ;;
    --map-frame=*)
      TF_TARGET_FRAME="${1#*=}"
      NAVIGATE_ARGS+=("$1")
      shift
      ;;
    --base-frame)
      TF_SOURCE_FRAME="${2:-}"
      if [[ -z "${TF_SOURCE_FRAME}" ]]; then
        echo "[navigate_debug_bag] --base-frame requires a value" >&2
        exit 1
      fi
      NAVIGATE_ARGS+=("$1" "$2")
      shift 2
      ;;
    --base-frame=*)
      TF_SOURCE_FRAME="${1#*=}"
      NAVIGATE_ARGS+=("$1")
      shift
      ;;
    --)
      shift
      NAVIGATE_ARGS+=("$@")
      break
      ;;
    *)
      NAVIGATE_ARGS+=("$1")
      shift
      ;;
  esac
done

apply_navigation_profile_defaults "${NAV_PROFILE}"

if [[ ! -f "${BAG_TOPICS_HELPER}" ]]; then
  echo "[navigate_debug_bag] missing bag topics helper: ${BAG_TOPICS_HELPER}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${BAG_TOPICS_HELPER}"

for required_file in \
  "${BAG_TOPICS_HELPER}" \
  "${ROS_SETUP_FILE}" \
  "${INSTALL_SETUP_FILE}" \
  "${WAYPOINT_FILE}" \
  "${NAVIGATE_BIN}"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "[navigate_debug_bag] missing required file: ${required_file}" >&2
    exit 1
  fi
done

if [[ "${START_BRINGUP}" == "1" ]]; then
  for required_file in \
    "${XT16_BIN}" \
    "${SLAM_MAP_FILE}.data" \
    "${SLAM_MAP_FILE}.posegraph"; do
    if [[ ! -e "${required_file}" ]]; then
      echo "[navigate_debug_bag] missing required bringup file: ${required_file}" >&2
      exit 1
    fi
  done

  if [[ "${WAIT_FOR_TF_READY}" == "1" && ! -e "${WAIT_FOR_TF_BIN}" ]]; then
    echo "[navigate_debug_bag] missing TF wait helper: ${WAIT_FOR_TF_BIN}" >&2
    exit 1
  fi
fi

cleanup() {
  local exit_code=$?
  trap - EXIT HUP INT QUIT TERM

  stop_pid_if_running "${NAVIGATE_PID}" "navigate_to_waypoint"
  stop_pid_if_running "${BAG_PID}" "rosbag recorder" INT 5 2 "${BAG_USE_PROCESS_GROUP}"
  stop_pid_if_running "${LAUNCH_PID}" "slam_rf2o.launch.py" INT 5 2 "${LAUNCH_USE_PROCESS_GROUP}"
  stop_pid_if_running "${XT16_PID}" "xt16_driver" TERM 5 2 "${XT16_USE_PROCESS_GROUP}"

  exit "${exit_code}"
}

trap cleanup EXIT HUP INT QUIT TERM

safe_source "${ROS_SETUP_FILE}"
if [[ -n "${UNITREE_SETUP_FILE}" && -f "${UNITREE_SETUP_FILE}" ]]; then
  safe_source "${UNITREE_SETUP_FILE}"
fi
safe_source "${INSTALL_SETUP_FILE}"

mkdir -p "${OUTPUT_ROOT}"
printf '%s\n' "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" > "${OUTPUT_ROOT}/topics.txt"

if [[ "${START_BRINGUP}" == "1" ]]; then
  echo "[navigate_debug_bag] starting xt16_driver from ${XT16_BIN}"
  if command -v setsid >/dev/null 2>&1; then
    setsid "${XT16_BIN}" >"${XT16_LOG_FILE}" 2>&1 &
    XT16_USE_PROCESS_GROUP="1"
  else
    "${XT16_BIN}" >"${XT16_LOG_FILE}" 2>&1 &
  fi
  XT16_PID=$!
  echo "[navigate_debug_bag] started xt16_driver (pid=${XT16_PID})"
  sleep "${XT16_WARMUP_SEC}"

  if ! kill -0 "${XT16_PID}" 2>/dev/null; then
    echo "[navigate_debug_bag] xt16_driver exited early, check ${XT16_LOG_FILE}" >&2
    exit 1
  fi

  echo "[navigate_debug_bag] launching slam_rf2o localization with map ${SLAM_MAP_FILE}"
  if command -v setsid >/dev/null 2>&1; then
    setsid "${ROS2_BIN}" launch go2w_real slam_rf2o.launch.py \
      "slam_mode:=${SLAM_MODE}" \
      "slam_map_file:=${SLAM_MAP_FILE}" >"${LAUNCH_LOG_FILE}" 2>&1 &
    LAUNCH_USE_PROCESS_GROUP="1"
  else
    "${ROS2_BIN}" launch go2w_real slam_rf2o.launch.py \
      "slam_mode:=${SLAM_MODE}" \
      "slam_map_file:=${SLAM_MAP_FILE}" >"${LAUNCH_LOG_FILE}" 2>&1 &
  fi
  LAUNCH_PID=$!
  echo "[navigate_debug_bag] started slam_rf2o.launch.py (pid=${LAUNCH_PID})"
  sleep 1

  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[navigate_debug_bag] slam_rf2o.launch.py exited early, check ${LAUNCH_LOG_FILE}" >&2
    exit 1
  fi

  if [[ "${WAIT_FOR_NAV_READY}" == "1" ]]; then
    echo "[navigate_debug_bag] waiting for bt_navigator to become ready..."
    deadline=$((SECONDS + NAV_READY_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
      if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[navigate_debug_bag] slam_rf2o.launch.py exited while waiting for readiness, check ${LAUNCH_LOG_FILE}" >&2
        exit 1
      fi

      if "${ROS2_BIN}" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
        break
      fi

      sleep 1
    done

    if ! "${ROS2_BIN}" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
      echo "[navigate_debug_bag] bt_navigator did not become ready within ${NAV_READY_TIMEOUT_SEC}s, check ${LAUNCH_LOG_FILE}" >&2
      exit 1
    fi
  fi

  if [[ "${WAIT_FOR_TF_READY}" == "1" ]]; then
    echo "[navigate_debug_bag] waiting for TF ${TF_TARGET_FRAME} <- ${TF_SOURCE_FRAME} to become ready..."
    if ! "${WAIT_FOR_TF_BIN}" \
      --ros-args \
      -p "target_frame:=${TF_TARGET_FRAME}" \
      -p "source_frame:=${TF_SOURCE_FRAME}" \
      -p "timeout_sec:=${TF_READY_TIMEOUT_SEC}" \
      -p "poll_period:=0.2" \
      -p "log_period:=2.0" >"${TF_WAIT_LOG_FILE}" 2>&1; then
      echo "[navigate_debug_bag] TF ${TF_TARGET_FRAME} <- ${TF_SOURCE_FRAME} did not become ready within ${TF_READY_TIMEOUT_SEC}s, check ${TF_WAIT_LOG_FILE}" >&2
      exit 1
    fi
    echo "[navigate_debug_bag] TF ${TF_TARGET_FRAME} <- ${TF_SOURCE_FRAME} is ready."
  fi
fi

echo "[navigate_debug_bag] profile: ${NAV_PROFILE}"
echo "[navigate_debug_bag] waypoint file: ${WAYPOINT_FILE}"

if command -v setsid >/dev/null 2>&1; then
  setsid "${ROS2_BIN}" bag record \
    -o "${BAG_OUTPUT_DIR}" \
    --compression-mode file \
    --compression-format zstd \
    --include-hidden-topics \
    "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" >"${BAG_LOG_FILE}" 2>&1 &
  BAG_USE_PROCESS_GROUP="1"
else
  "${ROS2_BIN}" bag record \
    -o "${BAG_OUTPUT_DIR}" \
    --compression-mode file \
    --compression-format zstd \
    --include-hidden-topics \
    "${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}" >"${BAG_LOG_FILE}" 2>&1 &
fi
BAG_PID=$!
echo "[navigate_debug_bag] started rosbag recorder (pid=${BAG_PID})"
echo "[navigate_debug_bag] recording to ${OUTPUT_ROOT}"
sleep 1

if ! kill -0 "${BAG_PID}" 2>/dev/null; then
  echo "[navigate_debug_bag] rosbag recorder exited early, check ${BAG_LOG_FILE}" >&2
  exit 1
fi

echo "[navigate_debug_bag] rosbag is recording. You can choose a waypoint now."
if GO2W_WAYPOINT_FILE="${WAYPOINT_FILE}" "${NAVIGATE_BIN}" "${NAVIGATE_ARGS[@]}"; then
  nav_exit_code=0
else
  nav_exit_code=$?
fi

exit "${nav_exit_code}"
