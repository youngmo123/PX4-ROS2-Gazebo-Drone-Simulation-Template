#!/usr/bin/env bash
# Discovers the Gazebo camera topic and launches the ros_gz_image bridge.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_TOPIC="/camera/image_raw"
GZ_TOPIC_OVERRIDE=""
INFO_TOPIC_OVERRIDE=""

usage() {
  cat <<'USAGE'
Usage: bridge_camera.sh [--ros-topic /camera/image_raw] [--gz-topic <topic>]
Auto-discovers a Gazebo image topic (preferring x500_mono_cam) and starts the
ros_gz_image image_bridge.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-topic) ROS_TOPIC="$2"; shift 2 ;;
    --gz-topic)  GZ_TOPIC_OVERRIDE="$2"; shift 2 ;;
    --info-topic) INFO_TOPIC_OVERRIDE="$2"; shift 2 ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "[!] Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

SETUP_FILE="$WS_DIR/install/setup.bash"
if [[ ! -f "$SETUP_FILE" ]]; then
  echo "[!] Workspace not built. Run 'colcon build --symlink-install' first." >&2
  exit 1
fi

# shellcheck disable=SC1090
if [[ $- == *u* ]]; then
  set +u
  _RESTORE_U=1
else
  _RESTORE_U=0
fi
source "$SETUP_FILE"
if [[ $_RESTORE_U -eq 1 ]]; then
  set -u
fi
unset _RESTORE_U

wait_for_image_topics() {
  local attempts=0
  local max_attempts=30
  topics=()
  while (( attempts < max_attempts )); do
    echo "[*] Scanning Gazebo topics for images (attempt $((attempts+1))/${max_attempts})..." >&2
    mapfile -t all_topics < <(gz topic -l 2>/dev/null || true)
    topics=()
    for topic in "${all_topics[@]}"; do
      info="$(gz topic -i -t "$topic" 2>/dev/null || true)"
      if echo "$info" | grep -q "gz.msgs.Image"; then
        topics+=("$topic")
      fi
    done
    if (( ${#topics[@]} > 0 )); then
      printf '[+] Found Gazebo image topics: %s\n' "${topics[*]}" >&2
      return 0
    fi
    sleep 2
    ((attempts++))
  done
  return 1
}

if [[ -n "$GZ_TOPIC_OVERRIDE" ]]; then
  SELECTED_TOPIC="$GZ_TOPIC_OVERRIDE"
else
  if ! command -v gz >/dev/null 2>&1; then
    echo "[!] 'gz' CLI not found in PATH" >&2
    exit 1
  fi
  if ! wait_for_image_topics; then
    echo "[!] No Gazebo image topics detected after waiting." >&2
    exit 1
  fi
  candidates=()
  for topic in "${topics[@]}"; do
    candidates+=("$topic")
  done

  preferred=()
  for topic in "${candidates[@]}"; do
    if [[ "$topic" == *"x500"* ]]; then
      preferred+=("$topic")
    fi
  done

  if (( ${#preferred[@]} == 1 )); then
    SELECTED_TOPIC="${preferred[0]}"
  elif (( ${#preferred[@]} > 1 )); then
    SELECTED_TOPIC="${preferred[0]}"
    echo "[=] Multiple x500 image topics found; choosing ${SELECTED_TOPIC}" >&2
  else
    if (( ${#candidates[@]} == 1 )); then
      SELECTED_TOPIC="${candidates[0]}"
    else
      echo "[?] Multiple image topics detected:" >&2
      for idx in "${!candidates[@]}"; do
        printf '  [%d] %s\n' "$idx" "${candidates[idx]}" >&2
      done
      read -rp "Select topic index: " choice
      if [[ ! "$choice" =~ ^[0-9]+$ ]] || [[ -z "${candidates[choice]:-}" ]]; then
        echo "[!] Invalid selection" >&2
        exit 1
      fi
      SELECTED_TOPIC="${candidates[choice]}"
    fi
  fi
fi

CAMERA_INFO_TOPIC=""
if [[ -n "$INFO_TOPIC_OVERRIDE" ]]; then
  CAMERA_INFO_TOPIC="$INFO_TOPIC_OVERRIDE"
else
  for topic in "${all_topics[@]}"; do
    if [[ "$topic" == *"camera_info"* ]]; then
      CAMERA_INFO_TOPIC="$topic"
      break
    fi
  done
fi

echo "[+] Bridging Gazebo topic '${SELECTED_TOPIC}' to ROS topic '${ROS_TOPIC}'"

BRIDGE_CMD=(ros2 run ros_gz_bridge parameter_bridge)
BRIDGE_CMD+=("${SELECTED_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image")

if [[ -n "$CAMERA_INFO_TOPIC" ]]; then
  BRIDGE_CMD+=("${CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
fi

BRIDGE_CMD+=(--ros-args --remap "${SELECTED_TOPIC}:=${ROS_TOPIC}")

if [[ -n "$CAMERA_INFO_TOPIC" ]]; then
  BASE_PATH="${ROS_TOPIC%/*}"
  if [[ -z "$BASE_PATH" || "$BASE_PATH" == "$ROS_TOPIC" ]]; then
    ROS_INFO_TOPIC="${ROS_TOPIC}_info"
  else
    ROS_INFO_TOPIC="${BASE_PATH}/camera_info"
  fi
  echo "[+] Bridging camera info '${CAMERA_INFO_TOPIC}' -> '${ROS_INFO_TOPIC}'"
  BRIDGE_CMD+=(--remap "${CAMERA_INFO_TOPIC}:=${ROS_INFO_TOPIC}")
fi

exec "${BRIDGE_CMD[@]}"
