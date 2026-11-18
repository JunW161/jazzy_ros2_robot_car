#!/usr/bin/env bash
# Usage: ./start_ld14p_wifi.sh <ESP32_IP> [TCP_PORT]
# Requires: Ubuntu ROS 2 Jazzy, previously built ldlidar_sl_ros2, socat.
set -Eeuo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <ESP32_IP> [TCP_PORT, default 7777]" >&2
  exit 2
fi
ESP_IP="$1"
ESP_PORT="${2:-7777}"
PTY_LINK="/tmp/ld14p_wifi"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_PID=""
SOCAT_PID=""

command -v socat >/dev/null || { echo 'Missing socat. Run: sudo apt install socat' >&2; exit 1; }
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Missing ROS 2 Jazzy setup' >&2; exit 1; }
[[ -f "$HOME/ldlidar_ros2_ws/install/setup.bash" ]] || {
  echo 'Missing ~/ldlidar_ros2_ws/install/setup.bash' >&2; exit 1;
}
# ROS shell setup files use optional variables and are not safe under nounset.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ldlidar_ros2_ws/install/setup.bash"
set -u

if [[ -e "$PTY_LINK" && ! -L "$PTY_LINK" ]]; then
  echo "Refusing to overwrite non-symlink $PTY_LINK" >&2
  exit 1
fi

stop_children() {
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    kill "$ROS_PID" 2>/dev/null || true
  fi
  if [[ -n "$SOCAT_PID" ]] && kill -0 "$SOCAT_PID" 2>/dev/null; then
    kill "$SOCAT_PID" 2>/dev/null || true
  fi
  [[ -z "$ROS_PID" ]] || wait "$ROS_PID" 2>/dev/null || true
  [[ -z "$SOCAT_PID" ]] || wait "$SOCAT_PID" 2>/dev/null || true
  ROS_PID=""
  SOCAT_PID=""
  [[ ! -L "$PTY_LINK" ]] || rm -f "$PTY_LINK"
}
trap 'stop_children' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

while true; do
  stop_children
  echo "[bridge] Connecting $ESP_IP:$ESP_PORT -> $PTY_LINK"
  socat -d -d \
    "PTY,link=$PTY_LINK,raw,echo=0,mode=660" \
    "TCP4:$ESP_IP:$ESP_PORT,connect-timeout=5,keepalive" &
  SOCAT_PID="$!"
  available=0
  for _ in {1..60}; do
    if ! kill -0 "$SOCAT_PID" 2>/dev/null; then break; fi
    if [[ -L "$PTY_LINK" && -e "$PTY_LINK" ]]; then
      available=1
      break
    fi
    sleep 0.1
  done
  if [[ "$available" -eq 1 ]] && kill -0 "$SOCAT_PID" 2>/dev/null; then
    echo "[bridge] Virtual serial online: $PTY_LINK"
    ros2 launch "$HERE/ld14p_wifi.launch.py" serial_port:="$PTY_LINK" \
      laser_x:="${LASER_X:-0.0}" laser_y:="${LASER_Y:-0.0}" \
      laser_z:="${LASER_Z:-0.18}" laser_yaw:="${LASER_YAW:-0.0}" &
    ROS_PID="$!"
    wait -n "$SOCAT_PID" "$ROS_PID" || true
  else
    echo "[bridge] No connection to ESP32; retrying..." >&2
  fi
  stop_children
  sleep 2
done
