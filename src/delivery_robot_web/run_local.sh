#!/usr/bin/env bash
# The only daily launcher: source ROS and known overlays, then run one local server.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROBOT_WEB_WS:-$(cd "$HERE/../.." && pwd)}"
if [[ "$EUID" -eq 0 ]]; then echo 'Không chạy web bằng sudo.' >&2; exit 1; fi
unset VIRTUAL_ENV PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
source /opt/ros/jazzy/setup.bash
# Agent may be in /uros_ws on this machine; do not build or change it.
for setup in "$HOME/micro_ros_ws/install/local_setup.bash" /uros_ws/install/local_setup.bash "$HOME/ldlidar_ros2_ws/install/local_setup.bash"; do
  [[ ! -f "$setup" ]] || source "$setup"
done
if [[ ! -f "$WS/install/local_setup.bash" ]]; then
  echo "Chưa có $WS/install/local_setup.bash. Chạy bộ cài/build package trước." >&2; exit 1
fi
source "$WS/install/local_setup.bash"
if [[ $# -eq 0 ]]; then
  exec ros2 run delivery_robot_web web serve
else
  exec ros2 run delivery_robot_web web "$@"
fi
