#!/usr/bin/env bash
# Called with an argv array, NOT a browser-supplied shell string.
set -eo pipefail
unset VIRTUAL_ENV PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Thiếu ROS 2 Jazzy.' >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
WS="${ROBOT_WEB_WORKSPACE:-$HOME/ros2_ws}"
for setup in "$HOME/micro_ros_ws/install/local_setup.bash" /uros_ws/install/local_setup.bash "$HOME/ldlidar_ros2_ws/install/local_setup.bash" "$WS/install/local_setup.bash"; do
  if [[ -f "$setup" ]]; then source "$setup"; fi
done
[[ $# -gt 0 ]] || exit 2
exec "$@"
