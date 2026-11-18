#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
TEMP="$(mktemp -d)"
trap 'rm -rf "$TEMP"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_robot_delivery_main_outer.cpp -o "$TEMP/outer"
"$TEMP/outer"
bash tests/run_heading_pid_tests.sh
bash tests/run_rep103_tests.sh
bash tests/run_v831_tests.sh
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_drive.cpp
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_sensors.cpp
python3 -m py_compile host_ros2/heading_pid_monitor.py host_ros2/heading_pid_log.py
printf 'Robot_Delivery_Main OFFLINE TESTS PASS; physical ESP-IDF hardware build NOT verified.\n'
