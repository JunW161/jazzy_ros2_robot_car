#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v12_heading.cpp -o "$T/test_v12"
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v12_imu.cpp -o "$T/test_imu"
"$T/test_imu"
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v12_ff.cpp -o "$T/test_ff"
"$T/test_ff"
"$T/test_v12"
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v12_imu.cpp -o "$T/test_imu"
"$T/test_imu"
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v12_ff.cpp -o "$T/test_ff"
"$T/test_ff"
c++ -std=c++17 -O2 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_drive.cpp
c++ -std=c++17 -O2 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_sensors.cpp
python3 tests/test_v12_source.py
python3 -m py_compile host_ros2/heading_toggle_v12.py host_ros2/heading_monitor_v12.py
printf 'V12 OFFLINE TESTS PASS. ESP-IDF hardware build not verified.\n'
