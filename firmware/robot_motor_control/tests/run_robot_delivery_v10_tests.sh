#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
TEMP="$(mktemp -d)"
trap 'rm -rf "$TEMP"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v10_imu.cpp -o "$TEMP/imu"; "$TEMP/imu"
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v10_ff.cpp -o "$TEMP/ff"; "$TEMP/ff"
python3 tests/test_v10_contract.py
bash tests/run_robot_delivery_main_tests.sh
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_drive.cpp
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Imain -Itests/stubs main/robot_sensors.cpp
python3 -m py_compile host_ros2/ff_tune_v10.py
printf 'V10 OFFLINE PASS; full ESP-IDF compilation and real robot test still required.\n'
