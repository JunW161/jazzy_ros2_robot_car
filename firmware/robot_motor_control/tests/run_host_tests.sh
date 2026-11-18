#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p /tmp/robot_motor_control_tests
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/test_controller_math.cpp -o /tmp/robot_motor_control_tests/controller
/tmp/robot_motor_control_tests/controller
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/test_kinematics.cpp -o /tmp/robot_motor_control_tests/kinematics
/tmp/robot_motor_control_tests/kinematics
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m compileall -q host_ros2
echo 'PASS: host math/Python tests; real ESP-IDF build/hardware tests still required.'
