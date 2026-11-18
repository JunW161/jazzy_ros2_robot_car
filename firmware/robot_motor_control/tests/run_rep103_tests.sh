#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Independent tests: do not run old V7/V8 yaw policy tests after migration.
TD="$(mktemp -d)"
trap 'rm -rf "$TD"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -Itests/stubs -Imain \
    tests/test_i2c_steps.cpp main/sensor_bus.cpp main/mpu6500_device.cpp \
    main/vl6180x_device.cpp -o "$TD/i2c"
"$TD/i2c"
g++ -std=c++17 -Wall -Wextra -Werror -O2 -Imain tests/test_controller_math.cpp -o "$TD/math"
"$TD/math"
g++ -std=c++17 -Wall -Wextra -Werror -O2 -Imain tests/test_kinematics.cpp -o "$TD/kinematics"
"$TD/kinematics"
g++ -std=c++17 -Wall -Wextra -Werror -O2 tests/test_rep103_path_controller.cpp -o "$TD/path"
"$TD/path"
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Itests/stubs -Imain main/robot_drive.cpp
python3 tests/test_rep103_source_contract.py
python3 -m py_compile host_ros2/safe_cmd_vel.py host_ros2/bench_wheel_test_v8.py host_ros2/trajectory_monitor_rep103.py
printf 'REP103 driver/math/motion-source tests PASS. ESP-IDF hardware build UNVERIFIED.\n'
