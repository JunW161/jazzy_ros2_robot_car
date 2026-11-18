#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
bash tests/run_i2c_owner_tests.sh
c++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_v8_interlocks.cpp -o /tmp/test_v8_interlocks
/tmp/test_v8_interlocks
echo "Checking motor/FF/PID C++ syntax against ESP-IDF test stubs..."
c++ -std=c++17 -fsyntax-only -Wall -Wextra -Werror -Itests/stubs -Imain main/robot_drive.cpp
python3 -m py_compile host_ros2/diagnose_v8.py host_ros2/bench_wheel_test_v8.py
python3 tests/test_v8_source_contract.py
printf 'V8 host/source tests PASSED. Hardware and ESP-IDF build remain UNVERIFIED.\n'
