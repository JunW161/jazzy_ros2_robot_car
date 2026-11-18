#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
TD="$(mktemp -d)";trap 'rm -rf "$TD"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_heading_pid_v1.cpp -o "$TD/heading_pid"
"$TD/heading_pid"
c++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Itests/stubs -Imain main/robot_drive.cpp
python3 tests/test_heading_pid_contract.py
python3 -m py_compile host_ros2/heading_pid_monitor.py
