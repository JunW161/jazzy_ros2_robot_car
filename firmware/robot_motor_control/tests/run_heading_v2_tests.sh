#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -Imain tests/test_heading_guard_v2.cpp -o "$tmp/heading_guard"
"$tmp/heading_guard"
# Check the actual embedded sources compile with existing ESP-IDF stubs.
g++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Itests/stubs -Imain main/robot_drive.cpp
g++ -std=c++17 -Wall -Wextra -Werror -fsyntax-only -Itests/stubs -Imain main/robot_sensors.cpp
bash tests/run_heading_pid_tests.sh
python3 tests/test_heading_v2_contract.py
