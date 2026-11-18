#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
g++ -std=c++17 -Wall -Wextra -Werror -I main tests/test_v6_policy.cpp -o /tmp/robot_v6_policy_test
/tmp/robot_v6_policy_test
bash tests/run_host_tests.sh
python3 tests/test_pose_math.py
