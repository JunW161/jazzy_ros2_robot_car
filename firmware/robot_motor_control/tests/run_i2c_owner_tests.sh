#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -I "$here/stubs" -I "$here/../main" \
 "$here/test_i2c_steps.cpp" "$here/../main/sensor_bus.cpp" \
 "$here/../main/mpu6500_device.cpp" "$here/../main/vl6180x_device.cpp" \
 -o "$out/test_i2c_steps"
"$out/test_i2c_steps"
"$here/run_v7_host_tests.sh"
