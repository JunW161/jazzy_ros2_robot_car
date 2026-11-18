#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror "$here/test_v7_policy.cpp" -o "$out/test_v7_policy"
"$out/test_v7_policy"
# Host mocks only: verify VL6180X register protocol and init/write sequence.
g++ -std=c++17 -Wall -Wextra -I "$here/stubs" \
  -I "$here/../main" "$here/test_vl6180x_stub.cpp" "$here/../main/vl6180x_device.cpp" "$here/../main/sensor_bus.cpp" -o "$out/vl6180_mock"
"$out/vl6180_mock"
bash "$here/run_host_tests.sh"
echo 'V7 pure-math tests PASSED (hardware not verified)'
