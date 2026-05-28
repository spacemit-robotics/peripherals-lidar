#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/peripherals/lidar/${SROBOTIS_TEST_NAME:-lidar-hardware-uart-smoke}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/lidar_hardware_uart_smoke.log"
build_dir="$artifact_dir/build"

: "${LIDAR_TEST_MODEL:?Set LIDAR_TEST_MODEL to the lidar model name, e.g. YDLIDAR or RPLIDAR}"
: "${LIDAR_TEST_DEV_PATH:?Set LIDAR_TEST_DEV_PATH to the lidar serial device path, e.g. /dev/ttyUSB0}"

lidar_test_baud="${LIDAR_TEST_BAUD:-115200}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] model=$LIDAR_TEST_MODEL dev=$LIDAR_TEST_DEV_PATH baud=$lidar_test_baud"

    cmake -S "$module_root" -B "$build_dir" \
        -DLIDAR_BUILD_TESTS=ON

    cmake --build "$build_dir" -j"$(nproc)"

    "$build_dir/test_lidar_uart" "$LIDAR_TEST_MODEL" "$LIDAR_TEST_DEV_PATH" "$lidar_test_baud"
} | tee "$log_file"

grep -q "Test completed" "$log_file"
grep -Eq "Total frames received: [1-9][0-9]*" "$log_file"