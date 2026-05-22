#!/usr/bin/env bash
# Verify the lidar UART test binary has no missing runtime dependencies.

set -euo pipefail

readonly BINARY="${LIDAR_TEST_BINARY:-}"

if [[ -z "${BINARY}" ]]; then
    echo "environment variable LIDAR_TEST_BINARY is not set" >&2
    exit 1
fi

if [[ ! -x "${BINARY}" ]]; then
    echo "missing executable binary: ${BINARY}" >&2
    exit 1
fi

ldd_output="$(ldd "${BINARY}")"
printf '%s\n' "${ldd_output}"

if [[ "${ldd_output}" == *"not found"* ]]; then
    echo "found unresolved runtime library dependency" >&2
    exit 1
fi

echo "lidar runtime dependency check passed"