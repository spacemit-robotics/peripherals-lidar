#!/usr/bin/env bash
# Verify the lidar UART test binary prints help repeatedly without failures.

set -euo pipefail

readonly BINARY="${LIDAR_TEST_BINARY:-}"
readonly REPEAT_COUNT=20

if [[ -z "${BINARY}" ]]; then
    echo "environment variable LIDAR_TEST_BINARY is not set" >&2
    exit 1
fi

if [[ ! -x "${BINARY}" ]]; then
    echo "missing executable binary: ${BINARY}" >&2
    exit 1
fi

for ((i = 1; i <= REPEAT_COUNT; i++)); do
    help_output="$(${BINARY} --help)"

    if [[ "${help_output}" != *"Usage:"* ]]; then
        echo "iteration ${i}: expected help output to contain 'Usage:'" >&2
        exit 1
    fi
done

echo "lidar repeated help stability test passed (${REPEAT_COUNT} runs)"