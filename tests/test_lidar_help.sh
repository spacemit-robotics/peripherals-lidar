#!/usr/bin/env bash
# Verify the lidar UART test binary prints the expected help output.

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

help_output="$(${BINARY} --help)"
printf '%s\n' "${help_output}"

if [[ "${help_output}" != *"Usage:"* ]]; then
    echo "expected help output to contain 'Usage:'" >&2
    exit 1
fi

if [[ "${help_output}" != *"MODEL [DEV_PATH [BAUD]]"* ]]; then
    echo "expected help output to describe positional arguments" >&2
    exit 1
fi

if [[ "${help_output}" != *"registered driver name"* ]]; then
    echo "expected help output to mention registered driver name" >&2
    exit 1
fi

echo "lidar help functional test passed"