#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
INPUT_DIR="${ROOT_DIR}/input"
OUTPUT_DIR="${ROOT_DIR}/output"
TARGET="${BUILD_DIR}/uav_scheduler"

if [[ ! -d "${INPUT_DIR}" ]]; then
    echo "Input directory not found: ${INPUT_DIR}" >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}"

mkdir -p "${OUTPUT_DIR}"

if [[ ! -x "${TARGET}" ]]; then
    echo "Expected executable not found: ${TARGET}" >&2
    exit 1
fi

(cd "${BUILD_DIR}" && ./uav_scheduler)
