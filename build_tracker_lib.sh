#!/bin/bash
# ============================================================
# 编译跟踪器库的快捷脚本
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/tracker_lib/build"

echo "[INFO] Building tracker library in: ${BUILD_DIR}"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)

echo ""
echo "[INFO] Build complete!"
echo "[INFO] Library: ${BUILD_DIR}/lib/libtracker.so"
