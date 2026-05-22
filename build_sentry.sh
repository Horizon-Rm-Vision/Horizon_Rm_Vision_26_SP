#!/bin/bash
# ============================================================
# 一键编译脚本 - 哨兵
# OpenVINO + 哨兵 + Buff + 无全向感知 + 无Test + 无标定
# ============================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

echo "=============================================="
echo "  哨兵 (Sentry) 编译脚本"
echo "  推理引擎: OpenVINO"
echo "  哨兵: 启用 | Buff: 启用 | 全向感知: 关闭"
echo "=============================================="

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
    -DUSE_OPENVINO=ON \
    -DUSE_CUDA=OFF \
    -DBUILD_SENTRY=ON \
    -DBUILD_OMNI=OFF \
    -DBUILD_BUFF=ON \
    -DBUILD_TEST=OFF \
    -DBUILD_CALIBRATION=OFF \
    -DBUILD_ALL=OFF

make -j$(nproc)

echo ""
echo "=============================================="
echo "  哨兵 (Sentry) 编译完成!"
echo "=============================================="

# 编译完成后进入 build 文件夹
cd "${BUILD_DIR}"
exec bash