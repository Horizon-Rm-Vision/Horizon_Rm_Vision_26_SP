#!/bin/bash
# ============================================================
# 一键编译脚本 - 全向步兵/飞机
# TensorRT + 无哨兵 + 无Buff + 无全向感知 + 无Test + 无标定
# ============================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

echo "=============================================="
echo "  全向步兵/飞机 (Omni Infantry/Drone) 编译脚本"
echo "  推理引擎: TensorRT (CUDA)"
echo "  哨兵: 关闭 | Buff: 关闭 | 全向感知: 关闭"
echo "=============================================="

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
    -DUSE_OPENVINO=OFF \
    -DUSE_CUDA=ON \
    -DBUILD_SENTRY=OFF \
    -DBUILD_OMNI=OFF \
    -DBUILD_BUFF=OFF \
    -DBUILD_TEST=OFF \
    -DBUILD_CALIBRATION=OFF \
    -DBUILD_ALL=OFF

make -j$(nproc)

echo ""
echo "=============================================="
echo "  全向步兵/飞机 (Omni Infantry/Drone) 编译完成!"
echo "  可执行文件: ${BUILD_DIR}/auto_aim_debug_mpc"
echo "=============================================="

# 编译完成后进入 build 文件夹
cd "${BUILD_DIR}"
exec bash