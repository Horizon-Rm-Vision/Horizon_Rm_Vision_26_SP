#!/bin/bash

# GPU优化性能测试脚本
# 用途：验证GPU优化的性能提升

set -e

PROJECT_DIR="/home/ad/Horizon_Rm_Vision_26_SP"
BUILD_DIR="${PROJECT_DIR}/build"
EXECUTABLE="${BUILD_DIR}/standard"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查依赖
check_dependencies() {
    echo -e "${BLUE}[检查依赖]${NC}"
    
    if ! command -v nvidia-smi &> /dev/null; then
        echo -e "${RED}✗ nvidia-smi not found${NC}"
        return 1
    fi
    
    if ! command -v nsys &> /dev/null; then
        echo -e "${YELLOW}⚠ nsys not found (性能分析跳过)${NC}"
        NSYS_AVAILABLE=0
    else
        NSYS_AVAILABLE=1
    fi
    
    if [ ! -f "$EXECUTABLE" ]; then
        echo -e "${RED}✗ Executable not found: $EXECUTABLE${NC}"
        return 1
    fi
    
    echo -e "${GREEN}✓ 所有依赖检查完成${NC}\n"
    return 0
}

# 获取GPU信息
get_gpu_info() {
    echo -e "${BLUE}[GPU 信息]${NC}"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -1
    echo ""
}

# 检查CUDA内存
check_cuda_memory() {
    echo -e "${BLUE}[CUDA内存检查]${NC}"
    nvidia-smi --query-gpu=memory.free,memory.used --format=csv,noheader | head -1
    echo ""
}

# 运行性能测试
run_performance_test() {
    echo -e "${BLUE}[运行性能测试]${NC}"
    echo "测试时长: 30秒，多帧测试..."
    
    # 在后台运行程序并监控GPU使用
    {
        nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader -l 1 > /tmp/gpu_stats.txt &
        GPU_MONITOR_PID=$!
        
        sleep 2  # 等待程序启动
        
        # 运行程序（假设有30秒的测试输入）
        timeout 30 "$EXECUTABLE" > /tmp/detection_output.txt 2>&1 || true
        
        kill $GPU_MONITOR_PID 2>/dev/null || true
    }
    
    # 分析GPU统计
    if [ -f /tmp/gpu_stats.txt ]; then
        AVG_UTIL=$(awk -F',' '{sum+=$1; count++} END {print int(sum/count)}' /tmp/gpu_stats.txt)
        MAX_MEM=$(awk -F',' '{gsub(/[^0-9]/, ""); print $2}' /tmp/gpu_stats.txt | sort -n | tail -1)
        
        echo -e "${GREEN}平均GPU利用率: ${AVG_UTIL}%${NC}"
        echo -e "${GREEN}峰值GPU内存: ${MAX_MEM}MB${NC}"
    fi
    
    # 从检测输出提取性能信息
    if [ -f /tmp/detection_output.txt ]; then
        grep -E "(Detection|detection|Time|time|ms)" /tmp/detection_output.txt | tail -10
    fi
    echo ""
}

# 使用nsys进行详细性能分析
run_nsys_profile() {
    if [ $NSYS_AVAILABLE -eq 0 ]; then
        return
    fi
    
    echo -e "${BLUE}[NVIDIA Nsys 性能分析]${NC}"
    echo "收集性能数据（约30秒）..."
    
    cd "$BUILD_DIR"
    nsys profile -o gpu_profile -w true -t cuda,cudnn,cublas \
        timeout 30 "$EXECUTABLE" > /dev/null 2>&1 || true
    
    echo -e "${GREEN}✓ 性能数据已收集: gpu_profile.nsys-rep${NC}"
    echo "查看报告:"
    echo "  nsys stats gpu_profile.nsys-rep"
    echo ""
}

# 比较GPU利用率
compare_gpu_usage() {
    echo -e "${BLUE}[GPU利用率对比]${NC}"
    echo "监控10秒的GPU状态..."
    
    for i in {1..10}; do
        nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader | head -1
        sleep 1
    done
    echo ""
}

# 内存检查
memory_check() {
    echo -e "${BLUE}[GPU内存详细检查]${NC}"
    nvidia-smi --query-gpu=memory.total,memory.reserved,memory.free,memory.used \
               --format=csv,noheader,unit=MB | head -1
    echo ""
}

# 编译检查
check_build() {
    echo -e "${BLUE}[编译检查]${NC}"
    
    if [ ! -f "${BUILD_DIR}/CMakeCache.txt" ]; then
        echo -e "${YELLOW}⚠ 需要重新编译${NC}"
        cd "$PROJECT_DIR"
        rm -rf build
        mkdir -p build && cd build
        cmake .. > /dev/null 2>&1
        cmake --build . -j4 > /dev/null 2>&1
        echo -e "${GREEN}✓ 编译完成${NC}"
    else
        echo -e "${GREEN}✓ 已编译${NC}"
    fi
    echo ""
}

# 验证CUDA kernel编译
verify_cuda_compilation() {
    echo -e "${BLUE}[验证CUDA Kernel编译]${NC}"
    
    CUDA_OBJ="${BUILD_DIR}/tasks/auto_aim/CMakeFiles/auto_aim.dir/yolos/yolov5_trt_gpu_kernel.cu.o"
    
    if [ -f "$CUDA_OBJ" ]; then
        SIZE=$(stat -f%z "$CUDA_OBJ" 2>/dev/null || stat -c%s "$CUDA_OBJ")
        SIZE_MB=$(echo "scale=2; $SIZE / 1024 / 1024" | bc)
        echo -e "${GREEN}✓ CUDA kernel 已编译 (${SIZE_MB}MB)${NC}"
    else
        echo -e "${RED}✗ CUDA kernel 编译失败${NC}"
        return 1
    fi
    echo ""
}

# 主函数
main() {
    echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║  GPU优化性能测试脚本                    ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${NC}\n"
    
    # 执行各项检查和测试
    check_dependencies || exit 1
    get_gpu_info
    check_cuda_memory
    
    echo -e "${BLUE}[构建检查]${NC}"
    check_build
    
    echo -e "${BLUE}[编译验证]${NC}"
    verify_cuda_compilation || exit 1
    
    echo -e "${BLUE}[内存分析]${NC}"
    memory_check
    
    echo -e "${BLUE}[GPU状态监控]${NC}"
    compare_gpu_usage
    
    echo -e "${BLUE}[性能测试]${NC}"
    run_performance_test
    
    # 如果nsys可用，进行详细分析
    if [ $NSYS_AVAILABLE -eq 1 ]; then
        run_nsys_profile
    fi
    
    echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  测试完成！                            ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════╝${NC}\n"
    
    # 打印建议
    echo -e "${YELLOW}[建议]${NC}"
    echo "1. 检查GPU利用率是否 > 80%"
    echo "2. 查看内存使用是否在预期范围内"
    echo "3. 运行 'nsys stats gpu_profile.nsys-rep' 获取详细信息"
    echo "4. 对比CPU版本性能提升"
    echo ""
}

# 处理参数
case "${1:-}" in
    --help|-h)
        echo "用法: $0 [选项]"
        echo "选项:"
        echo "  --deps     仅检查依赖"
        echo "  --gpu      查看GPU信息"
        echo "  --build    仅编译"
        echo "  --profile  运行完整性能分析（需要nsys）"
        echo "  --help     显示此帮助信息"
        ;;
    --deps)
        check_dependencies
        ;;
    --gpu)
        get_gpu_info
        memory_check
        compare_gpu_usage
        ;;
    --build)
        check_build
        verify_cuda_compilation
        ;;
    --profile)
        check_dependencies || exit 1
        check_build
        run_nsys_profile
        ;;
    *)
        main
        ;;
esac
