#ifndef AUTO_AIM__YOLOV5_TRT_GPU_KERNEL_CUH
#define AUTO_AIM__YOLOV5_TRT_GPU_KERNEL_CUH

#include <cuda_runtime.h>

namespace auto_aim {
namespace gpu_kernel {

/**
 * @brief GPU sigmoid激活函数kernel
 * @param d_input 输入张量
 * @param d_output 输出张量
 * @param size 数据大小
 */
__global__ void sigmoid_kernel(const float* d_input, float* d_output, int size);

/**
 * @brief GPU解析检测结果kernel - 第一阶段：计算置信度并筛选
 * @param d_output TensorRT输出 (output_rows x output_cols)
 * @param output_rows 输出行数
 * @param output_cols 输出列数
 * @param score_threshold 置信度阈值
 * @param scale 缩放比例
 * @param pad_x X方向填充
 * @param pad_y Y方向填充
 * @param d_valid_indices 有效检测索引输出
 * @param d_valid_count 有效检测计数
 */
__global__ void parse_detections_stage1_kernel(
    const float* d_output, 
    int output_rows, 
    int output_cols,
    float score_threshold,
    float scale,
    int pad_x,
    int pad_y,
    int* d_valid_indices,
    int* d_valid_count
);

/**
 * @brief GPU解析检测结果kernel - 第二阶段：计算边界框和关键点
 * @param d_output TensorRT输出
 * @param output_rows 输出行数
 * @param output_cols 输出列数
 * @param d_valid_indices 有效检测索引
 * @param num_valid 有效检测数量
 * @param scale 缩放比例
 * @param pad_x X方向填充
 * @param pad_y Y方向填充
 * @param d_color_ids 颜色ID输出 (num_valid)
 * @param d_num_ids 类别ID输出 (num_valid)
 * @param d_confidences 置信度输出 (num_valid)
 * @param d_boxes 边界框输出 (num_valid x 4: x,y,w,h)
 * @param d_keypoints 关键点输出 (num_valid x 8: 4个点的x,y坐标)
 */
__global__ void parse_detections_stage2_kernel(
    const float* d_output,
    int output_rows,
    int output_cols,
    const int* d_valid_indices,
    int num_valid,
    float scale,
    int pad_x,
    int pad_y,
    int* d_color_ids,
    int* d_num_ids,
    float* d_confidences,
    int* d_boxes,
    float* d_keypoints
);

/**
 * @brief GPU NMS kernel - 排序阶段
 * @param d_boxes 所有边界框 (count x 4)
 * @param d_confidences 所有置信度 (count)
 * @param count 边界框总数
 * @param d_sorted_indices 排序后的索引输出
 */
__global__ void nms_prepare_kernel(
    const int* d_boxes,
    const float* d_confidences,
    int count,
    int* d_sorted_indices
);

/**
 * @brief GPU NMS kernel - 计算IoU
 * @param d_boxes 所有边界框
 * @param d_valid_mask 有效掩码
 * @param count 边界框总数
 * @param iou_threshold IoU阈值
 */
__global__ void nms_iou_kernel(
    const int* d_boxes,
    int* d_valid_mask,
    int count,
    float iou_threshold
);

/**
 * @brief 计算两个边界框的IoU
 */
__device__ inline float compute_iou(
    const int* box1, 
    const int* box2
);

/**
 * @brief 辅助函数：在GPU上进行并行扫描（前缀和）
 * @param d_input 输入数据
 * @param d_output 输出数据
 * @param count 数据总数
 */
void parallel_scan(
    const int* d_input,
    int* d_output,
    int count,
    cudaStream_t stream = 0
);

/**
 * @brief GPU版本的后处理主函数
 * @param d_output TensorRT推理输出
 * @param output_rows 输出行数
 * @param output_cols 输出列数
 * @param score_threshold 置信度阈值
 * @param nms_threshold NMS阈值
 * @param scale 缩放比例
 * @param pad_x X方向填充
 * @param pad_y Y方向填充
 * @param d_color_ids 颜色ID输出
 * @param d_num_ids 类别ID输出
 * @param d_confidences 置信度输出
 * @param d_boxes 边界框输出
 * @param d_keypoints 关键点输出
 * @param d_valid_count 有效检测计数输出
 * @param stream CUDA流
 * @return 返回有效检测的数量
 */
int gpu_parse_detections_nms(
    const float* d_output,
    int output_rows,
    int output_cols,
    float score_threshold,
    float nms_threshold,
    float scale,
    int pad_x,
    int pad_y,
    int* d_color_ids,
    int* d_num_ids,
    float* d_confidences,
    int* d_boxes,
    float* d_keypoints,
    int* d_valid_count,
    cudaStream_t stream = 0
);

/**
 * @brief 从GPU内存复制数据到CPU
 */
void copy_detections_to_host(
    const int* d_color_ids,
    const int* d_num_ids,
    const float* d_confidences,
    const int* d_boxes,
    const float* d_keypoints,
    int num_detections,
    int* h_color_ids,
    int* h_num_ids,
    float* h_confidences,
    int* h_boxes,
    float* h_keypoints
);

}  // namespace gpu_kernel
}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_GPU_KERNEL_CUH
