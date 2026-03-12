#include "yolov5_trt_gpu_kernel.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>
#include <vector>
#include <opencv2/opencv.hpp>

namespace auto_aim {
namespace gpu_kernel {

// ======================= Sigmoid Kernel =======================
__global__ void sigmoid_kernel(const float* d_input, float* d_output, int size)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        float x = d_input[idx];
        d_output[idx] = 1.0f / (1.0f + expf(-x));
    }
}

// ======================= 计算IoU =======================
__device__ inline float compute_iou(const int* box1, const int* box2)
{
    // box格式: x, y, w, h
    int x1_1 = box1[0];
    int y1_1 = box1[1];
    int w1 = box1[2];
    int h1 = box1[3];
    
    int x1_2 = box2[0];
    int y1_2 = box2[1];
    int w2 = box2[2];
    int h2 = box2[3];
    
    // 处理无效的边界框
    if (w1 <= 0 || h1 <= 0 || w2 <= 0 || h2 <= 0) return 0.0f;
    
    int x_min1 = x1_1;
    int y_min1 = y1_1;
    int x_max1 = x1_1 + w1;
    int y_max1 = y1_1 + h1;
    
    int x_min2 = x1_2;
    int y_min2 = y1_2;
    int x_max2 = x1_2 + w2;
    int y_max2 = y1_2 + h2;
    
    // 计算交集
    int inter_x_min = max(x_min1, x_min2);
    int inter_y_min = max(y_min1, y_min2);
    int inter_x_max = min(x_max1, x_max2);
    int inter_y_max = min(y_max1, y_max2);
    
    int inter_w = max(0, inter_x_max - inter_x_min);
    int inter_h = max(0, inter_y_max - inter_y_min);
    float inter_area = (float)inter_w * inter_h;
    
    // 计算并集
    float area1 = (float)w1 * h1;
    float area2 = (float)w2 * h2;
    float union_area = area1 + area2 - inter_area;
    
    if (union_area <= 0) return 0.0f;
    return inter_area / union_area;
}

// ======================= 第一阶段：筛选有效检测 =======================
__global__ void parse_detections_stage1_kernel(
    const float* d_output, 
    int output_rows, 
    int output_cols,
    float score_threshold,
    float scale,
    int pad_x,
    int pad_y,
    int* d_valid_indices,
    int* d_valid_count)
{
    int row_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (row_idx < output_rows) {
        // 读取置信度分数 (位置8)
        float score = d_output[row_idx * output_cols + 8];
        
        // 计算sigmoid
        score = 1.0f / (1.0f + expf(-score));
        
        // 判断是否超过阈值
        if (score >= score_threshold) {
            // 原子操作：计数+1，并获得这个检测的索引
            int idx = atomicAdd(d_valid_count, 1);
            if (idx < 1000) {  // 限制最多1000个检测
                d_valid_indices[idx] = row_idx;
            }
        }
    }
}

// ======================= 第二阶段：计算边界框和关键点 =======================
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
    float* d_keypoints)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < num_valid) {
        int row_idx = d_valid_indices[idx];
        const float* row_data = &d_output[row_idx * output_cols];
        
        // 1. 计算置信度
        float score = row_data[8];
        score = 1.0f / (1.0f + expf(-score));
        d_confidences[idx] = score;
        
        // 2. 计算颜色ID (位置9-12)
        // 这些是logits，我们直接取最大值，不需要softmax用于分类
        float max_color_score = row_data[9];
        int color_id = 0;
        #pragma unroll
        for (int i = 1; i < 4; i++) {
            float s = row_data[9 + i];
            if (s > max_color_score) {
                max_color_score = s;
                color_id = i;
            }
        }
        d_color_ids[idx] = color_id;
        
        // 3. 计算类别ID (位置13-21，共9个类别)
        float max_class_score = row_data[13];
        int class_id = 0;
        #pragma unroll
        for (int i = 1; i < 9; i++) {
            float s = row_data[13 + i];
            if (s > max_class_score) {
                max_class_score = s;
                class_id = i;
            }
        }
        d_num_ids[idx] = class_id;
        
        // 4. 计算关键点并转换坐标（保持与CPU版本一致的顺序）
        // 原始顺序：点1(0,1), 点4(6,7), 点3(4,5), 点2(2,3)
        float x1 = (row_data[0] - pad_x) / scale;
        float y1 = (row_data[1] - pad_y) / scale;
        float x4 = (row_data[6] - pad_x) / scale;
        float y4 = (row_data[7] - pad_y) / scale;
        float x3 = (row_data[4] - pad_x) / scale;
        float y3 = (row_data[5] - pad_y) / scale;
        float x2 = (row_data[2] - pad_x) / scale;
        float y2 = (row_data[3] - pad_y) / scale;
        
        // 存储8个浮点数：4个点的x,y坐标
        d_keypoints[idx * 8 + 0] = x1;
        d_keypoints[idx * 8 + 1] = y1;
        d_keypoints[idx * 8 + 2] = x4;
        d_keypoints[idx * 8 + 3] = y4;
        d_keypoints[idx * 8 + 4] = x3;
        d_keypoints[idx * 8 + 5] = y3;
        d_keypoints[idx * 8 + 6] = x2;
        d_keypoints[idx * 8 + 7] = y2;
        
        // 5. 计算边界框
        float min_x = fminf(fminf(x1, x4), fminf(x3, x2));
        float max_x = fmaxf(fmaxf(x1, x4), fmaxf(x3, x2));
        float min_y = fminf(fminf(y1, y4), fminf(y3, y2));
        float max_y = fmaxf(fmaxf(y1, y4), fmaxf(y3, y2));
        
        // 存储为 x, y, width, height (整数格式)
        d_boxes[idx * 4 + 0] = (int)min_x;
        d_boxes[idx * 4 + 1] = (int)min_y;
        d_boxes[idx * 4 + 2] = (int)(max_x - min_x);
        d_boxes[idx * 4 + 3] = (int)(max_y - min_y);
    }
}

// ======================= NMS准备 =======================
__global__ void nms_prepare_kernel(
    const int* d_boxes,
    const float* d_confidences,
    int count,
    int* d_sorted_indices)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        d_sorted_indices[idx] = idx;
    }
}

// ======================= NMS IoU计算 =======================
__global__ void nms_iou_kernel(
    const int* d_boxes,
    int* d_valid_mask,
    int count,
    float iou_threshold)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i < count && d_valid_mask[i] == 1) {
        const int* box_i = &d_boxes[i * 4];
        
        for (int j = i + 1; j < count; j++) {
            if (d_valid_mask[j] == 1) {
                const int* box_j = &d_boxes[j * 4];
                float iou = compute_iou(box_i, box_j);
                
                if (iou > iou_threshold) {
                    // 将IoU高的框标记为无效
                    atomicExch(&d_valid_mask[j], 0);
                }
            }
        }
    }
}

// ======================= 前缀和（简化版，不使用CUB） =======================
void parallel_scan(
    const int* d_input,
    int* d_output,
    int count,
    cudaStream_t stream)
{
    // 简化实现：直接复制输入到输出（前缀和在CPU上实现）
    cudaMemcpyAsync(d_output, d_input, sizeof(int) * count, cudaMemcpyDeviceToDevice, stream);
}

// ======================= GPU解析检测结果和NMS主函数 =======================
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
    cudaStream_t stream)
{
    // 临时GPU内存
    int* d_valid_indices = nullptr;
    int h_valid_count = 0;
    
    // 分配临时存储用于有效索引
    cudaMalloc(&d_valid_indices, sizeof(int) * 1000);
    
    // 初始化有效计数为0
    cudaMemset(d_valid_count, 0, sizeof(int));
    
    // 第一阶段：筛选有效检测
    int threads = 256;
    int blocks = (output_rows + threads - 1) / threads;
    parse_detections_stage1_kernel<<<blocks, threads, 0, stream>>>(
        d_output, output_rows, output_cols, score_threshold,
        scale, pad_x, pad_y, d_valid_indices, d_valid_count
    );
    cudaStreamSynchronize(stream);
    
    // 从GPU读取有效检测的数量
    cudaMemcpy(&h_valid_count, d_valid_count, sizeof(int), cudaMemcpyDeviceToHost);
    
    if (h_valid_count == 0) {
        cudaFree(d_valid_indices);
        return 0;
    }
    
    // 限制检测数量
    h_valid_count = min(h_valid_count, 1000);
    
    // 第二阶段：计算边界框和关键点
    blocks = (h_valid_count + threads - 1) / threads;
    parse_detections_stage2_kernel<<<blocks, threads, 0, stream>>>(
        d_output, output_rows, output_cols, d_valid_indices, h_valid_count,
        scale, pad_x, pad_y, d_color_ids, d_num_ids, d_confidences,
        d_boxes, d_keypoints
    );
    cudaStreamSynchronize(stream);
    
    // 将数据复制到CPU用于NMS处理
    std::vector<int> h_color_ids(h_valid_count);
    std::vector<int> h_num_ids(h_valid_count);
    std::vector<float> h_confidences(h_valid_count);
    std::vector<int> h_boxes(h_valid_count * 4);
    std::vector<float> h_keypoints(h_valid_count * 8);
    
    cudaMemcpy(h_color_ids.data(), d_color_ids, sizeof(int) * h_valid_count, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_num_ids.data(), d_num_ids, sizeof(int) * h_valid_count, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_confidences.data(), d_confidences, sizeof(float) * h_valid_count, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_boxes.data(), d_boxes, sizeof(int) * h_valid_count * 4, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_keypoints.data(), d_keypoints, sizeof(float) * h_valid_count * 8, cudaMemcpyDeviceToHost);
    
    // 在CPU上进行简单的NMS处理（与原始CPU版本兼容）
    std::vector<cv::Rect> cv_boxes(h_valid_count);
    for (int i = 0; i < h_valid_count; i++) {
        cv_boxes[i] = cv::Rect(h_boxes[i * 4 + 0], h_boxes[i * 4 + 1],
                               h_boxes[i * 4 + 2], h_boxes[i * 4 + 3]);
    }
    
    std::vector<int> indices;
    cv::dnn::NMSBoxes(cv_boxes, h_confidences, score_threshold, nms_threshold, indices);
    
    int final_count = indices.size();
    
    if (final_count == 0) {
        cudaFree(d_valid_indices);
        return 0;
    }
    
    // 重新排列保留下来的检测结果
    std::vector<int> temp_color_ids(final_count);
    std::vector<int> temp_num_ids(final_count);
    std::vector<float> temp_confidences(final_count);
    std::vector<int> temp_boxes(final_count * 4);
    std::vector<float> temp_keypoints(final_count * 8);
    
    for (int i = 0; i < final_count; i++) {
        int idx = indices[i];
        temp_color_ids[i] = h_color_ids[idx];
        temp_num_ids[i] = h_num_ids[idx];
        temp_confidences[i] = h_confidences[idx];
        for (int j = 0; j < 4; j++) {
            temp_boxes[i * 4 + j] = h_boxes[idx * 4 + j];
        }
        for (int j = 0; j < 8; j++) {
            temp_keypoints[i * 8 + j] = h_keypoints[idx * 8 + j];
        }
    }
    
    // 复制回GPU
    cudaMemcpy(d_color_ids, temp_color_ids.data(), sizeof(int) * final_count, cudaMemcpyHostToDevice);
    cudaMemcpy(d_num_ids, temp_num_ids.data(), sizeof(int) * final_count, cudaMemcpyHostToDevice);
    cudaMemcpy(d_confidences, temp_confidences.data(), sizeof(float) * final_count, cudaMemcpyHostToDevice);
    cudaMemcpy(d_boxes, temp_boxes.data(), sizeof(int) * final_count * 4, cudaMemcpyHostToDevice);
    cudaMemcpy(d_keypoints, temp_keypoints.data(), sizeof(float) * final_count * 8, cudaMemcpyHostToDevice);
    
    // 清理
    cudaFree(d_valid_indices);
    
    return final_count;
}

// ======================= 复制检测结果到主机 =======================
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
    float* h_keypoints)
{
    cudaMemcpy(h_color_ids, d_color_ids, sizeof(int) * num_detections, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_num_ids, d_num_ids, sizeof(int) * num_detections, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_confidences, d_confidences, sizeof(float) * num_detections, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_boxes, d_boxes, sizeof(int) * num_detections * 4, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_keypoints, d_keypoints, sizeof(float) * num_detections * 8, cudaMemcpyDeviceToHost);
}

}  // namespace gpu_kernel
}  // namespace auto_aim
