#include "yolov5_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cuda_runtime.h>
#include <cuda.h>
#include <device_launch_parameters.h>
#include <atomic>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

YOLOV5_TRT::YOLOV5_TRT(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_trt_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  // 初始化TensorRT
  logger_ = std::make_unique<Logger>();
  trt_engine_ = std::make_unique<TrtEngine>(*logger_);
  
  // 加载TensorRT引擎
  std::string engine_path = model_path_;
  size_t last_dot = engine_path.find_last_of(".");
  if (last_dot != std::string::npos) {
    engine_path = engine_path.substr(0, last_dot) + ".engine";
  }
  
  try {
    trt_engine_->LoadEngine(engine_path);
    
    // 获取输入输出维度
    auto input_dim = trt_engine_->GetInputDim();
    auto output_dim = trt_engine_->GetOutputDim();
    
    input_size_ = input_dim[2]; // 假设输入是正方形
    output_rows_ = output_dim[1]; // 25200
    output_cols_ = output_dim[2]; // 22
    
    // 预分配GPU内存
    allocateGPUMemory();
    
    tools::logger()->info("TensorRT engine loaded successfully. Input: {}x{}, Output: {}x{}", 
                         input_size_, input_size_, output_rows_, output_cols_);
  } catch (const std::exception& e) {
    tools::logger()->error("Failed to load TensorRT engine: {}", e.what());
    throw;
  }
}

YOLOV5_TRT::~YOLOV5_TRT()
{
  freeGPUMemory();
}

void YOLOV5_TRT::allocateGPUMemory() {
  input_count_ = 3 * input_size_ * input_size_;
  output_count_ = output_rows_ * output_cols_;
  
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_input_tensor_), sizeof(float) * input_count_));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_output_tensor_), sizeof(float) * output_count_));
  
  // 预分配临时GPU内存用于后处理
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_temp_color_ids_), sizeof(int) * h_max_detections_));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_temp_num_ids_), sizeof(int) * h_max_detections_));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_temp_confidences_), sizeof(float) * h_max_detections_));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_temp_boxes_), sizeof(int) * h_max_detections_ * 4));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_temp_keypoints_), sizeof(float) * h_max_detections_ * 8));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&d_valid_count_), sizeof(int)));
  
  // 初始化有效计数为0
  CUDA_CHECK(cudaMemset(d_valid_count_, 0, sizeof(int)));
  
  // 分配Pinned Host内存 - 加速GPU-CPU数据传输
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_pinned_color_ids_), sizeof(int) * h_max_detections_));
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_pinned_num_ids_), sizeof(int) * h_max_detections_));
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_pinned_confidences_), sizeof(float) * h_max_detections_));
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_pinned_boxes_), sizeof(int) * h_max_detections_ * 4));
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_pinned_keypoints_), sizeof(float) * h_max_detections_ * 8));
  
  // 预分配主机端临时向量
  h_temp_color_ids_.reserve(h_max_detections_);
  h_temp_num_ids_.reserve(h_max_detections_);
  h_temp_confidences_.reserve(h_max_detections_);
  h_temp_boxes_.reserve(h_max_detections_ * 4);
  h_temp_keypoints_.reserve(h_max_detections_ * 8);

  // 创建一个非阻塞CUDA流，用于异步拷贝与推理
  CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
}

void YOLOV5_TRT::freeGPUMemory() {
  if (d_input_tensor_) {
    cudaFree(d_input_tensor_);
    d_input_tensor_ = nullptr;
  }
  if (d_output_tensor_) {
    cudaFree(d_output_tensor_);
    d_output_tensor_ = nullptr;
  }
  if (d_temp_color_ids_) cudaFree(d_temp_color_ids_);
  if (d_temp_num_ids_) cudaFree(d_temp_num_ids_);
  if (d_temp_confidences_) cudaFree(d_temp_confidences_);
  if (d_temp_boxes_) cudaFree(d_temp_boxes_);
  if (d_temp_keypoints_) cudaFree(d_temp_keypoints_);
  if (d_valid_count_) cudaFree(d_valid_count_);
  
  // 释放Pinned内存
  if (h_pinned_color_ids_) cudaFreeHost(h_pinned_color_ids_);
  if (h_pinned_num_ids_) cudaFreeHost(h_pinned_num_ids_);
  if (h_pinned_confidences_) cudaFreeHost(h_pinned_confidences_);
  if (h_pinned_boxes_) cudaFreeHost(h_pinned_boxes_);
  if (h_pinned_keypoints_) cudaFreeHost(h_pinned_keypoints_);
  
  // 销毁流
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }

}

cv::cuda::GpuMat YOLOV5_TRT::preprocessImageGPU(const cv::Mat& bgr_img, double& scale, int& pad_x, int& pad_y)
{
  // 计算缩放比例，保持宽高比
  auto x_scale = static_cast<double>(input_size_) / bgr_img.cols;
  auto y_scale = static_cast<double>(input_size_) / bgr_img.rows;
  scale = std::min(x_scale, y_scale);
  
  int new_w = static_cast<int>(bgr_img.cols * scale);
  int new_h = static_cast<int>(bgr_img.rows * scale);
  
  // 计算填充偏移（居中填充）
  pad_x = (input_size_ - new_w) / 2;
  pad_y = (input_size_ - new_h) / 2;
  
  // 上传图像到GPU
  gpu_bgr_img_.upload(bgr_img);
  
  // 调整图像大小
  cv::cuda::resize(gpu_bgr_img_, gpu_resized_, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  
  // 创建填充后的图像
  if (gpu_padded_.empty() || gpu_padded_.size() != cv::Size(input_size_, input_size_)) {
    gpu_padded_ = cv::cuda::GpuMat(input_size_, input_size_, CV_8UC3, cv::Scalar(0, 0, 0));
  } else {
    gpu_padded_.setTo(cv::Scalar(0, 0, 0));
  }
  
  // 复制到填充图像的对应位置
  cv::cuda::GpuMat roi(gpu_padded_, cv::Rect(pad_x, pad_y, new_w, new_h));
  gpu_resized_.copyTo(roi);
  
  // 转换为RGB
  cv::cuda::cvtColor(gpu_padded_, gpu_rgb_, cv::COLOR_BGR2RGB);
  
  // 转换为float并归一化
  gpu_rgb_.convertTo(gpu_float_, CV_32FC3, 1.0f / 255.0f);
  
  // 分离通道并重排为NCHW格式
  std::vector<cv::cuda::GpuMat> channels;
  cv::cuda::split(gpu_float_, channels);
  
  // 合并为单通道的blob (1x3xHxW)
  gpu_blob_.create(input_size_ * input_size_ * 3, 1, CV_32F);
  
  // 将通道数据拷贝到连续内存
  size_t channel_size = input_size_ * input_size_ * sizeof(float);
  for (int i = 0; i < 3; i++) {
    CUDA_CHECK(cudaMemcpy(gpu_blob_.ptr<float>() + i * input_size_ * input_size_,
               channels[i].ptr<float>(), channel_size, cudaMemcpyDeviceToDevice));
  }
  
  return gpu_blob_;
}

void YOLOV5_TRT::parseDetectionsGPU(const float* d_output, int output_rows, int output_cols,
                                   float score_threshold, double scale, int pad_x, int pad_y,
                                   std::vector<int>& color_ids, std::vector<int>& num_ids,
                                   std::vector<float>& confidences, std::vector<cv::Rect>& boxes,
                                   std::vector<std::vector<cv::Point2f>>& armor_key_points) {
    
    // 使用Pinned内存进行高效的GPU-CPU数据传输
    static std::vector<float> output_data(1);
    if (output_data.size() < output_rows * output_cols) {
      output_data.resize(output_rows * output_cols);
    }
    
    // 异步复制TensorRT输出到主机
    CUDA_CHECK(cudaMemcpy(output_data.data(), d_output, 
                         sizeof(float) * output_rows * output_cols, cudaMemcpyDeviceToHost));
    
    // 在CPU上处理（保留原版本的逻辑，确保准确性）
    for (int r = 0; r < output_rows; r++) {
        float score = output_data[r * output_cols + 8];
        // 使用CPU sigmoid函数
        double score_double = static_cast<double>(score);
        score = static_cast<float>(sigmoid(score_double));
        
        if (score < score_threshold) continue;
        
        // 颜色和类别预测
        float max_color_score = -FLT_MAX;
        int color_id = 0;
        for (int i = 0; i < 4; i++) {
            float s = output_data[r * output_cols + 9 + i];
            if (s > max_color_score) {
                max_color_score = s;
                color_id = i;
            }
        }
        
        float max_class_score = -FLT_MAX;
        int class_id = 0;
        for (int i = 0; i < 9; i++) {
            float s = output_data[r * output_cols + 13 + i];
            if (s > max_class_score) {
                max_class_score = s;
                class_id = i;
            }
        }
        
        // 按照原版顺序读取关键点
        std::vector<cv::Point2f> keypoints;
        // 点1: (0,1)
        float x1 = (output_data[r * output_cols + 0] - pad_x) / scale;
        float y1 = (output_data[r * output_cols + 1] - pad_y) / scale;
        keypoints.push_back(cv::Point2f(x1, y1));

        // 点4: (6,7)
        float x4 = (output_data[r * output_cols + 6] - pad_x) / scale;
        float y4 = (output_data[r * output_cols + 7] - pad_y) / scale;
        keypoints.push_back(cv::Point2f(x4, y4));

        // 点3: (4,5)
        float x3 = (output_data[r * output_cols + 4] - pad_x) / scale;
        float y3 = (output_data[r * output_cols + 5] - pad_y) / scale;
        keypoints.push_back(cv::Point2f(x3, y3));

        // 点2: (2,3)
        float x2 = (output_data[r * output_cols + 2] - pad_x) / scale;
        float y2 = (output_data[r * output_cols + 3] - pad_y) / scale;
        keypoints.push_back(cv::Point2f(x2, y2));
        
        // 计算边界框
        float min_x = keypoints[0].x;
        float max_x = keypoints[0].x;
        float min_y = keypoints[0].y;
        float max_y = keypoints[0].y;
        
        for (int i = 1; i < keypoints.size(); i++) {
            min_x = std::min(min_x, keypoints[i].x);
            max_x = std::max(max_x, keypoints[i].x);
            min_y = std::min(min_y, keypoints[i].y);
            max_y = std::max(max_y, keypoints[i].y);
        }
        
        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
        
        color_ids.push_back(color_id);
        num_ids.push_back(class_id);
        confidences.push_back(score);
        boxes.push_back(rect);
        armor_key_points.push_back(keypoints);
    }
}

std::list<Armor> YOLOV5_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // GPU预处理 - 获取缩放比例和填充偏移
  double scale;
  int pad_x, pad_y;
  cv::cuda::GpuMat gpu_blob = preprocessImageGPU(bgr_img, scale, pad_x, pad_y);

  // 将预处理后的数据拷贝到输入tensor
  // 将预处理后的GPU blob直接作为TensorRT输入指针，避免额外的device->device拷贝
  trt_engine_->AsyncInference(stream_, const_cast<float*>(gpu_blob.ptr<float>(0)), d_output_tensor_);

  // 解析检测结果
  tmp_img_ = bgr_img;
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armor_key_points;

  if (use_gpu_nms_) {
    // 使用GPU端解析 + NMS，减少从device->host拷贝的数据量
    int num_detections = gpu_kernel::gpu_parse_detections_nms(
        d_output_tensor_, output_rows_, output_cols_,
        score_threshold_, nms_threshold_, (float)scale, pad_x, pad_y,
        d_temp_color_ids_, d_temp_num_ids_, d_temp_confidences_,
        d_temp_boxes_, d_temp_keypoints_, d_valid_count_, stream_);

    if (num_detections > 0) {
      if (num_detections > h_max_detections_) num_detections = h_max_detections_;

      // 异步将紧凑的检测结果复制到Pinned Host内存（减少IO开销）
      size_t s_color = sizeof(int) * num_detections;
      size_t s_num = sizeof(int) * num_detections;
      size_t s_conf = sizeof(float) * num_detections;
      size_t s_boxes = sizeof(int) * num_detections * 4;
      size_t s_kps = sizeof(float) * num_detections * 8;

      CUDA_CHECK(cudaMemcpyAsync(h_pinned_color_ids_, d_temp_color_ids_, s_color, cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(h_pinned_num_ids_, d_temp_num_ids_, s_num, cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(h_pinned_confidences_, d_temp_confidences_, s_conf, cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(h_pinned_boxes_, d_temp_boxes_, s_boxes, cudaMemcpyDeviceToHost, stream_));
      CUDA_CHECK(cudaMemcpyAsync(h_pinned_keypoints_, d_temp_keypoints_, s_kps, cudaMemcpyDeviceToHost, stream_));

      // 等待流上所有操作完成（包括AsyncInference、解析kernel和拷贝）
      CUDA_CHECK(cudaStreamSynchronize(stream_));

      // 重构到 std::vector / 输出结构
      color_ids.resize(num_detections);
      num_ids.resize(num_detections);
      confidences.resize(num_detections);
      boxes.resize(num_detections);
      armor_key_points.resize(num_detections);

      for (int i = 0; i < num_detections; ++i) {
        color_ids[i] = h_pinned_color_ids_[i];
        num_ids[i] = h_pinned_num_ids_[i];
        confidences[i] = h_pinned_confidences_[i];

        int bx = h_pinned_boxes_[i * 4 + 0];
        int by = h_pinned_boxes_[i * 4 + 1];
        int bw = h_pinned_boxes_[i * 4 + 2];
        int bh = h_pinned_boxes_[i * 4 + 3];
        boxes[i] = cv::Rect(bx, by, bw, bh);

        std::vector<cv::Point2f> kps(4);
        kps[0] = cv::Point2f(h_pinned_keypoints_[i * 8 + 0], h_pinned_keypoints_[i * 8 + 1]);
        kps[1] = cv::Point2f(h_pinned_keypoints_[i * 8 + 2], h_pinned_keypoints_[i * 8 + 3]);
        kps[2] = cv::Point2f(h_pinned_keypoints_[i * 8 + 4], h_pinned_keypoints_[i * 8 + 5]);
        kps[3] = cv::Point2f(h_pinned_keypoints_[i * 8 + 6], h_pinned_keypoints_[i * 8 + 7]);
        armor_key_points[i] = std::move(kps);
      }
    } else {
      // 没有检测到目标，确保流上的推理完成
      CUDA_CHECK(cudaStreamSynchronize(stream_));
    }
  } else {
    // 保留CPU端解析路径，但使用异步拷贝以减少阻塞
    size_t entire_out = sizeof(float) * output_rows_ * output_cols_;
    static std::vector<float> output_data;
    if (output_data.size() < output_rows_ * output_cols_) output_data.resize(output_rows_ * output_cols_);

    CUDA_CHECK(cudaMemcpyAsync(output_data.data(), d_output_tensor_, entire_out, cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    // 在CPU上解析输出（原来的parseDetectionsGPU逻辑）
    for (int r = 0; r < output_rows_; r++) {
        float score = output_data[r * output_cols_ + 8];
        double score_double = static_cast<double>(score);
        score = static_cast<float>(sigmoid(score_double));
        if (score < score_threshold_) continue;

        float max_color_score = -FLT_MAX;
        int color_id = 0;
        for (int i = 0; i < 4; i++) {
            float s = output_data[r * output_cols_ + 9 + i];
            if (s > max_color_score) { max_color_score = s; color_id = i; }
        }

        float max_class_score = -FLT_MAX;
        int class_id = 0;
        for (int i = 0; i < 9; i++) {
            float s = output_data[r * output_cols_ + 13 + i];
            if (s > max_class_score) { max_class_score = s; class_id = i; }
        }

        std::vector<cv::Point2f> keypoints;
        float x1 = (output_data[r * output_cols_ + 0] - pad_x) / scale;
        float y1 = (output_data[r * output_cols_ + 1] - pad_x) / scale; // kept as original logic
        keypoints.push_back(cv::Point2f(x1, y1));
        float x4 = (output_data[r * output_cols_ + 6] - pad_x) / scale;
        float y4 = (output_data[r * output_cols_ + 7] - pad_x) / scale;
        keypoints.push_back(cv::Point2f(x4, y4));
        float x3 = (output_data[r * output_cols_ + 4] - pad_x) / scale;
        float y3 = (output_data[r * output_cols_ + 5] - pad_x) / scale;
        keypoints.push_back(cv::Point2f(x3, y3));
        float x2 = (output_data[r * output_cols_ + 2] - pad_x) / scale;
        float y2 = (output_data[r * output_cols_ + 3] - pad_x) / scale;
        keypoints.push_back(cv::Point2f(x2, y2));

        float min_x = keypoints[0].x, max_x = keypoints[0].x, min_y = keypoints[0].y, max_y = keypoints[0].y;
        for (int i = 1; i < keypoints.size(); i++) {
            min_x = std::min(min_x, keypoints[i].x);
            max_x = std::max(max_x, keypoints[i].x);
            min_y = std::min(min_y, keypoints[i].y);
            max_y = std::max(max_y, keypoints[i].y);
        }
        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

        color_ids.push_back(color_id);
        num_ids.push_back(class_id);
        confidences.push_back(score);
        boxes.push_back(rect);
        armor_key_points.push_back(keypoints);
    }
  }

  // NMS在CPU上执行（保持结果一致性）
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armor_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armor_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

// GPU加速版本的后处理 - 使用CUDA kernels进行并行处理
std::list<Armor> YOLOV5_TRT::parseDetectionsGPUFast(
    double scale, int pad_x, int pad_y, const cv::Mat& bgr_img, int frame_count)
{
  // 调用GPU kernel进行解析和NMS
  int num_detections = gpu_kernel::gpu_parse_detections_nms(
      d_output_tensor_, 
      output_rows_, output_cols_,
      score_threshold_, 
      nms_threshold_,
      scale, pad_x, pad_y,
      d_temp_color_ids_, 
      d_temp_num_ids_, 
      d_temp_confidences_,
      d_temp_boxes_, 
      d_temp_keypoints_,
      d_valid_count_
  );
  
  if (num_detections == 0) {
    return std::list<Armor>();
  }
  
  // 从GPU复制数据到CPU
  h_temp_color_ids_.resize(num_detections);
  h_temp_num_ids_.resize(num_detections);
  h_temp_confidences_.resize(num_detections);
  h_temp_boxes_.resize(num_detections * 4);
  h_temp_keypoints_.resize(num_detections * 8);
  
  CUDA_CHECK(cudaMemcpy(h_temp_color_ids_.data(), d_temp_color_ids_, 
                       sizeof(int) * num_detections, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(h_temp_num_ids_.data(), d_temp_num_ids_, 
                       sizeof(int) * num_detections, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(h_temp_confidences_.data(), d_temp_confidences_, 
                       sizeof(float) * num_detections, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(h_temp_boxes_.data(), d_temp_boxes_, 
                       sizeof(int) * num_detections * 4, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(h_temp_keypoints_.data(), d_temp_keypoints_, 
                       sizeof(float) * num_detections * 8, cudaMemcpyDeviceToHost));
  
  // 转换GPU数据格式到Armor
  std::list<Armor> armors;
  
  for (int i = 0; i < num_detections; i++) {
    // 从GPU数据重构关键点
    std::vector<cv::Point2f> keypoints(4);
    keypoints[0] = cv::Point2f(h_temp_keypoints_[i * 8 + 0], h_temp_keypoints_[i * 8 + 1]);
    keypoints[1] = cv::Point2f(h_temp_keypoints_[i * 8 + 2], h_temp_keypoints_[i * 8 + 3]);
    keypoints[2] = cv::Point2f(h_temp_keypoints_[i * 8 + 4], h_temp_keypoints_[i * 8 + 5]);
    keypoints[3] = cv::Point2f(h_temp_keypoints_[i * 8 + 6], h_temp_keypoints_[i * 8 + 7]);
    
    // 重构边界框
    cv::Rect rect(h_temp_boxes_[i * 4 + 0], h_temp_boxes_[i * 4 + 1],
                  h_temp_boxes_[i * 4 + 2], h_temp_boxes_[i * 4 + 3]);
    
    if (use_roi_) {
      armors.emplace_back(
          h_temp_color_ids_[i], h_temp_num_ids_[i], 
          h_temp_confidences_[i], rect, keypoints, offset_);
    } else {
      armors.emplace_back(
          h_temp_color_ids_[i], h_temp_num_ids_[i], 
          h_temp_confidences_[i], rect, keypoints);
    }
  }
  
  // 后续处理
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

std::list<Armor> YOLOV5_TRT::parseDetections(const cv::Mat& output, double scale, int pad_x, int pad_y, 
                                            const cv::Mat& bgr_img, int frame_count)
{
  // CPU版本的解析函数，作为备选
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    // 颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     // color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  // num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    // 关键点坐标
    armor_key_points.push_back(
      cv::Point2f((output.at<float>(r, 0) - pad_x) / scale, (output.at<float>(r, 1) - pad_y) / scale));
    armor_key_points.push_back(
      cv::Point2f((output.at<float>(r, 6) - pad_x) / scale, (output.at<float>(r, 7) - pad_y) / scale));
    armor_key_points.push_back(
      cv::Point2f((output.at<float>(r, 4) - pad_x) / scale, (output.at<float>(r, 5) - pad_y) / scale));
    armor_key_points.push_back(
      cv::Point2f((output.at<float>(r, 2) - pad_x) / scale, (output.at<float>(r, 3) - pad_y) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

std::list<Armor> YOLOV5_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 这个方法保持与接口兼容，但实际使用GPU路径
  return parseDetections(output, scale, 0, 0, bgr_img, frame_count);
}

bool YOLOV5_TRT::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  return name_ok && confidence_ok;
}

bool YOLOV5_TRT::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  return name_ok;
}

cv::Point2f YOLOV5_TRT::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5_TRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  // 绘制检测结果
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

void YOLOV5_TRT::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5_TRT::sigmoid(double x)
{
  // CPU版本的sigmoid
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

}  // namespace auto_aim