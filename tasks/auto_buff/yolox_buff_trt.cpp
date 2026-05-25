#include "yolox_buff_trt.hpp"

#include <algorithm>

#include "tools/ui_manager.hpp"

namespace auto_buff
{
YOLOX_BUFF_TRT::YOLOX_BUFF_TRT(const std::string & config)
{
  auto yaml = YAML::LoadFile(config);
  std::string model_path = yaml["buff_yolox_trt_model_path"].as<std::string>();
  conf_threshold_ = yaml["confidence_threshold"].as<float>(0.7f);
  nms_threshold_ = yaml["iou_threshold"].as<float>(0.4f);

  // Initialize TensorRT engine
  logger_ = std::make_unique<Logger>();
  trt_engine_ = std::make_unique<TrtEngine>(*logger_);
  trt_engine_->LoadEngine(model_path);

  // Get input/output dimensions from engine
  auto input_dim = trt_engine_->GetInputDim();
  auto output_dim = trt_engine_->GetOutputDim();

  // Use input dimension from the engine, but keep INPUT_W/INPUT_H as fallback
  if (input_dim.size() >= 4) {
    input_size_ = input_dim[2];  // NCHW: H
  } else if (input_dim.size() >= 2) {
    input_size_ = input_dim[1];
  }

  output_rows_ = output_dim[output_dim.size() - 2];  // second-to-last dim: rows/anchors
  output_cols_ = output_dim[output_dim.size() - 1];  // last dim: cols/channels

  // Allocate GPU memory
  allocateGPUMemory();

  // Generate grid strides for YOLOX decoding
  strides_ = {8, 16, 32};
  generateGridsAndStride(input_size_, input_size_, strides_, grid_strides_);

  tools::logger()->info("[YOLOX_BUFF_TRT] Engine loaded: {} (conf={}, nms={}, input={}, out={}x{})",
                        model_path, conf_threshold_, nms_threshold_,
                        input_size_, output_rows_, output_cols_);
}

YOLOX_BUFF_TRT::~YOLOX_BUFF_TRT()
{
  freeGPUMemory();
}

void YOLOX_BUFF_TRT::allocateGPUMemory()
{
  size_t input_count = 3 * input_size_ * input_size_;
  size_t output_count = output_rows_ * output_cols_;

  CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_input_tensor_), sizeof(float) * input_count));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_output_tensor_), sizeof(float) * output_count));
  CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
}

void YOLOX_BUFF_TRT::freeGPUMemory()
{
  if (d_input_tensor_) {
    cudaFree(d_input_tensor_);
    d_input_tensor_ = nullptr;
  }
  if (d_output_tensor_) {
    cudaFree(d_output_tensor_);
    d_output_tensor_ = nullptr;
  }
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

cv::Mat YOLOX_BUFF_TRT::letterbox(const cv::Mat & img, Eigen::Matrix3f & transform_matrix)
{
  int img_h = img.rows;
  int img_w = img.cols;

  // Compute scale ratio and target size
  float scale = std::min(input_size_ * 1.0f / img_h, input_size_ * 1.0f / img_w);
  int resize_h = static_cast<int>(std::round(img_h * scale));
  int resize_w = static_cast<int>(std::round(img_w * scale));

  // Compute padding
  int pad_h = input_size_ - resize_h;
  int pad_w = input_size_ - resize_w;

  // Resize
  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

  // Divide padding into 2 sides
  float half_h = pad_h * 1.0f / 2;
  float half_w = pad_w * 1.0f / 2;

  int top = static_cast<int>(std::round(half_h - 0.1));
  int bottom = static_cast<int>(std::round(half_h + 0.1));
  int left = static_cast<int>(std::round(half_w - 0.1));
  int right = static_cast<int>(std::round(half_w + 0.1));

  // Transform matrix from resized image to source image
  transform_matrix << 1.0f / scale, 0, -half_w / scale, 0, 1.0f / scale, -half_h / scale, 0, 0, 1;

  // Add border
  cv::copyMakeBorder(resized_img, resized_img, top, bottom, left, right, cv::BORDER_CONSTANT,
                     cv::Scalar(114, 114, 114));

  return resized_img;
}

void YOLOX_BUFF_TRT::generateGridsAndStride(
  int target_w, int target_h, std::vector<int> & strides,
  std::vector<GridAndStride> & grid_strides)
{
  for (auto stride : strides) {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;

    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides.push_back({g0, g1, stride});
      }
    }
  }
}

void YOLOX_BUFF_TRT::generateProposals(
  std::vector<Object> & output_objs, const cv::Mat & output_buffer,
  const Eigen::Matrix3f & transform_matrix, float conf_threshold,
  const std::vector<GridAndStride> & grid_strides)
{
  const int num_anchors = std::min(static_cast<int>(grid_strides.size()), output_buffer.rows);

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
    float confidence = output_buffer.at<float>(anchor_idx, NUM_POINTS * 2);
    if (confidence < conf_threshold) continue;

    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    // Color and class scores (YOLOX format)
    double color_score, class_score;
    cv::Point color_id, class_id;
    cv::Mat color_scores =
      output_buffer.row(anchor_idx).colRange(NUM_POINTS * 2 + 1, NUM_POINTS * 2 + 1 + NUM_COLORS);
    cv::Mat class_scores =
      output_buffer.row(anchor_idx)
        .colRange(NUM_POINTS * 2 + 1 + NUM_COLORS, NUM_POINTS * 2 + 1 + NUM_COLORS + NUM_CLASSES);
    cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_id);
    cv::minMaxLoc(class_scores, nullptr, &class_score, nullptr, &class_id);

    // Decode 5 points from YOLOX output (offset format)
    float x_1 = (output_buffer.at<float>(anchor_idx, 0) + grid0) * stride;
    float y_1 = (output_buffer.at<float>(anchor_idx, 1) + grid1) * stride;
    float x_2 = (output_buffer.at<float>(anchor_idx, 2) + grid0) * stride;
    float y_2 = (output_buffer.at<float>(anchor_idx, 3) + grid1) * stride;
    float x_3 = (output_buffer.at<float>(anchor_idx, 4) + grid0) * stride;
    float y_3 = (output_buffer.at<float>(anchor_idx, 5) + grid1) * stride;
    float x_4 = (output_buffer.at<float>(anchor_idx, 6) + grid0) * stride;
    float y_4 = (output_buffer.at<float>(anchor_idx, 7) + grid1) * stride;
    float x_5 = (output_buffer.at<float>(anchor_idx, 8) + grid0) * stride;
    float y_5 = (output_buffer.at<float>(anchor_idx, 9) + grid1) * stride;

    // Apply transform matrix to get coordinates in original image
    Eigen::Matrix<float, 3, 5> apex_norm;
    Eigen::Matrix<float, 3, 5> apex_dst;

    apex_norm << x_1, x_2, x_3, x_4, x_5, y_1, y_2, y_3, y_4, y_5, 1, 1, 1, 1, 1;

    apex_dst = transform_matrix * apex_norm;

    // ROS-style 5 points:
    // pts[0] = r_center, pts[1] = bottom_left, pts[2] = top_left
    // pts[3] = top_right, pts[4] = bottom_right
    cv::Point2f r_center(apex_dst(0, 0), apex_dst(1, 0));
    cv::Point2f bottom_left(apex_dst(0, 1), apex_dst(1, 1));
    cv::Point2f top_left(apex_dst(0, 2), apex_dst(1, 2));
    cv::Point2f top_right(apex_dst(0, 3), apex_dst(1, 3));
    cv::Point2f bottom_right(apex_dst(0, 4), apex_dst(1, 4));

    // Convert to SP-style 6 keypoints
    std::vector<cv::Point2f> kpt =
      convertPointsToKpts(r_center, bottom_left, top_left, top_right, bottom_right);

    // Compute bounding box from the 4 corner points for NMS
    std::vector<cv::Point2f> corners = {kpt[0], kpt[1], kpt[2], kpt[3]};
    cv::Rect rect = cv::boundingRect(corners);

    Object obj;
    obj.rect = rect;
    obj.prob = confidence;
    obj.label = static_cast<int>(class_id.x);
    obj.color = static_cast<int>(color_id.x);
    obj.kpt = kpt;

    output_objs.push_back(std::move(obj));
  }
}

std::vector<cv::Point2f> YOLOX_BUFF_TRT::convertPointsToKpts(
  const cv::Point2f & r_center, const cv::Point2f & bottom_left,
  const cv::Point2f & top_left, const cv::Point2f & top_right,
  const cv::Point2f & bottom_right) const
{
  std::vector<cv::Point2f> kpt(6);

  // kpt[0] = midpoint of top_left and top_right (top edge midpoint of rune structure)
  kpt[0] = (top_left + top_right) * 0.5f;

  // kpt[2] = midpoint of bottom_left and bottom_right (bottom edge midpoint)
  kpt[2] = (bottom_left + bottom_right) * 0.5f;

  // kpt[1] and kpt[3]: orthogonal points around the midpoint of kpt[0]-kpt[2]
  // V = kpt[0] - kpt[2], M = (kpt[0] + kpt[2]) / 2
  // Rotate V 90 degrees CCW: V_perp = (-V.y, V.x)
  // kpt[1] = M - V_perp/2  (left)
  // kpt[3] = M + V_perp/2  (right)
  cv::Point2f V = kpt[0] - kpt[2];
  cv::Point2f M = (kpt[0] + kpt[2]) * 0.5f;
  cv::Point2f V_perp(-V.y, V.x);

  kpt[1] = M - V_perp * 0.5f;
  kpt[3] = M + V_perp * 0.5f;

  // kpt[4] = fan blade center = average of ROS 4 corner points
  kpt[4] = (bottom_left + top_left + top_right + bottom_right) * 0.25f;

  // kpt[5] = r_center (raw from YOLOX, will be refined by detectRTag later)
  kpt[5] = r_center;

  return kpt;
}

std::vector<YOLOX_BUFF_TRT::Object> YOLOX_BUFF_TRT::get_multicandidateboxes(cv::Mat & image)
{
  const int64 start = cv::getTickCount();

  if (image.empty()) {
    tools::logger()->warn("[YOLOX_BUFF_TRT] Empty img!");
    return {};
  }

  // Preprocess: letterbox
  Eigen::Matrix3f transform_matrix;
  cv::Mat resized_img = letterbox(image, transform_matrix);

  // BGR->RGB, u8->f32, HWC->NCHW
  cv::Mat blob = cv::dnn::blobFromImage(resized_img, 1.0, cv::Size(input_size_, input_size_),
                                         cv::Scalar(0, 0, 0), true);

  // Upload blob to GPU
  size_t input_size_bytes = 3 * input_size_ * input_size_ * sizeof(float);
  CUDA_CHECK(cudaMemcpyAsync(d_input_tensor_, blob.ptr<float>(), input_size_bytes,
                              cudaMemcpyHostToDevice, stream_));

  // TensorRT inference (async on stream)
  trt_engine_->AsyncInference(stream_, d_input_tensor_, d_output_tensor_);

  // Synchronize stream to ensure inference completes
  CUDA_CHECK(cudaStreamSynchronize(stream_));

  // Download output from GPU
  size_t output_size = output_rows_ * output_cols_;
  std::vector<float> output_data(output_size);
  CUDA_CHECK(cudaMemcpy(output_data.data(), d_output_tensor_,
                        sizeof(float) * output_size, cudaMemcpyDeviceToHost));

  // Wrap output as cv::Mat for generateProposals (same format as original YOLOX)
  cv::Mat output_buffer(output_rows_, output_cols_, CV_32F, output_data.data());

  // Parse proposals
  std::vector<Object> objs;
  generateProposals(objs, output_buffer, transform_matrix, conf_threshold_, grid_strides_);

  // Sort by probability
  std::sort(objs.begin(), objs.end(),
            [](const Object & a, const Object & b) { return a.prob > b.prob; });

  // NMS
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  for (auto & obj : objs) {
    boxes.push_back(obj.rect);
    confidences.push_back(obj.prob);
  }
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

  std::vector<Object> results;
  for (size_t i = 0; i < indices.size(); ++i) {
    results.push_back(objs[indices[i]]);
  }

  // Draw results
  if (tools::UIManager::isUIEnabled()) {
    for (auto & obj : results) {
      cv::rectangle(image, obj.rect, cv::Scalar(255, 255, 255), 1, 8);
      const std::string label = "buff:" + std::to_string(obj.prob).substr(0, 4);
      const cv::Size textSize =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
      const cv::Rect textBox(obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width,
                             textSize.height + 5);
      cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
      cv::putText(image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

      const int radius = 2;
      for (int i = 0; i < 6; ++i) {
        cv::circle(image, obj.kpt[i], radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
      }
    }
  }

  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  // cv::putText(image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40),
  //             cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 0), 2, 8);

  return results;
}

std::vector<YOLOX_BUFF_TRT::Object> YOLOX_BUFF_TRT::get_onecandidatebox(cv::Mat & image)
{
  // For YOLOX mode, get_multicandidateboxes with NMS is more reliable.
  // Use it but return only the top-1 result.
  auto results = get_multicandidateboxes(image);
  if (results.empty()) return {};

  // Return only the highest confidence result
  std::sort(results.begin(), results.end(),
            [](const Object & a, const Object & b) { return a.prob > b.prob; });
  return {results.front()};
}

}  // namespace auto_buff
