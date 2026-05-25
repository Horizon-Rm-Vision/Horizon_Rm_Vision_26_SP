#ifndef AUTO_BUFF__YOLOX_BUFF_TRT_HPP
#define AUTO_BUFF__YOLOX_BUFF_TRT_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <vector>
#include <memory>

#include "tasks/auto_aim/trt_engine.h"
#include "tasks/auto_aim/utils.h"
#include "tools/logger.hpp"
#include "buff_type.hpp"

namespace auto_buff
{
class YOLOX_BUFF_TRT
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    int color = 0;  // 0=blue, 1=red (from model output)
    std::vector<cv::Point2f> kpt;  // 6 keypoints, converted to YOLO11-compatible format
  };

  YOLOX_BUFF_TRT(const std::string & config);
  ~YOLOX_BUFF_TRT();

  std::vector<Object> get_multicandidateboxes(cv::Mat & image);

  std::vector<Object> get_onecandidatebox(cv::Mat & image);

private:
  // Letterbox preprocessing with padding
  cv::Mat letterbox(const cv::Mat & img, Eigen::Matrix3f & transform_matrix);

  // Generate grids and stride for post processing
  void generateGridsAndStride(
    int target_w, int target_h, std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides);

  // Post-process YOLOX output into Object list
  void generateProposals(
    std::vector<Object> & output_objs, const cv::Mat & output_buffer,
    const Eigen::Matrix3f & transform_matrix, float conf_threshold,
    const std::vector<GridAndStride> & grid_strides);

  // Convert ROS-style 5 points to SP-style 6 keypoints
  // ROS pts layout: [r_center, bottom_left, top_left, top_right, bottom_right]
  // SP kpt layout: [4 corners, blade_center, r_center]
  std::vector<cv::Point2f> convertPointsToKpts(
    const cv::Point2f & r_center, const cv::Point2f & bottom_left,
    const cv::Point2f & top_left, const cv::Point2f & top_right,
    const cv::Point2f & bottom_right) const;

  // GPU memory management
  void allocateGPUMemory();
  void freeGPUMemory();

  // TensorRT engine
  std::unique_ptr<Logger> logger_;
  std::unique_ptr<TrtEngine> trt_engine_;

  // Model dimensions (set from config / engine)
  int input_size_{480};
  int output_rows_{0};
  int output_cols_{0};

  // GPU memory
  float* d_input_tensor_ = nullptr;
  float* d_output_tensor_ = nullptr;
  cudaStream_t stream_ = nullptr;

  float conf_threshold_;
  float nms_threshold_;
  std::vector<int> strides_;
  std::vector<GridAndStride> grid_strides_;

  static constexpr int INPUT_W = 480;
  static constexpr int INPUT_H = 480;
  static constexpr int NUM_POINTS = 5;
  static constexpr int NUM_CLASSES = 2;
  static constexpr int NUM_COLORS = 2;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__YOLOX_BUFF_TRT_HPP
