#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <string>
#include <vector>
#include <memory>
#include <cuda_runtime.h>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/trt_engine.h"

namespace auto_aim
{
class YOLOV5_TRT : public YOLOBase
{
public:
  YOLOV5_TRT(const std::string & config_path, bool debug);
  ~YOLOV5_TRT();

  std::list<Armor> detect(const cv::Mat & raw_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_, use_traditional_;

  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;

  // TensorRT相关成员
  std::unique_ptr<Logger> logger_;
  std::unique_ptr<TrtEngine> trt_engine_;
  size_t input_size_;
  int output_rows_;
  int output_cols_;

  // GPU内存指针
  float* d_input_tensor_ = nullptr;
  float* d_output_tensor_ = nullptr;
  size_t input_count_;
  size_t output_count_;
  
  // 临时GPU内存用于后处理
  int* d_temp_color_ids_ = nullptr;
  int* d_temp_num_ids_ = nullptr;
  float* d_temp_confidences_ = nullptr;
  float* d_temp_boxes_ = nullptr;
  float* d_temp_keypoints_ = nullptr;
  int* d_valid_count_ = nullptr;
  
  // OpenCV CUDA对象
  cv::cuda::GpuMat gpu_bgr_img_;
  cv::cuda::GpuMat gpu_resized_;
  cv::cuda::GpuMat gpu_padded_;
  cv::cuda::GpuMat gpu_rgb_;
  cv::cuda::GpuMat gpu_float_;
  cv::cuda::GpuMat gpu_blob_;

  // 预处理和后处理相关
  cv::cuda::GpuMat preprocessImageGPU(const cv::Mat& bgr_img, double& scale, int& pad_x, int& pad_y);
  std::list<Armor> parseDetections(const cv::Mat& output, double scale, int pad_x, int pad_y, 
                                   const cv::Mat& bgr_img, int frame_count);
  void parseDetectionsGPU(const float* d_output, int output_rows, int output_cols,
                         float score_threshold, double scale, int pad_x, int pad_y,
                         std::vector<int>& color_ids, std::vector<int>& num_ids,
                         std::vector<float>& confidences, std::vector<cv::Rect>& boxes,
                         std::vector<std::vector<cv::Point2f>>& armor_key_points);

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  double sigmoid(double x);
  
  // CUDA相关辅助函数
  void allocateGPUMemory();
  void freeGPUMemory();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP