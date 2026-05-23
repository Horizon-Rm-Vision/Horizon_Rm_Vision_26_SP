#ifndef AUTO_AIM__YOLOX_TRT_HPP
#define AUTO_AIM__YOLOX_TRT_HPP

#ifdef USE_CUDA

#include <list>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/trt_engine.h"

namespace auto_aim
{

class YOLOX_TRT : public YOLOBase
{
public:
  YOLOX_TRT(const std::string & config_path, bool debug);
  ~YOLOX_TRT();

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

  // ---- YOLOX internal types (from yolox_ov) ----
  struct ArmorObject
  {
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
    int area;
    cv::Point2f apex[4];
  };

  struct GridAndStride
  {
    int grid0;
    int grid1;
    int stride;
  };

  // ---- HOG+SVM Number Classifier (from yolox_ov) ----
  class SVMClassifier
  {
  public:
    SVMClassifier(const std::string & model_path);
    ~SVMClassifier();
    std::pair<int, double> predict(const cv::Mat & frame, const std::vector<cv::Point2f> & corners);

  private:
    cv::Ptr<cv::ml::SVM> svm_model_;
    cv::HOGDescriptor * hog_;
    bool affineNumber(const cv::Mat & frame, const std::vector<cv::Point2f> & corners);
    cv::Mat autoGammaCorrect(const cv::Mat & image);
    cv::Mat pixel(const cv::Mat & image, double gamma, double mu, double sigma, int flag);
    float getDistance(const cv::Point2f & p1, const cv::Point2f & p2);
    cv::Mat number_roi_;
    cv::Mat class_;
  };

  // ---- LeNet Number Classifier (from yolox_ov) ----
  class LeNetClassifier
  {
  public:
    LeNetClassifier(
      const std::string & model_path, const std::string & label_path, double threshold);
    std::pair<ArmorName, double> predict(
      const cv::Mat & frame, const std::vector<cv::Point2f> & corners);
    double threshold;

  private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::mutex mutex_;
    cv::Mat number_roi_;
    bool extractNumber(const cv::Mat & frame, const std::vector<cv::Point2f> & corners);
  };

  // ---- ResNet Number Classifier (from yolox_ov) ----
  class ResNetClassifier
  {
  public:
    ResNetClassifier(const std::string & model_path);
    std::pair<ArmorName, double> predict(
      const cv::Mat & frame, const std::vector<cv::Point2f> & corners);

  private:
    cv::dnn::Net net_;
    cv::Mat number_roi_;
    bool extractNumber(const cv::Mat & frame, const std::vector<cv::Point2f> & corners);
  };

private:
  // ---- YOLOX decode helpers ----
  static void generate_grids_and_stride(
    int target_w, int target_h, const std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides);

  void generateYoloxProposals(
    const std::vector<GridAndStride> & grid_strides, const float * feat_ptr,
    const Eigen::Matrix<float, 3, 3> & transform_matrix,
    std::vector<ArmorObject> & objects);

  void decodeOutputs(
    const float * prob, std::vector<ArmorObject> & objects,
    const Eigen::Matrix<float, 3, 3> & transform_matrix);

  cv::Mat scaledResize(cv::Mat & img, Eigen::Matrix<float, 3, 3> & transform_matrix);

  static void sort_keypoints(std::vector<cv::Point2f> & keypoints);

  // ---- Conversion to SP Armor ----
  std::list<Armor> parse(
    double scale, const cv::Mat & bgr_img, int frame_count,
    std::vector<ArmorObject> & objects);

  // ---- Filtering ----
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  // ---- Debug ----
  void draw_detections(
    const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;

  // ---- GPU memory management ----
  void allocateGPUMemory();
  void freeGPUMemory();

  // ---- Config ----
  std::string device_, model_path_, svm_model_path_;
  std::string label_correction_mode_ = "svm";
  std::string lenet_model_path_, lenet_label_path_;
  double lenet_threshold_ = 0.7;
  std::string resnet_model_path_;
<<<<<<< HEAD
  bool debug_, use_roi_, use_svm_ = true, use_traditional_;
=======
  bool debug_, use_roi_, use_traditional_;
>>>>>>> origin/main
  double min_confidence_;

  // ---- SVM classifier ----
  std::unique_ptr<SVMClassifier> classifier_;

  // ---- LeNet classifier (tra mode) ----
  std::unique_ptr<LeNetClassifier> lenet_classifier_;

  // ---- ResNet classifier (traditional_cv mode) ----
  std::unique_ptr<ResNetClassifier> resnet_classifier_;

  // YOLOX params
  int input_w_ = 416;
  int input_h_ = 416;
  int num_classes_ = 8;
  int num_colors_ = 8;
  int topk_ = 128;
  float bbox_conf_thresh_ = 0.6f;
  float nms_thresh_ = 0.3f;
  float merge_conf_error_ = 0.15f;
  float merge_min_iou_ = 0.9f;

  // ROI
  cv::Rect roi_;
  cv::Point2f offset_;

  // TensorRT engine
  std::unique_ptr<Logger> logger_;
  std::unique_ptr<TrtEngine> trt_engine_;

  // GPU memory
  float* d_input_tensor_ = nullptr;
  float* d_output_tensor_ = nullptr;
  cudaStream_t stream_ = nullptr;

  // Model dimensions
  int output_rows_ = 0;
  int output_cols_ = 0;

  // Traditional CV detector for secondary refinement
  Detector detector_;

  // Eigen transform matrix for letterbox→original coordinate mapping
  Eigen::Matrix<float, 3, 3> transform_matrix_;
};

}  // namespace auto_aim

#endif  // USE_CUDA
#endif  // AUTO_AIM__YOLOX_TRT_HPP
