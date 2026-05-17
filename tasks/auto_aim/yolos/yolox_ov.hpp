#ifndef AUTO_AIM__YOLOX_OV_HPP
#define AUTO_AIM__YOLOX_OV_HPP

#ifdef USE_OPENVINO

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLOX_OV : public YOLOBase
{
public:
  YOLOX_OV(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

  // ---- YOLOX internal types (from Horizon_Hero_Aim_26) ----
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

  // ---- HOG+SVM Number Classifier (ported from Hero NumberClassifier) ----
  class SVMClassifier
  {
  public:
    SVMClassifier(const std::string & model_path);
    ~SVMClassifier();

    // Returns (class_id, confidence). class_id: 0=Undefined, 1=Hero, 2=Engineer,
    // 3/4/5=Infantry, 6=Sentry, 7=Outpost, 8=Base. Returns (-1,0) on failure.
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

  // ---- LeNet Number Classifier (from tra mode TraditionalDetector) ----
  class LeNetClassifier
  {
  public:
    LeNetClassifier(
      const std::string & model_path, const std::string & label_path, double threshold);

    // Returns (ArmorName, confidence). Returns (not_armor, 0) on failure.
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

  // ---- ResNet Number Classifier (from Classifier via tiny_resnet.onnx) ----
  class ResNetClassifier
  {
  public:
    ResNetClassifier(const std::string & model_path);

    // Returns (ArmorName, confidence). Returns (not_armor, 0) on failure.
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

  // ---- Config ----
  std::string device_, model_path_, svm_model_path_;
  std::string label_correction_mode_ = "svm";
  std::string lenet_model_path_, lenet_label_path_;
  double lenet_threshold_ = 0.7;
  std::string resnet_model_path_;
  bool debug_, use_roi_, use_svm_ = true, use_traditional_;
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

  // OpenVINO
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  // Traditional CV detector for secondary refinement
  Detector detector_;

  // Eigen transform matrix for letterbox→original coordinate mapping
  Eigen::Matrix<float, 3, 3> transform_matrix_;
};

}  // namespace auto_aim

#endif  // USE_OPENVINO
#endif  // AUTO_AIM__YOLOX_OV_HPP
