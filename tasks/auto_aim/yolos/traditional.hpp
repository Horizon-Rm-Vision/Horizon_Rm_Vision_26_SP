#ifndef AUTO_AIM__TRADITIONAL_HPP
#define AUTO_AIM__TRADITIONAL_HPP

#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim {

// Ported from Horizon_Rm_Vision_26_ROS rm_auto_aim/armor_detector
// Traditional CV-based armor detector using LeNet for classification

struct TraLight {
  cv::RotatedRect rect;
  Color color;
  cv::Point2f top, bottom, center;
  cv::Point2f axis;
  double length;
  double width;
  float tilt_angle;

  TraLight() = default;
  explicit TraLight(const std::vector<cv::Point> &contour);
};

struct TraArmor {
  TraLight left_light, right_light;
  cv::Point2f center;
  ArmorType type;  // big or small
  cv::Mat number_img;
  std::string number;
  float confidence;

  TraArmor() = default;
  TraArmor(const TraLight &l1, const TraLight &l2);
};

class NumberClassifier {
public:
  NumberClassifier(const std::string &model_path,
                   const std::string &label_path,
                   double threshold,
                   const std::vector<std::string> &ignore_classes = {});

  cv::Mat extractNumber(const cv::Mat &src, const TraArmor &armor);
  void classify(const cv::Mat &src, TraArmor &armor);
  void eraseIgnoreClasses(std::vector<TraArmor> &armors);

  double threshold;

private:
  std::mutex mutex_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

struct SymmetryAxis {
  cv::Point2f centroid;
  cv::Point2f direction;
  float mean_val;
};

class LightCornerCorrector {
public:
  LightCornerCorrector() = default;
  void correctCorners(TraArmor &armor, const cv::Mat &gray_img);

private:
  SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const TraLight &light);
  cv::Point2f findCorner(const cv::Mat &gray_img, const TraLight &light,
                         const SymmetryAxis &axis, const std::string &order);
};

class TraditionalDetector : public YOLOBase {
public:
  TraditionalDetector(const std::string &config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat &img, int frame_count) override;

  std::list<Armor> postprocess(double scale, cv::Mat &output,
                               const cv::Mat &bgr_img, int frame_count) override;

private:
  struct LightParams {
    double min_ratio;
    double max_ratio;
    double max_angle;
    int color_diff_thresh;
  };

  struct ArmorParams {
    double min_light_ratio;
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    double max_angle;
  };

  // Parameters
  int binary_thres_;
  Color detect_color_;
  bool enemy_color_auto_{false};
  LightParams light_params_;
  ArmorParams armor_params_;
  bool use_pca_;
  bool debug_;

  // Classifier and corner corrector
  std::unique_ptr<NumberClassifier> classifier_;
  std::unique_ptr<LightCornerCorrector> corner_corrector_;

  // Internal state
  cv::Mat gray_img_;
  std::vector<TraLight> lights_;
  std::vector<TraArmor> armors_;

  // Detection pipeline
  cv::Mat preprocessImage(const cv::Mat &input);
  std::vector<TraLight> findLights(const cv::Mat &rgb_img, const cv::Mat &binary_img);
  std::vector<TraArmor> matchLights(const std::vector<TraLight> &lights);

  bool isLight(const TraLight &light);
  bool containLight(int i, int j, const std::vector<TraLight> &lights);
  ArmorType isArmor(const TraLight &light_1, const TraLight &light_2);

  // Conversion from TraArmor to SP Armor
  Armor convertToArmor(const TraArmor &tra_armor, const cv::Mat &bgr_img);

  // Color management
  void refresh_enemy_color_from_serial();

  // Debug
  void drawResults(cv::Mat &img);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRADITIONAL_HPP
