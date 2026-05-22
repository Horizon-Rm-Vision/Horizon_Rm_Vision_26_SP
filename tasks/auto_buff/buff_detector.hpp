#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <memory>
#include <optional>

#include "buff_type.hpp"
#include "tools/ui_manager.hpp"
#ifdef USE_OPENVINO
#include "yolo11_buff.hpp"
#include "yolox_buff.hpp"
#endif
#ifdef USE_CUDA
#include "yolox_buff_trt.hpp"
#endif

const int LOSE_MAX = 20;  // 丢失的阙值

namespace auto_buff
{

enum DetectorMode { YOLO11_MODE, YOLOX_MODE, YOLOX_TRT_MODE };

class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img); //支持多目标的检测接口

  std::optional<PowerRune> detect(cv::Mat & bgr_img); //只支持单目标的检测接口

std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  // R tag detection using traditional vision method (ported from ROS)
  std::tuple<cv::Point2f, cv::Mat> detectRTag(
    const cv::Mat & img, int binary_thresh, const cv::Point2f & prior);

  // Shared logic for detect_24 and detect (runs after inference)
  std::optional<PowerRune> processResults(
    std::vector<std::vector<cv::Point2f>> & all_kpts, cv::Mat & bgr_img);

  void handle_lose();

  // Detector instances (only the selected mode is constructed)
#ifdef USE_OPENVINO
  std::unique_ptr<YOLO11_BUFF> MODE_;
  std::unique_ptr<YOLOX_BUFF> MODE_YOLOX_;
#endif
#ifdef USE_CUDA
  std::unique_ptr<YOLOX_BUFF_TRT> MODE_YOLOX_TRT_;
#endif

  // Mode selection
  DetectorMode mode_;

  // Color filtering (for yoloX modes that can distinguish fan blade color)
  // enemy_color in YAML: red → target blue; blue → target red; auto → self_color
  Color target_color_ = Color::unknown;
  bool enemy_color_auto_ = false;
  void refresh_enemy_color_from_serial();

  // R tag detection parameters
  bool detect_r_tag_;
  int binary_thresh_;

  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP
