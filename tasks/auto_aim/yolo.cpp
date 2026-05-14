#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/traditional.hpp"
#ifdef USE_OPENVINO
#include "yolos/yolo11.hpp"
#include "yolos/yolov5_ov.hpp"
#include "yolos/yolov8.hpp"
#include "yolos/yolox_ov.hpp"
#endif

#ifdef USE_CUDA
#include "yolos/yolov5_trt.hpp"
#include "yolos/yolox_trt.hpp"
#endif

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  if (yolo_name == "tra") {
    yolo_ = std::make_unique<TraditionalDetector>(config_path, debug);
  }

#ifdef USE_OPENVINO
  else if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5_ov") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }

  else if (yolo_name == "yolox_ov") {
    yolo_ = std::make_unique<YOLOX_OV>(config_path, debug);
  }
#endif

#ifdef USE_CUDA
  else if (yolo_name == "yolov5_trt") {
    yolo_ = std::make_unique<YOLOV5_TRT>(config_path, debug);
  }
  else if (yolo_name == "yolox_trt") {
    yolo_ = std::make_unique<YOLOX_TRT>(config_path, debug);
  }
#endif
  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim