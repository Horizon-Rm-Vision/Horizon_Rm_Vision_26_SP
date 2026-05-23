#include "yolox_ov.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

#include "tools/logger.hpp"
#include "tools/ui_manager.hpp"

namespace auto_aim
{

// ============================================================================
// SVMClassifier implementation (ported from Horizon_Hero_Aim_26 NumberClassifier)
// ============================================================================

YOLOX_OV::SVMClassifier::SVMClassifier(const std::string & model_path)
{
  svm_model_ = cv::ml::SVM::load(model_path);
  if (svm_model_.empty()) {
    tools::logger()->error("Failed to load SVM model from {}", model_path);
    throw std::runtime_error("SVM model load failed: " + model_path);
  }
  hog_ = new cv::HOGDescriptor(
    cv::Size(32, 32), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 16);
}

YOLOX_OV::SVMClassifier::~SVMClassifier() { delete hog_; }

float YOLOX_OV::SVMClassifier::getDistance(const cv::Point2f & p1, const cv::Point2f & p2)
{
  float x = (p1 - p2).x;
  float y = (p1 - p2).y;
  return std::sqrt(x * x + y * y);
}

bool YOLOX_OV::SVMClassifier::affineNumber(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  static float classify_width_ratio = 0.2f;
  static float classify_height_ratio = 0.5f;

  cv::Point2f correct_points[4];
  cv::Point2f width_vec =
    (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
  cv::Point2f height_vec =
    (corners[3] - corners[0] + corners[2] - corners[1]) / 2;

  correct_points[0] =
    corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[1] =
    corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[2] =
    corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
  correct_points[3] =
    corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

  int width = getDistance(correct_points[0], correct_points[1]);
  int height = getDistance(correct_points[1], correct_points[2]);

  cv::Point2f min_point(9999.0f, 9999.0f);
  cv::Point2f max_point(0.0f, 0.0f);
  for (int i = 0; i < 4; i++) {
    min_point.x = std::min(min_point.x, correct_points[i].x);
    min_point.y = std::min(min_point.y, correct_points[i].y);
    max_point.x = std::max(max_point.x, correct_points[i].x);
    max_point.y = std::max(max_point.y, correct_points[i].y);
  }
  min_point.x = std::max(min_point.x, 0.0f);
  min_point.y = std::max(min_point.y, 0.0f);
  max_point.x = std::min(max_point.x, static_cast<float>(frame.cols));
  max_point.y = std::min(max_point.y, static_cast<float>(frame.rows));

  if (max_point.x <= min_point.x || max_point.y <= min_point.y) return false;

  cv::Mat m_number_roi = frame(cv::Rect(min_point, max_point));

  for (int i = 0; i < 4; i++) correct_points[i] -= min_point;

  cv::Point2f remap_points[4] = {
    cv::Point2f(0, 0), cv::Point2f(static_cast<float>(width), 0),
    cv::Point2f(static_cast<float>(width), static_cast<float>(height)),
    cv::Point2f(0, static_cast<float>(height))};

  cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
  cv::Mat output_roi;
  output_roi.create(cv::Size(width, height), CV_8UC3);

  if (m_number_roi.empty() || output_roi.empty()) return false;

  cv::warpPerspective(m_number_roi, output_roi, trans_matrix, output_roi.size());
  cv::resize(output_roi, number_roi_, cv::Size(32, 32));
  return true;
}

cv::Mat YOLOX_OV::SVMClassifier::pixel(
  const cv::Mat & image, double gamma, double mu, double sigma, int flag)
{
  double K;
  cv::Mat img;
  image.copyTo(img);
  int rows = image.rows;
  int cols = image.cols;
  int channels = image.channels();

  if (flag) {
    for (int i = 0; i < cols; ++i) {
      for (int j = 0; j < rows; ++j) {
        if (channels == 3)
          for (int k = 0; k < channels; ++k) {
            float pix = float(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
            img.at<cv::Vec3b>(i, j)[k] =
              cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
          }
        else {
          float pix = float(img.at<uchar>(i, j)) / 255.0;
          img.at<uchar>(i, j) = cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
        }
      }
    }
  } else {
    for (int i = 0; i < cols; ++i) {
      for (int j = 0; j < rows; ++j) {
        if (channels == 3)
          for (int k = 0; k < channels; ++k) {
            float pix = float(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
            double t = std::pow(mu, gamma);
            K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
            img.at<cv::Vec3b>(i, j)[k] =
              cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
          }
        else {
          float pix = float(img.at<uchar>(i, j)) / 255.0;
          double t = std::pow(mu, gamma);
          K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
          img.at<uchar>(i, j) =
            cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
        }
      }
    }
  }
  return img;
}

cv::Mat YOLOX_OV::SVMClassifier::autoGammaCorrect(const cv::Mat & image)
{
  cv::Mat img, dst;
  image.copyTo(dst);
  image.copyTo(img);

  cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
  cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);

  std::vector<cv::Mat> channels(3);
  cv::split(img, channels);
  cv::Mat V = channels[2];
  V.convertTo(V, CV_32F);
  double max_v;
  cv::minMaxIdx(V, 0, &max_v);
  V /= 255.0;

  cv::Mat Mean, Sigma;
  cv::meanStdDev(V, Mean, Sigma);

  double mu = Mean.at<double>(0);
  double sigma = Sigma.at<double>(0);

  bool High_contrast = (4 * sigma > 0.3333);
  bool High_bright = (mu > 0.65);

  double gamma;
  if (High_contrast)
    gamma = std::exp((1 - (mu + sigma)) / 2.0);
  else
    gamma = -std::log(sigma) / std::log(2);

  return pixel(dst, gamma, mu, sigma, High_bright ? 1 : 0);
}

std::pair<int, double> YOLOX_OV::SVMClassifier::predict(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  if (!affineNumber(frame, corners) || number_roi_.empty())
    return std::pair<int, double>(-1, 0);

  number_roi_ = autoGammaCorrect(number_roi_);

  std::vector<float> hog_descriptors;
  hog_->compute(number_roi_, hog_descriptors, cv::Size(8, 8), cv::Size(0, 0));

  cv::Mat descriptors_mat(1, static_cast<int>(hog_descriptors.size()), CV_32FC1);
  for (size_t i = 0; i < hog_descriptors.size(); ++i)
    descriptors_mat.at<float>(0, i) = hog_descriptors[i] * 100;

  svm_model_->predict(descriptors_mat, class_);
  return std::pair<int, double>(static_cast<int>(class_.at<float>(0)), 1);
}

// ============================================================================
// LeNetClassifier implementation (adapted from tra mode NumberClassifier)
// ============================================================================

static ArmorName stringToArmorName(const std::string & number)
{
  if (number == "1") return ArmorName::one;
  if (number == "2") return ArmorName::two;
  if (number == "3") return ArmorName::three;
  if (number == "4") return ArmorName::four;
  if (number == "5") return ArmorName::five;
  if (number == "outpost") return ArmorName::outpost;
  if (number == "sentry") return ArmorName::sentry;
  if (number == "base") return ArmorName::base;
  return ArmorName::not_armor;
}

YOLOX_OV::LeNetClassifier::LeNetClassifier(
  const std::string & model_path, const std::string & label_path, double threshold)
: threshold(threshold)
{
  net_ = cv::dnn::readNetFromONNX(model_path);
  if (net_.empty()) {
    tools::logger()->error("Failed to load LeNet model from {}", model_path);
    throw std::runtime_error("LeNet model load failed: " + model_path);
  }

  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    tools::logger()->error("Failed to open LeNet label file: {}", label_path);
    throw std::runtime_error("LeNet label file open failed: " + label_path);
  }
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
  tools::logger()->info("LeNetClassifier loaded with {} classes", class_names_.size());
}

bool YOLOX_OV::LeNetClassifier::extractNumber(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  static float classify_width_ratio = 0.2f;
  static float classify_height_ratio = 0.5f;

  cv::Point2f correct_points[4];
  cv::Point2f width_vec = (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
  cv::Point2f height_vec = (corners[3] - corners[0] + corners[2] - corners[1]) / 2;

  correct_points[0] =
    corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[1] =
    corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[2] =
    corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
  correct_points[3] =
    corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

  auto getDistance = [](const cv::Point2f & p1, const cv::Point2f & p2) -> float {
    float x = (p1 - p2).x;
    float y = (p1 - p2).y;
    return std::sqrt(x * x + y * y);
  };

  int width = getDistance(correct_points[0], correct_points[1]);
  int height = getDistance(correct_points[1], correct_points[2]);

  cv::Point2f min_point(9999.0f, 9999.0f);
  cv::Point2f max_point(0.0f, 0.0f);
  for (int i = 0; i < 4; i++) {
    min_point.x = std::min(min_point.x, correct_points[i].x);
    min_point.y = std::min(min_point.y, correct_points[i].y);
    max_point.x = std::max(max_point.x, correct_points[i].x);
    max_point.y = std::max(max_point.y, correct_points[i].y);
  }
  min_point.x = std::max(min_point.x, 0.0f);
  min_point.y = std::max(min_point.y, 0.0f);
  max_point.x = std::min(max_point.x, static_cast<float>(frame.cols));
  max_point.y = std::min(max_point.y, static_cast<float>(frame.rows));

  if (max_point.x <= min_point.x || max_point.y <= min_point.y) return false;

  cv::Mat m_number_roi = frame(cv::Rect(min_point, max_point));

  for (int i = 0; i < 4; i++) correct_points[i] -= min_point;

  cv::Point2f remap_points[4] = {
    cv::Point2f(0, 0), cv::Point2f(static_cast<float>(width), 0),
    cv::Point2f(static_cast<float>(width), static_cast<float>(height)),
    cv::Point2f(0, static_cast<float>(height))};

  cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
  cv::Mat output_roi;
  output_roi.create(cv::Size(width, height), CV_8UC3);

  if (m_number_roi.empty() || output_roi.empty()) return false;

  cv::warpPerspective(m_number_roi, output_roi, trans_matrix, output_roi.size());

  // LeNet-specific preprocessing: grayscale → OTSU threshold → 28x28
  cv::cvtColor(output_roi, number_roi_, cv::COLOR_BGR2GRAY);
  cv::threshold(number_roi_, number_roi_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::resize(number_roi_, number_roi_, cv::Size(28, 28));

  return true;
}

std::pair<ArmorName, double> YOLOX_OV::LeNetClassifier::predict(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  if (!extractNumber(frame, corners) || number_roi_.empty())
    return {ArmorName::not_armor, 0};

  cv::Mat input = number_roi_ / 255.0f;

  cv::Mat blob;
  cv::dnn::blobFromImage(input, blob);

  mutex_.lock();
  net_.setInput(blob);
  cv::Mat outputs = net_.forward().clone();
  mutex_.unlock();

  double confidence;
  cv::Point class_id_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
  int label_id = class_id_point.x;

  if (label_id < 0 || label_id >= static_cast<int>(class_names_.size()))
    return {ArmorName::not_armor, 0};

  return {stringToArmorName(class_names_[label_id]), confidence};
}

// ============================================================================
// ResNetClassifier implementation (adapted from Classifier / tiny_resnet.onnx)
// ============================================================================

YOLOX_OV::ResNetClassifier::ResNetClassifier(const std::string & model_path)
{
  net_ = cv::dnn::readNetFromONNX(model_path);
  if (net_.empty()) {
    tools::logger()->error("Failed to load ResNet model from {}", model_path);
    throw std::runtime_error("ResNet model load failed: " + model_path);
  }
  tools::logger()->info("ResNetClassifier loaded from {}", model_path);
}

bool YOLOX_OV::ResNetClassifier::extractNumber(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  static float classify_width_ratio = 0.2f;
  static float classify_height_ratio = 0.5f;

  cv::Point2f correct_points[4];
  cv::Point2f width_vec = (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
  cv::Point2f height_vec = (corners[3] - corners[0] + corners[2] - corners[1]) / 2;

  correct_points[0] =
    corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[1] =
    corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
  correct_points[2] =
    corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
  correct_points[3] =
    corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

  auto getDistance = [](const cv::Point2f & p1, const cv::Point2f & p2) -> float {
    float x = (p1 - p2).x;
    float y = (p1 - p2).y;
    return std::sqrt(x * x + y * y);
  };

  int width = getDistance(correct_points[0], correct_points[1]);
  int height = getDistance(correct_points[1], correct_points[2]);

  cv::Point2f min_point(9999.0f, 9999.0f);
  cv::Point2f max_point(0.0f, 0.0f);
  for (int i = 0; i < 4; i++) {
    min_point.x = std::min(min_point.x, correct_points[i].x);
    min_point.y = std::min(min_point.y, correct_points[i].y);
    max_point.x = std::max(max_point.x, correct_points[i].x);
    max_point.y = std::max(max_point.y, correct_points[i].y);
  }
  min_point.x = std::max(min_point.x, 0.0f);
  min_point.y = std::max(min_point.y, 0.0f);
  max_point.x = std::min(max_point.x, static_cast<float>(frame.cols));
  max_point.y = std::min(max_point.y, static_cast<float>(frame.rows));

  if (max_point.x <= min_point.x || max_point.y <= min_point.y) return false;

  cv::Mat m_number_roi = frame(cv::Rect(min_point, max_point));

  for (int i = 0; i < 4; i++) correct_points[i] -= min_point;

  cv::Point2f remap_points[4] = {
    cv::Point2f(0, 0), cv::Point2f(static_cast<float>(width), 0),
    cv::Point2f(static_cast<float>(width), static_cast<float>(height)),
    cv::Point2f(0, static_cast<float>(height))};

  cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
  cv::Mat output_roi;
  output_roi.create(cv::Size(width, height), CV_8UC3);

  if (m_number_roi.empty() || output_roi.empty()) return false;

  cv::warpPerspective(m_number_roi, output_roi, trans_matrix, output_roi.size());

  // ResNet-specific preprocessing: grayscale → resize to 32x32
  cv::cvtColor(output_roi, number_roi_, cv::COLOR_BGR2GRAY);

  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / number_roi_.cols;
  auto y_scale = static_cast<double>(32) / number_roi_.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(number_roi_.rows * scale);
  auto w = static_cast<int>(number_roi_.cols * scale);

  if (h == 0 || w == 0) return false;

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(number_roi_, input(roi), {w, h});
  number_roi_ = input;

  return true;
}

std::pair<ArmorName, double> YOLOX_OV::ResNetClassifier::predict(
  const cv::Mat & frame, const std::vector<cv::Point2f> & corners)
{
  if (!extractNumber(frame, corners) || number_roi_.empty())
    return {ArmorName::not_armor, 0};

  auto blob = cv::dnn::blobFromImage(number_roi_, 1.0 / 255.0, cv::Size(), cv::Scalar());

  net_.setInput(blob);
  cv::Mat outputs = net_.forward();

  // softmax
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  if (label_id < 0 || label_id >= 9) return {ArmorName::not_armor, 0};

  return {static_cast<ArmorName>(label_id), confidence};
}

// ============================================================================
// YOLOX decode helpers (ported from Horizon_Hero_Aim_26 Inference.cpp)
// ============================================================================

static inline int argmax(const float * ptr, int len)
{
  int max_arg = 0;
  for (int i = 1; i < len; i++) {
    if (ptr[i] > ptr[max_arg]) max_arg = i;
  }
  return max_arg;
}

cv::Mat YOLOX_OV::scaledResize(cv::Mat & img, Eigen::Matrix<float, 3, 3> & transform_matrix)
{
  float r =
    std::min(input_w_ / (img.cols * 1.0f), input_h_ / (img.rows * 1.0f));
  int unpad_w = static_cast<int>(r * img.cols);
  int unpad_h = static_cast<int>(r * img.rows);

  int dw = input_w_ - unpad_w;
  int dh = input_h_ - unpad_h;
  dw /= 2;
  dh /= 2;

  transform_matrix << 1.0f / r, 0, -dw / r, 0, 1.0f / r, -dh / r, 0, 0, 1;

  cv::Mat re;
  cv::resize(img, re, cv::Size(unpad_w, unpad_h));
  cv::Mat out;
  cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
  return out;
}

void YOLOX_OV::generate_grids_and_stride(
  int target_w, int target_h, const std::vector<int> & strides,
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

void YOLOX_OV::generateYoloxProposals(
  const std::vector<GridAndStride> & grid_strides, const float * feat_ptr,
  const Eigen::Matrix<float, 3, 3> & transform_matrix,
  std::vector<ArmorObject> & objects)
{
  const int num_anchors = static_cast<int>(grid_strides.size());
  const int channels_per_anchor = 9 + num_colors_ + num_classes_;  // 8+1 + 8+8 = 25

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;
    const int basic_pos = anchor_idx * channels_per_anchor;

    float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
    float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
    float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
    float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
    float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
    float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
    float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
    float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

    float box_objectness = feat_ptr[basic_pos + 8];
    int box_color = argmax(feat_ptr + basic_pos + 9, num_colors_);
    int box_class = argmax(feat_ptr + basic_pos + 9 + num_colors_, num_classes_);

    if (box_objectness >= bbox_conf_thresh_) {
      ArmorObject obj;

      Eigen::Matrix<float, 3, 4> apex_norm;
      Eigen::Matrix<float, 3, 4> apex_dst;

      apex_norm << x_1, x_2, x_3, x_4, y_1, y_2, y_3, y_4, 1, 1, 1, 1;
      apex_dst = transform_matrix * apex_norm;

      for (int i = 0; i < 4; i++) {
        obj.apex[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
        obj.pts.push_back(obj.apex[i]);
      }

      std::vector<cv::Point2f> tmp(obj.apex, obj.apex + 4);
      obj.rect = cv::boundingRect(tmp);
      obj.cls = box_class;
      obj.color = box_color;
      obj.prob = box_objectness;

      objects.push_back(obj);
    }
  }
}

static inline float intersection_area(
  const YOLOX_OV::ArmorObject & a, const YOLOX_OV::ArmorObject & b)
{
  cv::Rect_<float> inter = a.rect & b.rect;
  return inter.area();
}

static void qsort_descent_inplace(
  std::vector<YOLOX_OV::ArmorObject> & objects, int left, int right)
{
  int i = left;
  int j = right;
  float p = objects[(left + right) / 2].prob;

  while (i <= j) {
    while (objects[i].prob > p) i++;
    while (objects[j].prob < p) j--;
    if (i <= j) {
      std::swap(objects[i], objects[j]);
      i++;
      j--;
    }
  }
  if (left < j) qsort_descent_inplace(objects, left, j);
  if (i < right) qsort_descent_inplace(objects, i, right);
}

static void qsort_descent_inplace(std::vector<YOLOX_OV::ArmorObject> & objects)
{
  if (objects.empty()) return;
  qsort_descent_inplace(objects, 0, static_cast<int>(objects.size()) - 1);
}

static void nms_sorted_bboxes(
  std::vector<YOLOX_OV::ArmorObject> & objects, std::vector<int> & picked,
  float nms_threshold, float merge_min_iou, float merge_conf_error)
{
  picked.clear();
  const int n = static_cast<int>(objects.size());

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++) areas[i] = objects[i].rect.area();

  for (int i = 0; i < n; i++) {
    YOLOX_OV::ArmorObject & a = objects[i];
    int keep = 1;
    for (int j = 0; j < static_cast<int>(picked.size()); j++) {
      YOLOX_OV::ArmorObject & b = objects[picked[j]];
      float inter_area = intersection_area(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      float iou = inter_area / union_area;
      if (iou > nms_threshold || std::isnan(iou)) {
        keep = 0;
        // Merge logic: if boxes overlap heavily with similar confidence,
        // accumulate corner points for later averaging
        if (iou > merge_min_iou && std::abs(a.prob - b.prob) < merge_conf_error &&
            a.cls == b.cls && a.color == b.color) {
          for (int k = 0; k < 4; k++) b.pts.push_back(a.apex[k]);
        }
      }
    }
    if (keep) picked.push_back(i);
  }
}

void YOLOX_OV::decodeOutputs(
  const float * prob, std::vector<ArmorObject> & objects,
  const Eigen::Matrix<float, 3, 3> & transform_matrix)
{
  std::vector<ArmorObject> proposals;
  std::vector<int> strides = {8, 16, 32};
  std::vector<GridAndStride> grid_strides;

  generate_grids_and_stride(input_w_, input_h_, strides, grid_strides);
  generateYoloxProposals(grid_strides, prob, transform_matrix, proposals);
  qsort_descent_inplace(proposals);

  if (static_cast<int>(proposals.size()) >= topk_) proposals.resize(topk_);

  std::vector<int> picked;
  nms_sorted_bboxes(proposals, picked, nms_thresh_, merge_min_iou_, merge_conf_error_);

  int count = static_cast<int>(picked.size());
  objects.resize(count);
  for (int i = 0; i < count; i++) objects[i] = proposals[picked[i]];
}

static float calcTriangleArea(cv::Point2f pts[3])
{
  auto a = std::sqrt(
    std::pow((pts[0] - pts[1]).x, 2) + std::pow((pts[0] - pts[1]).y, 2));
  auto b = std::sqrt(
    std::pow((pts[1] - pts[2]).x, 2) + std::pow((pts[1] - pts[2]).y, 2));
  auto c = std::sqrt(
    std::pow((pts[2] - pts[0]).x, 2) + std::pow((pts[2] - pts[0]).y, 2));
  auto p = (a + b + c) / 2.0f;
  return std::sqrt(p * (p - a) * (p - b) * (p - c));
}

static float calcTetragonArea(cv::Point2f pts[4])
{
  return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
}

void YOLOX_OV::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) return;

  std::sort(
    keypoints.begin(), keypoints.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  keypoints[0] = top_points[0];      // TL
  keypoints[1] = top_points[1];      // TR
  keypoints[2] = bottom_points[1];   // BR
  keypoints[3] = bottom_points[0];   // BL
}

// ============================================================================
// YOLOX_OV main class
// ============================================================================

YOLOX_OV::YOLOX_OV(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolox_ov_model_path"].as<std::string>();
  svm_model_path_ = yaml["yolox_svm_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  min_confidence_ = yaml["min_confidence"].as<double>();

<<<<<<< HEAD
  // Optional YOLOX-specific params
  if (yaml["yolox_ov"]) {
    auto yx = yaml["yolox_ov"];
=======
  // Optional YOLOX-specific params (shared yolox section for both ov and trt)
  if (yaml["yolox"]) {
    auto yx = yaml["yolox"];
>>>>>>> origin/main
    if (yx["input_width"]) input_w_ = yx["input_width"].as<int>();
    if (yx["input_height"]) input_h_ = yx["input_height"].as<int>();
    if (yx["num_classes"]) num_classes_ = yx["num_classes"].as<int>();
    if (yx["num_colors"]) num_colors_ = yx["num_colors"].as<int>();
    if (yx["topk"]) topk_ = yx["topk"].as<int>();
    if (yx["bbox_conf_thresh"]) bbox_conf_thresh_ = yx["bbox_conf_thresh"].as<float>();
    if (yx["nms_thresh"]) nms_thresh_ = yx["nms_thresh"].as<float>();
    if (yx["merge_conf_error"]) merge_conf_error_ = yx["merge_conf_error"].as<float>();
    if (yx["merge_min_iou"]) merge_min_iou_ = yx["merge_min_iou"].as<float>();
<<<<<<< HEAD
    if (yx["use_svm"]) use_svm_ = yx["use_svm"].as<bool>();
=======
>>>>>>> origin/main
    if (yx["label_correction_mode"])
      label_correction_mode_ = yx["label_correction_mode"].as<std::string>();
    if (yx["lenet_model_path"])
      lenet_model_path_ = yx["lenet_model_path"].as<std::string>();
    if (yx["lenet_label_path"])
      lenet_label_path_ = yx["lenet_label_path"].as<std::string>();
    if (yx["lenet_threshold"])
      lenet_threshold_ = yx["lenet_threshold"].as<double>();
    if (yx["resnet_model_path"])
      resnet_model_path_ = yx["resnet_model_path"].as<std::string>();
  }

<<<<<<< HEAD
  // If no label_correction_mode specified, derive from use_svm_ for backward compat
  if (label_correction_mode_.empty()) {
    label_correction_mode_ = use_svm_ ? "svm" : "none";
=======
  // "none" or empty/missing disables label correction
  if (label_correction_mode_.empty() || label_correction_mode_ == "none") {
    label_correction_mode_.clear();
>>>>>>> origin/main
  }

  // ROI
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  // Init classifier based on label correction mode
  if (label_correction_mode_ == "svm") {
    classifier_ = std::make_unique<SVMClassifier>(svm_model_path_);
    tools::logger()->info("YOLOX_OV: using SVM label correction");
  } else if (label_correction_mode_ == "lenet") {
    lenet_classifier_ = std::make_unique<LeNetClassifier>(
      lenet_model_path_, lenet_label_path_, lenet_threshold_);
    tools::logger()->info("YOLOX_OV: using LeNet label correction, threshold={}", lenet_threshold_);
  } else if (label_correction_mode_ == "resnet") {
    resnet_classifier_ = std::make_unique<ResNetClassifier>(resnet_model_path_);
    tools::logger()->info("YOLOX_OV: using ResNet label correction");
  } else {
    tools::logger()->info("YOLOX_OV: no label correction (using YOLOX built-in classes)");
  }

  // Init OpenVINO model
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  ppp.input().tensor().set_element_type(ov::element::f32);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  infer_request_ = compiled_model_.create_infer_request();
}

std::list<Armor> YOLOX_OV::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("YOLOX_OV: Empty image!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width <= 0) roi_.width = raw_img.cols;
    if (roi_.height <= 0) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // Letterbox resize to model input size
  cv::Mat pr_img = scaledResize(bgr_img, transform_matrix_);

  // Prepare float32 NCHW input
  cv::Mat pre;
  cv::Mat pre_split[3];
  pr_img.convertTo(pre, CV_32F);
  cv::split(pre, pre_split);

  ov::Tensor input_tensor = infer_request_.get_input_tensor(0);
  float * tensor_data = input_tensor.data<float>();
  auto img_offset = input_w_ * input_h_;
  for (int c = 0; c < 3; c++) {
    std::memcpy(tensor_data, pre_split[c].data, img_offset * sizeof(float));
    tensor_data += img_offset;
  }

  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  ov::Tensor output_tensor = infer_request_.get_output_tensor();
  float * output = output_tensor.data<float>();

  std::vector<ArmorObject> objects;
  decodeOutputs(output, objects, transform_matrix_);

  // Average merged corner points and apply ROI offset
  for (auto & object : objects) {
    if (object.pts.size() >= 8) {
      auto N = object.pts.size();
      cv::Point2f pts_final[4] = {};
      for (size_t i = 0; i < N; i++) pts_final[i % 4] += object.pts[i];
      for (int i = 0; i < 4; i++) {
        pts_final[i].x /= (N / 4);
        pts_final[i].y /= (N / 4);
      }
      if (use_roi_) {
        for (int i = 0; i < 4; i++)
          object.apex[i] = pts_final[i] + offset_;
      } else {
        for (int i = 0; i < 4; i++) object.apex[i] = pts_final[i];
      }
    } else {
      if (use_roi_) {
        for (int i = 0; i < 4; i++) object.apex[i] = object.apex[i] + offset_;
      }
    }
    object.area = static_cast<int>(calcTetragonArea(object.apex));
  }

  // Parse: classify and convert to SP Armors
  return parse(0 /* unused scale */, raw_img, frame_count, objects);
}

std::list<Armor> YOLOX_OV::parse(
  double /*scale*/, const cv::Mat & bgr_img, int frame_count,
  std::vector<ArmorObject> & objects)
{
  // Class ID to YOLOV5-style num_id mapping.
  // SVM: 0=Undef, 1=Hero, 2=Eng, 3/4/5=Inf, 6=Sentry, 7=Outpost, 8=Base
  // YOLOX cls: 0=Sentry, 1=Hero, 2=Eng, 3/4/5=Inf, 6=Outpost, 7=Base
<<<<<<< HEAD
  // num_id: -1=skip, 0=Sentry, 1=Hero, 2=Eng, 3/4/5=Inf, 6=Outpost, 7=Base
  static const int svm_to_num_id[] = {-1, 1, 2, 3, 4, 5, 0, 6, 7};
  static const int yolox_cls_to_num_id[] = {0, 1, 2, 3, 4, 5, 6, 7};
=======
  // num_id encoding (YOLOV5 constructor): 0=Sentry, 1=Hero, 2=Eng,
  //   3/4/5=Inf, 6=Outpost, 7=Base
  // ArmorName enum: one=0, two=1, three=2, four=3, five=4,
  //   sentry=5, outpost=6, base=7, not_armor=8
  static const int svm_to_num_id[] = {-1, 1, 2, 3, 4, 5, 0, 6, 7};
  static const int yolox_cls_to_num_id[] = {0, 1, 2, 3, 4, 5, 6, 7};
  // ArmorName enum → YOLOV5 num_id (reverse of constructor's name mapping)
  static const int armor_name_to_num_id[] = {1, 2, 3, 4, 5, 0, 6, 7, -1};
  //                                          one two thr fou fiv sen out bas not_armor
>>>>>>> origin/main

  std::list<Armor> armors;

  for (auto & object : objects) {
<<<<<<< HEAD
    // Build sorted keypoints from apex (TL, TR, BR, BL order)
    std::vector<cv::Point2f> keypoints = {
      object.apex[0], object.apex[1], object.apex[2], object.apex[3]};
    sort_keypoints(keypoints);
=======
    // Build keypoints from apex in TL, TR, BR, BL order.
    // YOLOX model outputs corners clockwise from TL:
    //   apex[0]=TL, apex[1]=BL, apex[2]=BR, apex[3]=TR
    // Reorder via Hero_Aim convention (swap apex[1]↔apex[3]):
    //   {apex[0], apex[3], apex[2], apex[1]} = {TL, TR, BR, BL}
    std::vector<cv::Point2f> keypoints = {
      object.apex[0], object.apex[3], object.apex[2], object.apex[1]};
>>>>>>> origin/main

    int num_id = -1, svm_class = -1;
    double cls_confidence = object.prob;

    if (label_correction_mode_ == "svm") {
      // ---- SVM label correction (original) ----
      std::vector<cv::Point2f> corners = {keypoints[0], keypoints[1], keypoints[2], keypoints[3]};
      auto [cls, svm_conf] = classifier_->predict(bgr_img, corners);

      if (cls < 1 || cls > 8) continue;  // skip Undefined
      num_id = svm_to_num_id[cls];
      svm_class = cls;
    } else if (label_correction_mode_ == "lenet") {
      // ---- LeNet label correction (tra mode) ----
      std::vector<cv::Point2f> corners = {keypoints[0], keypoints[1], keypoints[2], keypoints[3]};
      auto [name, conf] = lenet_classifier_->predict(bgr_img, corners);

      if (name == ArmorName::not_armor) continue;
      if (conf < lenet_classifier_->threshold) continue;

<<<<<<< HEAD
      // Type-matching rules from NumberClassifier::eraseIgnoreClasses
      ArmorType determined_type = ArmorType::small;
      if (determined_type == ArmorType::big) {
        if (name == ArmorName::outpost || name == ArmorName::two || name == ArmorName::sentry || name == ArmorName::base)
          continue;
      } else {
        if (name == ArmorName::one)
          continue;
      }

      num_id = static_cast<int>(name);
      svm_class = (name == ArmorName::base) ? 8 : 1;
=======
      // Type-matching rules from NumberClassifier::eraseIgnoreClasses.
      // Determine expected type from LeNet-predicted name (Hero→big, others→small),
      // matching the type override logic applied downstream.
      ArmorType determined_type =
        (name == ArmorName::one) ? ArmorType::big : ArmorType::small;
      if (determined_type == ArmorType::big) {
        if (name == ArmorName::outpost || name == ArmorName::two ||
            name == ArmorName::sentry || name == ArmorName::base)
          continue;
      } else {
        if (name == ArmorName::one) continue;
      }

      num_id = armor_name_to_num_id[static_cast<int>(name)];
>>>>>>> origin/main
      cls_confidence = conf;
    } else if (label_correction_mode_ == "resnet") {
      // ---- ResNet label correction (traditional_cv mode) ----
      std::vector<cv::Point2f> corners = {keypoints[0], keypoints[1], keypoints[2], keypoints[3]};
      auto [name, conf] = resnet_classifier_->predict(bgr_img, corners);

      if (name == ArmorName::not_armor) continue;

<<<<<<< HEAD
      num_id = static_cast<int>(name);
      svm_class = (name == ArmorName::base) ? 8 : 1;
=======
      // Type-matching rules (same as LeNet path for consistency)
      ArmorType determined_type =
        (name == ArmorName::one) ? ArmorType::big : ArmorType::small;
      if (determined_type == ArmorType::big) {
        if (name == ArmorName::outpost || name == ArmorName::two ||
            name == ArmorName::sentry || name == ArmorName::base)
          continue;
      } else {
        if (name == ArmorName::one) continue;
      }

      num_id = armor_name_to_num_id[static_cast<int>(name)];
>>>>>>> origin/main
      cls_confidence = conf;
    } else {
      // ---- No label correction (use YOLOX built-in class) ----
      if (object.cls < 0 || object.cls >= num_classes_) continue;
      num_id = yolox_cls_to_num_id[object.cls];
      svm_class = object.cls + 1;  // YOLOX cls 0-based, adjust for Base check below
    }

    if (num_id < 0) continue;

    // color_id for YOLOV5 constructor: 0=Blue, 1=Red, 2+=extinguish
    int color_id = object.color / 2;
    if (color_id > 1) color_id = 2;  // Gray/Purple → extinguish

    auto box = cv::boundingRect(keypoints);

    // Use YOLOV5-style constructor (color_id, num_id, ...)
    armors.emplace_back(color_id, num_id, cls_confidence, box, keypoints);

    auto & armor = armors.back();

    // ---- SVM vs YOLOX label comparison ----
    if (label_correction_mode_ == "svm") {
      int yolox_num_id = (object.cls >= 0 && object.cls < num_classes_)
                           ? yolox_cls_to_num_id[object.cls] : -1;
      if (yolox_num_id != num_id && yolox_num_id >= 0) {
        // Mirror Armor constructor num_id→name mapping
        ArmorName yolo_name = yolox_num_id == 0    ? ArmorName::sentry
                            : yolox_num_id > 5     ? static_cast<ArmorName>(yolox_num_id)
                                                   : static_cast<ArmorName>(yolox_num_id - 1);
        armor.class_id = static_cast<int>(yolo_name);
        tools::logger()->info(
          "SVM override: {} -> {} (conf={:.2f})",
          ARMOR_NAMES[yolo_name], ARMOR_NAMES[armor.name], armor.confidence);
      } else {
        armor.class_id = -1;
      }
    } else {
      armor.class_id = -1;
    }

<<<<<<< HEAD
    // Override type: only Base (svm_class 8) is big in SP convention
    armor.type = (svm_class == 1) ? ArmorType::big : ArmorType::small;
=======
    // Type based on armor name, matching check_type expectations.
    // Hero is treated as "big" for ballistics (legacy Hero_Aim convention);
    // all other classes default to small. The Armor constructor may have
    // already set type, but we override here for consistency across modes.
    armor.type =
      (armor.name == ArmorName::one) ? ArmorType::big : ArmorType::small;
>>>>>>> origin/main
  }

  // Filter
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

bool YOLOX_OV::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool YOLOX_OV::check_type(const Armor & armor) const
{
  return (armor.type == ArmorType::small)
           ? (armor.name != ArmorName::one)
           : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
              armor.name != ArmorName::outpost && armor.name != ArmorName::base);
}

cv::Point2f YOLOX_OV::get_center_norm(
  const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

std::list<Armor> YOLOX_OV::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // For multi-threaded pipeline: output tensor is already available
  // Decode and parse from raw output
  std::vector<ArmorObject> objects;
  decodeOutputs(output.ptr<float>(), objects, transform_matrix_);

  // Apply merge averaging and ROI offset
  for (auto & object : objects) {
    if (object.pts.size() >= 8) {
      auto N = object.pts.size();
      cv::Point2f pts_final[4] = {};
      for (size_t i = 0; i < N; i++) pts_final[i % 4] += object.pts[i];
      for (int i = 0; i < 4; i++) {
        pts_final[i].x /= (N / 4);
        pts_final[i].y /= (N / 4);
      }
      if (use_roi_) {
        for (int i = 0; i < 4; i++)
          object.apex[i] = pts_final[i] + offset_;
      } else {
        for (int i = 0; i < 4; i++) object.apex[i] = pts_final[i];
      }
    } else {
      if (use_roi_)
        for (int i = 0; i < 4; i++) object.apex[i] = object.apex[i] + offset_;
    }
    object.area = static_cast<int>(calcTetragonArea(object.apex));
  }

  return parse(scale, bgr_img, frame_count, objects);
}

void YOLOX_OV::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  // auto detection = img.clone();
  // tools::draw_text(
  //   detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  // for (const auto & armor : armors) {
  //   auto info = fmt::format(
  //     "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
  //     ARMOR_TYPES[armor.type]);
  //   if (armor.class_id >= 0 && armor.class_id <= 7)
  //     info += fmt::format("({})", ARMOR_NAMES[armor.class_id]);
  //   tools::draw_points(detection, armor.points, {0, 255, 0});
  //   tools::draw_text(
  //     detection, info, cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)),
  //     {0, 255, 0});
  // }

  // if (use_roi_) {
  //   cv::rectangle(detection, roi_, cv::Scalar(0, 0, 255), 2);
  // }

  // cv::resize(detection, detection, {}, 0.5, 0.5);
  // cv::imshow("detection", detection);
}

}  // namespace auto_aim
