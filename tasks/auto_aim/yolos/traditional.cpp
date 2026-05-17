#include "tasks/auto_aim/yolos/traditional.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"

namespace auto_aim {

// ============================================================================
// TraLight — ported from ROS Light (types.hpp)
// ============================================================================
TraLight::TraLight(const std::vector<cv::Point> &contour) {
  rect = cv::minAreaRect(contour);

  center = std::accumulate(contour.begin(), contour.end(), cv::Point2f(0, 0),
                           [n = static_cast<float>(contour.size())](const cv::Point2f &a,
                                                                     const cv::Point &b) {
                             return a + cv::Point2f(b.x, b.y) / n;
                           });

  cv::Point2f p[4];
  rect.points(p);
  std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
  top = (p[0] + p[1]) / 2;
  bottom = (p[2] + p[3]) / 2;

  length = cv::norm(top - bottom);
  width = cv::norm(p[0] - p[1]);

  axis = top - bottom;
  axis = axis / cv::norm(axis);

  tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
  tilt_angle = tilt_angle / CV_PI * 180;

  color = Color::extinguish;  // default placeholder
}

// ============================================================================
// TraArmor — ported from ROS Armor (types.hpp)
// ============================================================================
TraArmor::TraArmor(const TraLight &l1, const TraLight &l2) {
  if (l1.center.x < l2.center.x) {
    left_light = l1;
    right_light = l2;
  } else {
    left_light = l2;
    right_light = l1;
  }
  center = (left_light.center + right_light.center) / 2;
}

// ============================================================================
// NumberClassifier — ported from ROS number_classifier.hpp / .cpp
// ============================================================================
NumberClassifier::NumberClassifier(const std::string &model_path,
                                   const std::string &label_path,
                                   double thre,
                                   const std::vector<std::string> &ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat &src, const TraArmor &armor) {
  static const int light_length = 12;
  static const int warp_height = 28;
  static const int small_armor_width = 32;
  static const int large_armor_width = 54;
  static const cv::Size roi_size(20, 28);
  static const cv::Size input_size(28, 28);

  cv::Point2f lights_vertices[4] = {armor.left_light.bottom, armor.left_light.top,
                                    armor.right_light.top, armor.right_light.bottom};

  const int top_light_y = (warp_height - light_length) / 2 - 1;
  const int bottom_light_y = top_light_y + light_length;
  const int warp_width =
      armor.type == ArmorType::big ? large_armor_width : small_armor_width;
  cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y), cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y), cv::Point(warp_width - 1, bottom_light_y)};

  cv::Mat number_image;
  auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
  cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

  number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

  cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
  cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::resize(number_image, number_image, input_size);
  return number_image;
}

void NumberClassifier::classify(const cv::Mat &src, TraArmor &armor) {
  cv::Mat input = armor.number_img / 255.0f;

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

  armor.confidence = static_cast<float>(confidence);
  armor.number = class_names_[label_id];
}

void NumberClassifier::eraseIgnoreClasses(std::vector<TraArmor> &armors) {
  armors.erase(
      std::remove_if(armors.begin(), armors.end(), [this](const TraArmor &a) {
        if (a.confidence < threshold) return true;

        for (const auto &ignore : ignore_classes_) {
          if (a.number == ignore) return true;
        }

        bool mismatch = false;
        if (a.type == ArmorType::big) {
          mismatch = a.number == "outpost" || a.number == "2" || a.number == "sentry" ||
                     a.number == "base";
        } else if (a.type == ArmorType::small) {
          mismatch = a.number == "1";
        }
        return mismatch;
      }),
      armors.end());
}

// ============================================================================
// LightCornerCorrector — ported from ROS light_corner_corrector.hpp / .cpp
// ============================================================================
void LightCornerCorrector::correctCorners(TraArmor &armor, const cv::Mat &gray_img) {
  constexpr int PASS_OPTIMIZE_WIDTH = 3;

  if (armor.left_light.width > PASS_OPTIMIZE_WIDTH) {
    SymmetryAxis left_axis = findSymmetryAxis(gray_img, armor.left_light);
    armor.left_light.center = left_axis.centroid;
    armor.left_light.axis = left_axis.direction;
    if (cv::Point2f t = findCorner(gray_img, armor.left_light, left_axis, "top"); t.x > 0) {
      armor.left_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.left_light, left_axis, "bottom"); b.x > 0) {
      armor.left_light.bottom = b;
    }
  }

  if (armor.right_light.width > PASS_OPTIMIZE_WIDTH) {
    SymmetryAxis right_axis = findSymmetryAxis(gray_img, armor.right_light);
    armor.right_light.center = right_axis.centroid;
    armor.right_light.axis = right_axis.direction;
    if (cv::Point2f t = findCorner(gray_img, armor.right_light, right_axis, "top"); t.x > 0) {
      armor.right_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.right_light, right_axis, "bottom"); b.x > 0) {
      armor.right_light.bottom = b;
    }
  }
}

SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img,
                                                     const TraLight &light) {
  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.07;

  cv::Rect light_box = light.rect.boundingRect();
  light_box.x -= light_box.width * SCALE;
  light_box.y -= light_box.height * SCALE;
  light_box.width += light_box.width * SCALE * 2;
  light_box.height += light_box.height * SCALE * 2;

  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid =
      cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
      cv::Point2f(light_box.x, light_box.y);

  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);
  cv::PCA pca(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  cv::Point2f axis =
      cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis = axis / cv::norm(axis);
  if (axis.y > 0) {
    axis = -axis;
  }

  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                              const TraLight &light,
                                              const SymmetryAxis &axis,
                                              const std::string &order) {
  constexpr float START = 0.8f / 2;
  constexpr float END = 1.2f / 2;

  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  int oper = order == "top" ? 1 : -1;
  float L = static_cast<float>(light.length);
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  int n = static_cast<int>(light.width) - 2;
  int half_n = std::round(n / 2.0f);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;

    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      float brightness_diff =
          static_cast<float>(gray_img.at<uchar>(prev)) - static_cast<float>(gray_img.at<uchar>(cur));
      if (brightness_diff > max_brightness_diff &&
          static_cast<float>(gray_img.at<uchar>(prev)) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }
      prev = cur;
    }

    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }

  if (!candidates.empty()) {
    cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }
  return cv::Point2f(-1, -1);
}

// ============================================================================
// TraditionalDetector — main detection class implementing YOLOBase
// ============================================================================
TraditionalDetector::TraditionalDetector(const std::string &config_path, bool debug)
    : debug_(debug) {
  auto yaml = YAML::LoadFile(config_path);

  binary_thres_ = yaml["traditional"]["binary_thres"].as<int>(90);

  const auto enemy_color_cfg = yaml["enemy_color"].as<std::string>("red");
  if (enemy_color_cfg == "auto") {
    enemy_color_auto_ = true;
    detect_color_ = Color::red;
    refresh_enemy_color_from_serial();
  } else {
    detect_color_ = (enemy_color_cfg == "red") ? Color::red : Color::blue;
  }

  use_pca_ = yaml["traditional"]["use_pca"].as<bool>(true);

  light_params_.min_ratio = yaml["traditional"]["light"]["min_ratio"].as<double>(0.0001);
  light_params_.max_ratio = yaml["traditional"]["light"]["max_ratio"].as<double>(1.0);
  light_params_.max_angle = yaml["traditional"]["light"]["max_angle"].as<double>(40.0);
  light_params_.color_diff_thresh =
      yaml["traditional"]["light"]["color_diff_thresh"].as<int>(20);

  armor_params_.min_light_ratio =
      yaml["traditional"]["armor"]["min_light_ratio"].as<double>(0.8);
  armor_params_.min_small_center_distance =
      yaml["traditional"]["armor"]["min_small_center_distance"].as<double>(0.8);
  armor_params_.max_small_center_distance =
      yaml["traditional"]["armor"]["max_small_center_distance"].as<double>(3.5);
  armor_params_.min_large_center_distance =
      yaml["traditional"]["armor"]["min_large_center_distance"].as<double>(3.5);
  armor_params_.max_large_center_distance =
      yaml["traditional"]["armor"]["max_large_center_distance"].as<double>(8.0);
  armor_params_.max_angle = yaml["traditional"]["armor"]["max_angle"].as<double>(35.0);

  double classifier_threshold =
      yaml["traditional"]["classifier"]["threshold"].as<double>(0.7);
  auto ignore_classes =
      yaml["traditional"]["classifier"]["ignore_classes"].as<std::vector<std::string>>(
          std::vector<std::string>{"negative"});

  auto model_path = yaml["traditional"]["classifier"]["model_path"].as<std::string>();
  auto label_path = yaml["traditional"]["classifier"]["label_path"].as<std::string>();

  classifier_ = std::make_unique<NumberClassifier>(model_path, label_path,
                                                    classifier_threshold, ignore_classes);
  corner_corrector_ = std::make_unique<LightCornerCorrector>();

  tools::logger()->info("[TraditionalDetector] initialized with binary_thres={}, "
                        "enemy_color={}, use_pca={}",
                        binary_thres_, enemy_color_cfg, use_pca_);
}

void TraditionalDetector::refresh_enemy_color_from_serial()
{
  if (!enemy_color_auto_) return;

  const auto self_color = io::latest_self_color();
  if (self_color == 0) {
    detect_color_ = Color::blue;
  } else if (self_color == 1) {
    detect_color_ = Color::red;
  }
}

std::list<Armor> TraditionalDetector::detect(const cv::Mat &img, int frame_count) {
  refresh_enemy_color_from_serial();
  // Note: input is BGR from SP pipeline, but ROS pipeline uses RGB
  // We convert BGR -> RGB since ROS detector expects RGB
  cv::Mat rgb_img;
  cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);

  // 1. Preprocess
  cv::Mat binary_img = preprocessImage(rgb_img);

  // 2. Find lights
  lights_ = findLights(rgb_img, binary_img);
  tools::logger()->debug("[TraditionalDetector] frame={}: binary_thres={}, contours->lights={}",
                          frame_count, binary_thres_, lights_.size());

  // 3. Match lights -> armors
  armors_ = matchLights(lights_);
  tools::logger()->debug("[TraditionalDetector] frame={}: lights={} -> armors={}",
                          frame_count, lights_.size(), armors_.size());

  // 4. Classify + corner correct
  if (!armors_.empty() && classifier_) {
    for (auto &armor : armors_) {
      armor.number_img = classifier_->extractNumber(rgb_img, armor);
      classifier_->classify(rgb_img, armor);
      // if (armor.number == "base") armor.type = ArmorType::small;
      // if (armor.number == "1") armor.type = ArmorType::big;
      if (corner_corrector_ != nullptr && use_pca_) {
        corner_corrector_->correctCorners(armor, gray_img_);
      }
    }
    classifier_->eraseIgnoreClasses(armors_);
  }

  // 5. Convert to SP Armor format
  std::list<Armor> result;
  for (const auto &tra_armor : armors_) {
    result.push_back(convertToArmor(tra_armor, img));
  }

  tools::logger()->debug("[TraditionalDetector] frame={}: final output {} armors",
                          frame_count, result.size());

  return result;
}

std::list<Armor> TraditionalDetector::postprocess(double scale, cv::Mat &output,
                                                   const cv::Mat &bgr_img, int frame_count) {
  // Not used for traditional detector (no neural network postprocessing)
  (void)scale;
  (void)output;
  (void)bgr_img;
  (void)frame_count;
  return {};
}

// ============================================================================
// Detection pipeline — ported from ROS armor_detector.cpp
// ============================================================================
cv::Mat TraditionalDetector::preprocessImage(const cv::Mat &rgb_img) {
  cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img_, binary_img, binary_thres_, 255, cv::THRESH_BINARY);
  return binary_img;
}

std::vector<TraLight> TraditionalDetector::findLights(const cv::Mat &rgb_img,
                                                       const cv::Mat &binary_img) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);

  tools::logger()->debug("[TraditionalDetector] findLights: {} contours found", contours.size());

  std::vector<TraLight> lights;
  for (const auto &contour : contours) {
    if (contour.size() < 6) continue;

    auto light = TraLight(contour);

    if (isLight(light)) {
      int sum_r = 0, sum_b = 0;
      for (const auto &point : contour) {
        sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
        sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
      }
      if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) >
          light_params_.color_diff_thresh) {
        light.color = (sum_r > sum_b) ? Color::red : Color::blue;
      }
      lights.emplace_back(light);
    }
  }

  int red_count = 0, blue_count = 0, unknown_count = 0;
  for (auto &l : lights) {
    if (l.color == Color::red) red_count++;
    else if (l.color == Color::blue) blue_count++;
    else unknown_count++;
  }
  tools::logger()->debug("[TraditionalDetector] findLights: {} lights total (R={} B={} unk={}), "
                          "first_center.x={:.1f}, last_center.x={:.1f}",
                          lights.size(), red_count, blue_count, unknown_count,
                          lights.empty() ? -1 : lights.front().center.x,
                          lights.empty() ? -1 : lights.back().center.x);

  std::sort(lights.begin(), lights.end(),
            [](const TraLight &l1, const TraLight &l2) { return l1.center.x < l2.center.x; });

  return lights;
}

bool TraditionalDetector::isLight(const TraLight &light) {
  float ratio = static_cast<float>(light.width / light.length);
  bool ratio_ok = light_params_.min_ratio < ratio && ratio < light_params_.max_ratio;
  bool angle_ok = light.tilt_angle < light_params_.max_angle;
  return ratio_ok && angle_ok;
}

bool TraditionalDetector::containLight(int i, int j,
                                        const std::vector<TraLight> &lights) {
  const TraLight &light_1 = lights[i], &light_2 = lights[j];
  auto points =
      std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);
  double avg_length = (light_1.length + light_2.length) / 2.0;
  double avg_width = (light_1.width + light_2.width) / 2.0;

  for (int k = i + 1; k < j; k++) {
    const TraLight &test_light = lights[k];

    if (test_light.width > 2 * avg_width) continue;
    if (test_light.length < 0.5 * avg_length) continue;

    if (bounding_rect.contains(test_light.top) ||
        bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
      return true;
    }
  }
  return false;
}

ArmorType TraditionalDetector::isArmor(const TraLight &light_1, const TraLight &light_2) {
  float light_length_ratio =
      light_1.length < light_2.length
          ? static_cast<float>(light_1.length / light_2.length)
          : static_cast<float>(light_2.length / light_1.length);
  bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;

  float avg_light_length = static_cast<float>((light_1.length + light_2.length) / 2);
  float center_distance =
      cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok =
      (armor_params_.min_small_center_distance <= center_distance &&
       center_distance < armor_params_.max_small_center_distance) ||
      (armor_params_.min_large_center_distance <= center_distance &&
       center_distance < armor_params_.max_large_center_distance);

  cv::Point2f diff = light_1.center - light_2.center;
  // Use atan(dy/dx) not atan2 — lights are sorted left-to-right so dx < 0,
  // and we want the absolute horizontal deviation angle, not the vector direction
  float angle = std::abs(std::atan(diff.y / diff.x)) / static_cast<float>(CV_PI) * 180;
  bool angle_ok = angle < armor_params_.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  if (is_armor) {
    return center_distance > armor_params_.min_large_center_distance ? ArmorType::big
                                                                     : ArmorType::small;
  }
  // Use invalid indicator: we'll filter by checking type in matchLights
  // Return small as fallback and let caller check is_armor
  return ArmorType::small;
}

std::vector<TraArmor> TraditionalDetector::matchLights(const std::vector<TraLight> &lights) {
  std::vector<TraArmor> armors;

  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    if (light_1->color != detect_color_) continue;
    double max_iter_width = light_1->length * armor_params_.max_large_center_distance;

    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_2->color != detect_color_) continue;

      if (containLight(static_cast<int>(light_1 - lights.begin()),
                       static_cast<int>(light_2 - lights.begin()), lights)) {
        continue;
      }

      if (light_2->center.x - light_1->center.x > max_iter_width) break;

      int idx1 = static_cast<int>(light_1 - lights.begin());
      int idx2 = static_cast<int>(light_2 - lights.begin());

      // Re-use isArmor logic: check light_ratio_ok, center_distance_ok, angle_ok
      float light_length_ratio =
          light_1->length < light_2->length
              ? static_cast<float>(light_1->length / light_2->length)
              : static_cast<float>(light_2->length / light_1->length);
      bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;
      if (!light_ratio_ok) continue;

      float avg_light_length =
          static_cast<float>((light_1->length + light_2->length) / 2);
      float center_distance =
          cv::norm(light_1->center - light_2->center) / avg_light_length;
      bool center_distance_ok =
          (armor_params_.min_small_center_distance <= center_distance &&
           center_distance < armor_params_.max_small_center_distance) ||
          (armor_params_.min_large_center_distance <= center_distance &&
           center_distance < armor_params_.max_large_center_distance);
      if (!center_distance_ok) continue;

      cv::Point2f diff = light_1->center - light_2->center;
      // Use atan(dy/dx) not atan2 — lights sorted left-to-right, dx < 0
      float angle =
          std::abs(std::atan(diff.y / diff.x)) / static_cast<float>(CV_PI) * 180;
      bool angle_ok = angle < armor_params_.max_angle;
      if (!angle_ok) continue;

      auto armor = TraArmor(*light_1, *light_2);
      armor.type =
          center_distance > armor_params_.min_large_center_distance ? ArmorType::big : ArmorType::small;
      armors.emplace_back(armor);
    }
  }

  return armors;
}

// ============================================================================
// Conversion helpers
// ============================================================================
static ArmorName stringToArmorName(const std::string &number) {
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

Armor TraditionalDetector::convertToArmor(const TraArmor &tra_armor, const cv::Mat &bgr_img) {
  // Construct Lightbar objects from TraLight data
  Lightbar left_lightbar;
  left_lightbar.center = tra_armor.left_light.center;
  left_lightbar.top = tra_armor.left_light.top;
  left_lightbar.bottom = tra_armor.left_light.bottom;
  left_lightbar.top2bottom = tra_armor.left_light.bottom - tra_armor.left_light.top;
  left_lightbar.length = tra_armor.left_light.length;
  left_lightbar.width = tra_armor.left_light.width;
  left_lightbar.angle = std::atan2(left_lightbar.top2bottom.y, left_lightbar.top2bottom.x);
  left_lightbar.angle_error = std::abs(left_lightbar.angle - CV_PI / 2);
  left_lightbar.ratio = left_lightbar.length / left_lightbar.width;

  Lightbar right_lightbar;
  right_lightbar.center = tra_armor.right_light.center;
  right_lightbar.top = tra_armor.right_light.top;
  right_lightbar.bottom = tra_armor.right_light.bottom;
  right_lightbar.top2bottom = tra_armor.right_light.bottom - tra_armor.right_light.top;
  right_lightbar.length = tra_armor.right_light.length;
  right_lightbar.width = tra_armor.right_light.width;
  right_lightbar.angle = std::atan2(right_lightbar.top2bottom.y, right_lightbar.top2bottom.x);
  right_lightbar.angle_error = std::abs(right_lightbar.angle - CV_PI / 2);
  right_lightbar.ratio = right_lightbar.length / right_lightbar.width;

  // Use the Armor(Lightbar, Lightbar) constructor which sets:
  // left, right, color, center, points[4]=[TL,TR,BR,BL], ratio, side_ratio, rectangular_error
  Armor armor(left_lightbar, right_lightbar);
  armor.color = tra_armor.left_light.color;

  armor.center_norm =
      cv::Point2f(tra_armor.center.x / bgr_img.cols, tra_armor.center.y / bgr_img.rows);

  armor.type = tra_armor.type;
  armor.name = stringToArmorName(tra_armor.number);

  // Assign priority based on name
  switch (armor.name) {
    case ArmorName::one:
      armor.priority = ArmorPriority::first;
      break;
    case ArmorName::two:
      armor.priority = ArmorPriority::second;
      break;
    case ArmorName::three:
      armor.priority = ArmorPriority::third;
      break;
    case ArmorName::four:
      armor.priority = ArmorPriority::forth;
      break;
    case ArmorName::five:
      armor.priority = ArmorPriority::fifth;
      break;
    default:
      armor.priority = ArmorPriority::first;
      break;
  }

  armor.confidence = tra_armor.confidence;
  armor.class_id = -1;
  armor.duplicated = false;

  // Pattern for debug
  armor.pattern = tra_armor.number_img.clone();

  // bounding box from points
  if (!armor.points.empty()) {
    std::vector<cv::Point> int_pts;
    for (const auto &pt : armor.points) {
      int_pts.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
    }
    armor.box = cv::boundingRect(int_pts);
  }

  return armor;
}

void TraditionalDetector::drawResults(cv::Mat &img) {
  for (const auto &armor : armors_) {
    cv::line(img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
    cv::line(img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
    cv::line(img, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
    cv::line(img, armor.right_light.bottom, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1,
             cv::LINE_AA);
  }
  for (const auto &armor : armors_) {
    std::string text =
        fmt::format("{} {}", (armor.type == ArmorType::big ? "big" : "small"), armor.number);
    cv::putText(img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace auto_aim
