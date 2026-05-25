#include "buff_detector.hpp"

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"

namespace auto_buff
{
Buff_Detector::Buff_Detector(const std::string & config)
: status_(LOSE), lose_(0)
{
  auto yaml = YAML::LoadFile(config);
  std::string mode_str = yaml["detector_mode"].as<std::string>("yolo11");
  if (mode_str == "yolox_ov") {
#ifdef USE_OPENVINO
    mode_ = YOLOX_MODE;
    tools::logger()->info("[Buff_Detector] Using YOLOX OV mode");
    MODE_YOLOX_ = std::make_unique<YOLOX_BUFF>(config);
#else
    tools::logger()->error("[Buff_Detector] YOLOX OV mode requires OpenVINO (USE_OPENVINO=OFF)");
#endif
  } else if (mode_str == "yolox_trt") {
#ifdef USE_CUDA
    mode_ = YOLOX_TRT_MODE;
    tools::logger()->info("[Buff_Detector] Using YOLOX TRT mode");
    MODE_YOLOX_TRT_ = std::make_unique<YOLOX_BUFF_TRT>(config);
#else
    #ifdef USE_OPENVINO
    tools::logger()->warn("[Buff_Detector] YOLOX TRT mode requested but USE_CUDA not enabled, falling back to YOLO11");
    mode_ = YOLO11_MODE;
    MODE_ = std::make_unique<YOLO11_BUFF>(config);
    #else
    tools::logger()->warn("[Buff_Detector] YOLOX TRT mode requested but neither CUDA nor OpenVINO enabled");
    #endif
#endif
  } else {
#ifdef USE_OPENVINO
    mode_ = YOLO11_MODE;
    tools::logger()->info("[Buff_Detector] Using YOLO11 mode");
    MODE_ = std::make_unique<YOLO11_BUFF>(config);
#else
    tools::logger()->error("[Buff_Detector] YOLO11 mode requires OpenVINO (USE_OPENVINO=OFF)");
#endif
  }

  // R tag detection parameters (ROS port)
  detect_r_tag_ = yaml["detect_r_tag"].as<bool>(true);
  binary_thresh_ = yaml["binary_thresh"].as<int>(100);
  r_tag_roi_size_ = yaml["r_tag_roi_size"].as<int>(200);

  // Read enemy_color for target color filtering (yoloX modes only)
  // Unlike auto_aim, auto_buff targets the OPPOSITE color:
  //   enemy_color=red  → target blue buff
  //   enemy_color=blue → target red buff
  //   enemy_color=auto → target = self_color (same as own car, for energy buff)
  const auto enemy_color_cfg = yaml["enemy_color"].as<std::string>("red");
  if (enemy_color_cfg == "auto") {
    enemy_color_auto_ = true;
    refresh_enemy_color_from_serial();
  } else {
    target_color_ = (enemy_color_cfg == "red") ? Color::blue : Color::red;
  }
}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);  // 彩色图转灰度图
  // cv::imshow("gray", gray_img);  // 调试用

  // 进行二值化           :把高于100变成255，低于100变成0
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
  // cv::imshow("binary", binary_img);  // 调试用

  // 膨胀
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 使用矩形核
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  /// error

  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  /// 算出大概位置

  cv::Point2f r_center_t = {0, 0};
  for (auto & fanblade : fanblades) {
    auto point5 = fanblade.points[4];  // point5是扇叶的中心
    auto point6 = fanblade.points[5];
    r_center_t += (point6 - point5) * 1.4 + point5;  // TODO
    // r_center_t += 4.7 * point - (4.7 - 1) * fanblade.center;
  }
  r_center_t /= float(fanblades.size());

  /// 处理图片,mask选出大概范围

  cv::Mat dilated_img;
  handle_img(bgr_img, dilated_img);
  double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
  cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // mask
  circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);               // 将遮罩应用于二值化图像
  tools::draw_point(bgr_img, r_center_t, {255, 255, 0}, 5);  // 调试用
  // cv::imshow("Dilated Image", dilated_img);                // 调试用

  /// 获取轮廓点,矩阵框筛选  TODO

  std::vector<std::vector<cv::Point>> contours;
  auto r_center = r_center_t;
  cv::findContours(
    dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // external找外部区域
  double ratio_1 = INF;
  for (auto & it : contours) {
    auto rotated_rect = cv::minAreaRect(it);
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3);
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }
  return r_center;
};

std::tuple<cv::Point2f, cv::Mat> Buff_Detector::detectRTag(
  const cv::Mat & img, int binary_thresh, int roi_size, const cv::Point2f & prior)
{
  int half = roi_size / 2;

  // Check if prior is within image bounds
  if (prior.x < 0 || prior.x > img.cols || prior.y < 0 || prior.y > img.rows) {
    cv::Mat empty_roi = cv::Mat::zeros(cv::Size(roi_size, roi_size), CV_8UC3);
    return {prior, empty_roi};
  }

  // Create ROI around prior point
  cv::Rect roi = cv::Rect(prior.x - half, prior.y - half, roi_size, roi_size) &
                 cv::Rect(0, 0, img.cols, img.rows);
  cv::Point2f prior_in_roi = prior - cv::Point2f(roi.tl());

  cv::Mat img_roi = img(roi);

  // Gray -> Binary -> Dilate
  cv::Mat gray_img;
  cv::cvtColor(img_roi, gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thresh, 255, cv::THRESH_BINARY);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(binary_img, binary_img, kernel);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // Find contour containing the prior point
  auto it =
    std::find_if(contours.begin(), contours.end(),
                 [p = prior_in_roi](const std::vector<cv::Point> & contour) -> bool {
                   return cv::boundingRect(contour).contains(p);
                 });

  // Convert to BGR for visualization
  cv::cvtColor(binary_img, binary_img, cv::COLOR_GRAY2BGR);

  if (it == contours.end()) {
    return {prior, binary_img};
  }

  cv::drawContours(binary_img, contours, static_cast<int>(it - contours.begin()),
                   cv::Scalar(0, 255, 0), 2);

  // Compute center of the contour
  cv::Point2f center(0, 0);
  for (const auto & p : *it) center += cv::Point2f(p);
  center /= static_cast<float>(it->size());
  center += cv::Point2f(roi.tl());

  return {center, binary_img};
}

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
    return; //SP原版无此行,为bug,无此行会导致永远无法进入LOSE状态,无条件执行。但是是status_没有被任何调用方使用（只在 detector 内部读/写），且关键的 last_powerrune_ 确实被正确清空
  }
  status_ = TEM_LOSE;
}

void Buff_Detector::refresh_enemy_color_from_serial()
{
  if (!enemy_color_auto_) return;
  const auto self_color = io::latest_self_color();
  if (self_color == 0) {
    target_color_ = Color::red;   // self is red team → target own red buff
  } else if (self_color == 1) {
    target_color_ = Color::blue;  // self is blue team → target own blue buff
  }
}

std::optional<PowerRune> Buff_Detector::processResults(
  std::vector<std::vector<cv::Point2f>> & all_kpts, cv::Mat & bgr_img)
{
  if (all_kpts.empty()) {
    handle_lose();
    return std::nullopt;
  }

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades;
  for (auto & kpt : all_kpts) fanblades.emplace_back(FanBlade(kpt, kpt[4], _light));

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
#ifdef USE_CUDA
  if (mode_ == YOLOX_TRT_MODE) {
    /// YOLOX TRT model detection

    std::vector<YOLOX_BUFF_TRT::Object> results = MODE_YOLOX_TRT_->get_multicandidateboxes(bgr_img);

    /// 处理未获得的情况

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF_TRT::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) {
        handle_lose();
        return std::nullopt;
      }
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF_TRT::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 提取原始r_center（用于detectRTag）, 构建FanBlade

    std::vector<cv::Point2f> raw_r_centers;
    std::vector<std::vector<cv::Point2f>> all_kpts;
    std::vector<Color> result_colors;
    for (auto & result : results) {
      raw_r_centers.push_back(result.kpt[5]);
      all_kpts.push_back(result.kpt);
      result_colors.push_back((result.color == 0) ? Color::red : Color::blue);
    }

    std::vector<FanBlade> fanblades;
    for (size_t i = 0; i < all_kpts.size(); ++i)
      fanblades.emplace_back(FanBlade(all_kpts[i], all_kpts[i][4], _light, result_colors[i]));

    /// 使用ROS版的r_tag机制获取r_center

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_ && !raw_r_centers.empty()) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_centers[0]);
    } else {
      r_center = raw_r_centers.empty() ? cv::Point2f(0, 0) : raw_r_centers[0];
    }

    /// 右下角二值化小窗口（移植自ROS）

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    /// 生成PowerRune
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;

  }
#endif
#ifdef USE_OPENVINO
  if (mode_ == YOLOX_MODE) {
    /// YOLOX model detection

    std::vector<YOLOX_BUFF::Object> results = MODE_YOLOX_->get_multicandidateboxes(bgr_img);

    /// 处理未获得的情况

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) {
        handle_lose();
        return std::nullopt;
      }
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 提取原始r_center（用于detectRTag）, 构建FanBlade

    std::vector<cv::Point2f> raw_r_centers;
    std::vector<std::vector<cv::Point2f>> all_kpts;
    std::vector<Color> result_colors;
    for (auto & result : results) {
      raw_r_centers.push_back(result.kpt[5]);
      all_kpts.push_back(result.kpt);
      result_colors.push_back((result.color == 0) ? Color::red : Color::blue);
    }

    std::vector<FanBlade> fanblades;
    for (size_t i = 0; i < all_kpts.size(); ++i)
      fanblades.emplace_back(FanBlade(all_kpts[i], all_kpts[i][4], _light, result_colors[i]));

    /// 使用ROS版的r_tag机制获取r_center

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_ && !raw_r_centers.empty()) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_centers[0]);
    } else {
      r_center = raw_r_centers.empty() ? cv::Point2f(0, 0) : raw_r_centers[0];
    }

    /// 右下角二值化小窗口（移植自ROS）

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    /// 生成PowerRune
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;

  } else {
    /// YOLO11 model detection (original)

    std::vector<YOLO11_BUFF::Object> results = MODE_->get_multicandidateboxes(bgr_img);

    /// 处理未获得的情况

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// results转扇叶FanBlade

    std::vector<FanBlade> fanblades;
    for (auto & result : results) fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

    /// 生成PowerRune
    auto r_center = get_r_center(fanblades, bgr_img);
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    /// handle error
    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;
  }
#else
    handle_lose();
    return std::nullopt;
#endif
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
#ifdef USE_CUDA
  if (mode_ == YOLOX_TRT_MODE) {
    /// YOLOX TRT model detection (use NMS, take top-1)

    std::vector<YOLOX_BUFF_TRT::Object> results = MODE_YOLOX_TRT_->get_multicandidateboxes(bgr_img);

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF_TRT::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) {
        handle_lose();
        return std::nullopt;
      }
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF_TRT::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 取置信度最高的结果
    std::sort(results.begin(), results.end(),
              [](const YOLOX_BUFF_TRT::Object & a, const YOLOX_BUFF_TRT::Object & b) {
                return a.prob > b.prob;
              });

    /// 提取原始r_center, 构建FanBlade

    cv::Point2f raw_r_center = results[0].kpt[5];
    std::vector<cv::Point2f> kpt = results[0].kpt;
    Color obj_color = (results[0].color == 0) ? Color::red : Color::blue;

    std::vector<FanBlade> fanblades;
    fanblades.emplace_back(FanBlade(kpt, kpt[4], _light, obj_color));

    /// ROS版r_tag机制获取r_center

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_center);
    } else {
      r_center = raw_r_center;
    }

    /// 右下角二值化小窗口

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    /// 生成PowerRune
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;

  }
#endif
#ifdef USE_OPENVINO
  if (mode_ == YOLOX_MODE) {
    /// YOLOX model detection (use NMS, take top-1)

    std::vector<YOLOX_BUFF::Object> results = MODE_YOLOX_->get_multicandidateboxes(bgr_img);

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) {
        handle_lose();
        return std::nullopt;
      }
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// 取置信度最高的结果
    std::sort(results.begin(), results.end(),
              [](const YOLOX_BUFF::Object & a, const YOLOX_BUFF::Object & b) {
                return a.prob > b.prob;
              });

    /// 提取原始r_center, 构建FanBlade

    cv::Point2f raw_r_center = results[0].kpt[5];
    std::vector<cv::Point2f> kpt = results[0].kpt;
    Color obj_color = (results[0].color == 0) ? Color::red : Color::blue;

    std::vector<FanBlade> fanblades;
    fanblades.emplace_back(FanBlade(kpt, kpt[4], _light, obj_color));

    /// ROS版r_tag机制获取r_center

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_center);
    } else {
      r_center = raw_r_center;
    }

    /// 右下角二值化小窗口

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    /// 生成PowerRune
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;

  } else {
    /// YOLO11 model detection (original)

    std::vector<YOLO11_BUFF::Object> results = MODE_->get_onecandidatebox(bgr_img);

    /// 处理未获得的情况

    if (results.empty()) {
      handle_lose();
      return std::nullopt;
    }

    /// results转扇叶FanBlade

    std::vector<FanBlade> fanblades;
    auto result = results[0];
    fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

    /// 生成PowerRune
    auto r_center = get_r_center(fanblades, bgr_img);
    PowerRune powerrune(fanblades, r_center, last_powerrune_);

    /// handle error
    if (powerrune.is_unsolve()) {
      handle_lose();
      return std::nullopt;
    }

    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;
  }
#else
    handle_lose();
    return std::nullopt;
#endif
}

std::optional<PowerRune> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f v)
{
#ifdef USE_CUDA
  if (mode_ == YOLOX_TRT_MODE) {
    /// YOLOX TRT model detection

    std::vector<YOLOX_BUFF_TRT::Object> results = MODE_YOLOX_TRT_->get_multicandidateboxes(bgr_img);

    if (results.empty()) return std::nullopt;

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF_TRT::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) return std::nullopt;
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF_TRT::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) return std::nullopt;

    /// 提取原始r_center, 构建FanBlades

    std::vector<cv::Point2f> raw_r_centers;
    std::vector<std::vector<cv::Point2f>> all_kpts;
    std::vector<Color> result_colors;
    for (auto & result : results) {
      raw_r_centers.push_back(result.kpt[5]);
      all_kpts.push_back(result.kpt);
      result_colors.push_back((result.color == 0) ? Color::red : Color::blue);
    }

    std::vector<FanBlade> fanblades_t;
    for (size_t i = 0; i < all_kpts.size(); ++i)
      fanblades_t.emplace_back(FanBlade(all_kpts[i], all_kpts[i][4], _light, result_colors[i]));

    /// 计算r_center,筛选fanblade

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_ && !raw_r_centers.empty()) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_centers[0]);
    } else {
      r_center = raw_r_centers.empty() ? cv::Point2f(0, 0) : raw_r_centers[0];
    }

    /// 右下角二值化小窗口

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    std::vector<FanBlade> fanblades;
    for (auto & fanblade : fanblades_t) {
      if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
        fanblades.emplace_back(fanblade);
        break;
      }
    }
    if (fanblades.empty()) return std::nullopt;
    PowerRune powerrune(fanblades, r_center, std::nullopt);

    std::optional<PowerRune> P;
    P.emplace(powerrune);
    return P;

  }
#endif
#ifdef USE_OPENVINO
  if (mode_ == YOLOX_MODE) {
    /// YOLOX model detection

    std::vector<YOLOX_BUFF::Object> results = MODE_YOLOX_->get_multicandidateboxes(bgr_img);

    if (results.empty()) return std::nullopt;

    /// 颜色过滤（auto模式刷新串口颜色）

    if (enemy_color_auto_) refresh_enemy_color_from_serial();
    if (target_color_ != Color::unknown) {
      results.erase(std::remove_if(results.begin(), results.end(),
          [this](const YOLOX_BUFF::Object & obj) {
            Color obj_color = (obj.color == 0) ? Color::red : Color::blue;
            return obj_color != target_color_;
          }), results.end());
      if (results.empty()) return std::nullopt;
    }

    /// 过滤已激活的扇叶 (label=1=ACTIVATED), 只保留未激活的
    results.erase(std::remove_if(results.begin(), results.end(),
                                 [](const YOLOX_BUFF::Object & obj) { return obj.label == 1; }),
                  results.end());
    if (results.empty()) return std::nullopt;

    /// 提取原始r_center, 构建FanBlades

    std::vector<cv::Point2f> raw_r_centers;
    std::vector<std::vector<cv::Point2f>> all_kpts;
    std::vector<Color> result_colors;
    for (auto & result : results) {
      raw_r_centers.push_back(result.kpt[5]);
      all_kpts.push_back(result.kpt);
      result_colors.push_back((result.color == 0) ? Color::red : Color::blue);
    }

    std::vector<FanBlade> fanblades_t;
    for (size_t i = 0; i < all_kpts.size(); ++i)
      fanblades_t.emplace_back(FanBlade(all_kpts[i], all_kpts[i][4], _light, result_colors[i]));

    /// 计算r_center,筛选fanblade

    cv::Point2f r_center;
    cv::Mat binary_roi;
    if (detect_r_tag_ && !raw_r_centers.empty()) {
      std::tie(r_center, binary_roi) = detectRTag(bgr_img, binary_thresh_, r_tag_roi_size_, raw_r_centers[0]);
    } else {
      r_center = raw_r_centers.empty() ? cv::Point2f(0, 0) : raw_r_centers[0];
    }

    /// 右下角二值化小窗口

    if (!binary_roi.empty() && binary_roi.cols > 1 && binary_roi.rows > 1) {
      cv::Rect roi_rect =
        cv::Rect(bgr_img.cols - binary_roi.cols, bgr_img.rows - binary_roi.rows, binary_roi.cols, binary_roi.rows);
      if (roi_rect.x >= 0 && roi_rect.y >= 0 && roi_rect.br().x <= bgr_img.cols &&
          roi_rect.br().y <= bgr_img.rows) {
        if (tools::UIManager::isUIEnabled()) {
          binary_roi.copyTo(bgr_img(roi_rect));
          cv::rectangle(bgr_img, roi_rect, cv::Scalar(150, 150, 150), 2);
        }
      }
    }

    std::vector<FanBlade> fanblades;
    for (auto & fanblade : fanblades_t) {
      if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
        fanblades.emplace_back(fanblade);
        break;
      }
    }
    if (fanblades.empty()) return std::nullopt;
    PowerRune powerrune(fanblades, r_center, std::nullopt);

    std::optional<PowerRune> P;
    P.emplace(powerrune);
    return P;

  } else {
    /// YOLO11 model detection (original)

    std::vector<YOLO11_BUFF::Object> results = MODE_->get_multicandidateboxes(bgr_img);

    /// 处理未获得的情况

    if (results.empty()) return std::nullopt;

    /// results转扇叶FanBlade

    std::vector<FanBlade> fanblades_t;
    for (auto & result : results)
      fanblades_t.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

    /// 计算r_center,筛选fanblade
    auto r_center = get_r_center(fanblades_t, bgr_img);
    std::vector<FanBlade> fanblades;
    for (auto & fanblade : fanblades_t) {
      if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
        fanblades.emplace_back(fanblade);
        break;
      }
    }
    if (fanblades.empty()) return std::nullopt;
    PowerRune powerrune(fanblades, r_center, std::nullopt);

    std::optional<PowerRune> P;
    P.emplace(powerrune);
    return P;
  }
#else
    handle_lose();
    return std::nullopt;
#endif
}

}  // namespace auto_buff
