#pragma once

#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools {

enum class UIDrawType { Point, Points, Text, Line };

struct UIDrawItem {
  UIDrawType type{UIDrawType::Points};
  std::vector<cv::Point2f> points;
  cv::Point2f p1{0.0F, 0.0F};
  cv::Point2f p2{0.0F, 0.0F};
  std::string text;
  cv::Scalar color{0, 255, 0};
  int thickness{1};
  int radius{1};
  double font_scale{0.6};
};

struct UIStreamFrame {
  int width{0};
  int height{0};
  uint64_t frame_id{0};
  std::vector<UIDrawItem> draws;
};

class UIStreamRecorder {
public:
  static UIStreamRecorder & instance();

  void setEnabled(bool enabled);
  bool isEnabled() const;

  void resetFrame(int width, int height);
  void recordPoint(const cv::Point & point, const cv::Scalar & color, int radius);
  void recordPoints(
    const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness);
  void recordText(
    const std::string & text, const cv::Point & position, const cv::Scalar & color,
    double font_scale, int thickness);
  void recordLine(const cv::Point & pt1, const cv::Point & pt2, const cv::Scalar & color, int thickness);

  UIStreamFrame snapshot() const;

private:
  UIStreamRecorder() = default;

  mutable std::mutex mutex_;
  bool enabled_{false};
  UIStreamFrame frame_{};
};

}  // namespace tools
