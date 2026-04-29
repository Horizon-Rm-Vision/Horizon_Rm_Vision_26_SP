#include "ui_stream_recorder.hpp"

namespace tools {

UIStreamRecorder & UIStreamRecorder::instance()
{
  static UIStreamRecorder recorder;
  return recorder;
}

void UIStreamRecorder::setEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
}

bool UIStreamRecorder::isEnabled() const
{
  return enabled_;
}

void UIStreamRecorder::resetFrame(int width, int height)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  frame_.width = width;
  frame_.height = height;
  frame_.frame_id++;
  frame_.draws.clear();
}

void UIStreamRecorder::recordPoint(const cv::Point & point, const cv::Scalar & color, int radius)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  UIDrawItem item;
  item.type = UIDrawType::Point;
  item.p1 = cv::Point2f(point.x, point.y);
  item.color = color;
  item.radius = radius;
  frame_.draws.push_back(std::move(item));
}

void UIStreamRecorder::recordPoints(
  const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  UIDrawItem item;
  item.type = UIDrawType::Points;
  item.points = points;
  item.color = color;
  item.thickness = thickness;
  frame_.draws.push_back(std::move(item));
}

void UIStreamRecorder::recordText(
  const std::string & text, const cv::Point & position, const cv::Scalar & color,
  double font_scale, int thickness)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  UIDrawItem item;
  item.type = UIDrawType::Text;
  item.text = text;
  item.p1 = cv::Point2f(position.x, position.y);
  item.color = color;
  item.font_scale = font_scale;
  item.thickness = thickness;
  frame_.draws.push_back(std::move(item));
}

void UIStreamRecorder::recordLine(
  const cv::Point & pt1, const cv::Point & pt2, const cv::Scalar & color, int thickness)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  UIDrawItem item;
  item.type = UIDrawType::Line;
  item.p1 = cv::Point2f(pt1.x, pt1.y);
  item.p2 = cv::Point2f(pt2.x, pt2.y);
  item.color = color;
  item.thickness = thickness;
  frame_.draws.push_back(std::move(item));
}

UIStreamFrame UIStreamRecorder::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_;
}

}  // namespace tools
