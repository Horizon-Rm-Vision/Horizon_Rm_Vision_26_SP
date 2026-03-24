#include "img_tools.hpp"
#include "ui_manager.hpp"

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  // 检查UI是否启用
  if (!UIManager::isUIEnabled()) return;
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  // 检查UI是否启用
  if (!UIManager::isUIEnabled()) return;
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  // 检查UI是否启用
  if (!UIManager::isUIEnabled()) return;
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  // 检查UI是否启用
  if (!UIManager::isUIEnabled()) return;
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

void draw_line(
  cv::Mat & img, const cv::Point & pt1, const cv::Point & pt2, const cv::Scalar & color, int thickness)
{
  // 检查UI是否启用
  if (!UIManager::isUIEnabled()) return;
  cv::line(img, pt1, pt2, color, thickness);
}

}  // namespace tools