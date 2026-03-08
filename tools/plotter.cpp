#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  //原socket发送数据部分
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());


  // 配置Plotter的曲线绘制功能
  setPlotSize(1400, 300, 500);  // 宽度、高度、最大点数
}

Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

void Plotter::addCurve(const std::string& name, cv::Scalar color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  CurveData curve;
  curve.name = name;
  curve.color = color;
  curves_.push_back(curve);
}

cv::Scalar Plotter::generateColor(size_t index)
{
  // 预定义颜色集合
  static const std::vector<cv::Scalar> colors = {
    cv::Scalar(230, 97, 203),   // 粉色
    cv::Scalar(76, 177, 34),    // 绿色
    cv::Scalar(255, 128, 0),    // 橙色
    cv::Scalar(0, 128, 255),    // 蓝色
  };
  
  return colors[index % colors.size()];
}

void Plotter::addData(const std::vector<double>& values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 如果曲线数量不匹配,自动调整
  if (curves_.size() != values.size()) {
    curves_.clear();
    for (size_t i = 0; i < values.size(); i++) {
      CurveData curve;
      curve.name = "curve_" + std::to_string(i);
      curve.color = generateColor(i);
      curves_.push_back(curve);
    }
  }
  
  // 添加数据点
  for (size_t i = 0; i < curves_.size(); i++) {
    curves_[i].data.push_back(values[i]);
    if (curves_[i].data.size() > max_points_) {
      curves_[i].data.pop_front();
    }
  }
  frame_count_++;
}

void Plotter::addData(const std::vector<double>& values, const std::vector<std::string>& names)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (values.size() != names.size()) return;
  
  // 如果曲线数量或名称不匹配,重新创建曲线
  bool need_recreate = (curves_.size() != values.size());
  if (!need_recreate) {
    for (size_t i = 0; i < curves_.size(); i++) {
      if (curves_[i].name != names[i]) {
        need_recreate = true;
        break;
      }
    }
  }
  
  if (need_recreate) {
    curves_.clear();
    for (size_t i = 0; i < values.size(); i++) {
      CurveData curve;
      curve.name = names[i];
      curve.color = generateColor(i);
      curves_.push_back(curve);
    }
  }
  
  // 添加数据点
  for (size_t i = 0; i < curves_.size(); i++) {
    curves_[i].data.push_back(values[i]);
    if (curves_[i].data.size() > max_points_) {
      curves_[i].data.pop_front();
    }
  }
  frame_count_++;
}

//新增：本地plot数据绘制（from infantryA by WCY）
void Plotter::setPlotSize(int width, int height, int max_points)
{
  std::lock_guard<std::mutex> lock(mutex_);
  width_ = width;
  height_ = height;
  max_points_ = max_points;
}

cv::Mat Plotter::drawCurves()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
  
  if (curves_.empty() || curves_[0].data.empty()) return img;

  // 计算绘图区域
  int plot_width = width_ - margin_left_ - margin_right_;
  int plot_height = height_ - margin_top_ - margin_bottom_;

  // 找到Y轴范围
  double min_val = 1e9, max_val = -1e9;
  for (const auto& curve : curves_) {
    for (double v : curve.data) {
      min_val = std::min(min_val, v);
      max_val = std::max(max_val, v);
    }
  }
  
  double range = max_val - min_val;
  if (range < 0.01) range = 0.01;
  min_val -= range * 0.1;
  max_val += range * 0.1;
  range = max_val - min_val;

  // 绘制网格和坐标轴
  drawGrid(img, plot_width, plot_height, min_val, max_val);

  // 绘制所有曲线
  for (const auto& curve : curves_) {
    drawCurve(img, curve, plot_width, plot_height, min_val, range);
  }

  // 绘制图例
  drawLegend(img);

  return img;
}

void Plotter::drawGrid(cv::Mat& img, int plot_width, int plot_height, 
                       double min_val, double max_val)
{
  // 绘制边框
  cv::rectangle(img, 
               cv::Point(margin_left_, margin_top_),
               cv::Point(margin_left_ + plot_width, margin_top_ + plot_height),
               cv::Scalar(200, 200, 200), 1);

  // 绘制水平网格线和Y轴刻度
  int num_h_lines = 6;
  for (int i = 0; i <= num_h_lines; i++) {
    int y = margin_top_ + plot_height * i / num_h_lines;
    
    // 网格线
    cv::line(img, 
            cv::Point(margin_left_, y),
            cv::Point(margin_left_ + plot_width, y),
            cv::Scalar(230, 230, 230), 1);
    
    // Y轴刻度值
    double value = max_val - (max_val - min_val) * i / num_h_lines;
    char label[32];
    snprintf(label, sizeof(label), "%.2f", value);
    cv::putText(img, label,
               cv::Point(5, y + 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.45, 
               cv::Scalar(80, 80, 80), 1);
  }

  // 绘制垂直网格线
  int num_v_lines = 10;
  for (int i = 0; i <= num_v_lines; i++) {
    int x = margin_left_ + plot_width * i / num_v_lines;
    cv::line(img, 
            cv::Point(x, margin_top_),
            cv::Point(x, margin_top_ + plot_height),
            cv::Scalar(230, 230, 230), 1);
  }

  // X轴标签 (帧数)
  for (int i = 0; i <= num_v_lines; i++) {
    int x = margin_left_ + plot_width * i / num_v_lines;
    int frame_num = frame_count_ - max_points_ + (max_points_ * i / num_v_lines);
    if (frame_num < 0) frame_num = 0;
    
    cv::putText(img, std::to_string(frame_num),
               cv::Point(x - 15, margin_top_ + plot_height + 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, 
               cv::Scalar(80, 80, 80), 1);
  }
}

void Plotter::drawCurve(cv::Mat& img, const CurveData& curve, 
                        int plot_width, int plot_height, double min_val, double range)
{
  if (curve.data.size() < 2) return;

  size_t data_size = curve.data.size();
  for (size_t i = 1; i < data_size; i++) {
    double x1_ratio = (double)(i - 1) / max_points_;
    double x2_ratio = (double)i / max_points_;
    
    int x1 = margin_left_ + x1_ratio * plot_width;
    int x2 = margin_left_ + x2_ratio * plot_width;
    
    int y1 = margin_top_ + plot_height - (curve.data[i-1] - min_val) / range * plot_height;
    int y2 = margin_top_ + plot_height - (curve.data[i] - min_val) / range * plot_height;
    
    // 限制范围
    y1 = std::max(margin_top_, std::min(margin_top_ + plot_height, y1));
    y2 = std::max(margin_top_, std::min(margin_top_ + plot_height, y2));
    
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), curve.color, 2);
  }
}

void Plotter::drawLegend(cv::Mat& img)
{
  int legend_x = width_ - margin_right_ + 10;
  int legend_y = margin_top_ + 10;
  int line_height = 25;

  for (size_t i = 0; i < curves_.size(); i++) {
    int y = legend_y + i * line_height;
    
    // 绘制颜色线
    cv::line(img, 
            cv::Point(legend_x, y + 5),
            cv::Point(legend_x + 30, y + 5),
            curves_[i].color, 3);
    
    // 绘制图例文字
    cv::putText(img, curves_[i].name,
               cv::Point(legend_x + 40, y + 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.45, 
               cv::Scalar(0, 0, 0), 1);
  }
}

void Plotter::drawData(const std::vector<double>& values)
{
  addData(values);
  cv::Mat yaw_plot = drawCurves();
  cv::imshow("YAW Tracking", yaw_plot);
}

void Plotter::drawData(const std::vector<double>& values, const std::vector<std::string>& names)
{
  addData(values, names);
  cv::Mat yaw_plot = drawCurves();
  cv::imshow("YAW Tracking", yaw_plot);
}

}  // namespace tools