#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>
#include <deque>
#include <vector>
#include <string>
#include <mutex>

namespace tools
{

// 单个曲线数据结构
struct CurveData {
    std::string name;
    cv::Scalar color;
    std::deque<double> data;
};

class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);
  ~Plotter();

  void plot(const nlohmann::json & json);
  
  // 实时曲线绘制功能
  void addCurve(const std::string& name, cv::Scalar color);
  void addData(const std::vector<double>& values);
  void addData(const std::vector<double>& values, const std::vector<std::string>& names);
  cv::Mat drawCurves();
  void setPlotSize(int width, int height, int max_points);
  void drawData(const std::vector<double>& values);
  void drawData(const std::vector<double>& values, const std::vector<std::string>& names);

private:
  void drawGrid(cv::Mat& img, int plot_width, int plot_height, double min_val, double max_val);
  void drawCurve(cv::Mat& img, const CurveData& curve, int plot_width, int plot_height, 
                 double min_val, double range);
  void drawLegend(cv::Mat& img);
  cv::Scalar generateColor(size_t index);

  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;

  // 曲线绘制相关
  std::vector<CurveData> curves_;
  int width_{1400};
  int height_{300};
  int max_points_{500};
  int margin_left_{60};
  int margin_right_{150};
  int margin_top_{40};
  int margin_bottom_{40};
  int frame_count_{0};
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP