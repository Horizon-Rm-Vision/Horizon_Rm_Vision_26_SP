#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

//新增本地plot所需头文件
#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>

namespace tools
{

// 单个曲线数据结构
struct CurveData {
    std::string name;
    cv::Scalar color;
    std::deque<double> data;
};

// 子图（坐标轴）数据结构
struct Subplot {
    std::string name;
    std::vector<CurveData> curves;
};

class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);
  ~Plotter();

  void plot(const nlohmann::json & json);
  void configureWebStreamFromConfig(const std::string & config_path);
  void setWebStream(const std::string & host, uint16_t port, bool enabled);
  void setWindowName(const std::string & name);

  // 多子图绘图 - 一行一个子图，按名称自动管理
  // 示例: plotter.subplot("Yaw", {yaw, target_yaw}, {"gimbal_yaw", "target_yaw"});
  void subplot(const std::string& name, const std::vector<double>& values,
               const std::vector<std::string>& names);
  void subplot(const std::vector<double>& values, const std::vector<std::string>& names);
  void subplot(const std::vector<double>& values);

  // 统一渲染一帧（subplot 只推数据，最后调用 draw 一次性绘制+发送）
  void draw();

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;

  int web_socket_{-1};
  sockaddr_in web_destination_{};
  bool web_enabled_{false};
  std::string web_host_{"127.0.0.1"};
  uint16_t web_port_{9876};

  // 曲线绘制相关函数和变量
  void setPlotSize(int width, int height, int max_points);
  int addSubplot(const std::string& name);
  void addData(int subplot_idx, const std::vector<double>& values);
  void addData(int subplot_idx, const std::vector<double>& values, const std::vector<std::string>& names);
  void drawSubplots();
  void drawGrid(cv::Mat& img, int plot_width, int plot_height, double min_val, double max_val, int y_offset = 0);
  void drawCurve(cv::Mat& img, const CurveData& curve, int plot_width, int plot_height,
                 double min_val, double range, int y_offset = 0);
  void drawLegend(cv::Mat& img, const std::vector<CurveData>& curves, int y_offset = 0);
  cv::Scalar generateColor(size_t index);
  std::vector<Subplot> subplots_;
  int width_{1400};
  int height_{300};
  int max_points_{500};
  int margin_left_{60};
  int margin_right_{150};
  int margin_top_{40};
  int margin_bottom_{40};
  int frame_count_{0};
  std::string window_name_{"Plotter"};

  void initWebSocket();
  void closeWebSocket();
  void sendWebSnapshot(const std::vector<double> & values);
  void sendSubplotsSnapshot();
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP
