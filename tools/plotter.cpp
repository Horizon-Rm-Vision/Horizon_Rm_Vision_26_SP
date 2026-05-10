#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

#include "yaml.hpp"

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

Plotter::~Plotter()
{
  ::close(socket_);
  closeWebSocket();
}

void Plotter::configureWebStreamFromConfig(const std::string & config_path)
{
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    bool enabled = false;
    std::string host = web_host_;
    uint16_t port = web_port_;

    if (config["ui"] && config["ui"]["web"]) {
      auto ui_web = config["ui"]["web"];
      if (ui_web["enabled"]) {
        enabled = ui_web["enabled"].as<bool>();
      }
      if (ui_web["host"]) {
        host = ui_web["host"].as<std::string>();
      }
      if (ui_web["port"]) {
        port = static_cast<uint16_t>(ui_web["port"].as<int>());
      }
    }

    if (config["plotter"] && config["plotter"]["web"]) {
      auto plot_web = config["plotter"]["web"];
      if (plot_web["enabled"]) {
        enabled = plot_web["enabled"].as<bool>();
      }
      if (plot_web["host"]) {
        host = plot_web["host"].as<std::string>();
      }
      if (plot_web["port"]) {
        port = static_cast<uint16_t>(plot_web["port"].as<int>());
      }
      if (plot_web["width"]) {
        width_ = plot_web["width"].as<int>();
      }
      if (plot_web["height"]) {
        height_ = plot_web["height"].as<int>();
      }
      if (plot_web["max_points"]) {
        max_points_ = plot_web["max_points"].as<int>();
      }
    }

    setWebStream(host, port, enabled);
  } catch (const std::exception &) {
    setWebStream(web_host_, web_port_, false);
  }
}

void Plotter::setWebStream(const std::string & host, uint16_t port, bool enabled)
{
  web_host_ = host;
  web_port_ = port;
  web_enabled_ = enabled;
  if (web_enabled_) {
    initWebSocket();
  } else {
    closeWebSocket();
  }
}

void Plotter::setWindowName(const std::string & name)
{
  window_name_ = name;
}

void Plotter::initWebSocket()
{
  if (web_socket_ >= 0) return;
  web_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  web_destination_.sin_family = AF_INET;
  web_destination_.sin_port = ::htons(web_port_);
  web_destination_.sin_addr.s_addr = ::inet_addr(web_host_.c_str());
}

void Plotter::closeWebSocket()
{
  if (web_socket_ >= 0) {
    ::close(web_socket_);
    web_socket_ = -1;
  }
}

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

// ============ 子图管理 ============

int Plotter::addSubplot(const std::string& name)
{
  Subplot sp;
  sp.name = name;
  subplots_.push_back(sp);
  return subplots_.size() - 1;
}

static void addDataToSubplot(Subplot& sp, const std::vector<double>& values,
                              const std::vector<std::string>& names,
                              size_t max_points, int& frame_count,
                              const std::function<cv::Scalar(size_t)>& genColor)
{
  if (!names.empty() && values.size() != names.size()) return;

  bool need_recreate = (sp.curves.size() != values.size());
  if (!need_recreate && !names.empty()) {
    for (size_t i = 0; i < sp.curves.size(); i++) {
      if (sp.curves[i].name != names[i]) {
        need_recreate = true;
        break;
      }
    }
  }

  if (need_recreate) {
    sp.curves.clear();
    for (size_t i = 0; i < values.size(); i++) {
      CurveData curve;
      curve.name = names.empty() ? "curve_" + std::to_string(i) : names[i];
      curve.color = genColor(i);
      sp.curves.push_back(curve);
    }
  }

  for (size_t i = 0; i < sp.curves.size(); i++) {
    sp.curves[i].data.push_back(values[i]);
    if (sp.curves[i].data.size() > max_points) {
      sp.curves[i].data.pop_front();
    }
  }
  frame_count++;
}

cv::Scalar Plotter::generateColor(size_t index)
{
  static const std::vector<cv::Scalar> colors = {
    cv::Scalar(230, 97, 203),   // 粉色
    cv::Scalar(76, 177, 34),    // 绿色
    cv::Scalar(255, 128, 0),    // 橙色
    cv::Scalar(0, 128, 255),    // 蓝色
  };
  return colors[index % colors.size()];
}

void Plotter::addData(int subplot_idx, const std::vector<double>& values)
{
  if (subplot_idx < 0 || subplot_idx >= (int)subplots_.size()) return;
  auto& sp = subplots_[subplot_idx];
  addDataToSubplot(sp, values, {}, max_points_, frame_count_,
                   [this](size_t i) { return generateColor(i); });
}

void Plotter::addData(int subplot_idx, const std::vector<double>& values,
                      const std::vector<std::string>& names)
{
  if (subplot_idx < 0 || subplot_idx >= (int)subplots_.size()) return;
  auto& sp = subplots_[subplot_idx];
  addDataToSubplot(sp, values, names, max_points_, frame_count_,
                   [this](size_t i) { return generateColor(i); });
}

// ============ 简化多子图接口 ============

void Plotter::subplot(const std::string& name, const std::vector<double>& values,
                      const std::vector<std::string>& names)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 按名称查找或创建子图
  int idx = -1;
  for (size_t i = 0; i < subplots_.size(); i++) {
    if (subplots_[i].name == name) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    idx = addSubplot(name);
  }

  addDataToSubplot(subplots_[idx], values, names, max_points_, frame_count_,
                   [this](size_t i) { return generateColor(i); });
}

void Plotter::subplot(const std::vector<double>& values, const std::vector<std::string>& names)
{
  std::string name = names.empty() ? "Plot" : names[0];
  subplot(name, values, names);
}

void Plotter::subplot(const std::vector<double>& values)
{
  subplot("Plot", values, {});
}

void Plotter::draw()
{
  std::lock_guard<std::mutex> lock(mutex_);
  drawSubplots();
}

// ============ Web 快照 ============

void Plotter::sendWebSnapshot(const std::vector<double> & values)
{
  if (!web_enabled_ || web_socket_ < 0) return;
  if (subplots_.empty() || subplots_[0].curves.empty()) return;

  nlohmann::json payload;
  payload["type"] = "plotter";
  payload["width"] = width_;
  payload["height"] = height_;
  payload["max_points"] = max_points_;
  payload["margin_left"] = margin_left_;
  payload["margin_right"] = margin_right_;
  payload["margin_top"] = margin_top_;
  payload["margin_bottom"] = margin_bottom_;
  payload["frame_count"] = frame_count_;

  payload["names"] = nlohmann::json::array();
  payload["colors"] = nlohmann::json::array();
  for (const auto & curve : subplots_[0].curves) {
    payload["names"].push_back(curve.name);
    payload["colors"].push_back({curve.color[0], curve.color[1], curve.color[2]});
  }

  payload["values"] = nlohmann::json::array();
  for (auto value : values) {
    payload["values"].push_back(value);
  }

  auto data = payload.dump();
  ::sendto(
    web_socket_, data.c_str(), data.length(), 0,
    reinterpret_cast<sockaddr *>(&web_destination_), sizeof(web_destination_));
}

void Plotter::sendSubplotsSnapshot()
{
  if (!web_enabled_ || web_socket_ < 0) return;

  nlohmann::json payload;
  payload["type"] = "plotter_multi";
  payload["width"] = width_;
  payload["height"] = height_;
  payload["max_points"] = max_points_;
  payload["margin_left"] = margin_left_;
  payload["margin_right"] = margin_right_;
  payload["margin_top"] = margin_top_;
  payload["margin_bottom"] = margin_bottom_;
  payload["frame_count"] = frame_count_;

  payload["subplots"] = nlohmann::json::array();
  for (const auto & sp : subplots_) {
    nlohmann::json subplot_json;
    subplot_json["name"] = sp.name;
    subplot_json["names"] = nlohmann::json::array();
    subplot_json["colors"] = nlohmann::json::array();
    for (const auto & curve : sp.curves) {
      subplot_json["names"].push_back(curve.name);
      subplot_json["colors"].push_back({curve.color[0], curve.color[1], curve.color[2]});
    }
    subplot_json["values"] = nlohmann::json::array();
    for (const auto & curve : sp.curves) {
      subplot_json["values"].push_back(curve.data.empty() ? 0.0 : curve.data.back());
    }
    payload["subplots"].push_back(subplot_json);
  }

  auto data = payload.dump();
  ::sendto(
    web_socket_, data.c_str(), data.length(), 0,
    reinterpret_cast<sockaddr *>(&web_destination_), sizeof(web_destination_));
}

//新增：本地plot数据绘制（from infantryA by WCY）
void Plotter::setPlotSize(int width, int height, int max_points)
{
  std::lock_guard<std::mutex> lock(mutex_);
  width_ = width;
  height_ = height;
  max_points_ = max_points;
}

// ============ 绘制方法 ============

void Plotter::drawSubplots()
{
  if (subplots_.empty()) return;

  int num_subplots = subplots_.size();
  int total_height = num_subplots * height_;
  int plot_width = width_ - margin_left_ - margin_right_;

  cv::Mat big_img(total_height, width_, CV_8UC3, cv::Scalar(255, 255, 255));

  for (int s = 0; s < num_subplots; s++) {
    auto& sp = subplots_[s];
    int y_offset = s * height_;
    int plot_height = height_ - margin_top_ - margin_bottom_;

    // 子图标题
    cv::putText(big_img, sp.name,
                cv::Point(margin_left_, y_offset + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 0), 1);

    if (sp.curves.empty() || sp.curves[0].data.empty()) continue;

    // 计算该子图的Y轴范围
    double min_val = 1e9, max_val = -1e9;
    for (const auto& curve : sp.curves) {
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

    drawGrid(big_img, plot_width, plot_height, min_val, max_val, y_offset);

    for (const auto& curve : sp.curves) {
      drawCurve(big_img, curve, plot_width, plot_height, min_val, range, y_offset);
    }

    drawLegend(big_img, sp.curves, y_offset);
  }

  cv::imshow(window_name_, big_img);
  sendSubplotsSnapshot();
}

void Plotter::drawGrid(cv::Mat& img, int plot_width, int plot_height,
                       double min_val, double max_val, int y_offset)
{
  cv::rectangle(img,
               cv::Point(margin_left_, y_offset + margin_top_),
               cv::Point(margin_left_ + plot_width, y_offset + margin_top_ + plot_height),
               cv::Scalar(200, 200, 200), 1);

  int num_h_lines = 6;
  for (int i = 0; i <= num_h_lines; i++) {
    int y = y_offset + margin_top_ + plot_height * i / num_h_lines;
    cv::line(img,
            cv::Point(margin_left_, y),
            cv::Point(margin_left_ + plot_width, y),
            cv::Scalar(230, 230, 230), 1);
    double value = max_val - (max_val - min_val) * i / num_h_lines;
    char label[32];
    snprintf(label, sizeof(label), "%.2f", value);
    cv::putText(img, label,
               cv::Point(5, y + 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.45,
               cv::Scalar(80, 80, 80), 1);
  }

  int num_v_lines = 10;
  for (int i = 0; i <= num_v_lines; i++) {
    int x = margin_left_ + plot_width * i / num_v_lines;
    cv::line(img,
            cv::Point(x, y_offset + margin_top_),
            cv::Point(x, y_offset + margin_top_ + plot_height),
            cv::Scalar(230, 230, 230), 1);
  }

  for (int i = 0; i <= num_v_lines; i++) {
    int x = margin_left_ + plot_width * i / num_v_lines;
    int frame_num = frame_count_ - max_points_ + (max_points_ * i / num_v_lines);
    if (frame_num < 0) frame_num = 0;
    cv::putText(img, std::to_string(frame_num),
               cv::Point(x - 15, y_offset + margin_top_ + plot_height + 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(80, 80, 80), 1);
  }
}

void Plotter::drawCurve(cv::Mat& img, const CurveData& curve,
                        int plot_width, int plot_height, double min_val, double range, int y_offset)
{
  if (curve.data.size() < 2) return;

  size_t data_size = curve.data.size();
  for (size_t i = 1; i < data_size; i++) {
    double x1_ratio = (double)(i - 1) / max_points_;
    double x2_ratio = (double)i / max_points_;

    int x1 = margin_left_ + x1_ratio * plot_width;
    int x2 = margin_left_ + x2_ratio * plot_width;

    int y1 = y_offset + margin_top_ + plot_height - (curve.data[i-1] - min_val) / range * plot_height;
    int y2 = y_offset + margin_top_ + plot_height - (curve.data[i] - min_val) / range * plot_height;

    y1 = std::max(y_offset + margin_top_, std::min(y_offset + margin_top_ + plot_height, y1));
    y2 = std::max(y_offset + margin_top_, std::min(y_offset + margin_top_ + plot_height, y2));

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), curve.color, 2);
  }
}

void Plotter::drawLegend(cv::Mat& img, const std::vector<CurveData>& curves, int y_offset)
{
  int legend_x = width_ - margin_right_ + 10;
  int legend_y = y_offset + margin_top_ + 10;
  int line_height = 25;

  for (size_t i = 0; i < curves.size(); i++) {
    int y = legend_y + i * line_height;
    cv::line(img,
            cv::Point(legend_x, y + 5),
            cv::Point(legend_x + 30, y + 5),
            curves[i].color, 3);
    cv::putText(img, curves[i].name,
               cv::Point(legend_x + 40, y + 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.45,
               cv::Scalar(0, 0, 0), 1);
  }
}

}  // namespace tools
