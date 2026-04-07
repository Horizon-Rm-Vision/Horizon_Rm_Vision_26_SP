#include "ui_manager.hpp"
#include "yaml.hpp"

namespace tools {

// 初始化静态成员变量
bool UIManager::global_ui_enabled_ = true;

UIManager::UIManager(const std::string& config_path)
    : UIManager(true) {  // 默认启用，后面会根据配置调整
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["ui"] && config["ui"]["enabled"]) {
            enabled_ = config["ui"]["enabled"].as<bool>();
            // 更新全局UI启用状态
            global_ui_enabled_ = enabled_;
        }
        if (config["ui"] && config["ui"]["imshow"]) {
            imshow_enabled_ = config["ui"]["imshow"].as<bool>();
        }
    } catch (const std::exception& e) {
        // 如果配置文件不存在或格式错误，默认禁用UI
        enabled_ = false;
        imshow_enabled_ = false;
        global_ui_enabled_ = false;
    }
}

UIManager::UIManager(bool enabled)
    : enabled_(enabled),
      imshow_enabled_(true),  // 默认启用imshow
      last_fps_time_(std::chrono::steady_clock::now()),
      frame_count_(0),
      fps_(0.0f),
      fps_text_cache_("FPS: 0.0"),
      left_y_offset_(30),
      right_y_offset_(30),
      line_height_(25),
      font_face_(cv::FONT_HERSHEY_SIMPLEX),
      font_scale_(0.6),
      thickness_(2),
      default_color_(0, 255, 0),
      highlight_color_(0, 165, 255),
      program_mode_("default") {
    // 更新全局UI启用状态
    global_ui_enabled_ = enabled;
}

void UIManager::initialize(cv::Mat& img) {
    if (!enabled_) return;
    
    resetForFrame();
    
    // 添加基础UI元素
    addLeftText("fps", fmt::format("FPS: {:.1f}", fps_));
    addLeftText("mode", fmt::format("Mode: {}", program_mode_));
    
    // 右侧基础元素
    addRightText("status_title", "STATUS");
    
    // 系统时间
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&now_c);
    time_str = time_str.substr(11, 8); // 只取HH:MM:SS部分
    addRightText("time", fmt::format("Time: {}", time_str));
}

void UIManager::addLeftText(const std::string& key, const std::string& text, cv::Scalar color) {
    if (!enabled_) return;
    left_elements_.push_back({key, text, color, true});
}

void UIManager::addRightText(const std::string& key, const std::string& text, cv::Scalar color) {
    if (!enabled_) return;
    right_elements_.push_back({key, text, color, true});
}

void UIManager::updateFPS() {
    if (!enabled_) return;
    
    frame_count_++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time_).count();
    
    if (elapsed_time >= 1000) {
        fps_ = static_cast<float>(frame_count_) * 1000.0f / static_cast<float>(elapsed_time);
        frame_count_ = 0;
        last_fps_time_ = current_time;
        
        // 更新FPS文本缓存
        fps_text_cache_ = fmt::format("FPS: {:.1f}", fps_);
    }
}

void UIManager::setProgramMode(const std::string& mode) {
    program_mode_ = mode;
}

void UIManager::render(cv::Mat& img) {
    if (!enabled_) return;
    
    // 更新FPS显示（直接使用缓存）
    for (auto& elem : left_elements_) {
        if (elem.key == "fps") {
            elem.text = fps_text_cache_;
            break;
        }
    }
    
    drawLeftPanel(img);
    drawRightPanel(img);
    drawCommonElements(img);
    executeDrawCommands(img);
}

void UIManager::drawLeftPanel(cv::Mat& img) {
    int y_offset = left_y_offset_;
    
    for (const auto& elem : left_elements_) {
        if (!elem.enabled) continue;
        cv::putText(img, elem.text, cv::Point(10, y_offset), font_face_, font_scale_, elem.color, thickness_);
        y_offset += line_height_;
    }
}

void UIManager::drawRightPanel(cv::Mat& img) {
    int img_width = img.cols;
    int right_x = img_width - 180;
    int y_offset = right_y_offset_;
    
    // 分隔线
    cv::line(img, cv::Point(img_width - 200, 20), cv::Point(img_width - 200, 400), cv::Scalar(0, 255, 255), 2);
    
    for (const auto& elem : right_elements_) {
        if (!elem.enabled) continue;
        
        if (elem.key == "status_title") {
            cv::putText(img, elem.text, cv::Point(right_x, y_offset), font_face_, font_scale_ + 0.1, cv::Scalar(255, 255, 0), thickness_);
            y_offset += line_height_ + 10;
        } else {
            cv::putText(img, elem.text, cv::Point(right_x, y_offset), font_face_, font_scale_, elem.color, thickness_);
            y_offset += line_height_;
        }
    }
}

void UIManager::drawCommonElements(cv::Mat& img) {
    // 可以在这里添加通用的UI元素
}

void UIManager::executeDrawCommands(cv::Mat& img) {
    for (const auto& cmd : draw_commands_) {
        switch (cmd.type) {
            case UIDrawCommand::DRAW_POINTS:
                for (const auto& point : cmd.points) {
                    cv::circle(img, point, cmd.point_size, cmd.point_color, -1);
                }
                break;
            case UIDrawCommand::DRAW_TEXT:
                cv::putText(img, cmd.text, cmd.position, font_face_, font_scale_, cmd.text_color, thickness_);
                break;
            case UIDrawCommand::CUSTOM:
                if (cmd.custom_draw) {
                    cmd.custom_draw(img);
                }
                break;
            default:
                break;
        }
    }
}

void UIManager::resetForFrame() {
    left_elements_.clear();
    right_elements_.clear();
    draw_commands_.clear();
    
    // 重置偏移量
    left_y_offset_ = 30;
    right_y_offset_ = 30;
}

//原img_tools中的绘制函数，添加UI启用检查
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

} // namespace tools