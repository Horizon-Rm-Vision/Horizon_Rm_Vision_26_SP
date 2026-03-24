#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <chrono>
#include <fmt/core.h>

namespace tools {

struct UIElement {
    std::string key;
    std::string text;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    bool enabled = true;
};

struct UIDrawCommand {
    enum Type {
        TEXT_LEFT,
        TEXT_RIGHT,
        DRAW_POINTS,
        DRAW_TEXT,
        CUSTOM
    } type;
    
    // For TEXT_LEFT/TEXT_RIGHT
    std::string text;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    
    // For DRAW_POINTS
    std::vector<cv::Point2f> points;
    cv::Scalar point_color = cv::Scalar(0, 255, 0);
    int point_size = 5;
    
    // For DRAW_TEXT
    cv::Point position;
    cv::Scalar text_color = cv::Scalar(0, 255, 0);
    
    // For CUSTOM
    std::function<void(cv::Mat&)> custom_draw;
};

class UIManager {
public:
    explicit UIManager(const std::string& config_path);
    explicit UIManager(bool enabled = true);
    ~UIManager() = default;
    
    // 初始化UI，设置基础参数
    void initialize(cv::Mat& img);
    
    // 添加显示元素
    void addLeftText(const std::string& key, const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0));
    void addRightText(const std::string& key, const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0));
    
    // 添加绘制命令
    void addDrawPoints(const std::vector<cv::Point2f>& points, cv::Scalar color = cv::Scalar(0, 255, 0), int size = 5);
    void addDrawText(const std::string& text, const cv::Point& position, cv::Scalar color = cv::Scalar(0, 255, 0));
    void addCustomDraw(std::function<void(cv::Mat&)> draw_func);
    
    // 更新FPS
    void updateFPS();
    
    // 设置程序特定的显示模式
    void setProgramMode(const std::string& mode);
    
    // 应用所有UI绘制到图像
    void render(cv::Mat& img);
    
    // 检查UI是否启用
    bool isEnabled() const { return enabled_; }
    
    // 检查imshow是否启用
    bool isImshowEnabled() const { return imshow_enabled_; }
    
    // 静态方法：获取全局UI启用状态（供img_tools使用）
    static bool isUIEnabled() { return global_ui_enabled_; }
    
    // 静态方法：设置全局UI启用状态
    static void setGlobalUIEnabled(bool enabled) { global_ui_enabled_ = enabled; }

private:
    bool enabled_;
    bool imshow_enabled_;
    
    // 静态全局UI启用状态（供img_tools使用）
    static bool global_ui_enabled_;
    
    // FPS计算
    std::chrono::steady_clock::time_point last_fps_time_;
    int frame_count_;
    float fps_;
    std::string fps_text_cache_; // 缓存FPS文本，避免重复格式化
    
    // UI布局参数
    int left_y_offset_;
    int right_y_offset_;
    int line_height_;
    int font_face_;
    double font_scale_;
    int thickness_;
    cv::Scalar default_color_;
    cv::Scalar highlight_color_;
    
    // 显示元素列表
    std::vector<UIElement> left_elements_;
    std::vector<UIElement> right_elements_;
    
    // 绘制命令队列
    std::vector<UIDrawCommand> draw_commands_;
    
    // 程序模式
    std::string program_mode_;
    
    // 私有方法
    void drawLeftPanel(cv::Mat& img);
    void drawRightPanel(cv::Mat& img);
    void drawCommonElements(cv::Mat& img);
    void executeDrawCommands(cv::Mat& img);
    void resetForFrame();
};

} // namespace tools