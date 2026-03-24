// auto_aim_playback.cpp
#include <fmt/core.h>
#include <fstream>
#include <filesystem>
#include <sstream>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/ui_manager.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

// 回放帧数据结构
struct PlaybackFrame {
    int frame_id;
    double timestamp;
    cv::Mat raw_image;
    cv::Mat ui_image;
    
    // 云台数据
    double gimbal_yaw;
    double gimbal_yaw_vel;
    double gimbal_pitch;
    double gimbal_pitch_vel;
    double bullet_speed;
    
    // 四元数
    Eigen::Quaterniond q;
    
    // 原始数据（用于验证）
    std::vector<auto_aim::Armor> original_armors;
    std::optional<auto_aim::Target> original_target;
    std::optional<auto_aim::Plan> original_plan;
};

// 回放管理器
class PlaybackManager {
public:
    PlaybackManager(const std::string& record_dir) 
        : record_dir_(record_dir),
          current_frame_(0),
          total_frames_(0),
          playback_speed_(1.0) {
        
        // 读取元数据
        std::ifstream metadata_file(record_dir + "/metadata.json");
        if (metadata_file.is_open()) {
            nlohmann::json metadata;
            metadata_file >> metadata;
            total_frames_ = metadata["total_frames"];
            config_path_ = metadata["config_path"];
            tools::logger()->info("Loaded metadata: {} frames, config: {}", 
                                 total_frames_, config_path_);
        }
        
        // 读取CSV数据
        std::ifstream data_file(record_dir + "/record_data.csv");
        if (!data_file.is_open()) {
            throw std::runtime_error("Cannot open record data file");
        }
        
        std::string line;
        // 跳过标题行
        std::getline(data_file, line);
        
        while (std::getline(data_file, line)) {
            std::stringstream ss(line);
            std::string token;
            PlaybackFrame frame;
            
            // 解析CSV行
            std::getline(ss, token, ',');
            frame.frame_id = std::stoi(token);
            
            std::getline(ss, token, ',');
            frame.timestamp = std::stod(token);
            
            std::getline(ss, token, ',');
            frame.gimbal_yaw = std::stod(token);
            
            std::getline(ss, token, ',');
            frame.gimbal_yaw_vel = std::stod(token);
            
            std::getline(ss, token, ',');
            frame.gimbal_pitch = std::stod(token);
            
            std::getline(ss, token, ',');
            frame.gimbal_pitch_vel = std::stod(token);
            
            std::getline(ss, token, ',');
            frame.bullet_speed = std::stod(token);
            
            double q_w, q_x, q_y, q_z;
            std::getline(ss, token, ',');
            q_w = std::stod(token);
            std::getline(ss, token, ',');
            q_x = std::stod(token);
            std::getline(ss, token, ',');
            q_y = std::stod(token);
            std::getline(ss, token, ',');
            q_z = std::stod(token);
            frame.q = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
            
            // 跳过剩余字段（这些将在需要时重新计算）
            for (int i = 0; i < 17; i++) {
                std::getline(ss, token, ',');
            }
            
            // 加载原始图像
            std::string raw_image_path;
            std::getline(ss, raw_image_path, ',');
            frame.raw_image = cv::imread(raw_image_path);
            
            // 尝试加载UI图像
            std::string ui_image_path;
            std::getline(ss, ui_image_path);
            if (!ui_image_path.empty() && std::filesystem::exists(ui_image_path)) {
                frame.ui_image = cv::imread(ui_image_path);
            }
            
            frames_.push_back(frame);
        }
        
        tools::logger()->info("Loaded {} frames for playback", frames_.size());
        total_frames_ = frames_.size();
    }
    
    bool hasNext() const {
        return current_frame_ < total_frames_;
    }
    
    PlaybackFrame& nextFrame() {
        return frames_[current_frame_++];
    }
    
    PlaybackFrame& getFrame(int index) {
        if (index >= 0 && index < total_frames_) {
            current_frame_ = index + 1;
            return frames_[index];
        }
        throw std::out_of_range("Frame index out of range");
    }
    
    void reset() {
        current_frame_ = 0;
    }
    
    int getCurrentFrame() const { return current_frame_; }
    int getTotalFrames() const { return total_frames_; }
    std::string getConfigPath() const { return config_path_; }
    
    void setPlaybackSpeed(double speed) {
        playback_speed_ = speed;
    }
    
private:
    std::string record_dir_;
    std::string config_path_;
    std::vector<PlaybackFrame> frames_;
    int current_frame_;
    int total_frames_;
    double playback_speed_;
};

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明}"
  "{@record-dir    |                   | 记录数据目录路径}"
  "{speed s        | 1.0               | 回放速度（1.0为正常速度）}"
  "{start-frame    | 0                 | 起始帧}"
  "{end-frame      | -1                | 结束帧（-1表示所有帧）}"
  "{compare c      | false             | 启用结果对比模式}";

int main(int argc, char *argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    
    std::string record_dir = cli.get<std::string>(0);
    if (record_dir.empty()) {
        std::cerr << "必须指定记录目录" << std::endl;
        return -1;
    }
    
    double playback_speed = cli.get<double>("speed");
    int start_frame = cli.get<int>("start-frame");
    int end_frame = cli.get<int>("end-frame");
    bool compare_mode = cli.get<bool>("compare");
    
    try {
        // 初始化回放管理器
        PlaybackManager playback_manager(record_dir);
        
        // 获取配置路径并初始化组件
        std::string config_path = playback_manager.getConfigPath();
        if (config_path.empty()) {
            config_path = "../configs/standard3.yaml";
        }
        
        auto_aim::YOLO yolo(config_path, true);
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path, solver);
        auto_aim::Planner planner(config_path);
        
        tools::Plotter plotter;
        tools::Exiter exiter;
        
        // 设置回放范围
        if (end_frame == -1 || end_frame > playback_manager.getTotalFrames()) {
            end_frame = playback_manager.getTotalFrames();
        }
        
        start_frame = std::max(0, start_frame);
        end_frame = std::min(end_frame, playback_manager.getTotalFrames());
        
        playback_manager.reset();
        if (start_frame > 0) {
            playback_manager.getFrame(start_frame - 1);
        }
        
        tools::logger()->info("Playback started: frames {} to {}, speed: {}x", 
                             start_frame, end_frame, playback_speed);
        
        // 统计数据
        int match_count = 0;
        int total_processed = 0;
        std::vector<double> yaw_errors;
        std::vector<double> pitch_errors;
        
        // 主回放循环
        for (int frame_idx = start_frame; frame_idx < end_frame && !exiter.exit(); frame_idx++) {
            auto& frame = playback_manager.getFrame(frame_idx);
            
            // 设置求解器的旋转矩阵（使用记录的四元数）
            solver.set_R_gimbal2world(frame.q);
            
            // 使用记录的原始图像进行检测
            auto start_time = std::chrono::steady_clock::now();
            auto armors = yolo.detect(frame.raw_image);
            
            // 模拟时间戳（使用记录的时间戳）
            auto timestamp = std::chrono::steady_clock::now() + 
                            std::chrono::duration<double>(frame.timestamp);
            
            // 跟踪
            auto targets = tracker.track(armors, timestamp);
            
            // 规划
            std::optional<auto_aim::Target> target_opt;
            if (!targets.empty()) {
                target_opt = targets.front();
            }
            
            auto plan = planner.plan(target_opt, frame.bullet_speed);
            
            auto end_time = std::chrono::steady_clock::now();
            double process_time = std::chrono::duration<double>(end_time - start_time).count();
            
            // 准备显示图像
            cv::Mat display_image = frame.raw_image.clone();
            
            // 绘制检测结果
            for (const auto& armor : armors) {
                tools::draw_points(display_image, armor.points, {0, 255, 0});
                auto info = fmt::format("{:.2f} {} {}", 
                                       armor.confidence,
                                       auto_aim::COLORS[armor.color],
                                       auto_aim::ARMOR_NAMES[armor.name]);
                tools::draw_text(display_image, info, armor.center, {0, 255, 0});
            }
            
            // 如果存在目标，绘制目标信息
            if (!targets.empty()) {
                auto target = targets.front();
                std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
                for (const auto& xyza : armor_xyza_list) {
                    auto image_points = solver.reproject_armor(
                        xyza.head(3), xyza[3], target.armor_type, target.name);
                    tools::draw_points(display_image, image_points, {0, 255, 0});
                }
                
                // 绘制瞄准点
                Eigen::Vector4d aim_xyza = planner.debug_xyza;
                auto image_points = solver.reproject_armor(
                    aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
                tools::draw_points(display_image, image_points, {0, 0, 255});
            }
            
            // 绘制回放信息
            int y_offset = 30;
            int line_height = 25;
            cv::Scalar text_color(0, 255, 0);
            cv::Scalar replay_color(255, 255, 0); // 黄色表示回放
            int font_face = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.6;
            int thickness = 2;
            
            // 帧信息
            std::string frame_text = fmt::format("Frame: {}/{}", 
                                                frame_idx + 1, 
                                                playback_manager.getTotalFrames());
            cv::putText(display_image, frame_text, cv::Point(10, y_offset), 
                       font_face, font_scale, replay_color, thickness);
            y_offset += line_height;
            
            // 时间戳
            std::string time_text = fmt::format("Time: {:.3f}s", frame.timestamp);
            cv::putText(display_image, time_text, cv::Point(10, y_offset), 
                       font_face, font_scale, text_color, thickness);
            y_offset += line_height;
            
            // 处理时间
            std::string process_text = fmt::format("Process: {:.1f}ms", process_time * 1000);
            cv::putText(display_image, process_text, cv::Point(10, y_offset), 
                       font_face, font_scale, text_color, thickness);
            y_offset += line_height;
            
            // 检测结果
            std::string detect_text = fmt::format("Detect: {} armors", armors.size());
            cv::putText(display_image, detect_text, cv::Point(10, y_offset), 
                       font_face, font_scale, 
                       armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 
                       thickness);
            y_offset += line_height;
            
            // 目标跟踪
            std::string track_text = fmt::format("Track: {} targets", targets.size());
            cv::putText(display_image, track_text, cv::Point(10, y_offset), 
                       font_face, font_scale, 
                       targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 
                       thickness);
            y_offset += line_height;
            
            // 规划结果
            std::string plan_text = fmt::format("Plan Yaw: {:.2f} Pitch: {:.2f}", 
                                               -plan.yaw * 180.0 / M_PI, 
                                               -plan.pitch * 180.0 / M_PI);
            cv::putText(display_image, plan_text, cv::Point(10, y_offset), 
                       font_face, font_scale, cv::Scalar(255, 165, 0), thickness);
            y_offset += line_height;
            
            // 开火状态
            std::string fire_text = fmt::format("Fire: {}", plan.fire ? "YES" : "NO");
            cv::putText(display_image, fire_text, cv::Point(10, y_offset), 
                       font_face, font_scale, 
                       plan.fire ? cv::Scalar(0, 0, 255) : text_color, 
                       thickness);
            y_offset += line_height;
            
            // 对比模式（如果启用）
            if (compare_mode && frame.original_plan.has_value()) {
                double yaw_error = std::abs(plan.yaw - frame.original_plan->yaw);
                double pitch_error = std::abs(plan.pitch - frame.original_plan->pitch);
                
                yaw_errors.push_back(yaw_error);
                pitch_errors.push_back(pitch_error);
                
                std::string compare_text = fmt::format("Compare: ΔYaw={:.4f} ΔPitch={:.4f}", 
                                                      yaw_error, pitch_error);
                cv::putText(display_image, compare_text, cv::Point(10, y_offset), 
                           font_face, font_scale, 
                           (yaw_error < 0.01 && pitch_error < 0.01) ? 
                           cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 
                           thickness);
                y_offset += line_height;
                
                if (yaw_error < 0.01 && pitch_error < 0.01) {
                    match_count++;
                }
                total_processed++;
            }
            
            // 显示图像
            cv::namedWindow("Playback", cv::WINDOW_NORMAL);
            cv::imshow("Playback", display_image);
            
            // 控制播放速度
            int wait_time = static_cast<int>(30.0 / playback_speed);
            auto key = cv::waitKey(wait_time);
            
            // 快捷键控制
            switch (key) {
                case 'q': // 退出
                    return 0;
                case ' ': // 暂停/继续
                    cv::waitKey(0);
                    break;
                case 'r': // 重置
                    frame_idx = start_frame - 1;
                    tracker.reset(); // 重置跟踪器
                    break;
                case 'f': // 前进10帧
                    frame_idx += 9;
                    break;
                case 'b': // 后退10帧
                    frame_idx = std::max(start_frame, frame_idx - 10);
                    break;
                case '+': // 加速
                    playback_speed = std::min(10.0, playback_speed * 1.5);
                    break;
                case '-': // 减速
                    playback_speed = std::max(0.1, playback_speed / 1.5);
                    break;
            }
            
            // 定期输出进度
            if (frame_idx % 100 == 0) {
                tools::logger()->info("Processed {}/{} frames", 
                                     frame_idx + 1, end_frame - start_frame);
            }
        }
        
        // 输出统计结果
        if (compare_mode && total_processed > 0) {
            double yaw_mean = 0, pitch_mean = 0;
            double yaw_max = 0, pitch_max = 0;
            
            for (size_t i = 0; i < yaw_errors.size(); i++) {
                yaw_mean += yaw_errors[i];
                pitch_mean += pitch_errors[i];
                yaw_max = std::max(yaw_max, yaw_errors[i]);
                pitch_max = std::max(pitch_max, pitch_errors[i]);
            }
            
            yaw_mean /= yaw_errors.size();
            pitch_mean /= pitch_errors.size();
            
            tools::logger()->info("Comparison Results:");
            tools::logger()->info("  Matches: {}/{} ({:.1f}%)", 
                                 match_count, total_processed, 
                                 (match_count * 100.0) / total_processed);
            tools::logger()->info("  Mean Yaw Error: {:.6f} rad", yaw_mean);
            tools::logger()->info("  Mean Pitch Error: {:.6f} rad", pitch_mean);
            tools::logger()->info("  Max Yaw Error: {:.6f} rad", yaw_max);
            tools::logger()->info("  Max Pitch Error: {:.6f} rad", pitch_max);
        }
        
        tools::logger()->info("Playback completed");
        
    } catch (const std::exception& e) {
        tools::logger()->error("Playback error: {}", e.what());
        return -1;
    }
    
    return 0;
}