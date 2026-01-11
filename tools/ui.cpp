#include "tools/ui.hpp"

namespace tools
{
Drawui::Drawui(cv::Mat & img, 
  const std::vector<auto_aim::Target> & targets, 
  const std::vector<auto_aim::Armor> & armors,
  const io::GimbalState & gs,
  const auto_aim::Plan & plan,
  float fps)
    : img(img), targets(targets), armors(armors), gs(gs), plan(plan), fps(fps) 
{

}

Drawui::~Drawui() 
{

}

void Drawui::drawText()
{
    // FPS信息
    std::string fps_text = fmt::format("FPS: {:.1f}", fps);
    cv::putText(img, fps_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;

    // 运行模式
    // std::string mode_text = fmt::format("Mode: {}", gs.str(gs.mode()));
    // cv::putText(img, mode_text, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    // y_offset += line_height;

    // 目标检测状态
    bool has_target = !targets.empty();
    std::string detect_text = fmt::format("Detect: {}", has_target ? "YES" : "NO");
    cv::putText(img, detect_text, cv::Point(10, y_offset), font_face, font_scale, 
                has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), thickness);
    y_offset += line_height;

    // // 开火状态
    std::string fire_text = fmt::format("Fire: {}", plan.fire ? "YES" : "NO");
    cv::putText(img, fire_text, cv::Point(10, y_offset), font_face, font_scale, 
                plan.fire ? cv::Scalar(0, 0, 255) : text_color, thickness);
    y_offset += line_height;
    
    // 云台状态 - 接收到的数据
    std::string gimbal_status = fmt::format("Gimbal Yaw: {:.2f}  Pitch: {:.2f}", gs.yaw, gs.pitch);
    cv::putText(img, gimbal_status, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    std::string gimbal_vel = fmt::format("Gimbal Vel Y: {:.2f}  P: {:.2f}", gs.yaw_vel, gs.pitch_vel);
    cv::putText(img, gimbal_vel, cv::Point(10, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 规划状态 - 发送的数据
    std::string plan_status = fmt::format("Plan Yaw: {:.2f}  Pitch: {:.2f}", plan.yaw, plan.pitch);
    cv::putText(img, plan_status, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    std::string plan_vel = fmt::format("Plan Vel Y: {:.2f}  P: {:.2f}", plan.yaw_vel, plan.pitch_vel);
    cv::putText(img, plan_vel, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    std::string plan_acc = fmt::format("Plan Acc Y: {:.2f}  P: {:.2f}", plan.yaw_acc, plan.pitch_acc);
    cv::putText(img, plan_acc, cv::Point(10, y_offset), font_face, font_scale, highlight_color, thickness);
    y_offset += line_height;
    
    // 目标信息（如果存在）
    if (has_target) {
      auto& target = targets[0];
      std::string target_info = fmt::format("Target Z: {:.2f}  Vz: {:.2f}", target.ekf_x()[4], target.ekf_x()[5]);
      cv::putText(img, target_info, cv::Point(10, y_offset), font_face, font_scale, cv::Scalar(255, 255, 0), thickness);
      y_offset += line_height;
      
      if (target.ekf_x().size() > 7) {
        std::string target_w = fmt::format("Target W: {:.2f}", target.ekf_x()[7]);
        cv::putText(img, target_w, cv::Point(10, y_offset), font_face, font_scale, cv::Scalar(255, 255, 0), thickness);
        y_offset += line_height;
      }
    }
    
    // 在图像右侧添加分隔线和状态指示
    int img_width = img.cols;
    cv::line(img, cv::Point(img_width - 200, 20), cv::Point(img_width - 200, y_offset + 20), cv::Scalar(0, 255, 255), 2);
    
    // 右侧状态指示
    int right_x = img_width - 180;
    y_offset = 30;
    
    std::string status_title = "STATUS";
    cv::putText(img, status_title, cv::Point(right_x, y_offset), font_face, font_scale + 0.1, cv::Scalar(255, 255, 0), thickness);
    y_offset += line_height + 10;
    
    // 系统时间
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&now_c);
    time_str = time_str.substr(11, 8); // 只取HH:MM:SS部分
    std::string time_text = fmt::format("Time: {}", time_str);
    cv::putText(img, time_text, cv::Point(right_x, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 子弹速度
    std::string bullet_speed_text = fmt::format("Bullet Speed: {:.1f}", gs.bullet_speed);
    cv::putText(img, bullet_speed_text, cv::Point(right_x, y_offset), font_face, font_scale, text_color, thickness);
    y_offset += line_height;
    
    // 目标数量
    std::string target_count_text = fmt::format("Targets: {}", targets.size());
    cv::putText(img, target_count_text, cv::Point(right_x, y_offset), font_face, font_scale, 
                targets.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);
    y_offset += line_height;
    
    // 检测到的装甲板数量
    std::string armor_count_text = fmt::format("Armors: {}", armors.size());
    cv::putText(img, armor_count_text, cv::Point(right_x, y_offset), font_face, font_scale, 
                armors.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), thickness);
    y_offset += line_height;

}
} // namespace tools
