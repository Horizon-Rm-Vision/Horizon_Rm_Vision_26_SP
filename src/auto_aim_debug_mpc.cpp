// #include <fmt/core.h>

// #include <atomic>
// #include <chrono>
// #include <nlohmann/json.hpp>
// #include <opencv2/opencv.hpp>
// #include <thread>

// #include "io/camera.hpp"
// #include "io/gimbal/gimbal.hpp"
// #include "tasks/auto_aim/planner/planner.hpp"
// #include "tasks/auto_aim/solver.hpp"
// #include "tasks/auto_aim/tracker.hpp"
// #include "tasks/auto_aim/yolo.hpp"
// #include "tools/exiter.hpp"
// #include "tools/img_tools.hpp"
// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"
// #include "tools/plotter.hpp"
// #include "tools/thread_safe_queue.hpp"

// using namespace std::chrono_literals;

// const std::string keys =
//   "{help h usage ? |                        | 输出命令行参数说明}"
//   "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

// int main(int argc, char * argv[])
// {
//   tools::Exiter exiter;
//   tools::Plotter plotter;

//   cv::CommandLineParser cli(argc, argv, keys);
//   auto config_path = cli.get<std::string>(0);
//   if (cli.has("help") || config_path.empty()) {
//     cli.printMessage();
//     return 0;
//   }

//   io::Gimbal gimbal(config_path);
//   io::Camera camera(config_path);

//   auto_aim::YOLO yolo(config_path, true);
//   auto_aim::Solver solver(config_path);
//   auto_aim::Tracker tracker(config_path, solver);
//   auto_aim::Planner planner(config_path);

//   tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
//   target_queue.push(std::nullopt);

//   std::atomic<bool> quit = false;
//   auto plan_thread = std::thread([&]() {
//     auto t0 = std::chrono::steady_clock::now();
//     uint16_t last_bullet_count = 0;

//     while (!quit) {
//       auto target = target_queue.front();
//       auto gs = gimbal.state();
//       auto plan = planner.plan(target, gs.bullet_speed);

//       gimbal.send(
//         plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
//         plan.pitch_acc);

//       auto fired = gs.bullet_count > last_bullet_count;
//       last_bullet_count = gs.bullet_count;

//       nlohmann::json data;
//       data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

//       data["gimbal_yaw"] = gs.yaw;
//       data["gimbal_yaw_vel"] = gs.yaw_vel;
//       data["gimbal_pitch"] = gs.pitch;
//       data["gimbal_pitch_vel"] = gs.pitch_vel;

//       data["target_yaw"] = plan.target_yaw;
//       data["target_pitch"] = plan.target_pitch;

//       data["plan_yaw"] = plan.yaw;
//       data["plan_yaw_vel"] = plan.yaw_vel;
//       data["plan_yaw_acc"] = plan.yaw_acc;

//       data["plan_pitch"] = plan.pitch;
//       data["plan_pitch_vel"] = plan.pitch_vel;
//       data["plan_pitch_acc"] = plan.pitch_acc;

//       data["fire"] = plan.fire ? 1 : 0;
//       data["fired"] = fired ? 1 : 0;

//       if (target.has_value()) {
//         data["target_z"] = target->ekf_x()[4];   //z
//         data["target_vz"] = target->ekf_x()[5];  //vz
//       }

//       if (target.has_value()) {
//         data["w"] = target->ekf_x()[7];
//       } else {
//         data["w"] = 0.0;
//       }

//       plotter.plot(data);

//       std::this_thread::sleep_for(10ms);
//     }
//   });

//   cv::Mat img;
//   std::chrono::steady_clock::time_point t;

//   while (!exiter.exit()) {
//     camera.read(img, t);
//     auto q = gimbal.q(t);

//     solver.set_R_gimbal2world(q);
//     auto armors = yolo.detect(img);
//     auto targets = tracker.track(armors, t);
//     if (!targets.empty())
//       target_queue.push(targets.front());
//     else
//       target_queue.push(std::nullopt);

//     // 在图像上绘制检测结果（合并原 detection 窗口信息）
//     for (const auto & armor : armors) {
//       // 画装甲四点与标签
//       tools::draw_points(img, armor.points, {0, 255, 0});
//       auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],auto_aim::ARMOR_TYPES[armor.type]);
//       tools::draw_text(img, info, armor.center, {0, 255, 0});
//     }
    
//     if (!targets.empty()) {
//       auto target = targets.front();

//       // 当前帧target更新后
//       std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
//       for (const Eigen::Vector4d & xyza : armor_xyza_list) {
//         auto image_points =
//           solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
//         tools::draw_points(img, image_points, {0, 255, 0});
//       }

//       Eigen::Vector4d aim_xyza = planner.debug_xyza;
//       auto image_points =
//         solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
//       tools::draw_points(img, image_points, {0, 0, 255});
//     }

//     cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
//     cv::imshow("reprojection", img);
//     auto key = cv::waitKey(1);
//     if (key == 'q') break;
//   }

//   quit = true;
//   if (plan_thread.joinable()) plan_thread.join();
//   gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

//   return 0;
// }
#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::draw_picture draw;
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  
  // FPS计算相关变量
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  float fps = 0.0f;
  
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    auto loop_start_time = std::chrono::steady_clock::now();
    
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

     // 在图像上绘制检测结果（合并原 detection 窗口信息）
    for (const auto & armor : armors) {
      // 画装甲四点与标签
      cv::Point2f center;
      center.x=(armor.points[0].x+armor.points[1].x+armor.points[2].x+armor.points[3].x)/4;
      center.y=(armor.points[0].y+armor.points[1].y+armor.points[2].y+armor.points[3].y)/4;
      tools::draw_points(img, armor.points, {0, 255, 0});
      auto info = fmt::format("{:.2f} {} {} {}", armor.confidence, auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],auto_aim::ARMOR_TYPES[armor.type]);
      tools::draw_text(img, info, armor.center, {20, 0, 165});
      draw.draw_Picture_center(img,center);
    }
    
    // 获取云台状态和规划信息用于UI显示
    auto gs = gimbal.state();
    std::optional<auto_aim::Target> target_opt = target_queue.front();
    bool has_target = target_opt.has_value();
    auto plan = planner.plan(target_opt, gs.bullet_speed);
    if (!targets.empty()) {
      auto target = targets.front();
       
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
        solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
      solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      cv::Point2f center;
        center.x=(image_points[0].x+image_points[1].x+image_points[2].x+image_points[3].x)/4;
        center.y=(image_points[0].y+image_points[1].y+image_points[2].y+image_points[3].y)/4;
        draw.draw_Picture_center(img,center,cv::Scalar(0, 165, 255));
        plan.fire?draw.draw_Picture_center(img,center,cv::Scalar(0, 165, 255)):tools::draw_points(img, image_points, cv::Scalar(0,0,255));
        tools::draw_points(img, image_points, cv::Scalar(0, 0, 255));
    }
    

    // 计算FPS
    frame_count++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time).count();
    
    if (elapsed_time >= 1000) { // 每秒更新一次FPS
      fps = static_cast<float>(frame_count) * 1000.0f / static_cast<float>(elapsed_time);
      frame_count = 0;
      last_fps_time = current_time;
    }    
    draw.draw_set_img(img);//设置图像
    draw.draw_TxT("Mode: AutoAim",true);// 运行模式
    draw.draw_S("FPS",fps);// FPS信息
    draw.draw_TxT("Detect",has_target ? "YES" : "NO",true);// 目标检测状态
    draw.draw_TxT("Fire",plan.fire ? "YES" : "NO",true);// 开火状态
    draw.draw_ChangeX(img_width - 180);
    draw.draw_TxT("Gimbal",true);
    draw.draw_S("Yaw",gs.yaw/(-M_PI/180),"pitch",gs.pitch/(-M_PI/180));// 云台状态 - 接收到的数据
    draw.draw_S("Vel Y",gs.yaw_vel/(-M_PI/180),"P",gs.pitch_vel/(-M_PI/180));//

    draw.draw_TxT("Plan",true);
    draw.draw_S("Yaw",plan.yaw/(-M_PI/180),"Pitch",plan.pitch/(-M_PI/180));// 规划状态 - 发送的数据
    draw.draw_S("Vel Y",plan.yaw_vel/(-M_PI/180)," P",plan.pitch_vel/(-M_PI/180));
    draw.draw_S("Acc Y",plan.yaw_acc/(-M_PI/180)," P",plan.pitch_acc/(-M_PI/180));
    
    // 目标信息（如果存在）
    if (has_target) {
      auto& target = target_opt.value();
      draw.draw_S("Target Z",target.ekf_x()[4]/(-M_PI/180),"Vz",target.ekf_x()[5]/(-M_PI/180));
      if (target.ekf_x().size() > 7) {
        draw.draw_S("Target W",target.ekf_x()[7]/(-M_PI/180));
      }
    }

    int img_width = img.cols;
    
    // 在图像右侧添加分隔线和状态指示
    cv::line(img, cv::Point(img_width - 200, 20), cv::Point(img_width - 200, 50), cv::Scalar(0, 255, 255), 2);
    draw.draw_TxT("status_title");

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::string time_str = std::ctime(&now_c);
    time_str = time_str.substr(11, 8); // 只取HH:MM:SS部分

    draw.draw_TxT("Time",time_str);// 系统时间
    draw.draw_S("Bullet Speed",gs.bullet_speed);// 子弹速度
    draw.draw_S("Targets", targets.size());// 目标数量
    draw.draw_S("Armors", armors.size());// 检测到的装甲板数量
    draw.draw_clean();
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}