#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  aim_center_ = tools::read<bool>(yaml, "aim_center");
  armor_yaw_threshold_ = tools::read<double>(yaml, "armor_yaw_threshold");

  // 读取弹道模型配置
  if (yaml["ballistic_model"]) {
    auto str = yaml["ballistic_model"].as<std::string>();
    if (str == "hero")
      ballistic_model_ = BallisticModel::kHero;
    else
      ballistic_model_ = BallisticModel::kNoDrag;
  } else {
    ballistic_model_ = BallisticModel::kNoDrag;
  }

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(
    bullet_speed, min_dist, xyz.z(),
    ballistic_model_ == BallisticModel::kHero ? tools::Trajectory::Model::kHero
                                               : tools::Trajectory::Model::kNoDrag);
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  //auto gs = gimbal.state();
  //x0 << (gs.yaw - yaw0), gs.yaw_vel;
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  #ifdef SR_VEL
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  #endif

  // //保存上次状态供下次使用，暂时无用
  // plan.last_yaw = plan.yaw + plan.yaw_vel * DT;
  // plan.last_pitch = plan.pitch + plan.yaw_vel * DT;
  // last_yaw_ = plan.last_yaw;
  // last_pitch_ = plan.last_pitch;

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  if (aim_center_) {
    return aim_at_center(*target, bullet_speed);
  } else {
    return plan(*target, bullet_speed);
  }
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(
    bullet_speed, min_dist, xyz.z(),
    ballistic_model_ == BallisticModel::kHero ? tools::Trajectory::Model::kHero
                                               : tools::Trajectory::Model::kNoDrag);
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

Plan Planner::aim_at_center(Target target, double bullet_speed)
{
  target.v1 = 20;
  // target.v2 = 150;
  Plan plan;
  // 整车中心坐标减去半径为正对相机装甲板坐标
  Eigen::Vector3d xyz;
  xyz = {target.ekf_x()[0] - target.ekf_x()[8], target.ekf_x()[2], target.ekf_x()[4]};

  // 存储世界坐标系中的目标位置，仅重投影用
  if(target.ekf_x()[0]==0 && target.ekf_x()[2]==0 || target.armor_xyza_list().empty()) {
    center_points = Eigen::Vector3d(0, 0, 0);
  } else {
    center_points = Eigen::Vector3d(target.ekf_x()[0], target.ekf_x()[2], target.ekf_x()[4]);
  }

  // 计算弹道是否可解
  double center_dist = xyz.head<2>().norm();
  float yaw, pitch;
  //根据弹道飞行时间预测目标位置
  auto bullet_traj = tools::Trajectory(
  bullet_speed, center_dist, xyz.z(),
  ballistic_model_ == BallisticModel::kHero ? tools::Trajectory::Model::kHero
                                               : tools::Trajectory::Model::kNoDrag);
  target.predict(bullet_traj.fly_time);

  if (ballistic_model_ == BallisticModel::kHero) {
      auto azim = std::atan2(xyz.y(), xyz.x());
      auto bullet_traj = tools::Trajectory(
      bullet_speed, center_dist, xyz.z(),
      ballistic_model_ == BallisticModel::kHero ? tools::Trajectory::Model::kHero
                                                : tools::Trajectory::Model::kNoDrag);
      if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");
      yaw = tools::limit_rad(azim + yaw_offset_);
      pitch = -bullet_traj.pitch - pitch_offset_;
  }
  else{
    auto bullet_traj = tools::Trajectory(bullet_speed, center_dist, xyz.z(), tools::Trajectory::Model::kNoDrag);
    if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");
    
      // 计算yaw和pitch
      auto azim = std::atan2(xyz.y(), xyz.x());
      yaw = tools::limit_rad(azim + yaw_offset_);
      pitch = -bullet_traj.pitch - pitch_offset_;
  }


  // 开火条件,距离最近的装甲板的yaw与云台夹角小于armor_yaw_threshold_阈值
  Eigen::Vector3d armor_xyz;
  double armor_yaw;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      armor_xyz = xyza.head<3>();
      armor_yaw = xyza[3];
    }
  }
  if(armor_yaw  > armor_yaw_threshold_ || armor_yaw < -armor_yaw_threshold_) {
    plan.control = false;
    plan.fire = false;
  }
  if(armor_yaw  < armor_yaw_threshold_ && armor_yaw  > -armor_yaw_threshold_) {
    plan.control = true;
    plan.fire = true;
  }

  // 返回规划结果
  plan.target_yaw = yaw;
  plan.target_pitch = pitch; // 规划值没调用MPC求解，直接返回了计算的yaw和pitch
  plan.yaw = yaw;
  plan.yaw_vel = 0;
  plan.yaw_acc = 0;
  plan.pitch = pitch;
  plan.pitch_vel = 0;
  plan.pitch_acc = 0;

  return plan;
}

Plan Planner::go(Armor armor)
{
    Eigen::Vector3d xyz = armor.xyz_in_world;

    double horizontal = std::hypot(xyz.x(), xyz.y());

    double azim = std::atan2(xyz.y(), xyz.x());
    float yaw = tools::limit_rad(azim + yaw_offset_);

    double pitch_angle = std::atan2(xyz.z(), horizontal);
    float pitch = -pitch_angle - pitch_offset_;

    Plan plan;
    plan.control = true;
    plan.fire = false;
    plan.target_yaw = yaw;
    plan.target_pitch = pitch;
    plan.yaw = yaw;
    plan.yaw_vel = 0;
    plan.yaw_acc = 0;
    plan.pitch = pitch;
    plan.pitch_vel = 0;
    plan.pitch_acc = 0;
    return plan;
}

}  // namespace auto_aim