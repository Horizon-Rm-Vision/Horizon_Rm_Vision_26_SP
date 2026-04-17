#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

#ifdef NOVA_OUTPOST_V2
#include <algorithm>
#include <cmath>
#include <limits>
#endif

namespace auto_aim
{

#ifdef NOVA_OUTPOST_V2
namespace
{
double sin_interp(double x, double x0, double x1, double y0, double y1)
{
  if (x <= x0) return y0;
  if (x >= x1) return y1;
  double ratio = (x - x0) / (x1 - x0);
  double s = std::sin(ratio * CV_PI / 2.0);
  return y0 + (y1 - y0) * s;
}
}  // namespace
#endif

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  #ifdef NOVA_OUTPOST_V2
    current_id_(0),
  #endif
  t_(t),
  is_switch_(false),
  is_converged_(false),
  #ifdef NOVA_OUTPOST_V2
    is_h_converged_(false),
  #endif
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  //NOVA_OUTPOST_V1
  // x9 x10在前哨站中复用，分别表示与第一块装甲板的高度差outpost_z01 outpost_z02
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
#ifdef NOVA_OUTPOST_V2
  if (name == ArmorName::outpost) {
    observed_ids_.insert(0);
  }
#endif
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  #ifndef AIM_CENTER
  double v1, v2;
  #endif
  #ifdef NOVA_OUTPOST_V2
    double v_h = 0.0;
  #endif
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
    #ifdef NOVA_OUTPOST_V2
        v_h = 0.0006;  // 前哨站高度差过程噪声
    #endif
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  #ifdef NOVA_OUTPOST_V1
  auto q_r = 1e-4;
  auto q_l = 1e-4;
  auto q_h = 1e-4;
  // 预测过程噪声偏差的方差
  // clang-format off
  //std::cout<<"Using NOVA_OUTPOST_V1 Q matrix"<<std::endl;
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, q_r, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, q_l, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, q_h}
  };
  #endif
  #ifdef NOVA_OUTPOST_V2
  //std::cout<<"Using NOVA_OUTPOST_V2 Q matrix"<<std::endl;
    // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0,   0,   0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, v_h,   0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0,   0, v_h}
  };
  #endif
  #ifndef NOVA_OUTPOST_V1
  #ifndef NOVA_OUTPOST_V2
  //std::cout<<"Using DEFAULT Q matrix"<<std::endl;
    // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  #endif
  #endif
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

#ifndef NOVA_OUTPOST_V2
 // tracker update函数，传入检测到的armor
void Target::update(const Armor & armor)
{
  // 装甲板匹配
  #ifdef NOVA_OUTPOST_V1
  int id = 0;
  #else
  int id;
  #endif
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }
  
  // 按distance排序,匹配id
  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) { 
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;
  #ifdef NOVA_OUTPOST_V1
  ID = id;  // debug only
  #endif
  update_ypda(armor, id);
}
#endif
#ifdef NOVA_OUTPOST_V2
 // tracker update函数，传入检测到的armor
void Target::update(const Armor & armor)
{
  int id = 0;

  if (name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_NUM) {
    id = outpost_match_by_mahalanobis(armor);
    observed_ids_.insert(id);
    current_id_ = id;
  } else {
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++) {
      xyza_i_list.push_back({xyza_list[i], i});
    }

    std::sort(
      xyza_i_list.begin(), xyza_i_list.end(),
      [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
      });

    int candidate_count = std::min(3, static_cast<int>(xyza_i_list.size()));
    for (int i = 0; i < candidate_count; i++) {
      const auto & xyza = xyza_i_list[i].first;
      Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
      auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

      if (std::abs(angle_error) < std::abs(min_angle_error)) {
        id = xyza_i_list[i].second;
        min_angle_error = angle_error;
      }
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;
  update_ypda(armor, id);
}
#endif

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  // 观测量只传入中间装甲板的高度数据
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}
#ifdef NOVA_OUTPOST_V2
Eigen::VectorXd Target::outpost_predict_observation(const Eigen::VectorXd & x, int id) const
{
  Eigen::Vector3d xyz = h_armor_xyz(x, id);
  Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / OUTPOST_ARMOR_NUM);
  Eigen::VectorXd result(4);
  result << ypd[0], ypd[1], ypd[2], angle;
  return result;
}

Eigen::MatrixXd Target::outpost_compute_R_from_prediction(const Eigen::VectorXd & z_pred) const
{
  double delta_angle = tools::limit_rad(z_pred[3] - z_pred[0]);
  double abs_delta = std::abs(delta_angle);
  double distance = z_pred[2];

  Eigen::VectorXd R_dig(4);
  R_dig << 4e-3, 4e-3,
    sin_interp(abs_delta, 0.0, CV_PI / 2.0, 0.05, 0.07) + distance * distance * 0.01,
    std::log(std::abs(distance) + 1) * 0.005 +
      sin_interp(CV_PI / 2.0 - abs_delta, 0.0, CV_PI / 2.0, 0.09, 0.09);

  return R_dig.asDiagonal();
}

int Target::outpost_match_by_mahalanobis(const Armor & armor)
{
  Eigen::VectorXd z_obs(4);
  z_obs << armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2], armor.ypr_in_world[0];

  double d2_list[OUTPOST_ARMOR_NUM] = {0, 0, 0};
  double min_d2 = std::numeric_limits<double>::max();
  int best_id = -1;

  for (int id = 0; id < OUTPOST_ARMOR_NUM; id++) {
    Eigen::VectorXd z_pred = outpost_predict_observation(ekf_.x, id);
    Eigen::VectorXd nu = z_obs - z_pred;
    nu[0] = tools::limit_rad(nu[0]);
    nu[1] = tools::limit_rad(nu[1]);
    nu[3] = tools::limit_rad(nu[3]);

    Eigen::MatrixXd R = outpost_compute_R_from_prediction(z_pred);
    Eigen::MatrixXd R_inv = R.inverse();
    double d2 = (nu.transpose() * R_inv * nu)(0, 0);
    d2_list[id] = d2;

    if (std::isfinite(d2) && d2 < OUTPOST_MATCH_GATE) {
      if (d2 < min_d2) {
        min_d2 = d2;
        best_id = id;
      }
    }
  }

  if (best_id < 0) {
    double min_valid_d2 = std::numeric_limits<double>::max();
    int min_valid_id = -1;
    for (int id = 0; id < OUTPOST_ARMOR_NUM; id++) {
      if (std::isfinite(d2_list[id]) && d2_list[id] < min_valid_d2) {
        min_valid_d2 = d2_list[id];
        min_valid_id = id;
      }
    }
    best_id = (min_valid_id >= 0) ? min_valid_id : current_id_;
  }

  return best_id;
}

bool Target::outpost_h_converged() const
{
  double p_h1 = ekf_.P(9, 9);
  double p_h2 = ekf_.P(10, 10);
  bool variance_ok = (p_h1 < 0.05) && (p_h2 < 0.05);
  bool enough_ids = observed_ids_.size() >= 2;
  return variance_ok && enough_ids;
}
#endif

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  #ifdef NOVA_OUTPOST_V2
    if (name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_NUM) {
    if (
      std::abs(ekf_.x[9]) > OUTPOST_H_MAX_REASONABLE ||
      std::abs(ekf_.x[10]) > OUTPOST_H_MAX_REASONABLE)
      return true;

    if (ekf_.x.hasNaN() || ekf_.P.hasNaN()) return true;

    return false;
  }
  #endif
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}
#ifndef NOVA_OUTPOST_V2
bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}
#endif
#ifdef NOVA_OUTPOST_V2
bool Target::convergened()
{
  if (is_converged_) return true;
  if (this->diverged()) return false;

  if (name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_NUM) {
    if (update_count_ < 10) return false;
    if (!outpost_h_converged()) return false;

    double p_pos_max = std::max({ekf_.P(0, 0), ekf_.P(2, 2), ekf_.P(4, 4)});
    double p_vel_max = std::max({ekf_.P(1, 1), ekf_.P(3, 3), ekf_.P(5, 5)});

    if (p_pos_max > 0.5 || p_vel_max > 10.0) return false;

    is_converged_ = true;
    is_h_converged_ = true;
    return true;
  }

  if (update_count_ > 3) {
    is_converged_ = true;
  }
  return is_converged_;
}
#endif
#ifndef NOVA_OUTPOST_V2
// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  #ifdef NOVA_OUTPOST_V1
  // 前哨站特殊计算z坐标
  if(armor_num_ == 3){
    armor_z = id == 0 ? x[4] 
      : id == 1 ? x[4] + x[9]
      : id == 2 ? x[4] + x[10]
      : x[4];
  }
  #endif
  return {armor_x, armor_y, armor_z};
}
#endif

#ifdef NOVA_OUTPOST_V2
// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);

  double r;
  double armor_z;
  if (name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_NUM) {
    r = x[8];
    if (id == 0)
      armor_z = x[4];
    else if (id == 1)
      armor_z = x[4] + x[9];
    else
      armor_z = x[4] + x[10];
  } else {
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
    r = (use_l_h) ? x[8] + x[9] : x[8];
    armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  }

  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);

  return {armor_x, armor_y, armor_z};
}
#endif

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  #ifdef NOVA_OUTPOST_V2
    if (name == ArmorName::outpost && armor_num_ == OUTPOST_ARMOR_NUM) {
    auto r = x[8];
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);

    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);

    double dz_dh1 = (id == 1) ? 1.0 : 0.0;
    double dz_dh2 = (id == 2) ? 1.0 : 0.0;

    Eigen::MatrixXd H_armor_xyza{
      {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, 0,      0},
      {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, 0,      0},
      {0, 0, 0, 0, 1, 0,     0, 0,     0, dz_dh1, dz_dh2},
      {0, 0, 0, 0, 0, 0,     1, 0,     0, 0,      0}
    };

    Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
    Eigen::MatrixXd H_armor_ypda{
      {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
      {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
      {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
      {                0,                 0,                 0, 1}
    };

    return H_armor_ypda * H_armor_xyza;
  }
  #endif
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  #ifdef NOVA_OUTPOST_V1
  auto outpost_z01 = 0.0; //复用
  auto outpost_z02 = (use_l_h) ? 1.0 : 0.0; //复用 原本为dz_dh
  // 前哨站特殊计算z坐标导数
  if(armor_num_ == 3){
    dx_dl = 0;
    dy_dl = 0;
    outpost_z01 = (id==1) ? 1 : 0;
    outpost_z02 = (id==2) ? 1 : 0;
  }
  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     outpost_z01, outpost_z02},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  #else
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  #endif
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
