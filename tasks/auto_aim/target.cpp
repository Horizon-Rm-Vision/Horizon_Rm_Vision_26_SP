#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  z1_in_world = 0;
  z2_in_world = 0;
  z3_in_world = 0;
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
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
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
  if (name == ArmorName::outpost) {
    v1 = 10;    // 前哨站加速度方差
    v2 = 0.1;   // 前哨站角加速度方差
    v_vel = 1;  // 前哨站速度方差（用于平滑中心坐标）
  } else {
    v1 = 100;   // 加速度方差
    v2 = 400;   // 角加速度方差
    v_vel = 10; // 速度方差（中心坐标会通过速度积分更新）
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  
  // 预测过程噪声偏差的方差
  // x, y, z 加入速度项的过程噪声，使中心坐标更平稳
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1 + c * v_vel,      0,      0,      0,      0,      0,      0, 0, 0, 0},  // 增加速度噪声项
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1 + c * v_vel,      0,      0,      0,      0, 0, 0, 0},  // 增加速度噪声项
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1 + c * v_vel,      0,      0, 0, 0, 0},  // 增加速度噪声项
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
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

 // tracker update函数，传入检测到的armor
void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id;
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

  // 前哨站: 重新按z高度分配id
  if(armor.name == ArmorName::outpost && armor_num_ == 3) {
    // 按z坐标排序,让id=0/1/2对应下/中/上
    std::sort(
      xyza_i_list.begin(), xyza_i_list.end(),
      [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        return a.first[2] < b.first[2];  // 按z坐标排序
      });
    
    // 重新分配id: 按排序后的顺序赋值为0,1,2
    for (int i = 0; i < armor_num_; i++) {
      xyza_i_list[i].second = i;
    }
    
    // 然后再按距离排序用于后续匹配
    std::sort(
      xyza_i_list.begin(), xyza_i_list.end(),
      [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
      });
  }

  // 取前3个distance最小的装甲板
  for (int i = 0; i < std::min(3, armor_num_); i++) {
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

  // 前哨站z坐标处理: 现在id已经按高度排序,直接取中间id
  if(armor.name == ArmorName::outpost) {
    if(id == 0) z1_in_world = armor.xyz_in_world[2];
    if(id == 1) z2_in_world = armor.xyz_in_world[2];
    if(id == 2) z3_in_world = armor.xyz_in_world[2];
    
    // 将三个值放入数组并排序取中间值
    double z_values[3] = {z1_in_world, z2_in_world, z3_in_world};
    std::sort(z_values, z_values + 3);
    
    // 更新装甲板的z坐标为中间值
    Armor armor_modified = armor;
    armor_modified.xyz_in_world[2] = z_values[1];
    
    update_ypda(armor_modified, id);
    return;
  }
  update_ypda(armor, id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  // 从装甲板位置反演旋转中心坐标
  auto angle = tools::limit_rad(ekf_.x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto r = (use_l_h) ? ekf_.x[8] + ekf_.x[9] : ekf_.x[8];
  
  // 从装甲板坐标反演中心坐标
  // armor_x = center_x - r * cos(angle)
  // center_x = armor_x + r * cos(angle)
  auto inferred_center_x = armor.xyz_in_world[0] + r * std::cos(angle);
  auto inferred_center_y = armor.xyz_in_world[1] + r * std::sin(angle);
  auto inferred_center_z = (use_l_h) ? armor.xyz_in_world[2] - ekf_.x[10] : armor.xyz_in_world[2];

  // 扩展观测向量：[yaw_error, pitch, distance, roll, center_x, center_y, center_z]
  // 观测噪声设置
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  
  // 根据观测的可信度调整噪声
  // 中心坐标约束应该比装甲板约束宽松很多，因为反演的准确性依赖于角度精度
  double center_noise_xy = 0.1 * (1.0 + 2.0 * std::abs(delta_angle));  // xy方向噪声较大
  double center_noise_z = 0.2 * (1.0 + std::abs(delta_angle));          // z方向噪声更大
  
  Eigen::VectorXd R_dig(7);
  R_dig << 
    4e-3,  // yaw error
    4e-3,  // pitch
    log(std::abs(delta_angle) + 1) + 1,  // distance
    log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2,  // roll
    center_noise_xy,  // center_x noise (较大的噪声，降低约束权重)
    center_noise_xy,  // center_y noise
    center_noise_z;   // center_z noise

  Eigen::MatrixXd R = R_dig.asDiagonal();
  
  // 计算扩展Jacobian矩阵
  Eigen::MatrixXd H = h_jacobian_extended(ekf_.x, id);

  // 定义扩展的非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto a = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    
    // 从当前状态反演中心坐标
    auto use_l_h_local = (armor_num_ == 4) && (id == 1 || id == 3);
    auto r_local = (use_l_h_local) ? x[8] + x[9] : x[8];
    auto cx = xyz[0] + r_local * std::cos(a);
    auto cy = xyz[1] + r_local * std::sin(a);
    auto cz = (use_l_h_local) ? xyz[2] - x[10] : xyz[2];
    
    Eigen::VectorXd result(7);
    result << ypd[0], ypd[1], ypd[2], a, cx, cy, cz;
    return result;
  };

  // 防止角度求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);  // yaw
    c[3] = tools::limit_rad(c[3]);  // roll
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  
  // 扩展观测向量：[ypd(3), roll, center_xyz(3)]
  Eigen::VectorXd z(7);
  z << ypd[0], ypd[1], ypd[2], ypr[0], inferred_center_x, inferred_center_y, inferred_center_z;

  ekf_.update(z, H, R, h, z_subtract);
}

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
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

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

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4]; // 考虑高度差，新版前哨站应该改这部分
  if(armor_num_ == 3) { // 三装甲板高度特判
    if(id == 0) armor_z = x[4] - 0.1;
    if(id == 1) armor_z = x[4];
    if(id == 2) armor_z = x[4] + 0.1; 
  }

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
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

Eigen::MatrixXd Target::h_jacobian_extended(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // 扩展H矩阵，包含中心坐标的约束
  // 前4行：装甲板的ypd和角度
  // 后3行：中心坐标xyz的Jacobian
  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},  // armor_x
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},  // armor_y
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},  // armor_z
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}   // angle
  };
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

  Eigen::MatrixXd H_ypda = H_armor_ypda * H_armor_xyza;

  // 中心坐标的Jacobian
  // center_x = armor_x + r * cos(angle) 
  // center_y = armor_y + r * sin(angle)
  // center_z = armor_z (± offset)
  
  // d(center_x)/d(state)
  auto dcx_da = armor_xyz[0] * std::sin(angle) + r * std::cos(angle);  // armor_x*sin + r*cos
  auto dcx_dr = std::cos(angle);
  auto dcx_dl = (use_l_h) ? std::cos(angle) : 0.0;

  // d(center_y)/d(state)
  auto dcy_da = -armor_xyz[1] * std::cos(angle) + r * std::sin(angle);  // -armor_y*cos + r*sin
  auto dcy_dr = std::sin(angle);
  auto dcy_dl = (use_l_h) ? std::sin(angle) : 0.0;

  // d(center_z)/d(state)
  auto dcz_dh = (use_l_h) ? -1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_center{
    {1, 0, 0, 0, 0, 0, dcx_da, 0, dcx_dr, dcx_dl,     0},  // center_x
    {0, 0, 1, 0, 0, 0, dcy_da, 0, dcy_dr, dcy_dl,     0},  // center_y
    {0, 0, 0, 0, 1, 0,      0, 0,      0,      0, dcz_dh}   // center_z
  };
  // clang-format on

  // 组合Jacobian矩阵 [7x11]
  Eigen::MatrixXd H_extended(7, 11);
  H_extended.block(0, 0, 4, 11) = H_ypda;
  H_extended.block(4, 0, 3, 11) = H_center;

  return H_extended;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim