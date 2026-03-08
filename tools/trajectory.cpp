#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h, Model model)
{
  // 距离必须为正
  if (d < 0) {
    unsolvable = true;
    return;
  }

  if (model == Model::kNoDrag) {
    // 经典抛物线解算
    auto a = g * d * d / (2 * v0 * v0);
    auto b = -d;
    auto c = a + h;
    auto delta = b * b - 4 * a * c;

    if (delta < 0) {
      unsolvable = true;
      return;
    }

    unsolvable = false;
    auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
    auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
    auto pitch_1 = std::atan(tan_pitch_1);
    auto pitch_2 = std::atan(tan_pitch_2);
    auto t_1 = d / (v0 * std::cos(pitch_1));
    auto t_2 = d / (v0 * std::cos(pitch_2));

    pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
    fly_time = (t_1 < t_2) ? t_1 : t_2;
  } else {
    // Hero 弹道模型，带空气阻力，参考 Horizon_Hero_Aim_26 中的算法
    // 初始估计使用简单抛物线的俯仰角
    double theta = std::atan2(h, d);
    // 阻力系数计算，和 Hero 项目保持一致
    double k1 = 0.47 * 1.169 * (2 * M_PI * 0.02125 * 0.02125) / 2 / 0.041;
    double delta_z;
    double t = 0;
    unsolvable = false;

    for (int i = 0; i < 100; i++) {
      double cos_theta = std::cos(theta);
      if (cos_theta == 0) {
        unsolvable = true;
        break;
      }
      // 计算飞行时间
      t = (std::exp(k1 * d) - 1) / (k1 * v0 * cos_theta);
      // z 误差
      delta_z = h - v0 * std::sin(theta) * t / cos_theta + 0.5 * g * t * t / (cos_theta * cos_theta);
      if (std::fabs(delta_z) < 1e-6) {
        break;
      }
      // 牛顿迭代更新theta
      theta -= delta_z / (-(v0 * t) / (cos_theta * cos_theta) +
                          g * t * t / (v0 * v0) * std::sin(theta) / (cos_theta * cos_theta * cos_theta));
      if (std::isnan(theta)) {
        unsolvable = true;
        break;
      }
    }
    if (unsolvable) return;
    pitch = theta;
    fly_time = std::abs(d / (v0 * std::cos(theta)));
  }
}

}  // namespace tools