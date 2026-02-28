#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 选择弹道模型: 默认不考虑空气阻力（抛物线），
  // hero 模型来源于 Horizon_Hero_Aim_26 项目，带有空气阻力迭代解算。
  enum class Model { kNoDrag, kHero };

  // 构造弹道。默认使用抛物线模型。
  // 
  // v0   子弹初速度大小，单位：m/s
  // d    目标水平距离，单位：m
  // h    目标竖直高度，单位：m
  // model 弹道模型，见 Model 枚举。
  Trajectory(const double v0, const double d, const double h,
             Model model = Model::kNoDrag);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP