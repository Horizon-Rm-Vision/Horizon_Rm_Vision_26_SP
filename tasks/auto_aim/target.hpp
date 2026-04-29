#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

#ifdef NOVA_OUTPOST_V2
#include <set>
#endif

namespace auto_aim
{

class Target
{
public:
#ifdef NOVA_OUTPOST_V2
  struct OutpostV2Params
  {
    double h_max_reasonable = 0.25;
    double match_gate = 10.0;
    double q_h = 0.0006;
    double h_converged_variance = 0.05;
    double converged_pos_p_max = 0.5;
    double converged_vel_p_max = 10.0;
  };

  static void set_outpost_v2_params(const OutpostV2Params & params);
#endif

  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only
  #ifdef AIM_CENTER
  double v1, v2;
  #endif
  #ifdef NOVA_OUTPOST_V1
  int ID;       // debug only
  #endif

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

  // hero 模式：前哨站重投影后，仅当预测打到“下面那块装甲板”才允许 fire
  // 说明：以装甲板世界坐标 z 最小者作为“下面装甲板”。
  bool is_outpost_predicted_bottom_armor() const;

private:
#ifndef NOVA_OUTPOST_V2
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
#endif
#ifdef NOVA_OUTPOST_V2
  static constexpr int OUTPOST_ARMOR_NUM = 3;
  static OutpostV2Params outpost_v2_params_;

  int armor_num_ = 0;
  int switch_count_ = 0;
  int update_count_ = 0;
  int current_id_ = 0;

  bool is_switch_ = false, is_converged_ = false, is_h_converged_ = false;

  std::set<int> observed_ids_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  int outpost_match_by_mahalanobis(const Armor & armor);
  Eigen::VectorXd outpost_predict_observation(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd outpost_compute_R_from_prediction(const Eigen::VectorXd & z_pred) const;
  bool outpost_h_converged() const;
#endif
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP