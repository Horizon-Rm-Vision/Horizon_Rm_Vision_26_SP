#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "buff_detector.hpp"
#include "buff_type.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/ransac_sine_fitter.hpp"

namespace auto_buff
{
class Voter
{
public:
  Voter();
  void vote(const double angle_last, const double angle_now);
  int clockwise();

private:
  int clockwise_;
};

/// Target 基类

class Target
{
public:
  Target();
  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;  // 纯虚函数

  virtual void predict(double dt) = 0;  // 纯虚函数

  virtual std::unique_ptr<Target> clone() const = 0;

  void reset();

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  /// blade_id 重载: blade_id=0..4 对应 fanblades 中的 5 个位置偏移
  Eigen::Vector3d point_buff2world(
    const Eigen::Vector3d & point_in_buff, int blade_id) const;

  bool is_unsolve() const;

  Eigen::VectorXd ekf_x() const;

  /// 获取 EKF 估计的 roll 角, 供 Director 做叶片 ID 匹配
  double get_roll() const { return ekf_.x[5]; }

  double spd = 0;  //调试用

protected:
  virtual void init(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  virtual void update(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  tools::ExtendedKalmanFilter ekf_;
  double lasttime_ = 0;
  Voter voter;  // 逆时针-1 顺时针1
  bool first_in_;
  bool unsolvable_;
};

/// SmallTarget子类

class SmallTarget : public Target
{
public:
  SmallTarget();

  std::unique_ptr<Target> clone() const override;

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  const double SMALL_W = CV_PI / 3;
  // const double SMALL_W = 0;
};

/// BigTarget子类

class BigTarget : public Target
{
public:
  BigTarget();

  std::unique_ptr<Target> clone() const override;

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  tools::RansacSineFitter spd_fitter_;

  double fit_spd_;
};

}  // namespace auto_buff
#endif