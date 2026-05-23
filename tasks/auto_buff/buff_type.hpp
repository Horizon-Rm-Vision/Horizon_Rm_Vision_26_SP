#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "tools/math_tools.hpp"
namespace auto_buff
{
const int INF = 1000000;
enum PowerRune_type { SMALL, BIG };
enum FanBlade_type { _target, _unlight, _light };
enum Track_status { TRACK, TEM_LOSE, LOSE };
enum class Color { red, blue, unknown };

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

class FanBlade
{
public:
  cv::Point2f center;               // 扇页中心
  std::vector<cv::Point2f> points;  // 四个点从左上角开始逆时针
  double angle, width, height;
  FanBlade_type type;  // 类型
  Color color = Color::unknown;  // 扇叶颜色（仅yolox_ov/yolox_trt模式有效）

  explicit FanBlade() = default;

  explicit FanBlade(
    const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t,
    Color c = Color::unknown);

  explicit FanBlade(FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;
  std::vector<FanBlade> fanblades;  // 按target开始顺时针

  int light_num;

  /// 2026 大符双目标: 记录 fanblades[0..4] 中哪些位置是被点亮的目标
  std::vector<int> target_indices_;
  bool big_2026_mode_ = false;

  Eigen::Vector3d xyz_in_world;  // 单位：m
  Eigen::Vector3d ypr_in_world;  // 单位：rad
  Eigen::Vector3d ypd_in_world;  // 球坐标系

  Eigen::Vector3d blade_xyz_in_world;  // 单位：m
  Eigen::Vector3d blade_ypd_in_world;  // 球坐标系, 单位: m

  /// 多目标匹配: 记录每个可见扇叶匹配到的 blade_id + 尖端坐标
  struct MatchedBlade {
    int blade_id;                         ///< 匹配到的叶片编号 0..4
    Eigen::Vector3d blade_ypd_in_world;   ///< 扇叶尖端球坐标
    Eigen::Vector3d blade_xyz_in_world;   ///< 扇叶尖端笛卡尔坐标
  };
  std::vector<MatchedBlade> matched_blades_;

  explicit PowerRune(
    std::vector<FanBlade> & ts, const cv::Point2f r_center,
    std::optional<PowerRune> last_powerrune,
    bool big_2026_mode = false);
  explicit PowerRune() = default;

  FanBlade & target() { return fanblades[0]; };

  /// 2026: 返回所有被点亮的 fanblade 指针
  std::vector<FanBlade*> get_targets();

  bool is_unsolve() const { return unsolvable_; }

private:
  double target_angle_;
  bool unsolvable_ = false;

  double atan_angle(cv::Point2f v) const;  // [0, 2CV_PI]
};
}  // namespace auto_buff
#endif  // BUFF_TYPE_HPP
