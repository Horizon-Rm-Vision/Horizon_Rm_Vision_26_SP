#ifndef AUTO_BUFF__DIRECTOR_HPP
#define AUTO_BUFF__DIRECTOR_HPP

#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

#include "buff_target.hpp"
#include "buff_type.hpp"

namespace auto_buff
{

/// 2026 大符激活状态机
enum class ActivationState {
  IDLE,               ///< 未激活: 恒速旋转 π/3, 等待激活
  WAITING_FIRST,      ///< 正在激活: 2 块灯臂点亮, 等待击中第一块
  SECOND_WINDOW,      ///< 第一块已击中: 1s 窗口内击中第二块
  GROUP_TRANSITION,   ///< 组间过渡: 等待下一组出现
  ALL_COMPLETED,      ///< 5 组全部激活成功
  ACTIVATION_FAILED   ///< 2.5s 超时, 激活失败
};

/// 2026 大符激活管理器
///
/// 职责:
///   1. 跟踪 5 组激活进度
///   2. 确定当前瞄准哪一块灯臂 (blade_id)
///   3. 根据视觉检测结果 + 超时判定命中/切换
///
/// 设计思路 (借鉴 auto_aim Tracker → Aimer 的分离):
///   - BigTarget EKF 跟踪旋转状态 (与叶片数量无关)
///   - Director 利用 EKF roll + PowerRune.target_indices_ 做 ID 匹配
///   - Aimer 根据 Director 给出的 blade_id 计算瞄准点
class Buff2026Director
{
public:
  Buff2026Director();

  /// 每帧调用: 传入当前帧检测结果与 EKF 跟踪器
  void update(
    const std::optional<PowerRune> & power_rune,
    const std::chrono::steady_clock::time_point & now,
    const Target & target,
    cv::Point2f image_center = cv::Point2f(0, 0));

  /// 当前应瞄准的 blade_id (0..4), -1 表示不应射击
  int getAimBladeId() const { return current_target_idx_ >= 0 ? target_pair_[current_target_idx_] : -1; }

  /// 当前状态
  ActivationState getState() const { return state_; }

  /// 已完成组数 (0..5)
  int getCompletedGroups() const { return completed_groups_; }

  /// 是否应开火
  bool shouldFire() const { return state_ == ActivationState::WAITING_FIRST || state_ == ActivationState::SECOND_WINDOW; }

  /// 重置 (大符进入未激活)
  void reset();

private:
  ActivationState state_;

  /// 当前组的目标 blade_id [2]
  int target_pair_[2];
  int current_target_idx_;  ///< 0 或 1, 指向 target_pair_ 中正在瞄准的

  int completed_groups_;    ///< 已完成组数 0..5

  std::chrono::steady_clock::time_point group_start_time_;  ///< 当前组开始时间
  std::chrono::steady_clock::time_point first_hit_time_;    ///< 击中第一块的时间

  /// 上一帧的 target_indices_ 快照, 用于检测叶片消失
  std::vector<int> last_target_indices_;

  /// 图像中心 (用于选择距离最近的叶片优先瞄准)
  cv::Point2f image_center_;

  // ===== 超时参数 =====
  static constexpr double FIRST_HIT_TIMEOUT_S = 2.5;   ///< 击中第一块超时
  static constexpr double SECOND_HIT_TIMEOUT_S = 1.0;  ///< 击中第二块超时
  static constexpr double TRANSITION_TIMEOUT_S = 0.3;  ///< 组间过渡等待

  // ===== 内部辅助 =====
  /// 将当前检测到的 target_indices_ 与上一帧比较, 检测命中
  /// @return 被击中的 blade_id, -1 表示无命中
  int detectHit(const std::vector<int> & current_indices);

  /// 启动新一组 (按距图像中心距离排序, 近的优先)
  void startNewGroup(const std::vector<int> & target_indices, const PowerRune & pr);

  /// 进入下一组 (或完成)
  void advanceGroup();

  /// 检查 2026 大符是否处于激活 (视觉上看到 1-2 块亮)
  bool isActivating(const std::optional<PowerRune> & pr) const;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__DIRECTOR_HPP
