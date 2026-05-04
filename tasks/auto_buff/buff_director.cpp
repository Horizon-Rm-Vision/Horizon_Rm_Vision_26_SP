#include "buff_director.hpp"

#include <algorithm>

#include "tools/logger.hpp"

namespace auto_buff
{

Buff2026Director::Buff2026Director()
: state_(ActivationState::IDLE), current_target_idx_(-1), completed_groups_(0)
{
  target_pair_[0] = target_pair_[1] = -1;
}

void Buff2026Director::update(
  const std::optional<PowerRune> & power_rune,
  const std::chrono::steady_clock::time_point & now,
  const Target & target,
  cv::Point2f image_center)
{
  image_center_ = image_center;

  // 没有检测到 或 非 2026 大符模式
  if (!power_rune.has_value() || !power_rune->big_2026_mode_) {
    if (state_ != ActivationState::IDLE)
      tools::logger()->info("[Buff2026Director] 丢失目标, 回到 IDLE");
    reset();
    return;
  }

  const PowerRune & pr = power_rune.value();
  const auto & current_indices = pr.target_indices_;
  auto now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
  auto group_start_s = std::chrono::duration<double>(group_start_time_.time_since_epoch()).count();

  switch (state_) {
    // ===================== IDLE =====================
    case ActivationState::IDLE: {
      // 检测到 1-2 块叶片亮起 → 激活开始
      if (isActivating(power_rune)) {
        tools::logger()->info("[Buff2026Director] 检测到激活开始, {} 块亮",
                              current_indices.size());
        startNewGroup(current_indices, pr);
        state_ = ActivationState::WAITING_FIRST;
        group_start_time_ = now;
        for (int id : current_indices)
          tools::logger()->info("  -> target_indices_: {}", id);
      }
      break;
    }

    // ===================== WAITING_FIRST =====================
    case ActivationState::WAITING_FIRST: {
      // 超时检查: 2.5s 未命中第一块
      if (now_s - group_start_s > FIRST_HIT_TIMEOUT_S) {
        tools::logger()->warn("[Buff2026Director] 2.5s 超时, 激活失败");
        state_ = ActivationState::ACTIVATION_FAILED;
        break;
      }

      if (current_indices.empty()) {
        // 检测丢失, 保持状态等待
        break;
      }

      // 检测命中
      int hit_id = detectHit(current_indices);
      if (hit_id >= 0) {
        tools::logger()->info("[Buff2026Director] 击中 blade {} (第一块)", hit_id);
        first_hit_time_ = now;

        // 检查是否还有第二块
        if (current_indices.size() >= 2) {
          // 两块仍在 → 切换到第二块瞄准
          for (int i = 0; i < 2; ++i) {
            if (target_pair_[i] == hit_id) {
              current_target_idx_ = (i == 0) ? 1 : 0;
              break;
            }
          }
          state_ = ActivationState::SECOND_WINDOW;
        } else if (current_indices.size() == 1) {
          // 只剩一块 → 切换瞄准它
          current_target_idx_ = 0;
          target_pair_[0] = current_indices[0];
          target_pair_[1] = -1;
          state_ = ActivationState::SECOND_WINDOW;
        } else {
          // 没有叶片了 → 组完成
          advanceGroup();
        }
      } else if (current_indices.size() == 1 && last_target_indices_.size() == 2) {
        // 启发式: 2→1 自然过渡, 不管 detectHit 是否检出, 都认为命中了一块
        // (detectHit 可能漏检因为目标索引变了)
        tools::logger()->info("[Buff2026Director] 2→1 过渡, 推断命中");
        first_hit_time_ = now;
        target_pair_[0] = current_indices[0];
        target_pair_[1] = -1;
        current_target_idx_ = 0;
        state_ = ActivationState::SECOND_WINDOW;
      }
      break;
    }

    // ===================== SECOND_WINDOW =====================
    case ActivationState::SECOND_WINDOW: {
      auto first_hit_s = std::chrono::duration<double>(first_hit_time_.time_since_epoch()).count();

      // 超时检查: 1s 窗口
      if (now_s - first_hit_s > SECOND_HIT_TIMEOUT_S) {
        tools::logger()->info("[Buff2026Director] 1s 窗口超时, 组完成 (第二块未击中)");
        advanceGroup();
        break;
      }

      if (current_indices.empty()) {
        // 检测丢失, 但仍在窗口内
        break;
      }

      // 检测命中 (第二块)
      int hit_id = detectHit(current_indices);
      if (hit_id >= 0) {
        tools::logger()->info("[Buff2026Director] 击中 blade {} (第二块)", hit_id);
        advanceGroup();
        break;
      }

      // 如果当前 2 块与 target_pair_ 完全不同 → 组已自动切换
      if (current_indices.size() >= 2) {
        bool same_pair = false;
        for (int id : current_indices) {
          if (id == target_pair_[0] || id == target_pair_[1]) {
            same_pair = true;
            break;
          }
        }
        if (!same_pair) {
          tools::logger()->info("[Buff2026Director] 检测到新组, 跳过剩余窗口");
          advanceGroup();
          break;
        }
      }
      break;
    }

    // ===================== GROUP_TRANSITION =====================
    case ActivationState::GROUP_TRANSITION: {
      // 等待下一组出现 (新亮 2 块) 或超时
      if (current_indices.size() >= 2) {
        // 检查是否是新组
        bool is_new = (target_pair_[0] != current_indices[0] || target_pair_[1] != current_indices[1]) &&
                      (target_pair_[1] != current_indices[0] || target_pair_[0] != current_indices[1]);
        if (is_new) {
          tools::logger()->info("[Buff2026Director] 新组出现, 开始组 {}", completed_groups_ + 1);
          startNewGroup(current_indices, pr);
          group_start_time_ = now;
          state_ = ActivationState::WAITING_FIRST;
        }
      }

      if (now_s - group_start_s > TRANSITION_TIMEOUT_S) {
        // 超时无新组 → 回退到 IDLE (等待重新检测)
        tools::logger()->warn("[Buff2026Director] 过渡超时, 回到 IDLE");
        reset();
      }
      break;
    }

    // ===================== ALL_COMPLETED =====================
    case ActivationState::ALL_COMPLETED: {
      // 保持完成状态, 等待外部重置
      break;
    }

    // ===================== ACTIVATION_FAILED =====================
    case ActivationState::ACTIVATION_FAILED: {
      // 等待 2s 后回到 IDLE
      if (now_s - group_start_s > 2.0) {
        tools::logger()->info("[Buff2026Director] 激活失败后冷却完成, 回到 IDLE");
        reset();
      }
      break;
    }
  }

  // 记录当前帧用于下一帧命中检测
  last_target_indices_ = current_indices;
}

int Buff2026Director::detectHit(const std::vector<int> & current_indices)
{
  // 检测: 上一帧有, 这一帧没有了 → 被击中
  for (int last_id : last_target_indices_) {
    auto it = std::find(current_indices.begin(), current_indices.end(), last_id);
    if (it == current_indices.end()) {
      return last_id;  // 这块消失了
    }
  }
  return -1;
}

void Buff2026Director::startNewGroup(const std::vector<int> & target_indices, const PowerRune & pr)
{
  target_pair_[0] = target_pair_[1] = -1;

  // 按距图像中心距离升序排列, 优先瞄准最近的叶片
  std::vector<int> sorted = target_indices;
  std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
    float da = cv::norm(pr.fanblades[a].center - image_center_);
    float db = cv::norm(pr.fanblades[b].center - image_center_);
    return da < db;
  });

  for (size_t i = 0; i < sorted.size() && i < 2; ++i) {
    target_pair_[i] = sorted[i];
  }
  current_target_idx_ = 0;
  group_start_time_ = std::chrono::steady_clock::now();
}

void Buff2026Director::advanceGroup()
{
  completed_groups_++;
  tools::logger()->info("[Buff2026Director] 组完成! 进度 {}/5", completed_groups_);

  if (completed_groups_ >= 5) {
    state_ = ActivationState::ALL_COMPLETED;
    tools::logger()->info("[Buff2026Director] 全部 5 组激活成功!");
  } else {
    state_ = ActivationState::GROUP_TRANSITION;
    // 保持 target_pair_, current_target_idx_, group_start_time_ 以便检测新组
    current_target_idx_ = -1;
  }
}

bool Buff2026Director::isActivating(const std::optional<PowerRune> & pr) const
{
  if (!pr.has_value()) return false;
  const auto & idx = pr->target_indices_;
  return idx.size() >= 1 && idx.size() <= 2;
}

void Buff2026Director::reset()
{
  state_ = ActivationState::IDLE;
  target_pair_[0] = target_pair_[1] = -1;
  current_target_idx_ = -1;
  completed_groups_ = 0;
  last_target_indices_.clear();
}

}  // namespace auto_buff
