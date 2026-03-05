# Target参数在YOLO检测不到目标时的处理分析

## 执行流程总结

当YOLO检测不到目标时，**target参数为`std::nullopt`（空值），而不是用卡尔曼数据填入**。

## 详细分析

### 1. 主函数流程（auto_aim_debug_mpc.cpp）

```cpp
// 第118-130行：主循环中的目标获取
camera.read(img, t);
auto q = gimbal.q(t);

solver.set_R_gimbal2world(q);
auto armors = yolo.detect(img);           // YOLO检测装甲板
auto targets = tracker.track(armors, t);  // 跟踪器处理

if (!targets.empty())
    target_queue.push(targets.front());
else
    target_queue.push(std::nullopt);      // ← 检测不到目标时为std::nullopt
```

**关键点**：当YOLO检测不到装甲板，或跟踪器处理后没有有效目标时，直接推送`std::nullopt`到队列。

---

### 2. 跟踪器处理流程（tracker.cpp）

#### 2.1 track()函数的返回逻辑

```cpp
// 第60-95行：track函数的核心逻辑

bool found;
if (state_ == "lost") {
    found = set_target(armors, t);  // 尝试从检测的装甲板中建立新目标
} else {
    found = update_target(armors, t);  // 尝试更新现有目标
}

state_machine(found);  // 状态机切换

// 发散检测
if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};  // 返回空列表
}

if (state_ == "lost") return {};  // 返回空列表

std::list<Target> targets = {target_};
return targets;
```

**关键逻辑**：
- 当`state_ == "lost"`时，返回空列表`{}`
- 返回空列表表示没有有效目标

---

### 3. Planner处理空Target的方式（planner.cpp）

#### 3.1 接收std::optional<Target>

```cpp
// 第98-110行：Planner::plan()函数

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
    if (!target.has_value()) return {false};  // ← 直接返回无效Plan
    
    double delay_time = 
        std::abs(target->ekf_x()[7]) > decision_speed_ 
        ? high_speed_delay_time_ 
        : low_speed_delay_time_;
    
    auto future = std::chrono::steady_clock::now() 
        + std::chrono::microseconds(int(delay_time * 1e6));
    
    target->predict(future);
    
    return plan(*target, bullet_speed);
}
```

**处理方式**：
- 参数为`std::nullopt`时，**直接返回`{false}`**
- **不使用卡尔曼滤波数据**
- **不进行任何预测计算**

---

### 4. 计划线程的接收（auto_aim_debug_mpc.cpp）

```cpp
// 第64-81行：计划线程

auto plan_thread = std::thread([&]() {
    // ...
    while (!quit) {
        auto target = target_queue.front();  // ← 获取std::optional<Target>
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);  // ← 传入std::optional
        
        gimbal.send(
            plan.control, plan.fire, plan.yaw, plan.yaw_vel, 
            plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc);
        
        // ...
        std::this_thread::sleep_for(10ms);
    }
});
```

---

## 跟踪器状态机与目标管理

### 状态流转图

```
lost → [set_target(armors)] → detecting
         ↑                        ↓
         |         [有装甲检测]   ↓
         |         [min_detect_count次后]
         |                    tracking
         |                        ↓
         |     [无装甲检测]   temp_lost
         ←─────[继续预测]─────→
              [超过max_temp_lost_count帧]
```

### 关键判断条件

| 状态 | 检测结果 | 下一状态 | 返回值 |
|------|---------|---------|--------|
| lost | 无 | lost | `{}` |
| lost | 有 | detecting | `{}` |
| detecting | 无 | lost | `{}` |
| detecting | 有(少于min_count) | detecting | `{}` |
| detecting | 有(≥min_count) | tracking | `{target}` |
| tracking | 有 | tracking | `{target}` |
| tracking | 无 | temp_lost | `{target}`** |
| temp_lost | 有 | tracking | `{target}` |
| temp_lost | 无(超时) | lost | `{}` |

\** 在temp_lost状态下，使用**卡尔曼预测的目标位置**

---

## YOLO检测不到目标时的处理总结

### 立即返回空值的情况

1. **YOLO检测器返回空列表**：`armors.empty() == true`
2. **跟踪器在lost状态**：返回`{}`
3. **跟踪器在detecting状态但检测次数不足**：返回`{}`
4. **跟踪器因发散而进入lost状态**：返回`{}`
5. **跟踪器因temp_lost超时而进入lost状态**：返回`{}`

### 使用卡尔曼预测的情况

**仅在以下情况下使用卡尔曼预测**：
- 状态为`temp_lost`
- 检测已中断，但未超过`max_temp_lost_count`帧
- 此时tracker返回缓存的`target_`对象，其包含卡尔曼滤波器的预测状态 // 

---

## 代码流程图

```
YOLO.detect(img)
    ↓
armors = [装甲板列表] 或 []
    ↓
tracker.track(armors, t)
    ├─ armors.empty()?
    │  ├─ YES: lost状态 → return {}
    │  └─ NO: update_target/set_target
    │         ↓
    │         found = true/false
    │         ↓
    │         state_machine(found)
    │         ↓
    │         temp_lost超时?
    │         ├─ YES: state="lost" → return {}
    │         ├─ NO: state="temp_lost" → return {target_}** (卡尔曼预测)
    │         └─ state="tracking" → return {target_}
    │
    └─ return targets

    ↓
if (targets.empty())
    target_queue.push(std::nullopt)  ← 无目标
else
    target_queue.push(targets.front())  ← 有目标

    ↓
planner.plan(target_queue.front(), bullet_speed)
    ├─ target.has_value()?
    │  ├─ NO: return Plan{false}  ← 不发射，保持当前姿态
    │  └─ YES: 进行完整的MPC规划和预测
    │
    └─ return plan
```

---

## 结论

**参数target在YOLO检测不到目标时为`std::nullopt`（空值），而不是用卡尔曼数据填入**。

卡尔曼滤波数据仅在以下场景使用：
- ✅ 跟踪器处于`temp_lost`状态（短期目标丢失）
- ✅ 需要进行短期预测和外推

直接返回空值的场景：
- ✅ 完全丢失目标（`lost`状态）
- ✅ YOLO未检测到装甲板
- ✅ 检测次数不足（`detecting`状态）
- ✅ 目标发散（`diverged`状态）
- ✅ 短期丢失超时（`temp_lost`超时）

