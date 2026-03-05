# Target_由谁更新传递给Tracker

## 总结
**`target_`是Tracker类的成员变量，由Tracker内部的两个函数直接更新：**
- `set_target()` - 创建新目标
- `update_target()` - 更新现有目标

---

## 详细流程

### 1. Tracker类结构（tracker.hpp）

```cpp
class Tracker {
private:
    Solver & solver_;
    Color enemy_color_;
    int min_detect_count_;
    int max_temp_lost_count_;
    int detect_count_;
    int temp_lost_count_;
    int outpost_max_temp_lost_count_;
    int normal_temp_lost_count_;
    std::string state_, pre_state_;
    Target target_;  // ← 内部成员变量，需要被更新
    std::chrono::steady_clock::time_point last_timestamp_;
    ArmorPriority omni_target_priority_;
    
    void state_machine(bool found);
    bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
    bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};
```

---

### 2. 谁更新Target_？

#### 2.1 情况1：创建新目标 - `set_target()`函数

**触发条件**：状态为`lost`时，尝试从检测的装甲板中建立新目标

```cpp
// tracker.cpp - track()函数中的逻辑
bool found;
if (state_ == "lost") {
    found = set_target(armors, t);  // ← 调用set_target()
}
```

**set_target()函数实现**（tracker.cpp 第227-272行）：

```cpp
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
    if (armors.empty()) return false;
    
    auto & armor = armors.front();  // 取首个装甲板（优先级最高）
    solver_.solve(armor);  // 先利用Solver求解该装甲板的坐标
    
    // 根据兵种选择不同的初始化参数
    if (is_balance) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 2, P0_dig);  // ← 创建新Target对象
    }
    else if (armor.name == ArmorName::outpost) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
        target_ = Target(armor, t, 0.2765, 3, P0_dig);
    }
    else if (armor.name == ArmorName::base) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
        target_ = Target(armor, t, 0.3205, 3, P0_dig);
    }
    else {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 4, P0_dig);
    }
    
    return true;  // 返回true表示成功找到目标
}
```

**target_初始化过程**（target.cpp 构造函数）：
- 初始化卡尔曼滤波器状态向量
- 初始化卡尔曼滤波器协方差矩阵P0
- 设置目标的装甲板名称、类型、优先级等属性

---

#### 2.2 情况2：更新现有目标 - `update_target()`函数

**触发条件**：状态为非`lost`时，使用新检测到的装甲板更新现有目标

```cpp
// tracker.cpp - track()函数中的逻辑
else {
    found = update_target(armors, t);  // ← 调用update_target()
}
```

**update_target()函数实现**（tracker.cpp 第273-295行）：

```cpp
bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
    target_.predict(t);  // 第1步：使用卡尔曼滤波器进行时间更新（预测）
    
    // 第2步：在检测到的装甲板中寻找与当前追踪目标相匹配的装甲板
    int found_count = 0;
    double min_x = 1e10;
    for (const auto & armor : armors) {
        // 检查装甲板名称和类型是否匹配
        if (armor.name != target_.name || armor.type != target_.armor_type) continue;
        found_count++;
        min_x = armor.center.x < min_x ? armor.center.x : min_x;
    }
    
    if (found_count == 0) return false;  // 未找到匹配的装甲板
    
    // 第3步：用检测到的装甲板更新target_的卡尔曼滤波器
    for (auto & armor : armors) {
        if (armor.name != target_.name || armor.type != target_.armor_type)
            continue;
        
        solver_.solve(armor);  // 使用Solver计算装甲板的世界坐标
        
        target_.update(armor);  // ← 更新target_中的卡尔曼滤波器
    }
    
    return true;
}
```

**target_.update()函数做什么**（target.cpp 第193-...行）：
```cpp
void Target::update(const Armor & armor)
{
    // 1. 装甲板匹配：从目标的所有可能装甲板中，找到与检测装甲板最接近的那个
    int id;
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
    
    // 2. 根据装甲板匹配结果更新卡尔曼滤波器状态
    update_ypda(armor, id);  // ← 使用测量装甲板信息更新EKF
}
```

---

### 3. 完整的更新流程图

```
主函数: auto_aim_debug_mpc.cpp
    ↓
camera.read(img)
    ↓
armors = yolo.detect(img)  ← YOLO检测装甲板
    ↓
Tracker.track(armors, t)  ← 调用tracker的track方法
    ↓
    ├─ if (state_ == "lost"):
    │   └─ set_target(armors, t)
    │       ├─ solver.solve(armor)
    │       └─ target_ = Target(armor, t, radius, armor_num, P0)
    │           ↓
    │           [创建新的Target对象，初始化EKF]
    │           ↓
    │           target_内部包含ExtendedKalmanFilter
    │
    └─ else:
        └─ update_target(armors, t)
            ├─ target_.predict(t)  ← 时间更新
            │   └─ ekf_.predict(F, Q, f)  ← EKF预测步
            │
            ├─ [在armors中找匹配的装甲板]
            │
            └─ target_.update(armor)  ← 测量更新
                ├─ [装甲板匹配]
                └─ ekf_.update(...)  ← EKF更新步
                    └─ update_ypda()

    ↓
    [state_machine(found)]
    ↓
    [发散检测]
    ↓
    [返回目标列表或空列表]
    ↓
target_queue.push(target或std::nullopt)
```

---

### 4. Target_更新涉及的关键类

```
Tracker                    Solver                 Target
├─ target_: Target    ├─ solve()      ├─ ekf_: ExtendedKalmanFilter
├─ set_target()       └─ (计算装甲板   ├─ predict()
├─ update_target()        世界坐标)    ├─ update()
│                                     └─ armor_num_, armor_type等
├─ predict(t)                            属性
└─ update(armor)
   ↓
   [调用Target的方法]
```

---

### 5. 数据流向总结

```
YOLO.detect()
    ↓
Armor列表
    ↓
Tracker.track()
    ├─ Solver.solve() → 装甲板世界坐标
    │
    ├─ set_target() → 创建新Target对象
    │   └─ target_成员变量赋值：
    │       target_ = Target(armor, ...)
    │
    └─ update_target() → 更新现有Target对象
        └─ target_成员变量内部状态更新：
            ├─ target_.predict()
            └─ target_.update(armor)
                └─ target_.ekf_.update()

    ↓
Tracker返回std::list<Target>
    ├─ 包含target_的副本（如果状态valid）
    └─ 或返回空列表（如果状态lost）

    ↓
target_queue.push(targets.front()或std::nullopt)
```

---

## 关键点总结

| 项目 | 说明 |
|------|------|
| **target_来源** | Tracker类的私有成员变量 |
| **谁创建target_** | `set_target()`函数从YOLO检测的Armor创建 |
| **谁更新target_** | `update_target()`函数用新检测的Armor更新 |
| **什么时候创建** | 状态从`lost`转为`detecting`时 |
| **什么时候更新** | 处于`detecting`或`tracking`状态，检测到匹配的装甲板时 |
| **update_target步骤** | 1.predict(时间更新)→2.find_match(装甲板匹配)→3.update(测量更新) |
| **最终传递** | track()函数返回`std::list<Target>`，包含target_副本 |
| **卡尔曼作用** | target_内部的EKF在predict和update中完成时间更新和测量更新 |

