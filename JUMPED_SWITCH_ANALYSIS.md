# Target中的jumped和switch含义解析

## 总结

| 变量 | 类型 | 作用域 | 含义 |
|------|------|--------|------|
| **`jumped`** | `bool` | 公有成员 | **装甲板是否发生过"跳变"**（目标旋转跳过了某个装甲板编号） |
| **`is_switch_`** | `bool` | 私有成员 | **当前帧装甲板编号是否改变**（从上一帧的装甲板id切换到当前帧） |
| **`switch_count_`** | `int` | 私有成员 | **装甲板切换的累计次数**（统计目标被追踪过程中装甲板ID切换了多少次） |

---

## 详细说明

### 1. `jumped` - 装甲板跳变标志

#### 定义和初始化
```cpp
// target.hpp - 第23行
public:
  bool jumped;

// target.cpp - 第15行（构造函数）
: jumped(false),
```

#### 何时被设置
```cpp
// target.cpp - 第261行（update()函数中）
if (id != 0) jumped = true;
```

**含义**：
- 当检测到的装甲板ID不为0时，说明**目标发生了"跳变"**
- **装甲板跳变** = 目标在旋转过程中，检测到的装甲板编号**不是连续的**
- 例如：依次检测到 id=0 → id=2 → id=3 → id=0，中间跳过了id=1，就说明发生了跳变

#### 使用场景
```cpp
// aimer.cpp - 第150行
AimPoint Aimer::choose_aim_point(const Target & target)
{
    // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
    if (!target.jumped) return {true, armor_xyza_list[0]};
    
    // 如果发生了跳变，则需要根据多个装甲板位置来选择瞄准点
    // ...更复杂的选择逻辑...
}
```

**实际应用**：
- 如果`jumped == false`：说明目标是新检测的或旋转连续，只需瞄准第一个装甲板
- 如果`jumped == true`：说明目标旋转过程中跳过了某些装甲板，需要更复杂的逻辑来选择最佳瞄准点

---

### 2. `is_switch_` - 当前帧装甲板切换标志

#### 定义和初始化
```cpp
// target.hpp - 第56行
private:
  bool is_switch_, is_converged_;

// target.cpp - 第20行（构造函数）
: is_switch_(false),
```

#### 何时被设置
```cpp
// target.cpp - 第264-267行（update()函数中）
if (id != last_id) {
    is_switch_ = true;    // 当前帧的装甲板id与上一帧不同
} else {
    is_switch_ = false;   // 当前帧的装甲板id与上一帧相同
}
```

**含义**：
- **当前帧是否从上一个装甲板切换到了另一个装甲板**
- `is_switch_ == true`：装甲板ID改变了（例如 id从0变成了1）
- `is_switch_ == false`：装甲板ID未改变（保持在同一个id）

#### 实际例子
```
帧1: 检测到 id=0
帧2: 检测到 id=0  →  is_switch_ = false (0 == 0，未切换)
帧3: 检测到 id=1  →  is_switch_ = true  (0 != 1，发生切换)
帧4: 检测到 id=1  →  is_switch_ = false (1 == 1，未切换)
帧5: 检测到 id=2  →  is_switch_ = true  (1 != 2，发生切换)
```

---

### 3. `switch_count_` - 装甲板切换次数计数

#### 定义和初始化
```cpp
// target.hpp - 第53行
private:
  int switch_count_;

// target.cpp - 第22行（构造函数）
: switch_count_(0)
```

#### 何时被更新
```cpp
// target.cpp - 第269行（update()函数中）
if (is_switch_) switch_count_++;
```

**含义**：
- **统计目标被追踪过程中，装甲板ID发生切换的总次数**
- 每当`is_switch_ == true`时，计数器加1
- 反映了目标在旋转过程中发生的**装甲板切换事件数**

#### 实际例子
```
帧1: id=0, is_switch_=false, switch_count_=0
帧2: id=0, is_switch_=false, switch_count_=0
帧3: id=1, is_switch_=true,  switch_count_=1  ← 第1次切换
帧4: id=1, is_switch_=false, switch_count_=1
帧5: id=2, is_switch_=true,  switch_count_=2  ← 第2次切换
帧6: id=2, is_switch_=false, switch_count_=2
帧7: id=3, is_switch_=true,  switch_count_=3  ← 第3次切换
```

---

## 三个变量的关系

```
update()每帧执行：

1. 装甲板匹配，获得当前帧的id
   ↓
2. if (id != 0) jumped = true;
   │  ↓
   │  标记：目标是否发生过跳变（全局状态）
   │
3. if (id != last_id) is_switch_ = true;
   │  ↓
   │  标记：当前帧是否切换了装甲板（帧级状态）
   │
4. if (is_switch_) switch_count_++;
   │  ↓
   │  计数：累计发生切换的次数（统计量）
   │
5. last_id = id;
   └─ 更新上一帧id，供下次比较
```

---

## 代码执行流程图

```cpp
void Target::update(const Armor & armor) {
    // ...装甲板匹配逻辑...
    int id = ...;  // 获得匹配的装甲板编号
    
    // 第1步：标记是否发生过跳变
    if (id != 0) {
        jumped = true;  // ← 全局性标记：目标历史上是否跳过装甲板
    }
    
    // 第2步：检测当前帧是否切换装甲板
    if (id != last_id) {
        is_switch_ = true;   // ← 当前帧：ID改变了
    } else {
        is_switch_ = false;  // ← 当前帧：ID未改变
    }
    
    // 第3步：统计切换次数
    if (is_switch_) {
        switch_count_++;  // ← 累计计数：已切换几次
    }
    
    // 第4步：更新上一帧id
    last_id = id;
    
    // ...进行卡尔曼滤波更新...
}
```

---

## 应用场景

### `jumped`的应用

在Aimer类中选择瞄准点时：
```cpp
AimPoint Aimer::choose_aim_point(const Target & target) {
    if (!target.jumped) {
        // 如果未跳变，说明目标的所有装甲板位置未知
        // 保守策略：只瞄准已知位置的装甲板
        return {true, armor_xyza_list[0]};
    }
    
    // 如果发生过跳变，说明目标已进行过旋转，多个装甲板已被观测
    // 激进策略：基于卡尔曼预测的多个装甲板位置选择最优瞄准点
    // ...选择delta_angle最接近0的装甲板...
}
```

### `switch_count_`的应用

- **质量评估**：装甲板切换次数多 → 可能目标在快速旋转或检测不稳定
- **调试用途**：tracking状态下switch_count_过高 → 检测器或匹配算法可能有问题
- **性能指标**：统计目标追踪的稳定性

---

## 总结表格

| 变量 | 更新时机 | 状态 | 用途 | 示例 |
|------|---------|------|------|------|
| `jumped` | 每次update | 全局状态 | 判断目标是否完成初始旋转 | `false`→`true` 一次 |
| `is_switch_` | 每次update | 帧级状态 | 检测当前帧是否有装甲板切换 | 每帧可能`true`或`false` |
| `switch_count_` | 仅当`is_switch_=true` | 累计计数 | 统计总共切换了多少次 | 0→1→2→3→... |

