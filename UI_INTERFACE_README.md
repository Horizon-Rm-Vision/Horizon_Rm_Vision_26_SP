# 统一UI显示接口

本项目提供了一个统一的UI显示接口，用于在视觉系统中统一管理用户界面显示。该接口支持通过yaml配置启用/禁用UI，并提供简洁的API来添加各种显示元素。

## 主要特性

- **统一管理**：所有UI元素通过UIManager统一管理
- **配置化控制**：可以通过yaml配置文件控制UI的启用/禁用
- **程序差异化**：支持不同程序显示不同的UI元素
- **简洁代码**：减少主文件中的UI相关代码，提高可读性
- **扩展性**：方便添加新的显示元素

## 使用方法

### 1. 初始化UIManager

```cpp
#include "tools/ui_manager.hpp"

// 在main函数中创建UIManager实例
tools::UIManager ui_manager(true); // true表示启用UI
ui_manager.setProgramMode("Your Program Name");
```

### 2. 在主循环中更新FPS

```cpp
while (!exiter.exit()) {
    // UI FPS更新（必须在循环开始处调用）
    ui_manager.updateFPS();
    
    // ... 其他处理代码 ...
    
    // 初始化UI元素（每帧开始时调用）
    ui_manager.initialize(img);
    
    // 添加左侧UI元素
    ui_manager.addLeftText("detect", fmt::format("Detect: {}", has_target ? "YES" : "NO"), 
                          has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    
    // 添加右侧UI元素
    ui_manager.addRightText("fps", fmt::format("FPS: {:.1f}", fps));
    
    // 添加绘制命令
    ui_manager.addDrawPoints(armor_points, cv::Scalar(0, 255, 0));
    ui_manager.addDrawText("Custom Text", cv::Point(100, 100), cv::Scalar(255, 255, 0));
    
    // 渲染所有UI元素到图像
    ui_manager.render(img);
    
    // 显示图像
    cv::imshow("window", img);
}
```

### 3. API参考

#### UIManager类方法

- `UIManager(bool enabled)`: 构造函数，参数控制是否启用UI
- `void setProgramMode(const std::string& mode)`: 设置程序模式（显示在UI中）
- `void updateFPS()`: 更新FPS计算（每帧调用一次）
- `void initialize(cv::Mat& img)`: 初始化UI，为新帧做准备
- `void render(cv::Mat& img)`: 将所有UI元素渲染到图像上

#### 添加UI元素的方法

- `void addLeftText(const std::string& key, const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0))`: 添加左侧文本
- `void addRightText(const std::string& key, const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0))`: 添加右侧文本
- `void addDrawPoints(const std::vector<cv::Point2f>& points, cv::Scalar color = cv::Scalar(0, 255, 0), int size = 5)`: 添加点绘制
- `void addDrawText(const std::string& text, const cv::Point& position, cv::Scalar color = cv::Scalar(0, 255, 0))`: 添加文本绘制
- `void addCustomDraw(std::function<void(cv::Mat&)> draw_func)`: 添加自定义绘制函数

## 配置方法

### YAML配置

在配置文件中添加UI配置：

```yaml
#####-----UI显示配置-----#####
ui:
  enabled: true  # 是否启用UI显示
```

### 程序特定配置

不同程序可以通过setProgramMode设置不同的模式名称，并在initialize中添加程序特定的UI元素。

## 已集成程序

- `auto_aim_debug_mpc.cpp`: 自动瞄准MPC调试程序
- `standard_serial.cpp`: 标准串口程序

## 扩展UI元素

要添加新的UI显示元素：

1. 在程序的UI初始化部分调用相应的add方法
2. 如果需要新的绘制类型，可以扩展UIDrawCommand和相关方法

## 注意事项

- 必须在每帧开始时调用`updateFPS()`和`initialize()`
- 所有UI元素在`render()`调用前添加
- UI元素的key用于标识，不同key的元素会显示在不同行
- 左侧UI显示在图像左侧，右侧UI显示在图像右侧