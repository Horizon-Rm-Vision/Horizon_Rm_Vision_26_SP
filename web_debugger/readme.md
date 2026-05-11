# web_debugger - 自瞄系统网页调试器

基于 Flask + Chart.js 的实时远程调试面板，通过共享内存接收 C++ 端视频帧和 JSON 数据，在浏览器中实时显示。

## 文件结构

```
web_debugger/
├── web.py                 # Flask 服务端（Python）
├── shm_debug.hpp          # 共享内存写入工具（C++ header-only）
├── templates/
│   └── index.html         # 网页前端
└── static/
    ├── css/
    │   └── style.css      # 样式
    └── js/
        ├── main.js        # 主逻辑
        ├── json_view.js   # JSON 树形展示
        └── chart_logic.js # Chart.js 实时曲线
```

## 快速开始

### 1. Python 端

```bash
pip install flask setproctitle
cd web_debugger
python web.py
```

启动后访问 `http://<设备IP>:8000`。

### 2. C++ 端

`shm_debug.hpp` 是 header-only 工具，在 `standard.cpp`（或其他主循环文件）中：

```cpp
#include "web_debugger/shm_debug.hpp"

int main() {
    // ... 初始化代码 ...

    web_debugger::ShmDebug shm_debug;  // 创建调试对象

    while (!exiter.exit()) {
        // ... 读取图像、检测、跟踪、瞄准 ...

        shm_debug.write_frame(img);  // 写入帧到共享内存

        // 写入曲线数据（自动累积 + 限长 + 写盘）
        shm_debug.append_cmd("time", t);
        shm_debug.append_cmd("yaw", command.yaw * 57.3);
        shm_debug.append_cmd("pitch", command.pitch * 57.3);
        shm_debug.append_cmd("fire", command.shoot ? 1.0 : 0.0);
    }
}
```

### ShmDebug API

| 方法 | 说明 |
|------|------|
| `write_frame(img, quality=80)` | 编码 cv::Mat 为 JPEG 写入共享内存 + 回退文件 |
| `append_cmd(key, value, max_points=500)` | 追加一个数据点到 cmd_log，自动累积、限长、写盘 |
| `append_serial(key, value, max_points=500)` | 同上，写入 serial_log |
| `append_target(key, value, max_points=500)` | 同上，写入 target_log |
| `write_cmd_log(json_str)` | 直接覆盖写入 cmd_log |
| `write_serial_log(json_str)` | 直接覆盖写入 serial_log |
| `write_target_log(json_str)` | 直接覆盖写入 target_log |
| `reset_cmd_log()` | 清空累积的 cmd 日志 |

## 数据通道

| 通道 | 路径 | 方向 | 说明 |
|------|------|------|------|
| 视频帧 | `/dev/shm/debug_frame` | C++ → Python | 共享内存，前 4 字节为 uint32 JPEG 大小 |
| 视频帧(回退) | `/dev/shm/debug_frame.jpg` | C++ → Python | 文件模式，共享内存不可用时自动回退 |
| 命令日志 | `/dev/shm/cmd_log.json` | C++ → Python | 供 `/data` 接口和图表使用 |
| 串口日志 | `/dev/shm/serial_log.json` | C++ → Python | 供 `/serial_log` 接口使用 |
| 目标日志 | `/dev/shm/target_log.json` | C++ → Python | 供 `/target_log` 接口使用 |

## 网页功能

- **左上**：MJPEG 实时视频流，支持全屏
- **右上**：Target / Serial JSON 树形展示
- **左下**：多曲线总图 + 范围控制
- **右下**：独立子图，可固定 Y 轴范围

## 共享内存格式

```
偏移 0:  uint32_t (little-endian)  JPEG 数据大小 N
偏移 4:  uint8_t[N]                JPEG 字节流
```

## JSON 日志格式

```json
{
  "time": [0.1, 0.2, 0.3, ...],
  "yaw": [1.23, 1.45, 1.67, ...],
  "pitch": [0.12, 0.15, 0.18, ...],
  "fire": [0, 0, 1, ...]
}
```

每个 key 对应一个等长数组，`time` 为 X 轴，其余为 Y 轴数据。

## 依赖

| 端 | 依赖 |
|----|------|
| Python | flask, setproctitle |
| C++ | OpenCV, pthread, librt |
| 前端 | Chart.js (CDN) |
