# Web UI Receiver

该脚本用于接收 Jetson 端发送的 UI 数据，并使用 OpenCV 重绘调试 UI（不包含图像数据）。同时支持 Plotter 曲线重绘。

## 安装依赖

```bash
pip install -r requirements.txt
```

## 运行

```bash
<<<<<<< HEAD
python receiver.py --host 0.0.0.0 --port 9876
=======
python3 receiver.py --host 0.0.0.0 --port 9878
>>>>>>> origin/main
```

启动后会打开两个窗口：`Web UI Receiver` 与 `Plotter`。

## 配置发送端

在程序使用的配置文件中添加：

```yaml
ui:
  enabled: false
  imshow: false
  web:
    enabled: true
    host: "<接收端电脑IP>" #在接收端使用命令:ip -4 addr 获得接收端ip,示例:"192.168.1.75"
    port: 9878
```

> 说明：`enabled: false` 可关闭本机UI绘制与窗口显示，仅保留数据采集与发送。
