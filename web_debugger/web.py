#!/usr/bin/env python3
"""
web.py - 自瞄系统网页调试器 (Horizon_Rm_Vision)

功能：
- 从共享内存 /dev/shm/debug_frame 读取 JPEG 帧并通过 MJPEG 流式传输
- 回退到文件模式读取 /dev/shm/debug_frame.jpg
- 读取 JSON 日志文件并通过 /data、/serial_log、/target_log 接口提供数据
- 使用 Chart.js 实时绘制曲线
- 支持全屏视频

共享内存格式（与 C++ 端约定）：
  [uint32 little-endian jpg_size][jpg_bytes...]
"""

from flask import Flask, render_template, Response, jsonify
import time
import json
import socket
import os
import logging
import struct
import mmap
import threading
import subprocess
import atexit
import fcntl

import setproctitle

setproctitle.setproctitle("horizon_vision_web")

app = Flask(__name__)

# ===============================
# 参数设置
# ===============================
USE_SHARED_MEMORY_MODE = True  # True -> 强制共享内存模式, False -> 文件模式
STREAM_FPS = 60
FRAME_INTERVAL = 1.0 / STREAM_FPS

# 通信参数
SHARED_MEMORY_PATH = "/dev/shm/debug_frame"
SHARED_SIZE = 2 * 1024 * 1024  # 2MB
SHARED_FRAME_PATH = "/dev/shm/debug_frame.jpg"

# 日志文件路径
CMD_LOG_PATH = "/dev/shm/cmd_log.json"
SERIAL_LOG_PATH = "/dev/shm/serial_log.json"
TARGET_LOG_PATH = "/dev/shm/target_log.json"

# 初始化通信模式
use_shared_memory = False
mapfile = None
fd = None

# 权限修复锁
permission_lock = threading.Lock()

PORT = 8000


def ensure_shared_memory_permissions():
    """确保共享内存文件存在且权限正确"""
    with permission_lock:
        try:
            if not os.path.exists(SHARED_MEMORY_PATH):
                print(f"创建共享内存文件: {SHARED_MEMORY_PATH}")
                with open(SHARED_MEMORY_PATH, "wb") as f:
                    f.write(b"\0" * SHARED_SIZE)

            current_mode = oct(os.stat(SHARED_MEMORY_PATH).st_mode & 0o777)
            if current_mode != "0o777":
                print(f"修复权限 (当前: {current_mode} -> 目标: 777)")
                result = subprocess.run(
                    ["sudo", "chmod", "777", SHARED_MEMORY_PATH],
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    print("权限修复成功")
                    return True
                else:
                    print(f"权限修复失败: {result.stderr.strip()}")
                    return False
            return True
        except Exception as e:
            print(f"权限修复异常: {str(e)}")
            return False


def init_shared_memory():
    """初始化共享内存连接"""
    global use_shared_memory, mapfile, fd

    if not ensure_shared_memory_permissions():
        print("[WARN] 权限修复失败")
        use_shared_memory = False
        return False

    try:
        fd = os.open(SHARED_MEMORY_PATH, os.O_RDONLY)
        mapfile = mmap.mmap(fd, SHARED_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
        fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        use_shared_memory = True
        print("[INFO] 共享内存初始化成功")
        return True
    except Exception as e:
        print(f"[WARN] 共享内存初始化失败: {e}")
        if mapfile:
            try:
                mapfile.close()
            except Exception:
                pass
            mapfile = None
        if fd:
            try:
                os.close(fd)
            except Exception:
                pass
            fd = None
        use_shared_memory = False
        return False


# ===============================
# 初始化模式
# ===============================
if USE_SHARED_MEMORY_MODE:
    if init_shared_memory():
        print("✅ 使用共享内存模式")
    else:
        print("⚠️ 强制共享内存模式失败，回退到文件模式")
        use_shared_memory = False
else:
    use_shared_memory = False
    print("ℹ️ 使用文件模式")


# ===============================
# 清理函数
# ===============================
@atexit.register
def cleanup():
    if mapfile:
        try:
            mapfile.close()
        except Exception:
            pass
    if fd:
        try:
            os.close(fd)
        except Exception:
            pass


# ===============================
# MJPEG 流生成器
# ===============================
def mjpeg_stream():
    global use_shared_memory, mapfile
    last_fix_attempt = 0

    while True:
        try:
            if use_shared_memory and mapfile:
                try:
                    mapfile.seek(0)
                    size_bytes = mapfile.read(4)
                    if len(size_bytes) < 4:
                        time.sleep(FRAME_INTERVAL)
                        continue
                    jpg_size = struct.unpack("I", size_bytes)[0]
                    if jpg_size <= 0 or jpg_size > SHARED_SIZE - 4:
                        time.sleep(FRAME_INTERVAL)
                        continue
                    jpg_bytes = mapfile.read(jpg_size)
                    if len(jpg_bytes) != jpg_size:
                        time.sleep(FRAME_INTERVAL)
                        continue
                    if jpg_bytes[0:3] != b"\xff\xd8\xff":
                        time.sleep(FRAME_INTERVAL)
                        continue
                except (OSError, ValueError) as e:
                    current_time = time.time()
                    if current_time - last_fix_attempt > 60:
                        print("尝试重新初始化共享内存...")
                        if init_shared_memory():
                            continue
                        last_fix_attempt = current_time
                    use_shared_memory = False
                    continue

            if not use_shared_memory or not mapfile:
                try:
                    with open(SHARED_FRAME_PATH, "rb") as f:
                        jpg_bytes = f.read()
                    if jpg_bytes[0:3] != b"\xff\xd8\xff":
                        time.sleep(FRAME_INTERVAL)
                        continue
                except FileNotFoundError:
                    time.sleep(0.1)
                    continue
                except Exception:
                    time.sleep(0.1)
                    continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpg_bytes
                + b"\r\n"
            )
            time.sleep(FRAME_INTERVAL)
        except Exception:
            time.sleep(0.5)


# ===============================
# 辅助函数
# ===============================
def get_local_ip():
    """获取本机局域网 IP"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def read_json_safe(filepath):
    """安全读取 JSON 文件，失败返回错误字典"""
    try:
        with open(filepath, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"error": f"文件不存在: {filepath}"}
    except json.JSONDecodeError as e:
        return {"error": f"JSON 解析失败: {str(e)}"}
    except Exception as e:
        return {"error": str(e)}


# ===============================
# Flask 路由
# ===============================
@app.route("/")
def index():
    url = f"http://{get_local_ip()}:{PORT}"
    return render_template("index.html", server_url=url)


@app.route("/video")
def video_feed():
    return Response(
        mjpeg_stream(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/data")
def get_data():
    return jsonify(read_json_safe(CMD_LOG_PATH))


@app.route("/serial_log")
def serial_log():
    return jsonify(read_json_safe(SERIAL_LOG_PATH))


@app.route("/target_log")
def target_log():
    return jsonify(read_json_safe(TARGET_LOG_PATH))


# ===============================
# 主函数
# ===============================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    url = f"http://{get_local_ip()}:{PORT}"
    print(f"✅ Horizon 自瞄系统网页调试器已启动: {url}")
    print(f"   - 共享内存模式: {'是' if use_shared_memory else '否'}")
    print(f"   - 视频流地址: {url}/video")
    print(f"   - 数据接口: {url}/data")
    print(f"   - 串口日志: {url}/serial_log")
    print(f"   - 目标日志: {url}/target_log")

    app.run(host="0.0.0.0", port=PORT, threaded=True)
