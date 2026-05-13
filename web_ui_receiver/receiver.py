import argparse
import json
import mmap
import os
import socket
import struct
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np


def _color_from_list(color_list: List[Any]) -> Tuple[int, int, int]:
    if not color_list or len(color_list) < 3:
        return 0, 255, 0
    return int(color_list[0]), int(color_list[1]), int(color_list[2])


def _draw_left_panel(img, left_items, layout):
    x = 10
    y = int(layout.get("left_y_offset", 30))
    line_height = int(layout.get("line_height", 25))
    font_scale = float(layout.get("font_scale", 0.6))
    thickness = int(layout.get("thickness", 2))

    for item in left_items:
        text = item.get("text", "")
        color = _color_from_list(item.get("color", [0, 255, 0]))
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
        y += line_height


def _draw_right_panel(img, right_items, layout):
    height, width = img.shape[:2]
    right_x = width - 180
    y = int(layout.get("right_y_offset", 30))
    line_height = int(layout.get("line_height", 25))
    font_scale = float(layout.get("font_scale", 0.6))
    thickness = int(layout.get("thickness", 2))

    cv2.line(img, (width - 200, 20), (width - 200, 400), (0, 255, 255), 2)

    for item in right_items:
        key = item.get("key", "")
        text = item.get("text", "")
        color = _color_from_list(item.get("color", [0, 255, 0]))
        if key == "status_title":
            cv2.putText(
                img,
                text,
                (right_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale + 0.1,
                (255, 255, 0),
                thickness,
            )
            y += line_height + 10
        else:
            cv2.putText(img, text, (right_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
            y += line_height


def _draw_commands(img, draws):
    for draw in draws:
        draw_type = draw.get("type")
        color = _color_from_list(draw.get("color", [0, 255, 0]))

        if draw_type == "point":
            point = draw.get("point", [0, 0])
            radius = int(draw.get("radius", 3))
            cv2.circle(img, (int(point[0]), int(point[1])), radius, color, -1)
        elif draw_type == "points":
            points = draw.get("points", [])
            if points:
                pts = np.array([[int(p[0]), int(p[1])] for p in points], dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                thickness = int(draw.get("thickness", 2))
                cv2.drawContours(img, [pts], -1, color, thickness)
        elif draw_type == "text":
            text = draw.get("text", "")
            position = draw.get("position", [0, 0])
            font_scale = float(draw.get("font_scale", 0.6))
            thickness = int(draw.get("thickness", 2))
            cv2.putText(
                img,
                text,
                (int(position[0]), int(position[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                color,
                thickness,
            )
        elif draw_type == "line":
            p1 = draw.get("p1", [0, 0])
            p2 = draw.get("p2", [0, 0])
            thickness = int(draw.get("thickness", 2))
            cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, thickness)


# ─── 单子图状态（兼容旧格式） ──────────────────────────────

class SubplotState:
    """单个子图的数据状态。"""

    def __init__(self) -> None:
        self.name = ""
        self.names: List[str] = []
        self.colors: List[Tuple[int, int, int]] = []
        self.data: List[Deque[float]] = []

    def configure(self, name: str, names: List[str], colors: List[Any], max_points: int) -> None:
        self.name = name
        if self.names != names:
            self.names = list(names)
            self.colors = [_color_from_list(c) for c in colors]
            self.data = [deque(maxlen=max_points) for _ in self.names]

    def push_values(self, values: List[float]) -> None:
        for idx, value in enumerate(values):
            if idx < len(self.data):
                self.data[idx].append(float(value))


class PlotterState:
    def __init__(self) -> None:
        self.width = 1400
        self.height = 300
        self.max_points = 500
        self.margin_left = 60
        self.margin_right = 150
        self.margin_top = 40
        self.margin_bottom = 40
        self.subplots: List[SubplotState] = []

    def _update_common(self, payload: Dict[str, Any]) -> None:
        self.width = int(payload.get("width", self.width))
        self.height = int(payload.get("height", self.height))
        self.max_points = int(payload.get("max_points", self.max_points))
        self.margin_left = int(payload.get("margin_left", self.margin_left))
        self.margin_right = int(payload.get("margin_right", self.margin_right))
        self.margin_top = int(payload.get("margin_top", self.margin_top))
        self.margin_bottom = int(payload.get("margin_bottom", self.margin_bottom))

    def update_single(self, payload: Dict[str, Any]) -> None:
        """兼容旧格式：单子图。"""
        self._update_common(payload)
        names = payload.get("names", [])
        colors = payload.get("colors", [])
        values = payload.get("values", [])
        if not names or not values:
            return

        if not self.subplots:
            self.subplots.append(SubplotState())
        sp = self.subplots[0]
        sp.configure("Plot", names, colors, self.max_points)
        sp.push_values(values)

    def update_multi(self, payload: Dict[str, Any]) -> None:
        """新格式：多子图。"""
        self._update_common(payload)
        subplots_raw = payload.get("subplots", [])
        if not subplots_raw:
            return

        # 保证子图数量一致
        while len(self.subplots) < len(subplots_raw):
            self.subplots.append(SubplotState())

        for idx, raw in enumerate(subplots_raw):
            sp = self.subplots[idx]
            sp.configure(
                raw.get("name", ""),
                raw.get("names", []),
                raw.get("colors", []),
                self.max_points,
            )
            sp.push_values(raw.get("values", []))


# ─── 单子图绘制（兼容） ──────────────────────────────

def _draw_subplot_grid(
    img: np.ndarray,
    state: PlotterState,
    plot_width: int,
    plot_height: int,
    min_val: float,
    max_val: float,
    y_offset: int = 0,
) -> None:
    top = state.margin_top + y_offset
    bottom = top + plot_height
    left = state.margin_left
    right = left + plot_width

    cv2.rectangle(img, (left, top), (right, bottom), (200, 200, 200), 1)

    num_h_lines = 6
    for i in range(num_h_lines + 1):
        y = top + plot_height * i // num_h_lines
        cv2.line(img, (left, y), (right, y), (220, 220, 220), 1)
        val = max_val - (max_val - min_val) * i / num_h_lines
        cv2.putText(img, f"{val:.2f}", (5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    num_v_lines = 10
    for i in range(num_v_lines + 1):
        x = left + plot_width * i // num_v_lines
        cv2.line(img, (x, top), (x, bottom), (220, 220, 220), 1)


def _draw_subplot_curves(
    img: np.ndarray,
    state: PlotterState,
    subplot: SubplotState,
    plot_width: int,
    plot_height: int,
    min_val: float,
    max_val: float,
    y_offset: int = 0,
) -> None:
    value_range = max_val - min_val
    top = state.margin_top + y_offset

    for idx, curve in enumerate(subplot.data):
        if len(curve) < 2:
            continue
        color = subplot.colors[idx] if idx < len(subplot.colors) else (0, 255, 0)
        for i in range(1, len(curve)):
            x1_ratio = (i - 1) / max(state.max_points, 1)
            x2_ratio = i / max(state.max_points, 1)
            x1 = int(state.margin_left + x1_ratio * plot_width)
            x2 = int(state.margin_left + x2_ratio * plot_width)
            y1 = int(top + plot_height - (curve[i - 1] - min_val) / value_range * plot_height)
            y2 = int(top + plot_height - (curve[i] - min_val) / value_range * plot_height)
            y1 = max(top, min(top + plot_height, y1))
            y2 = max(top, min(top + plot_height, y2))
            cv2.line(img, (x1, y1), (x2, y2), color, 2)


def _draw_subplot_legend(
    img: np.ndarray,
    state: PlotterState,
    subplot: SubplotState,
    y_offset: int = 0,
) -> None:
    legend_x = state.width - state.margin_right + 10
    legend_y = state.margin_top + y_offset + 10
    line_height = 25
    for idx, name in enumerate(subplot.names):
        y = legend_y + idx * line_height
        color = subplot.colors[idx] if idx < len(subplot.colors) else (0, 255, 0)
        cv2.line(img, (legend_x, y + 5), (legend_x + 30, y + 5), color, 3)
        cv2.putText(img, name, (legend_x + 40, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)


def _draw_single_subplot(state: PlotterState, subplot: SubplotState) -> np.ndarray:
    """绘制单个子图的图像（兼容旧版单窗口）。"""
    img = np.full((state.height, state.width, 3), 255, dtype=np.uint8)
    plot_width = state.width - state.margin_left - state.margin_right
    plot_height = state.height - state.margin_top - state.margin_bottom
    if plot_width <= 0 or plot_height <= 0:
        return img

    min_val = None
    max_val = None
    for curve in subplot.data:
        if not curve:
            continue
        local_min = min(curve)
        local_max = max(curve)
        min_val = local_min if min_val is None else min(min_val, local_min)
        max_val = local_max if max_val is None else max(max_val, local_max)

    if min_val is None or max_val is None:
        min_val, max_val = 0.0, 1.0
    if abs(max_val - min_val) < 1e-6:
        max_val = min_val + 1.0

    _draw_subplot_grid(img, state, plot_width, plot_height, min_val, max_val)
    _draw_subplot_curves(img, state, subplot, plot_width, plot_height, min_val, max_val)
    _draw_subplot_legend(img, state, subplot)
    return img


# ─── 多子图绘制 ──────────────────────────────

def _draw_multi_subplots(state: PlotterState) -> np.ndarray:
    """绘制所有子图，竖直堆叠。"""
    num = len(state.subplots)
    total_height = num * state.height
    img = np.full((total_height, state.width, 3), 255, dtype=np.uint8)
    plot_width = state.width - state.margin_left - state.margin_right
    plot_height = state.height - state.margin_top - state.margin_bottom

    if plot_width <= 0 or plot_height <= 0:
        return img

    for s, subplot in enumerate(state.subplots):
        y_off = s * state.height

        if not subplot.data:
            continue

        # 子图标题
        cv2.putText(
            img,
            subplot.name,
            (state.margin_left, y_off + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
        )

        min_val = None
        max_val = None
        for curve in subplot.data:
            if not curve:
                continue
            local_min = min(curve)
            local_max = max(curve)
            if min_val is None or local_min < min_val:
                min_val = local_min
            if max_val is None or local_max > max_val:
                max_val = local_max

        if min_val is None or max_val is None:
            min_val, max_val = 0.0, 1.0
        if abs(max_val - min_val) < 1e-6:
            max_val = min_val + 1.0

        _draw_subplot_grid(img, state, plot_width, plot_height, min_val, max_val, y_off)
        _draw_subplot_curves(img, state, subplot, plot_width, plot_height, min_val, max_val, y_off)
        _draw_subplot_legend(img, state, subplot, y_off)

    return img


# ─── 共享内存图像读取 (参考 wust_vision web.py 的共享内存模式) ──

class ShmImageReader:
    """从 POSIX 共享内存读取 JPEG 图像 (C++ ShmWriter 写入的格式).

    格式: [4-byte uint32 size][JPEG data], 与 wust_vision 的 ShmWriter 一致.
    图像通过 cv2.imdecode 解码为 BGR 的 numpy 数组.
    """

    _SHM_PATH = "/dev/shm/nova_cam_frame"
    _SHM_SIZE = 2 * 1024 * 1024  # 2 MB, 与 C++ kShmMaxSize 保持一致

    def __init__(self, shm_path: Optional[str] = None) -> None:
        self._path = shm_path or self._SHM_PATH
        self._fd: Optional[int] = None
        self._map: Optional[mmap.mmap] = None
        self._connected = False
        self._last_attempt = 0.0
        self._retry_interval = 2.0  # seconds between reconnect attempts

    def _try_connect(self) -> None:
        now = time.time()
        if now - self._last_attempt < self._retry_interval:
            return
        self._last_attempt = now

        # Close any stale handle from a previous connect attempt
        self._disconnect()

        try:
            if not os.path.exists(self._path):
                return
            self._fd = os.open(self._path, os.O_RDONLY)
            # 验证文件大小
            size = os.fstat(self._fd).st_size
            if size < 4:
                os.close(self._fd)
                self._fd = None
                return
            self._map = mmap.mmap(self._fd, self._SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
            self._connected = True
        except (OSError, ValueError):
            self._disconnect()

    def _disconnect(self) -> None:
        self._connected = False
        if self._map is not None:
            try:
                self._map.close()
            except OSError:
                pass
            self._map = None
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def read(self) -> Optional[np.ndarray]:
        """返回 BGR 图像，若不可用则返回 None。"""
        if not self._connected:
            self._try_connect()
        if not self._connected or self._map is None:
            return None

        try:
            self._map.seek(0)
            raw_size = self._map.read(4)
            if len(raw_size) < 4:
                return None
            jpg_size = struct.unpack("I", raw_size)[0]
            if jpg_size <= 0 or jpg_size > self._SHM_SIZE - 4:
                return None
            jpg_bytes = self._map.read(jpg_size)
            if len(jpg_bytes) != jpg_size:
                return None
            if jpg_bytes[0:3] != b"\xff\xd8\xff":  # JPEG magic
                return None

            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except (OSError, ValueError):
            self._disconnect()
            return None

    def close(self) -> None:
        self._disconnect()


# ─── 主循环 ──────────────────────────────

def run_receiver(
    host: str,
    port: int,
    window_name: str,
    shm_path: Optional[str] = None,
    control_port: Optional[int] = None,
    remote_host: Optional[str] = None,
    exposure_min: float = 1.0,
    exposure_max: float = 50.0,
    exposure_step: float = 1.0,
    exposure_init: float = 10.0,
) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_addr: Optional[Tuple[str, int]] = None

    # Trackbar setup for exposure control
    if control_port is None:
        control_port = port + 1

    max_pos = int(round((exposure_max - exposure_min) / max(exposure_step, 1e-6)))
    init_pos = int(round((exposure_init - exposure_min) / max(exposure_step, 1e-6)))
    init_pos = max(0, min(max_pos, init_pos))

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Exposure(ms)", window_name, init_pos, max_pos, lambda _: None)
    last_sent_exposure = None

    # 图像来源：UDP NVIC 包 > SHM（本地回退）> 黑画布
    shm_reader = ShmImageReader(shm_path) if shm_path else ShmImageReader()
    latest_udp_img: Optional[np.ndarray] = None          # from NVIC UDP packets
    ui_frame_width: int = 1280
    ui_frame_height: int = 1024

    last_frame = None
    plotter_state = PlotterState()
    last_plotter_single = None
    last_plotter_multi = None
    last_time = time.time()
    fps = 0.0
    frame_count = 0
    rx_img_fps = 0.0
    rx_img_count = 0
    last_img_time = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(2**20)
        except BlockingIOError:
            data = None
            addr = None

        if data:
            if addr is not None and remote_host is None:
                target_addr = (addr[0], control_port)

            # ── Check for NVIC image packet ──
            if len(data) >= 8 and data[:4] == b'NVIC':
                jpg_size = struct.unpack("I", data[4:8])[0]
                if 0 < jpg_size <= len(data) - 8 and data[8:11] == b'\xff\xd8\xff':
                    arr = np.frombuffer(data[8:8+jpg_size], dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        latest_udp_img = img
                continue  # don't treat as JSON

            # ── JSON payload ──
            try:
                payload: Dict[str, Any] = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                payload = {}

            ptype = payload.get("type", "")

            if ptype == "plotter":
                plotter_state.update_single(payload)
                if plotter_state.subplots:
                    last_plotter_single = _draw_single_subplot(plotter_state, plotter_state.subplots[0])

            elif ptype == "plotter_multi":
                plotter_state.update_multi(payload)
                if plotter_state.subplots:
                    last_plotter_multi = _draw_multi_subplots(plotter_state)

            else:
                # UI 帧
                ui_frame_width = int(payload.get("width", ui_frame_width))
                ui_frame_height = int(payload.get("height", ui_frame_height))
                layout = payload.get("layout", {})
                left_items = payload.get("left", [])
                right_items = payload.get("right", [])
                draws = payload.get("draws", [])

                # 画布：NVIC UDP 图像 > SHM 图像 > 黑画布
                if latest_udp_img is not None:
                    canvas = cv2.resize(latest_udp_img, (ui_frame_width, ui_frame_height))
                    rx_img_count += 1
                else:
                    camera_img = shm_reader.read()
                    if camera_img is not None:
                        canvas = cv2.resize(camera_img, (ui_frame_width, ui_frame_height))
                        rx_img_count += 1
                    else:
                        canvas = np.zeros((ui_frame_height, ui_frame_width, 3), dtype=np.uint8)

                _draw_left_panel(canvas, left_items, layout)
                _draw_right_panel(canvas, right_items, layout)
                _draw_commands(canvas, draws)

                last_frame = canvas

        if last_frame is not None:
            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                fps = frame_count / (now - last_time)
                frame_count = 0
                last_time = now

            if now - last_img_time >= 1.0:
                rx_img_fps = rx_img_count / (now - last_img_time)
                rx_img_count = 0
                last_img_time = now

            cv2.putText(
                last_frame,
                f"RX FPS: {fps:.1f}  IMG: {rx_img_fps:.1f}",
                (10, last_frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            cv2.imshow(window_name, last_frame)

        # ── Exposure control send-back ──
        if remote_host is not None:
            target_addr = (remote_host, control_port)

        if target_addr is not None:
            pos = cv2.getTrackbarPos("Exposure(ms)", window_name)
            exposure_ms = exposure_min + pos * exposure_step
            if last_sent_exposure is None or abs(exposure_ms - last_sent_exposure) >= 1e-6:
                payload = {
                    "type": "control",
                    "name": "exposure_ms",
                    "value": float(exposure_ms),
                }
                try:
                    ctrl_sock.sendto(json.dumps(payload).encode("utf-8"), target_addr)
                    last_sent_exposure = exposure_ms
                except OSError:
                    pass

        if last_plotter_single is not None:
            cv2.imshow("Plotter", last_plotter_single)

        if last_plotter_multi is not None:
            cv2.imshow("Plotter Multi", last_plotter_multi)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    shm_reader.close()
    sock.close()
    ctrl_sock.close()
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Web UI receiver for Horizon_Rm_Vision_26_SP")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--window", default="Web UI Receiver")
    parser.add_argument("--shm-path", default=None,
                        help="Path to shared memory frame (default: /dev/shm/nova_cam_frame)")
    parser.add_argument("--control-port", type=int, default=None,
                        help="UDP port to send exposure control (default: port + 1)")
    parser.add_argument("--remote-host", default=None,
                        help="Override remote host for control packets")
    parser.add_argument("--exposure-min", type=float, default=1.0)
    parser.add_argument("--exposure-max", type=float, default=50.0)
    parser.add_argument("--exposure-step", type=float, default=1.0)
    parser.add_argument("--exposure-init", type=float, default=10.0)
    args = parser.parse_args()

    run_receiver(
        args.host,
        args.port,
        args.window,
        args.shm_path,
        control_port=args.control_port,
        remote_host=args.remote_host,
        exposure_min=args.exposure_min,
        exposure_max=args.exposure_max,
        exposure_step=args.exposure_step,
        exposure_init=args.exposure_init,
    )


if __name__ == "__main__":
    main()
