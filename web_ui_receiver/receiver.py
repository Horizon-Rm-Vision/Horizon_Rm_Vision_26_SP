import argparse
import json
import socket
import time
from collections import deque
from typing import Any, Deque, Dict, List, Tuple

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


class PlotterState:
    def __init__(self) -> None:
        self.width = 1400
        self.height = 300
        self.max_points = 500
        self.margin_left = 60
        self.margin_right = 150
        self.margin_top = 40
        self.margin_bottom = 40
        self.names: List[str] = []
        self.colors: List[Tuple[int, int, int]] = []
        self.data: List[Deque[float]] = []

    def update(self, payload: Dict[str, Any]) -> None:
        self.width = int(payload.get("width", self.width))
        self.height = int(payload.get("height", self.height))
        self.max_points = int(payload.get("max_points", self.max_points))
        self.margin_left = int(payload.get("margin_left", self.margin_left))
        self.margin_right = int(payload.get("margin_right", self.margin_right))
        self.margin_top = int(payload.get("margin_top", self.margin_top))
        self.margin_bottom = int(payload.get("margin_bottom", self.margin_bottom))

        names = payload.get("names", [])
        colors = payload.get("colors", [])
        values = payload.get("values", [])

        if not names or not values:
            return

        if self.names != names:
            self.names = list(names)
            self.colors = [_color_from_list(c) for c in colors]
            self.data = [deque(maxlen=self.max_points) for _ in self.names]

        for idx, value in enumerate(values):
            if idx >= len(self.data):
                break
            self.data[idx].append(float(value))


def _draw_plotter(state: PlotterState) -> np.ndarray:
    width = state.width
    height = state.height
    img = np.full((height, width, 3), 255, dtype=np.uint8)

    plot_width = width - state.margin_left - state.margin_right
    plot_height = height - state.margin_top - state.margin_bottom

    if plot_width <= 0 or plot_height <= 0:
        return img

    min_val = None
    max_val = None
    for curve in state.data:
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

    _draw_plotter_grid(img, state, plot_width, plot_height, min_val, max_val)
    _draw_plotter_curves(img, state, plot_width, plot_height, min_val, max_val)
    _draw_plotter_legend(img, state)
    return img


def _draw_plotter_grid(
    img: np.ndarray,
    state: PlotterState,
    plot_width: int,
    plot_height: int,
    min_val: float,
    max_val: float,
) -> None:
    cv2.rectangle(
        img,
        (state.margin_left, state.margin_top),
        (state.margin_left + plot_width, state.margin_top + plot_height),
        (200, 200, 200),
        1,
    )

    num_h_lines = 6
    for i in range(num_h_lines + 1):
        y = state.margin_top + plot_height * i // num_h_lines
        cv2.line(img, (state.margin_left, y), (state.margin_left + plot_width, y), (220, 220, 220), 1)
        val = max_val - (max_val - min_val) * i / num_h_lines
        cv2.putText(
            img,
            f"{val:.2f}",
            (5, y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (80, 80, 80),
            1,
        )

    num_v_lines = 10
    for i in range(num_v_lines + 1):
        x = state.margin_left + plot_width * i // num_v_lines
        cv2.line(img, (x, state.margin_top), (x, state.margin_top + plot_height), (220, 220, 220), 1)


def _draw_plotter_curves(
    img: np.ndarray,
    state: PlotterState,
    plot_width: int,
    plot_height: int,
    min_val: float,
    max_val: float,
) -> None:
    value_range = max_val - min_val
    for idx, curve in enumerate(state.data):
        if len(curve) < 2:
            continue
        color = state.colors[idx] if idx < len(state.colors) else (0, 255, 0)
        for i in range(1, len(curve)):
            x1_ratio = (i - 1) / max(state.max_points, 1)
            x2_ratio = i / max(state.max_points, 1)
            x1 = int(state.margin_left + x1_ratio * plot_width)
            x2 = int(state.margin_left + x2_ratio * plot_width)
            y1 = int(state.margin_top + plot_height - (curve[i - 1] - min_val) / value_range * plot_height)
            y2 = int(state.margin_top + plot_height - (curve[i] - min_val) / value_range * plot_height)
            y1 = max(state.margin_top, min(state.margin_top + plot_height, y1))
            y2 = max(state.margin_top, min(state.margin_top + plot_height, y2))
            cv2.line(img, (x1, y1), (x2, y2), color, 2)


def _draw_plotter_legend(img: np.ndarray, state: PlotterState) -> None:
    legend_x = state.width - state.margin_right + 10
    legend_y = state.margin_top + 10
    line_height = 25
    for idx, name in enumerate(state.names):
        y = legend_y + idx * line_height
        color = state.colors[idx] if idx < len(state.colors) else (0, 255, 0)
        cv2.line(img, (legend_x, y + 5), (legend_x + 30, y + 5), color, 3)
        cv2.putText(
            img,
            name,
            (legend_x + 40, y + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
        )


def run_receiver(host: str, port: int, window_name: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)

    last_frame = None
    plotter_state = PlotterState()
    last_plotter = None
    last_time = time.time()
    fps = 0.0
    frame_count = 0

    while True:
        try:
            data, _ = sock.recvfrom(2**20)
        except BlockingIOError:
            data = None

        if data:
            try:
                payload: Dict[str, Any] = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                payload = {}

            if payload.get("type") == "plotter":
                plotter_state.update(payload)
                last_plotter = _draw_plotter(plotter_state)
            else:
                width = int(payload.get("width", 1280))
                height = int(payload.get("height", 1024))
                layout = payload.get("layout", {})
                left_items = payload.get("left", [])
                right_items = payload.get("right", [])
                draws = payload.get("draws", [])

                canvas = np.zeros((height, width, 3), dtype=np.uint8)

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

            cv2.putText(
                last_frame,
                f"RX FPS: {fps:.1f}",
                (10, last_frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            cv2.imshow(window_name, last_frame)

        if last_plotter is not None:
            cv2.imshow("Plotter", last_plotter)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    sock.close()
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Web UI receiver for Horizon_Rm_Vision_26_SP")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--window", default="Web UI Receiver")
    args = parser.parse_args()

    run_receiver(args.host, args.port, args.window)


if __name__ == "__main__":
    main()
