#!/usr/bin/env python3
"""
一体式机器人视觉调试上位机 (全融合单文件 极致调参版)
包含了黑白反差渲染、详细的中文调参说明，以及抓取区速度、三圆环参数实时滑条调试。
新增：Tab 4 三圆环工位识别参数实时可视化调试。

"""

from __future__ import annotations

import sys
import types
import cv2
import numpy as np

# ==============================================================================
# 动态依赖修复：拦截底层的导包请求
# ==============================================================================
def _mask_from_hsv(hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.asarray(hsv_range["lower"], dtype=np.uint8), np.asarray(hsv_range["upper"], dtype=np.uint8)))
    return mask

def make_all_color_masks(frame: np.ndarray, config: dict) -> dict[int, np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {int(item["color_id"]): _mask_from_hsv(hsv, item["ranges"]) for item in config["colors"]}

def _find_color(document: dict, color: int | str) -> dict:
    return [item for item in document["colors"] if int(item["color_id"]) == int(color)][0]

def make_color_mask(frame: np.ndarray, color: int | str, config: dict) -> np.ndarray:
    item = _find_color(config, color)
    return _mask_from_hsv(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), item["ranges"])

mock_color_module = types.ModuleType('颜色标定')
mock_color_module._mask_from_hsv = _mask_from_hsv
mock_color_module.make_all_color_masks = make_all_color_masks
mock_color_module._find_color = _find_color
mock_color_module.make_color_mask = make_color_mask
sys.modules['颜色标定'] = mock_color_module

# ==============================================================================
# 算法核心库导入
# ==============================================================================
import argparse
import copy
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# 引入核心检测类
from material_pickup_detector import (
    build_default_material_pickup_config,
    load_material_pickup_config,
    save_material_pickup_config,
    MaterialPickupError,
    MaterialPickupDetector,
    draw_material_pickup_detection
)
from turntable_detector import (
    build_default_turntable_detector_document,
    load_turntable_detector_document,
    save_turntable_detector_config,
    turntable_config_from_document,
    TurntableDetector,
    TurntableDebugResult,
    TurntableDetectionError,
    DebugCircleCandidate,
    draw_turntable_detection
)
# 引入新增的三圆环检测类
from yuanhuan import (
    load_ring_station_config,
    RingStationDetector,
    draw_ring_station_detection
)

# ==============================================================================
# 全局共享工具与常量
# ==============================================================================
class DebuggerCalibrationError(RuntimeError): pass

def _read_matrix(storage: cv2.FileStorage, name: str):
    matrix = storage.getNode(name).mat()
    if matrix is None or matrix.size == 0:
        raise DebuggerCalibrationError(f"标定文件缺少矩阵：{name}")
    return matrix

def load_camera_calibration_shared(path: Path):
    resolved = path.expanduser().resolve()
    try:
        yaml_text = resolved.read_text(encoding="utf-8")
    except Exception as e:
        raise DebuggerCalibrationError(f"无法读取标定文件文本：{e}")

    storage = cv2.FileStorage(yaml_text, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
    if not storage.isOpened():
        raise DebuggerCalibrationError(f"无法打开摄像头标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = _read_matrix(storage, "camera_matrix")
        distortion = _read_matrix(storage, "distortion_coefficients")
        new_camera_matrix = _read_matrix(storage, "new_camera_matrix")
    finally:
        storage.release()
    if (width, height) != (1280, 720):
        raise DebuggerCalibrationError(f"标定要求1280x720内参，当前为{width}x{height}")
    return (width, height), camera_matrix, distortion, new_camera_matrix

def open_camera_shared(device, fps: float):
    if device == "/dev/video0": device = 0
    elif isinstance(device, str) and device.isdigit(): device = int(device)

    camera = cv2.VideoCapture(device)
    if not camera.isOpened():
        camera.release()
        raise DebuggerCalibrationError(f"无法打开摄像头：{device} (请确保没有被占用)")
        
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, float(fps))
    
    for _ in range(10):
        ok, frame = camera.read()
        if ok and frame is not None:
            if frame.shape[:2] != (720, 1280):
                camera.release()
                raise DebuggerCalibrationError("摄像头输出分辨率非1280x720")
            return camera
    camera.release()
    raise DebuggerCalibrationError("摄像头已打开但无法读取画面")

# ==============================================================================
# UI 参数滑块配置
# ==============================================================================
@dataclass(frozen=True)
class ParameterSpec:
    page: str; group: str; key: str; label: str; minimum: float; maximum: float; step: float; integer: bool = False
    def format_value(self, value) -> str:
        if self.integer: return str(int(value))
        if self.step >= 1.0: return f"{float(value):.0f}"
        if self.step >= 0.1: return f"{float(value):.1f}"
        return f"{float(value):.2f}"

PICKUP_SPECS = (
    ParameterSpec("pickup", "motion", "moving_speed_px_s", "转动判定速度 px/s", 10.0, 100.0, 1.0),
    ParameterSpec("pickup", "motion", "near_stop_speed_px_s", "停稳低速阈值 px/s", 5.0, 80.0, 1.0),
    ParameterSpec("pickup", "motion", "required_moving_frames", "转动确认要求帧数", 1, 10, 1, True),
    ParameterSpec("pickup", "motion", "required_low_speed_frames", "低速确认要求帧数", 1, 10, 1, True),
    ParameterSpec("pickup", "detection", "minimum_area_px", "物料过滤最小面积", 100, 2000, 10),
    ParameterSpec("pickup", "detection", "maximum_area_px", "物料过滤最大面积", 2000, 40000, 100),
    ParameterSpec("pickup", "detection", "maximum_match_distance_px", "测速最大允许跳变", 20.0, 200.0, 1.0),
)

TURNTABLE_SPECS = (
    ParameterSpec("common", "coarse_search", "image_scale", "处理缩放", 0.20, 1.00, 0.01),
    ParameterSpec("common", "coarse_search", "minimum_radius_px", "最小半径 px", 50, 700, 1, True),
    ParameterSpec("common", "coarse_search", "maximum_radius_px", "最大半径 px", 100, 900, 1, True),
    ParameterSpec("common", "coarse_search", "canny_threshold", "边缘阈值", 1, 255, 1, True),
    ParameterSpec("common", "coarse_search", "accumulator_threshold", "Hough 严格度", 5, 100, 1, True),
    ParameterSpec("common", "refinement", "radial_search_half_width_px", "径向搜索宽度", 10, 150, 1, True),
    ParameterSpec("common", "refinement", "minimum_visible_arc_deg", "最小可见圆弧 °", 10, 360, 1, True),
    ParameterSpec("common", "refinement", "maximum_fit_rms_px", "最大拟合 RMS", 0.5, 10.0, 0.1),
    ParameterSpec("advanced", "coarse_search", "hough_dp", "Hough dp", 1.0, 3.0, 0.1),
    ParameterSpec("advanced", "coarse_search", "minimum_center_distance_px", "候选圆心间距", 20, 500, 1, True),
    ParameterSpec("advanced", "coarse_search", "maximum_candidates", "最多候选数量", 1, 20, 1, True),
    ParameterSpec("advanced", "refinement", "angular_step_deg", "采样角度步长 °", 0.5, 10.0, 0.5),
    ParameterSpec("advanced", "refinement", "minimum_edge_strength", "最小边缘强度", 0.5, 20.0, 0.1),
    ParameterSpec("advanced", "refinement", "saturated_pixel_threshold", "彩色像素过滤", 0, 255, 1, True),
    ParameterSpec("advanced", "refinement", "local_recheck_confidence_below", "局部复核置信度", 0.0, 1.0, 0.01),
    ParameterSpec("advanced", "refinement", "local_recheck_rms_above_px", "局部复核 RMS", 0.1, 10.0, 0.1),
    ParameterSpec("advanced", "refinement", "local_recheck_interval_frames", "复核间隔帧", 1, 30, 1, True),
    ParameterSpec("advanced", "refinement", "reacquisition_jump_px", "重定位跳变 px", 1, 200, 1, True),
    ParameterSpec("advanced", "stability", "window_frames", "稳定窗口帧数", 2, 30, 1, True),
    ParameterSpec("advanced", "stability", "maximum_center_jitter_px", "最大圆心抖动", 0.5, 30.0, 0.5),
    ParameterSpec("advanced", "stability", "maximum_radius_jitter_px", "最大半径抖动", 0.5, 30.0, 0.5),
)

RING_STATION_SPECS = (
    ParameterSpec("ring", "detection", "canny_low", "边缘提取低阈值", 10, 120, 1, True),
    ParameterSpec("ring", "detection", "canny_high", "边缘提取高阈值", 50, 255, 1, True),
    ParameterSpec("ring", "detection", "min_contour_points", "最小轮廓点数", 10, 150, 1, True),
    ParameterSpec("ring", "detection", "min_minor_axis_px", "圆环短轴最小px", 10.0, 100.0, 1.0),
    ParameterSpec("ring", "detection", "max_major_axis_px", "圆环长轴最大px", 150.0, 500.0, 5.0),
    ParameterSpec("ring", "detection", "min_ellipse_aspect", "椭圆度下限(越圆越大)", 0.05, 0.95, 0.05),
    ParameterSpec("ring", "detection", "max_collinearity_error_ratio", "共线误差容忍", 0.01, 0.20, 0.01),
    ParameterSpec("ring", "reference", "minimum_confidence", "质量置信度门槛", 0.1, 1.0, 0.05),
)

class ParameterPanel:
    """通用的参数滑条面板管理器"""
    def __init__(self, document: dict, all_specs: tuple[ParameterSpec, ...]):
        self.document = copy.deepcopy(document)
        self.all_specs = all_specs
        self.page = all_specs[0].page
        self.dragging_index, self.selected_index = None, None
        self.dirty, self.slider_changed = False, False
        self.ROW_START, self.ROW_HEIGHT, self.TRACK_LEFT, self.TRACK_RIGHT = 126, 40, 18, 420 - 18

    @property
    def specs(self) -> tuple[ParameterSpec, ...]: return tuple(s for s in self.all_specs if s.page == self.page)

    def replace_document(self, document: dict):
        self.document = copy.deepcopy(document)
        self.dragging_index = self.selected_index = None
        self.dirty = True

    def toggle_page(self):
        pages = list(dict.fromkeys(s.page for s in self.all_specs))
        if len(pages) > 1:
            idx = pages.index(self.page)
            self.page = pages[(idx + 1) % len(pages)]
            self.dragging_index = self.selected_index = None

    def value(self, spec: ParameterSpec): return self.document[spec.group][spec.key]

    def set_value(self, spec: ParameterSpec, value: float):
        clipped = min(spec.maximum, max(spec.minimum, float(value)))
        quantized = spec.minimum + round((clipped - spec.minimum) / spec.step) * spec.step
        self.document[spec.group][spec.key] = int(round(quantized)) if spec.integer else float(quantized)
        
        # 联动保护
        if "coarse_search" in self.document:
            c, r = self.document["coarse_search"], self.document.get("refinement", {})
            if spec.key == "minimum_radius_px" and c["minimum_radius_px"] >= c["maximum_radius_px"]: c["maximum_radius_px"] = float(min(900, c["minimum_radius_px"] + 1))
            elif spec.key == "maximum_radius_px" and c["maximum_radius_px"] <= c["minimum_radius_px"]: c["minimum_radius_px"] = float(max(50, c["maximum_radius_px"] - 1))
            if spec.key == "maximum_fit_rms_px" and r.get("local_recheck_rms_above_px", 0) > r.get("maximum_fit_rms_px", 0): r["local_recheck_rms_above_px"] = float(r["maximum_fit_rms_px"])
        if "motion" in self.document:
            m, d = self.document["motion"], self.document["detection"]
            if spec.key == "moving_speed_px_s" and m["moving_speed_px_s"] <= m["near_stop_speed_px_s"]: m["near_stop_speed_px_s"] = float(max(0, m["moving_speed_px_s"] - 1.0))
            elif spec.key == "near_stop_speed_px_s" and m["near_stop_speed_px_s"] >= m["moving_speed_px_s"]: m["moving_speed_px_s"] = float(min(150, m["near_stop_speed_px_s"] + 1.0))
            if spec.key == "minimum_area_px" and d["minimum_area_px"] >= d["maximum_area_px"]: d["maximum_area_px"] = float(d["minimum_area_px"] + 10)
        
        self.dirty = self.slider_changed = True

    def mouse_callback(self, event, x, y, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONUP: self.dragging_index = None; return
        if x < 1280: return
        px = x - 1280
        ri = int((y - self.ROW_START) // self.ROW_HEIGHT)
        specs = self.specs
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= ri < len(specs):
            self.dragging_index = self.selected_index = ri
            self.set_value(specs[ri], specs[ri].minimum + ((px - self.TRACK_LEFT) / (self.TRACK_RIGHT - self.TRACK_LEFT)) * (specs[ri].maximum - specs[ri].minimum))
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging_index is not None:
            self.set_value(specs[self.dragging_index], specs[self.dragging_index].minimum + ((px - self.TRACK_LEFT) / (self.TRACK_RIGHT - self.TRACK_LEFT)) * (specs[self.dragging_index].maximum - specs[self.dragging_index].minimum))

# ==============================================================================
# 其它各功能模块类定义省略...（保持原样）
# ==============================================================================
REGION_ORDER = ("workspace",)
REGION_COLORS = {"workspace": (0, 220, 255)}

class RegionSelector:
    def __init__(self, panel: ParameterPanel):
        self.panel = panel
        self.rectangles: list[tuple[str, dict]] = []
        if bool(self.panel.document.get("calibrated")):
            if "workspace" in self.panel.document.get("regions", {}):
                self.rectangles.append(("workspace", copy.deepcopy(self.panel.document["regions"]["workspace"])))
            else:
                for name in REGION_ORDER:
                    if name in self.panel.document.get("regions", {}):
                        self.rectangles.append((name, copy.deepcopy(self.panel.document["regions"][name])))
        self.latest_frame, self.frozen_frame, self.drag_start, self.drag_end = None, None, None, None
        self.message = "准备就绪，请框选目标区域..."

    @property
    def complete(self) -> bool: return len(self.rectangles) >= len(REGION_ORDER)

    @property
    def current_name(self) -> str | None: return None if self.complete else REGION_ORDER[len(self.rectangles)]

    @staticmethod
    def _rectangle(first, second):
        x1, y1, x2, y2 = *first, *second
        return {"x": int(min(x1, x2)), "y": int(min(y1, y2)), "width": int(abs(x2 - x1)), "height": int(abs(y2 - y1))}

    def mouse_callback(self, event, x_value, y_value, _flags, _parameter):
        point = (max(0, min(1279, int(x_value))), max(0, min(719, int(y_value))))
        if event == cv2.EVENT_LBUTTONDOWN and not self.complete:
            self.drag_start = self.drag_end = point
            if self.latest_frame is not None: self.frozen_frame = self.latest_frame.copy()
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_end = point
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_end = point
            rectangle = self._rectangle(self.drag_start, self.drag_end)
            if rectangle["width"] >= 10 and rectangle["height"] >= 10:
                self.rectangles.append((self.current_name, rectangle))
                self.message = f"【记录】已框选 {self.rectangles[-1][0]}"
            else:
                self.message = "【提示】区域太小，请重新拖拽"
            self.drag_start = self.drag_end = self.frozen_frame = None

    def undo(self):
        if self.drag_start is not None: self.drag_start = self.drag_end = self.frozen_frame = None
        elif self.rectangles:
            name, _ = self.rectangles.pop()
            self.message = f"【撤销】移除了 {name}"

    def reset(self):
        self.rectangles.clear()
        self.drag_start = self.drag_end = self.frozen_frame = None
        self.message = "【重置】已清空框选"

    def save(self, path: Path):
        if not self.complete:
            self.message = "【警告】请先画好区域再保存"
            return False
        self.panel.document["calibrated"] = True
        self.panel.document["regions"] = {name: copy.deepcopy(rect) for name, rect in self.rectangles}
        try:
            save_material_pickup_config(path, self.panel.document)
            self.message = f"【保存成功】区域与参数已写入"
            return True
        except MaterialPickupError as error:
            self.message = f"【失败】 {error}"
            return False

    def draw(self, live_frame: np.ndarray) -> np.ndarray:
        self.latest_frame = live_frame
        base = self.frozen_frame.copy() if self.frozen_frame is not None else live_frame.copy()
        for name, rectangle in self.rectangles:
            first = (rectangle["x"], rectangle["y"])
            second = (rectangle["x"] + rectangle["width"], rectangle["y"] + rectangle["height"])
            cv2.rectangle(base, first, second, REGION_COLORS.get(name, (0, 220, 255)), 2)
            cv2.putText(base, name.upper(), (first[0] + 4, first[1] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, REGION_COLORS.get(name, (0, 220, 255)), 2, cv2.LINE_AA)
        
        if self.drag_start is not None and self.drag_end is not None and not self.complete:
            rectangle = self._rectangle(self.drag_start, self.drag_end)
            cv2.rectangle(base, (rectangle["x"], rectangle["y"]), (rectangle["x"] + rectangle["width"], rectangle["y"] + rectangle["height"]), REGION_COLORS.get(self.current_name, (0, 220, 255)), 2)

        return base

SLIDER_NAMES = ("H_LOW", "H_HIGH", "S_LOW", "S_HIGH", "V_LOW", "V_HIGH")
SLIDER_MAXIMA = (179, 179, 255, 255, 255, 255)
SLIDER_X_MIN, SLIDER_X_MAX = 1280 + 30, 1640 - 30
SLIDER_Y_POSITIONS = (150, 220, 290, 360, 430, 500)
COLOR_SPECS = {1: ("red", (0, 0, 255)), 2: ("yellow", (0, 255, 255)), 3: ("blue", (255, 0, 0)), 4: ("green", (0, 255, 0)), 5: ("black", (80, 80, 80)), 6: ("light_blue", (255, 255, 0))}
PRESET_BOUNDS = {1: (172, 8, 150, 255, 90, 255), 2: (20, 32, 170, 255, 120, 255), 3: (104, 116, 160, 255, 80, 230), 4: (63, 78, 110, 255, 80, 220), 5: (0, 179, 0, 130, 0, 60), 6: (94, 104, 130, 255, 100, 255)}

def bounds_to_ranges(bounds: Iterable[int]) -> list[dict]:
    values = tuple(int(value) for value in bounds)
    h_low, h_high, s_low, s_high, v_low, v_high = values
    if h_low <= h_high: return [{"lower": [h_low, s_low, v_low], "upper": [h_high, s_high, v_high]}]
    return [{"lower": [0, s_low, v_low], "upper": [h_high, s_high, v_high]}, {"lower": [h_low, s_low, v_low], "upper": [179, s_high, v_high]}]

def ranges_to_bounds(ranges: list[dict]) -> tuple[int, ...]:
    def triplet(r: dict, k: str): return tuple(int(v) for v in r[k])
    if len(ranges) == 1:
        l, u = triplet(ranges[0], "lower"), triplet(ranges[0], "upper")
        return (l[0], u[0], l[1], u[1], l[2], u[2])
    first, second = ranges
    fl, fu = triplet(first, "lower"), triplet(first, "upper")
    sl, su = triplet(second, "lower"), triplet(second, "upper")
    return (sl[0], fu[0], fl[1], fu[1], fl[2], fu[2])

def build_default_color_config(camera_device: str, calibration_file: str) -> dict:
    colors = [{"color_id": cid, "name": n, "display_bgr": list(b), "ranges": bounds_to_ranges(PRESET_BOUNDS[cid])} for cid, (n, b) in COLOR_SPECS.items()]
    return {"format_version": 1, "color_space": "OpenCV_HSV", "camera_device": str(camera_device), "image_width": 1280, "image_height": 720, "calibration_file": str(calibration_file), "colors": colors}

def load_color_config(path: Path | str) -> dict: return json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8"))

def save_color_config(path: Path | str, document: dict) -> None:
    resolved = Path(path).expanduser().resolve()
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=resolved.parent, delete=False) as tmp:
        json.dump(document, tmp, ensure_ascii=False, indent=2)
        tmp.write("\n")
        tmp_name = tmp.name
    os.chmod(tmp_name, 0o644)
    os.replace(tmp_name, resolved)

def estimate_hsv_bounds(sample_arrays: Iterable[np.ndarray], color_id: int) -> tuple[int, ...]:
    arrays = [np.asarray(i, dtype=np.float64).reshape(-1, 3) for i in sample_arrays if np.asarray(i).size]
    values = np.concatenate(arrays, axis=0)
    saturation, value = values[:, 1], values[:, 2]
    if int(color_id) == 5:
        return (0, 179, 0, min(255, int(math.ceil(np.percentile(saturation, 98))) + 20), 0, min(255, int(math.ceil(np.percentile(value, 98))) + 20))
    hue = values[:, 0]
    angles = hue * (2.0 * math.pi / 180.0)
    mean_angle = math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    center_hue = (mean_angle * 180.0 / (2.0 * math.pi)) % 180.0
    delta = ((hue - center_hue + 90.0) % 180.0) - 90.0
    h_low_unwrapped = math.floor(np.percentile(delta, 2)) + center_hue - 3
    h_high_unwrapped = math.ceil(np.percentile(delta, 98)) + center_hue + 3
    if h_high_unwrapped - h_low_unwrapped >= 179.0: h_low, h_high = 0, 179
    else: h_low, h_high = int(math.floor(h_low_unwrapped)) % 180, int(math.ceil(h_high_unwrapped)) % 180
    return (h_low, h_high, max(0, int(math.floor(np.percentile(saturation, 2))) - 15), min(255, int(math.ceil(np.percentile(saturation, 98))) + 15), max(0, int(math.floor(np.percentile(value, 2))) - 20), min(255, int(math.ceil(np.percentile(value, 98))) + 20))

class HSVCalibrationSession:
    def __init__(self, document: dict):
        self.document = copy.deepcopy(document)
        self.samples = {cid: [] for cid in COLOR_SPECS}
        self.baseline_bounds = {cid: None for cid in COLOR_SPECS}
        self.dirty = False

    def _item(self, color_id: int) -> dict: return [i for i in self.document["colors"] if int(i["color_id"]) == int(color_id)][0]
    def bounds(self, color_id: int) -> tuple[int, ...]: return ranges_to_bounds(self._item(color_id)["ranges"])
    def set_bounds(self, color_id: int, bounds: Iterable[int]) -> None:
        ranges = bounds_to_ranges(bounds)
        if ranges != self._item(color_id)["ranges"]:
            self._item(color_id)["ranges"] = ranges
            self.dirty = True

    def add_sample(self, color_id: int, hsv_pixels: np.ndarray) -> None:
        pixels = np.asarray(hsv_pixels, dtype=np.uint8).reshape(-1, 3)
        if len(pixels) > 50000: pixels = pixels[::int(math.ceil(len(pixels) / 50000))]
        if not self.samples[color_id]: self.baseline_bounds[color_id] = self.bounds(color_id)
        self.samples[color_id].append(pixels)
        self.set_bounds(color_id, estimate_hsv_bounds(self.samples[color_id], color_id))

    def undo_sample(self, color_id: int):
        if not self.samples[color_id]: return False
        self.samples[color_id].pop()
        if self.samples[color_id]: self.set_bounds(color_id, estimate_hsv_bounds(self.samples[color_id], color_id))
        else:
            if self.baseline_bounds[color_id]: self.set_bounds(color_id, self.baseline_bounds[color_id])
            self.baseline_bounds[color_id] = None
        return True

    def clear_samples(self, color_id: int):
        if self.samples[color_id]:
            if self.baseline_bounds[color_id]: self.set_bounds(color_id, self.baseline_bounds[color_id])
            self.samples[color_id].clear()
        self.baseline_bounds[color_id] = None

    def reset_to_preset(self, color_id: int):
        self.samples[color_id].clear()
        self.baseline_bounds[color_id] = None
        self.set_bounds(color_id, PRESET_BOUNDS[color_id])

class ColorCalibratorWindow:
    def __init__(self, session: HSVCalibrationSession, config_path: Path):
        self.session = session
        self.config_path = config_path.expanduser().resolve()
        self.selected_color_id = 1
        self.current_frame, self.frozen_frame, self.drag_start, self.drag_current = None, None, None, None
        self.active_slider_index = None
        self.sample_rectangles = {cid: [] for cid in COLOR_SPECS}
        self.slider_values = self.session.bounds(self.selected_color_id)

    def _sync_trackbars(self): self.slider_values = self.session.bounds(self.selected_color_id)
    def update_manual_thresholds(self):
        values = tuple(int(v) for v in self.slider_values)
        if values[2] <= values[3] and values[4] <= values[5]: self.session.set_bounds(self.selected_color_id, values)

    def _slider_index_at(self, x: int, y: int):
        if not SLIDER_X_MIN - 12 <= x <= SLIDER_X_MAX + 12: return None
        for i, sy in enumerate(SLIDER_Y_POSITIONS):
            if abs(int(y) - sy) <= 22: return i
        return None

    def _set_slider_from_x(self, slider_index: int, x: int):
        x = max(SLIDER_X_MIN, min(SLIDER_X_MAX, int(x)))
        vals = list(self.slider_values)
        vals[slider_index] = int(round(((x - SLIDER_X_MIN) / (SLIDER_X_MAX - SLIDER_X_MIN)) * SLIDER_MAXIMA[slider_index]))
        self.slider_values = tuple(vals)
        self.update_manual_thresholds()

    def _on_mouse(self, event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            idx = self._slider_index_at(x, y)
            if idx is not None:
                self.active_slider_index = idx
                self._set_slider_from_x(idx, x)
                return
        if self.active_slider_index is not None:
            if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP): self._set_slider_from_x(self.active_slider_index, x)
            if event == cv2.EVENT_LBUTTONUP: self.active_slider_index = None
            return

        if x >= 1280 and self.drag_start is None: return
        point = (max(0, min(1279, int(x))), max(0, min(719, int(y))))
        
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            self.frozen_frame = self.current_frame.copy()
            self.drag_start = self.drag_current = point
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_current = point
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_current = point
            x1, y1 = min(self.drag_start[0], point[0]), min(self.drag_start[1], point[1])
            x2, y2 = max(self.drag_start[0], point[0]), max(self.drag_start[1], point[1])
            if x2 - x1 >= 10 and y2 - y1 >= 10:
                hsv_pixels = cv2.cvtColor(self.frozen_frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV).reshape(-1, 3)
                self.session.add_sample(self.selected_color_id, hsv_pixels)
                self.sample_rectangles[self.selected_color_id].append((x1, y1, x2, y2))
                self._sync_trackbars()
            self.drag_start = self.drag_current = self.frozen_frame = None

    def select_color(self, color_id: int):
        self.selected_color_id = int(color_id)
        self._sync_trackbars()

    def save(self, device: str, calib_path: Path):
        self.update_manual_thresholds()
        self.session.document["camera_device"] = str(device)
        self.session.document["calibration_file"] = str(calib_path.expanduser().resolve())
        save_color_config(self.config_path, self.session.document)
        self.session.dirty = False

    def render(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = _mask_from_hsv(hsv, bounds_to_ranges(self.slider_values) if self.slider_values[2]<=self.slider_values[3] else [])
        
        if self.selected_color_id == 5:
            img_out = frame.copy()
            img_out[mask == 0] = (255, 255, 255)
        else:
            img_out = (frame.astype(np.float32) * 0.25).astype(np.uint8)
            img_out[mask > 0] = frame[mask > 0]
        
        color = COLOR_SPECS[self.selected_color_id][1]
        for (x1, y1, x2, y2) in self.sample_rectangles[self.selected_color_id]: cv2.rectangle(img_out, (x1, y1), (x2, y2), color, 2)
        if self.drag_start and self.drag_current: cv2.rectangle(img_out, self.drag_start, self.drag_current, (255, 255, 255), 2)

        canvas = np.full((720, 1640, 3), (26, 26, 26), dtype=np.uint8)
        canvas[:, :1280] = img_out
        cv2.line(canvas, (1280, 0), (1280, 719), (90, 90, 90), 2)

        for i, text in enumerate([f"Color {self.selected_color_id}", f"Samples={len(self.session.samples[self.selected_color_id])}"]):
            cv2.putText(canvas, text, (1298, 30 + i * 31), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)

        for i, (name, val, mx, y) in enumerate(zip(SLIDER_NAMES, self.slider_values, SLIDER_MAXIMA, SLIDER_Y_POSITIONS)):
            cv2.putText(canvas, f"{name}: {val}", (SLIDER_X_MIN, y - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1, cv2.LINE_AA)
            cv2.rectangle(canvas, (SLIDER_X_MIN, y - 6), (SLIDER_X_MAX, y + 8), (70, 70, 70), -1)
            knob_x = int(round(SLIDER_X_MIN + val / mx * (SLIDER_X_MAX - SLIDER_X_MIN)))
            cv2.rectangle(canvas, (SLIDER_X_MIN, y - 6), (knob_x, y + 8), color, -1)
        return canvas

def _accumulator_attempts(current: float) -> tuple[float, ...]:
    values = [float(current)]
    for value in (30.0, 26.0, 22.0, 18.0):
        if value < float(current) and value not in values: values.append(value)
    return tuple(values)

@dataclass(frozen=True)
class AutoDiagnosisResult:
    success: bool; document: dict; candidate: DebugCircleCandidate | None; message: str; accumulator_threshold: float

def auto_diagnose_frame(frame: np.ndarray, document: dict) -> AutoDiagnosisResult:
    original = copy.deepcopy(document)
    current_accumulator = float(original["coarse_search"]["accumulator_threshold"])
    failure_details = []
    for accumulator in _accumulator_attempts(current_accumulator):
        trial = copy.deepcopy(original)
        trial["coarse_search"]["minimum_radius_px"] = 120.0
        trial["coarse_search"]["maximum_radius_px"] = 700.0
        trial["coarse_search"]["accumulator_threshold"] = accumulator
        trial["coarse_search"]["maximum_candidates"] = max(8, int(trial["coarse_search"]["maximum_candidates"]))
        trial["refinement"]["radial_search_half_width_px"] = max(90.0, float(trial["refinement"]["radial_search_half_width_px"]))
        trial["refinement"]["minimum_visible_arc_deg"] = 40.0
        trial["refinement"]["maximum_fit_rms_px"] = 8.0
        trial["refinement"]["local_recheck_rms_above_px"] = min(6.0, float(trial["refinement"]["maximum_fit_rms_px"]))
        try:
            debug = TurntableDetector(turntable_config_from_document(trial)).inspect_frame(frame)
        except (TurntableDetectionError, cv2.error) as error:
            failure_details.append(str(error))
            continue
        if debug.selected_index is None:
            failure_details.append(debug.detail)
            continue

        candidate = debug.candidates[debug.selected_index]
        radius = float(candidate.fit.radius_px)
        proposed = copy.deepcopy(original)
        proposed["coarse_search"]["minimum_radius_px"] = float(max(50, math.floor(radius - 60.0)))
        proposed["coarse_search"]["maximum_radius_px"] = float(min(900, math.ceil(radius + 60.0)))
        if accumulator < current_accumulator: proposed["coarse_search"]["accumulator_threshold"] = accumulator
        return AutoDiagnosisResult(True, proposed, candidate, f"自动找到外圆：半径 {candidate.fit.radius_px:.1f}px", accumulator)
    return AutoDiagnosisResult(False, original, None, f"诊断失败：{failure_details[-1] if failure_details else '无'}", current_accumulator)

def draw_debug_overlay(frame: np.ndarray, debug: TurntableDebugResult, show_points: bool) -> np.ndarray:
    best = debug.candidates[debug.selected_index] if debug.selected_index is not None else None
    for i, c in enumerate(debug.candidates, start=1):
        cv2.circle(frame, (int(round(c.seed[0])), int(round(c.seed[1]))), int(round(c.seed[2])), (0, 220, 255), 1, cv2.LINE_8)
        if c.fit.radius_px > 0.0 and math.isfinite(c.fit.radius_px):
            color = (40, 255, 40) if c.fit.detected else (30, 30, 255)
            cv2.circle(frame, (int(round(c.fit.center_x_px)), int(round(c.fit.center_y_px))), int(round(c.fit.radius_px)), color, 3 if c is best else 1, cv2.LINE_8)
    if show_points and best:
        for pt in best.sample_points[::2]: cv2.circle(frame, (int(round(pt[0])), int(round(pt[1]))), 2, (255, 0, 255), -1)
    return frame

# ==============================================================================
# 统一 UI 调度与中文字体渲染框架
# ==============================================================================
GUIDES = {
    0: [
        "【 抓取区与测速调参指南 】",
        "",
        "1. 画框标定位置 (左边画面拖框)：",
        "   ① 框选 1 个目标工作区",
        "   [U] 撤销  [R] 重置",
        "",
        "2. 测速与过滤调参 (右边滑条)：",
        "   * [判定速度] 决定多快算转动",
        "   * [停稳速度] 决定多慢算停稳",
        "   * [过滤面积] 去除杂点或干扰",
        "",
        "3. 完成后按 [S] 一起保存所有配置"
    ],
    1: [
        "【 六色HSV标定指南 】",
        "1. 按键盘 [1-6] 键切换颜色",
        "   *注意：调试【5黑】时背景会全白",
        "2. 在左边拖框采样，右边滑条微调",
        "3. [U] 撤销上次采样",
        "   [C] 清空当前颜色样本",
        "   [R] 恢复默认预设",
        "4. 按 [S] 保存所有颜色配置"
    ],
    2: [
        "【 转盘参数调试指南 】",
        "目标：让绿圈稳定贴合转盘外沿。",
        "1. [A] 自动诊断推荐半径",
        "   [空格] 冻结/恢复画面",
        "   [P] 显示边缘采样点",
        "2. 右边滑条微调",
        "   [Tab] 切换 高级/常用 页",
        "3. 按 [S] 保存转盘配置"
    ],
    3: [
        "【 三圆环工位参数调试指南 】",
        "目标：绿色框中靶环，紫十字锁定圆心",
        "1. 画面必须同时完整出现三个靶环！",
        "2. 调 [边缘阈值] 使绿色椭圆包住靶环",
        "3. 若只找部分，放宽 [共线/长短轴比例]",
        "4. [空格] 冻结画面，[R] 重置稳定窗口",
        "5. 按 [S] 保存三圆环配置"
    ]
}

class UIPainter:
    def __init__(self):
        self.font = self.title_font = self.small_font = None
        if HAS_PIL:
            for p in ["C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc", "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"]:
                if Path(p).exists():
                    self.font = ImageFont.truetype(p, 16)
                    self.title_font = ImageFont.truetype(p, 20)
                    self.small_font = ImageFont.truetype(p, 13)
                    break
            if not self.font: self.font = self.title_font = self.small_font = ImageFont.load_default()

    def draw_tabs_and_guides(self, canvas, tabs, current_tab, tab_height):
        if not HAS_PIL or not self.font: return canvas
        img_pil = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil, "RGBA")
        
        tab_width = canvas.shape[1] // len(tabs)
        for i, name in enumerate(tabs):
            draw.rectangle([i * tab_width, 0, (i + 1) * tab_width, tab_height], fill=(60,60,60) if i == current_tab else (30,30,30))
            draw.line([(i + 1) * tab_width, 0, (i + 1) * tab_width, tab_height], fill=(0,0,0), width=2)
            draw.text((i * tab_width + 20, 6), name, font=self.title_font, fill=(255,255,255) if i == current_tab else (150,150,150))
        
        gx, gy = 20, 360 + tab_height
        lines = GUIDES[current_tab]
        draw.rectangle([gx, gy, gx + 330, gy + len(lines)*24 + 30], fill=(20,20,25,225), outline=(80,80,90,255), width=2)
        for i, line in enumerate(lines): draw.text((gx + 15, gy + 10 + i * 24), line, font=self.title_font if i==0 else self.font, fill=(0,255,150,255) if i==0 else (235,235,235,255))
        return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

    def draw_pickup_panel(self, panel: ParameterPanel, result, msg: str, fps: float):
        if not HAS_PIL or not self.font: return np.full((720, 420, 3), 26, dtype=np.uint8)
        img = Image.new("RGB", (420, 720), (25, 27, 31))
        draw = ImageDraw.Draw(img)
        draw.text((14, 8), "速度与检测参数设置", font=self.title_font, fill=(245, 245, 245))
        state = result.state if result else "WAIT"
        ready = result.ready if result else False
        speed = result.max_speed_px_s if result else 0.0
        mat_cnt = result.material_count if result else 0
        
        draw.text((14, 40), f"状态：{state}  数量：{mat_cnt}  FPS：{fps:.1f}", font=self.font, fill=(80, 240, 120) if ready else (255, 110, 90))
        draw.text((14, 66), f"当前实时速度: {speed:.1f} px/s", font=self.font, fill=(0, 220, 255))
        draw.text((14, 96), msg[:29], font=self.small_font, fill=(255, 205, 70))

        for idx, spec in enumerate(panel.specs):
            y = panel.ROW_START + idx * panel.ROW_HEIGHT + 20
            val = float(panel.value(spec))
            frac = (val - spec.minimum) / (spec.maximum - spec.minimum)
            draw.text((panel.TRACK_LEFT, y), f"{spec.label}   {spec.format_value(val)}", font=self.small_font, fill=(255, 220, 90) if idx == panel.selected_index else (230, 230, 230))
            draw.rectangle((panel.TRACK_LEFT, y + 26, panel.TRACK_RIGHT, y + 32), fill=(65, 68, 76))
            kx = int(round(panel.TRACK_LEFT + frac * (panel.TRACK_RIGHT - panel.TRACK_LEFT)))
            draw.rectangle((panel.TRACK_LEFT, y + 26, kx, y + 32), fill=(40, 170, 255))
            draw.ellipse((kx - 6, y + 22, kx + 6, y + 36), fill=(255, 255, 255))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    def draw_turntable_panel(self, panel: ParameterPanel, result, debug, msg: str, fps: float):
        if not HAS_PIL or not self.font: return np.full((720, 420, 3), 26, dtype=np.uint8)
        img = Image.new("RGB", (420, 720), (25, 27, 31))
        draw = ImageDraw.Draw(img)
        draw.text((14, 8), f"参数设置 · {'常用' if panel.page == 'common' else '高级'}", font=self.title_font, fill=(245, 245, 245))
        draw.text((14, 40), f"状态：{'稳定' if result.stable else ('已检' if result.detected else '未检')}  候选：{len(debug.candidates)}  FPS：{fps:.1f}", font=self.font, fill=(80, 240, 120) if result.detected else (255, 110, 90))
        draw.text((14, 66), f"半径 {result.radius_px:.1f} 弧 {result.visible_arc_deg:.0f}° RMS {result.fit_rms_px:.2f}" if result.detected else debug.detail[:38], font=self.small_font, fill=(220, 220, 220))
        draw.text((14, 86), msg[:29], font=self.small_font, fill=(255, 205, 70))

        for idx, spec in enumerate(panel.specs):
            y = panel.ROW_START + idx * panel.ROW_HEIGHT
            val = float(panel.value(spec))
            frac = (val - spec.minimum) / (spec.maximum - spec.minimum)
            draw.text((panel.TRACK_LEFT, y), f"{spec.label}   {spec.format_value(val)}", font=self.small_font, fill=(255, 220, 90) if idx == panel.selected_index else (230, 230, 230))
            draw.rectangle((panel.TRACK_LEFT, y + 26, panel.TRACK_RIGHT, y + 32), fill=(65, 68, 76))
            kx = int(round(panel.TRACK_LEFT + frac * (panel.TRACK_RIGHT - panel.TRACK_LEFT)))
            draw.rectangle((panel.TRACK_LEFT, y + 26, kx, y + 32), fill=(40, 170, 255))
            draw.ellipse((kx - 6, y + 22, kx + 6, y + 36), fill=(255, 255, 255))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        
    def draw_ring_station_panel(self, panel: ParameterPanel, result, msg: str, fps: float):
        if not HAS_PIL or not self.font: return np.full((720, 420, 3), 26, dtype=np.uint8)
        img = Image.new("RGB", (420, 720), (25, 27, 31))
        draw = ImageDraw.Draw(img)
        draw.text((14, 8), "三圆环识别参数设置", font=self.title_font, fill=(245, 245, 245))
        
        status_text = "稳定" if result.stable else ("已找到" if result.detected else "未找到")
        status_color = (80, 240, 120) if result.stable else ((255, 200, 50) if result.detected else (255, 110, 90))
        draw.text((14, 40), f"状态：{status_text}  FPS：{fps:.1f}", font=self.font, fill=status_color)
        
        detail_msg = result.detail[:38] if hasattr(result, 'detail') else "等待中..."
        draw.text((14, 66), detail_msg, font=self.small_font, fill=(220, 220, 220))
        draw.text((14, 86), msg[:29], font=self.small_font, fill=(255, 205, 70))

        for idx, spec in enumerate(panel.specs):
            y = panel.ROW_START + idx * panel.ROW_HEIGHT
            val = float(panel.value(spec))
            frac = (val - spec.minimum) / (spec.maximum - spec.minimum)
            draw.text((panel.TRACK_LEFT, y), f"{spec.label}   {spec.format_value(val)}", font=self.small_font, fill=(255, 220, 90) if idx == panel.selected_index else (230, 230, 230))
            draw.rectangle((panel.TRACK_LEFT, y + 26, panel.TRACK_RIGHT, y + 32), fill=(65, 68, 76))
            kx = int(round(panel.TRACK_LEFT + frac * (panel.TRACK_RIGHT - panel.TRACK_LEFT)))
            draw.rectangle((panel.TRACK_LEFT, y + 26, kx, y + 32), fill=(40, 170, 255))
            draw.ellipse((kx - 6, y + 22, kx + 6, y + 36), fill=(255, 255, 255))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

class IntegratedDebugger:
    def __init__(self, args):
        self.args = args
        self.ui_painter = UIPainter()
        self.canvas_width, self.frame_height, self.tab_height = 1700, 720, 40
        self.canvas_height = self.frame_height + self.tab_height
        self.tabs = ["1. 物料与速度调参", "2. 六色 HSV 标定", "3. 转盘边缘调试", "4. 三圆环对位调试"]
        self.current_tab = 0
        
        self.size, self.camera_matrix, self.dist, self.new_camera_matrix = load_camera_calibration_shared(args.calibration)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist, None, self.new_camera_matrix, self.size, cv2.CV_16SC2)
        
        # 1. 初始化物料面板
        try: doc_region = load_material_pickup_config(args.region_config)
        except Exception: doc_region = build_default_material_pickup_config()
        self.pickup_panel = ParameterPanel(doc_region, PICKUP_SPECS)
        self.region_selector = RegionSelector(self.pickup_panel)
        
        # 2. 初始化颜色面板
        doc_color = load_color_config(args.color_config) if args.color_config.expanduser().exists() else build_default_color_config(args.device, str(args.calibration))
        self.color_session = HSVCalibrationSession(doc_color)
        self.color_interface = ColorCalibratorWindow(self.color_session, args.color_config)
        self.pickup_detector = MaterialPickupDetector(self.pickup_panel.document, self.color_session.document)
        
        # 3. 初始化转盘面板
        doc_turntable = load_turntable_detector_document(args.turntable_config)
        self.turntable_panel = ParameterPanel(doc_turntable, TURNTABLE_SPECS)
        self.turntable_detector = TurntableDetector(turntable_config_from_document(doc_turntable))
        self.turntable_frozen, self.frozen_frame, self.turntable_show_points = False, None, False
        self.turntable_msg = "就绪"

        # 4. 初始化三圆环面板
        try:
            with open(args.ring_config, 'r', encoding='utf-8') as f: doc_ring = json.load(f)
        except Exception:
            doc_ring = {}
        self.ring_panel = ParameterPanel(doc_ring, RING_STATION_SPECS)
        self.ring_detector = RingStationDetector(load_ring_station_config(args.ring_config))
        self.ring_msg = "就绪"
        self.ring_frozen = False
        self.ring_frozen_frame = None
        
        self.frame_times, self.measured_fps = [], 0.0

    def mouse_callback(self, e, x, y, f, p):
        if e == cv2.EVENT_LBUTTONDOWN and y < self.tab_height:
            new_tab = min(x // (self.canvas_width // len(self.tabs)), len(self.tabs) - 1)
            if self.current_tab != new_tab:
                self.current_tab = new_tab
                self.color_interface.active_slider_index = self.turntable_panel.dragging_index = self.pickup_panel.dragging_index = self.ring_panel.dragging_index = None 
            return
        adj_y = y - self.tab_height
        if adj_y < 0: return
        
        if self.current_tab == 0:
            if x < 1280: self.region_selector.mouse_callback(e, x, adj_y, f, p)
            else: self.pickup_panel.mouse_callback(e, x, adj_y, f, p)
        elif self.current_tab == 1:
            self.color_interface._on_mouse(e, x, adj_y, f, p)
        elif self.current_tab == 2:
            self.turntable_panel.mouse_callback(e, x, adj_y, f, p)
        elif self.current_tab == 3:
            self.ring_panel.mouse_callback(e, x, adj_y, f, p)

    def run(self):
        camera = open_camera_shared(self.args.device, self.args.fps)
        cv2.namedWindow("Debugger", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Debugger", self.canvas_width, self.canvas_height)
        cv2.setMouseCallback("Debugger", self.mouse_callback)

        turn_res, turn_dbg = None, TurntableDebugResult((), None, 0.0, "Wait")
        
        try:
            while True:
                # 统一获取并纠正最新帧
                if not (self.turntable_frozen and self.current_tab == 2) and not (self.ring_frozen and self.current_tab == 3):
                    ok, raw = camera.read()
                    if not ok: break
                    latest = cv2.remap(raw, self.map1, self.map2, cv2.INTER_LINEAR)
                    if self.turntable_frozen: self.frozen_frame = latest.copy()
                    if self.ring_frozen: self.ring_frozen_frame = latest.copy()

                now = time.monotonic()
                self.frame_times.append(now)
                if len(self.frame_times) > 45: del self.frame_times[0]
                if len(self.frame_times) >= 2 and (self.frame_times[-1] - self.frame_times[0]) > 0:
                    self.measured_fps = (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])

                canvas = np.full((self.canvas_height, self.canvas_width, 3), 26, dtype=np.uint8)
                
                # --- Tab 1: 区域与测速 ---
                if self.current_tab == 0:
                    if self.pickup_panel.dirty or self.color_session.dirty:
                        self.pickup_detector = MaterialPickupDetector(self.pickup_panel.document, self.color_session.document)
                        self.pickup_panel.dirty = self.color_session.dirty = False
                    
                    pickup_res = self.pickup_detector.detect(latest, timestamp=now)
                    base = self.region_selector.draw(latest if self.region_selector.frozen_frame is None else self.region_selector.frozen_frame)
                    preview = draw_material_pickup_detection(base, pickup_res, self.pickup_panel.document)
                    
                    canvas[self.tab_height:, :1280] = preview
                    canvas[self.tab_height:, 1280:1700] = self.ui_painter.draw_pickup_panel(self.pickup_panel, pickup_res, self.region_selector.message, self.measured_fps)

                # --- Tab 2: 颜色标定 ---
                elif self.current_tab == 1:
                    self.color_interface.current_frame = latest
                    self.color_interface.update_manual_thresholds()
                    canvas[self.tab_height:, :1640] = self.color_interface.render(self.color_interface.frozen_frame if self.color_interface.frozen_frame is not None else latest)
                
                # --- Tab 3: 转盘参数 ---
                elif self.current_tab == 2:
                    if self.turntable_panel.dirty:
                        self.turntable_detector = TurntableDetector(turntable_config_from_document(self.turntable_panel.document))
                        self.turntable_panel.dirty = False
                        if self.turntable_panel.slider_changed: self.turntable_msg = "参数已应用"
                        self.turntable_panel.slider_changed = False
                        
                    src = self.frozen_frame if self.turntable_frozen and self.frozen_frame is not None else latest
                    if not self.turntable_frozen or turn_res is None or self.turntable_panel.slider_changed:
                        turn_res, turn_dbg = self.turntable_detector.detect_with_debug(src)
                        
                    preview = src.copy()
                    draw_debug_overlay(preview, turn_dbg, self.turntable_show_points)
                    draw_turntable_detection(preview, turn_res)
                    canvas[self.tab_height:, :1280] = preview
                    canvas[self.tab_height:, 1280:1700] = self.ui_painter.draw_turntable_panel(self.turntable_panel, turn_res, turn_dbg, self.turntable_msg, self.measured_fps)

                # --- Tab 4: 三圆环对位调试 ---
                elif self.current_tab == 3:
                    if self.ring_panel.dirty:
                        # 用临时文件重新实例化检测器
                        with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as tmp:
                            json.dump(self.ring_panel.document, tmp)
                            tmp_name = tmp.name
                        self.ring_detector = RingStationDetector(load_ring_station_config(tmp_name))
                        os.remove(tmp_name)
                        
                        self.ring_panel.dirty = False
                        if self.ring_panel.slider_changed: self.ring_msg = "参数已应用"
                        self.ring_panel.slider_changed = False

                    src = self.ring_frozen_frame if self.ring_frozen and self.ring_frozen_frame is not None else latest
                    ring_res = self.ring_detector.detect(src)
                    
                    preview = src.copy()
                    draw_ring_station_detection(preview, ring_res)
                    canvas[self.tab_height:, :1280] = preview
                    canvas[self.tab_height:, 1280:1700] = self.ui_painter.draw_ring_station_panel(self.ring_panel, ring_res, self.ring_msg, self.measured_fps)

                # 渲染导航栏和教程文字
                canvas = self.ui_painter.draw_tabs_and_guides(canvas, self.tabs, self.current_tab, self.tab_height)
                cv2.imshow("Debugger", canvas)

                # 键盘事件处理
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27): break
                
                if self.current_tab == 0:
                    if key in (ord("u"), ord("U")): self.region_selector.undo()
                    elif key in (ord("r"), ord("R")): self.region_selector.reset()
                    elif key in (ord("s"), ord("S")): self.region_selector.save(self.args.region_config)
                
                elif self.current_tab == 1:
                    if ord("1") <= key <= ord("6"): 
                        self.color_interface.select_color(key - ord("0"))
                    elif key in (ord("u"), ord("U")): 
                        if self.color_session.undo_sample(self.color_interface.selected_color_id):
                            if self.color_interface.sample_rectangles[self.color_interface.selected_color_id]:
                                self.color_interface.sample_rectangles[self.color_interface.selected_color_id].pop()
                    elif key in (ord("c"), ord("C")): 
                        self.color_session.clear_samples(self.color_interface.selected_color_id)
                        self.color_interface.sample_rectangles[self.color_interface.selected_color_id].clear()
                    elif key in (ord("r"), ord("R")): 
                        self.color_session.reset_to_preset(self.color_interface.selected_color_id)
                        self.color_interface.sample_rectangles[self.color_interface.selected_color_id].clear()
                    elif key in (ord("s"), ord("S")): 
                        self.color_interface.save(self.args.device, self.args.calibration)
                
                elif self.current_tab == 2:
                    if key == 9: self.turntable_panel.toggle_page()
                    elif key == 32:
                        self.turntable_frozen = not self.turntable_frozen
                        self.frozen_frame = latest.copy() if self.turntable_frozen else None
                        self.turntable_msg = "冻结" if self.turntable_frozen else "实时"
                    elif key in (ord("p"), ord("P")): self.turntable_show_points = not self.turntable_show_points
                    elif key in (ord("a"), ord("A")):
                        diagnosis = auto_diagnose_frame(src, self.turntable_panel.document)
                        self.turntable_msg = diagnosis.message
                        if diagnosis.success: self.turntable_panel.replace_document(diagnosis.document)
                    elif key in (ord("r"), ord("R")): self.turntable_panel.replace_document(load_turntable_detector_document(self.args.turntable_config))
                    elif key in (ord("d"), ord("D")): self.turntable_panel.replace_document(build_default_turntable_detector_document())
                    elif key in (ord("s"), ord("S")): save_turntable_detector_config(self.args.turntable_config, self.turntable_panel.document)
                
                elif self.current_tab == 3:
                    if key == 32:
                        self.ring_frozen = not self.ring_frozen
                        self.ring_frozen_frame = latest.copy() if self.ring_frozen else None
                        self.ring_msg = "冻结" if self.ring_frozen else "实时"
                    elif key in (ord('r'), ord('R')):
                        self.ring_detector.reset()
                        self.ring_msg = "已重置历史窗口"
                    elif key in (ord('s'), ord('S')):
                        with open(self.args.ring_config, 'w', encoding='utf-8') as f:
                            json.dump(self.ring_panel.document, f, indent=2, ensure_ascii=False)
                        self.ring_msg = "配置已保存至JSON"

        finally:
            camera.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    SCRIPT_DIR = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="一体式机器人视觉调试上位机")
    parser.add_argument("--device", default=0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", type=Path, default=SCRIPT_DIR / "config" / "realtek_rgb_camera.yaml")
    parser.add_argument("--region_config", type=Path, default=SCRIPT_DIR / "config" / "material_pickup_detector.json")
    parser.add_argument("--color_config", type=Path, default=SCRIPT_DIR / "config" / "material_color_thresholds.json")
    parser.add_argument("--turntable_config", type=Path, default=SCRIPT_DIR / "config" / "turntable_detector.json")
    parser.add_argument("--ring_config", type=Path, default=SCRIPT_DIR / "config" / "ring_station_detector.json")
    IntegratedDebugger(parser.parse_args()).run()