#!/usr/bin/env python3
"""颜色物料检测封装：兼容旧“颜色标定”依赖并提供稳定判定入口。

"""

from __future__ import annotations
import os
import json
import time
import sys
import types
import cv2
import numpy as np

# ==============================================================================
# 动态依赖修复：拦截底层的导包请求，免去新建 颜色标定.py
# ==============================================================================
def _mask_from_hsv(hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
    """按配置的 HSV 区间合并掩膜，保持原有区间遍历顺序。"""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.asarray(hsv_range["lower"], dtype=np.uint8), np.asarray(hsv_range["upper"], dtype=np.uint8)))
    return mask

def make_all_color_masks(frame: np.ndarray, config: dict) -> dict[int, np.ndarray]:
    """一次性生成所有颜色通道掩膜，供下游检测模块复用。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {int(item["color_id"]): _mask_from_hsv(hsv, item["ranges"]) for item in config["colors"]}

def _find_color(document: dict, color: int | str) -> dict:
    """按颜色 ID 查找配置条目，未找到时沿用原有 IndexError 行为。"""
    return [item for item in document["colors"] if int(item["color_id"]) == int(color)][0]

def make_color_mask(frame: np.ndarray, color: int | str, config: dict) -> np.ndarray:
    """生成单个颜色通道掩膜，供独立调试使用。"""
    item = _find_color(config, color)
    return _mask_from_hsv(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), item["ranges"])

mock_color_module = types.ModuleType('颜色标定')
mock_color_module._mask_from_hsv = _mask_from_hsv
mock_color_module.make_all_color_masks = make_all_color_masks
mock_color_module._find_color = _find_color
mock_color_module.make_color_mask = make_color_mask
sys.modules['颜色标定'] = mock_color_module
# ==============================================================================

from material_pickup_detector import (
    MaterialPickupDetector,
    load_material_pickup_config,
    draw_material_pickup_detection
)

def init_yanse_detector(material_config_file="config/material_pickup_detector.json", 
                        color_config_file="config/material_color_thresholds.json"):
    """初始化颜色物料检测器"""
    if not os.path.exists(color_config_file) and os.path.exists("config/颜色标定.json"):
        color_config_file = "config/颜色标定.json"
    try:
        material_config = load_material_pickup_config(material_config_file)
        with open(color_config_file, 'r', encoding='utf-8') as f:
            color_config = json.load(f)
        detector = MaterialPickupDetector(material_config, color_config)
        print(f"颜色物料检测器初始化完成 (加载配置: {color_config_file})。")
        return detector
    except Exception as e:
        print(f"颜色物料检测器初始化失败: {e}")
        return None

# 全局变量：用于记录颜色稳定时间
_last_color = 0
_color_start_time = 0.0

def process_yanse(raw_frame, detector, map1=None, map2=None, stable_duration=2.0):
    """
    处理单帧颜色检测，增加连续识别稳定时间判定
    :param stable_duration: 颜色持续多少秒才视为识别成功（默认 2.0 秒）
    :return: (is_stable, current_color, processed_frame)
    """
    global _last_color, _color_start_time
    
    if detector is None:
        return False, 0, raw_frame

    if map1 is not None and map2 is not None:
        frame_rect = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)
    else:
        frame_rect = raw_frame.copy()

    current_time = time.time()
    result = detector.detect(frame_rect, timestamp=current_time)
    
    current_color = result.slot_colors[0] if result.slot_colors else 0
    processed_frame = draw_material_pickup_detection(frame_rect, result, detector.config)
    
    is_stable = False
    
    if current_color == 0:
        _last_color = 0
        _color_start_time = current_time
    elif current_color == _last_color:
        if (current_time - _color_start_time) >= stable_duration:
            is_stable = True
    else:
        _last_color = current_color
        _color_start_time = current_time

    if _last_color != 0:
        elapsed = current_time - _color_start_time
        remaining = max(0, stable_duration - elapsed)
        if remaining > 0:
            cv2.putText(processed_frame, f"Locking... {remaining:.1f}s", (10, 150), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
        else:
            cv2.putText(processed_frame, "LOCKED!", (10, 150), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return is_stable, _last_color, processed_frame

if __name__ == "__main__":
    from wuliaopan import load_calibration
    print("正在启动颜色物料检测独立测试...")
    try:
        size, cam_mat, dist, new_cam_mat = load_calibration("config/realtek_rgb_camera.yaml")
        map1, map2 = cv2.initUndistortRectifyMap(cam_mat, dist, None, new_cam_mat, size, cv2.CV_16SC2)
    except Exception:
        map1, map2 = None, None
    detector = init_yanse_detector()

    cap_usb = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    color_map = {0: "Empty", 1: "Red", 2: "Yellow", 3: "Blue", 4: "Green", 5: "Black", 6: "L-Blue"}
    while True:
        ret, frame = cap_usb.read()
        if not ret or frame is None:
            continue
            
        is_stable, current_color, display_frame = process_yanse(frame, detector, map1, map2, stable_duration=2.0)
        
        color_name = color_map.get(current_color, str(current_color))
        print(f"当前颜色: {color_name:<8} | 发送状态: {'[SEND]' if is_stable else '...等待2秒...'}")
        
        cv2.imshow("Yanse Standalone Test", cv2.resize(display_frame, (640, 360)))
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        
    cap_usb.release()
    cv2.destroyAllWindows()