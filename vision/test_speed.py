"""颜色物料与转盘检测速度测试脚本。

"""

from __future__ import annotations  # 必须放在第一行！

import cv2
import time
import json
import os
import sys
import types
import numpy as np

# ==============================================================================
# 动态依赖修复：拦截底层的导包请求，免去新建 颜色标定.py
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

from material_pickup_detector import MaterialPickupDetector, load_material_pickup_config, draw_material_pickup_detection
from wuliaopan import load_calibration

def main():
    """速度测试入口：实时显示物料检测状态与速度估计。"""
    # 1. 加载配置文件
    calib_file = "config/realtek_rgb_camera.yaml"
    material_file = "config/material_pickup_detector.json"
    color_file = "config/material_color_thresholds.json"
    if not os.path.exists(color_file) and os.path.exists("config/颜色标定.json"):
        color_file = "config/颜色标定.json"

    # 初始化畸变映射表
    try:
        size, cam_mat, dist, new_cam_mat = load_calibration(calib_file)
        map1, map2 = cv2.initUndistortRectifyMap(cam_mat, dist, None, new_cam_mat, size, cv2.CV_16SC2)
    except Exception as e:
        print(f"标定加载失败: {e}")
        return

    # 初始化颜色物料检测器
    try:
        mat_config = load_material_pickup_config(material_file)
        with open(color_file, 'r', encoding='utf-8') as f:
            color_config = json.load(f)
        detector = MaterialPickupDetector(mat_config, color_config)
        print("检测器初始化成功，请放置物料并转动转盘进行测试...")
    except Exception as e:
        print(f"物料检测器初始化失败: {e}")
        return

    # 2. 初始化摄像头（去掉 cv2.CAP_V4L2，加入 BUFFERSIZE 修复）
    cap = cv2.VideoCapture(0)  # Windows 环境兼容写法
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 【修复核心】强制只保留最新一帧，丢弃积压帧
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("无法打开摄像头！请检查是否被其他程序占用，或者尝试修改设备号（如把 VideoCapture(0) 改成 1 或 2）。")
        return

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("正在等待摄像头画面...")
            time.sleep(0.5)
            continue

        # 【修复核心】在读取画面后立刻获取当前时间戳，跳过 remap 的耗时波动
        st = time.time()
        
        # 图像畸变矫正
        frame_rect = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        # 【修复核心】显式传入时间戳
        result = detector.detect(frame_rect, timestamp=st)

        # 在终端清晰打印速度和状态切换流
        print(f"状态: {result.state:<15} | 速度: {result.max_speed_px_s:>6.1f} px/s | "
              f"动:{result.moving_frames}/{result.required_moving_frames} | "
              f"静:{result.low_speed_frames}/{result.required_low_speed_frames}")

        # 绘制检测结果并缩放显示
        display_frame = draw_material_pickup_detection(frame_rect, result, detector.config)
        cv2.imshow("Speed Measurement Test", cv2.resize(display_frame, (640, 360)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()