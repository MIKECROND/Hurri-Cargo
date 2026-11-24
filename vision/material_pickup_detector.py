#!/usr/bin/env python3
"""固定高位视角下的转盘物料颜色、位置和低速状态检测。

本模块只处理已经完成畸变矫正的 BGR 图像。低位转盘圆心不能在摄像头
抬起后继续使用，因此高位视角单独保存一个物料活动区 (workspace)。
单区域版本：检测器在活动区内识别唯一 1 件物料并测定其转动和停稳状态。

"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import json
import math
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np

from 颜色标定 import make_all_color_masks

COLOR_NAMES = {
    0: "empty",
    1: "red",
    2: "yellow",
    3: "blue",
    4: "green",
    5: "black",
    6: "light_blue",
}


class MaterialPickupError(ValueError):
    """高位物料配置或输入图像不满足检测要求。"""


@dataclass(frozen=True)
class MaterialRegion:
    """一件通过颜色和形状过滤的物料。"""

    color_id: int
    name: str
    center_x_px: float
    center_y_px: float
    area_px: float
    confidence: float
    bounding_box: tuple[int, int, int, int]
    contour: object


@dataclass(frozen=True)
class MaterialPickupDetection:
    """一帧高位物料检测结果。

    ``ready`` 表示当前帧已经积累足够的低速和位置稳定证据。
    单区域版中 ``slot_colors`` 返回单元素元组，如 (1,)。
    """

    detected: bool
    ready: bool
    calibrated: bool
    state: str
    material_count: int
    slot_colors: tuple[int, ...]
    max_speed_px_s: float
    motion_confirmed: bool
    moving_frames: int
    required_moving_frames: int
    low_speed_frames: int
    required_low_speed_frames: int
    processing_ms: float
    detail: str
    regions: tuple[MaterialRegion, ...] = ()

    @property
    def stable(self) -> bool:
        """与现有检测结果保持一致的稳定状态别名。"""
        return self.ready


def build_default_material_pickup_config() -> dict:
    """构造未标定的安全默认配置 (单区域版)。"""

    return {
        "format_version": 1,
        "image_width": 1280,
        "image_height": 720,
        "calibrated": False,
        "regions": {
            "workspace": None
        },
        "detection": {
            "processing_scale": 0.5,
            "open_kernel_px": 5,
            "close_kernel_px": 11,
            "minimum_area_px": 450.0,
            "maximum_area_px": 26000.0,
            "minimum_dimension_px": 18.0,
            "maximum_dimension_px": 210.0,
            "maximum_aspect_ratio": 2.35,
            "minimum_solidity": 0.55,
            "minimum_extent": 0.28,
            "maximum_material_count": 1,
            "slot_interior_margin_px": 5.0,
            "maximum_match_distance_px": 80.0,
        },
        "motion": {
            "moving_speed_px_s": 30.0,
            "required_moving_frames": 2,
            "near_stop_speed_px_s": 25.0,
            "required_low_speed_frames": 3,
            "minimum_frame_interval_s": 0.02,
            "maximum_frame_interval_s": 0.25,
        },
    }


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise MaterialPickupError(message)


def _normalise_rectangle(value, name: str, width: int, height: int) -> dict:
    try:
        rectangle = {
            "x": int(value["x"]),
            "y": int(value["y"]),
            "width": int(value["width"]),
            "height": int(value["height"]),
        }
    except (KeyError, TypeError, ValueError) as error:
        raise MaterialPickupError(f"{name}区域格式无效") from error
    _require(rectangle["width"] > 0 and rectangle["height"] > 0,
             f"{name}区域宽高必须大于0")
    _require(rectangle["x"] >= 0 and rectangle["y"] >= 0,
             f"{name}区域不能超出图像左上边界")
    _require(
        rectangle["x"] + rectangle["width"] <= width
        and rectangle["y"] + rectangle["height"] <= height,
        f"{name}区域超出{width}x{height}图像边界",
    )
    return rectangle


def validate_material_pickup_config(document: dict) -> dict:
    """严格校验高位区域、形态参数和低速参数。"""
    try:
        _require(isinstance(document, dict), "物料配置根节点必须是对象")
        _require(int(document["format_version"]) == 1,
                 "不支持的物料配置版本")
        width = int(document["image_width"])
        height = int(document["image_height"])
        _require((width, height) == (1280, 720),
                 "物料配置分辨率必须为1280x720")
        calibrated = document["calibrated"]
        _require(isinstance(calibrated, bool), "calibrated必须为布尔值")
        regions = document["regions"]
        _require(isinstance(regions, dict), "regions必须是对象")
        
        expected = {"workspace"}
        _require(set(regions) == expected, "必须配置活动区(workspace)，无需多余区域")

        rectangles = {}
        if calibrated:
            rectangles["workspace"] = _normalise_rectangle(regions["workspace"], "workspace", width, height)
        else:
            _require(regions["workspace"] is None, "未标定配置的区域必须为null")

        detection = document["detection"]
        scale = float(detection["processing_scale"])
        _require(0.25 <= scale <= 1.0, "processing_scale必须在0.25～1.0")
        for key in ("open_kernel_px", "close_kernel_px"):
            value = int(detection[key])
            _require(value >= 1 and value % 2 == 1,
                     f"{key}必须是正奇数")
        minimum_area = float(detection["minimum_area_px"])
        maximum_area = float(detection["maximum_area_px"])
        _require(0.0 < minimum_area < maximum_area, "物料面积范围无效")
        minimum_dimension = float(detection["minimum_dimension_px"])
        maximum_dimension = float(detection["maximum_dimension_px"])
        _require(0.0 < minimum_dimension < maximum_dimension,
                 "物料尺寸范围无效")
        _require(float(detection["maximum_aspect_ratio"]) >= 1.0,
                 "maximum_aspect_ratio不能小于1")
        _require(0.0 < float(detection["minimum_solidity"]) <= 1.0,
                 "minimum_solidity必须在0～1")
        _require(0.0 < float(detection["minimum_extent"]) <= 1.0,
                 "minimum_extent必须在0～1")
        _require(int(detection["maximum_material_count"]) == 1,
                 "单点抓取模式下，当前协议最多支持1件物料")
        margin = float(detection["slot_interior_margin_px"])
        _require(margin >= 0.0, "内部边距不能为负数")
        _require(float(detection["maximum_match_distance_px"]) > 0.0,
                 "跨帧匹配距离必须大于0")
        if calibrated:
            rectangle = rectangles["workspace"]
            _require(
                margin * 2 < min(rectangle["width"], rectangle["height"]),
                f"workspace区域小于两倍内部边距",
            )

        motion = document["motion"]
        moving_speed = float(motion["moving_speed_px_s"])
        near_stop_speed = float(motion["near_stop_speed_px_s"])
        _require(moving_speed > near_stop_speed,
                 "转动确认速度必须大于接近停稳速度")
        _require(int(motion["required_moving_frames"]) >= 1,
                 "转动确认帧数必须大于0")
        _require(near_stop_speed > 0.0,
                 "低速阈值必须大于0")
        _require(int(motion["required_low_speed_frames"]) >= 1,
                 "低速确认帧数必须大于0")
        minimum_interval = float(motion["minimum_frame_interval_s"])
        maximum_interval = float(motion["maximum_frame_interval_s"])
        _require(0.0 < minimum_interval < maximum_interval,
                 "有效帧间隔范围无效")
    except (KeyError, TypeError, ValueError) as error:
        if isinstance(error, MaterialPickupError):
            raise
        raise MaterialPickupError("物料配置字段格式无效") from error
    return document


def load_material_pickup_config(path: Path | str) -> dict:
    """读取并校验高位物料 JSON 配置。"""
    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except OSError as error:
        raise MaterialPickupError(f"无法读取物料配置：{resolved}: {error}") from error
    except json.JSONDecodeError as error:
        raise MaterialPickupError(f"物料配置不是有效JSON：{resolved}") from error
    return validate_material_pickup_config(document)


def save_material_pickup_config(path: Path | str, document: dict) -> None:
    """在同目录原子保存配置。"""
    validate_material_pickup_config(document)
    resolved = Path(path).expanduser().resolve()
    _require(resolved.parent.is_dir(), f"配置目录不存在：{resolved.parent}")
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=resolved.parent,
            prefix=f".{resolved.name}.", suffix=".tmp", delete=False,
        ) as temporary:
            temporary_name = temporary.name
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, resolved)
    except OSError as error:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink(missing_ok=True)
            except OSError:
                pass
        raise MaterialPickupError(f"无法保存物料配置：{resolved}: {error}") from error


class MaterialPickupDetector:
    """单区域物料检测，并在接近停稳时输出位置颜色。"""

    def __init__(self, config: dict, color_config: dict):
        self.config = validate_material_pickup_config(config)
        self.color_config = color_config
        self.reset()

    def reset(self) -> None:
        self._previous_regions: tuple[MaterialRegion, ...] = ()
        self._previous_timestamp_s: float | None = None
        self._previous_slots: tuple[int, ...] | None = None
        self._motion_confirmed = False
        self._moving_frames = 0
        self._low_speed_frames = 0

    def _empty_result(
        self, started_s: float, state: str, detail: str,
        *, calibrated: bool = True, regions=(), slots=(0,), speed=0.0,
    ) -> MaterialPickupDetection:
        return MaterialPickupDetection(
            detected=bool(regions), ready=False, calibrated=calibrated,
            state=state, material_count=len(regions),
            slot_colors=tuple(slots), max_speed_px_s=float(speed),
            motion_confirmed=self._motion_confirmed,
            moving_frames=self._moving_frames,
            required_moving_frames=int(self.config["motion"]["required_moving_frames"]),
            low_speed_frames=self._low_speed_frames,
            required_low_speed_frames=int(self.config["motion"]["required_low_speed_frames"]),
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            detail=detail, regions=tuple(regions),
        )

    @staticmethod
    def _scaled_kernel(full_size: int, scale: float) -> int:
        value = max(1, int(round(full_size * scale)))
        return value if value % 2 else value + 1

    def _find_regions(self, frame: np.ndarray) -> tuple[MaterialRegion, ...]:
        detection = self.config["detection"]
        workspace = self.config["regions"]["workspace"]
        x0, y0 = workspace["x"], workspace["y"]
        width, height = workspace["width"], workspace["height"]
        cropped = frame[y0:y0 + height, x0:x0 + width]
        scale = float(detection["processing_scale"])
        if scale < 1.0:
            working = cv2.resize(cropped, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        else:
            working = cropped
        masks = make_all_color_masks(working, self.color_config)
        open_size = self._scaled_kernel(int(detection["open_kernel_px"]), scale)
        close_size = self._scaled_kernel(int(detection["close_kernel_px"]), scale)
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))

        candidates = []
        scale_area = scale * scale
        for color_id, raw_mask in masks.items():
            mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                working_area = float(cv2.contourArea(contour))
                area = working_area / scale_area
                if not (float(detection["minimum_area_px"]) <= area <= float(detection["maximum_area_px"])):
                    continue
                bx, by, bw, bh = cv2.boundingRect(contour)
                full_width, full_height = bw / scale, bh / scale
                if (min(full_width, full_height) < float(detection["minimum_dimension_px"])
                    or max(full_width, full_height) > float(detection["maximum_dimension_px"])):
                    continue
                aspect = max(full_width, full_height) / max(1.0, min(full_width, full_height))
                if aspect > float(detection["maximum_aspect_ratio"]):
                    continue
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                if hull_area <= 0.0: continue
                solidity = working_area / hull_area
                extent = working_area / max(1.0, float(bw * bh))
                if solidity < float(detection["minimum_solidity"]) or extent < float(detection["minimum_extent"]):
                    continue
                moments = cv2.moments(contour)
                if abs(moments["m00"]) < 1e-6: continue
                local_x = float(moments["m10"] / moments["m00"]) / scale
                local_y = float(moments["m01"] / moments["m00"]) / scale
                center_x, center_y = local_x + x0, local_y + y0
                full_contour = np.rint(contour.astype(np.float32) / scale).astype(np.int32)
                full_contour += np.asarray([[[x0, y0]]], dtype=np.int32)
                confidence = max(0.0, min(1.0, 0.55 * solidity + 0.45 * min(1.0, extent / 0.75)))
                candidates.append(MaterialRegion(
                    color_id=int(color_id), name=COLOR_NAMES[int(color_id)],
                    center_x_px=center_x, center_y_px=center_y,
                    area_px=area, confidence=confidence,
                    bounding_box=(
                        int(round(bx / scale + x0)), int(round(by / scale + y0)),
                        int(round(full_width)), int(round(full_height)),
                    ),
                    contour=full_contour,
                ))
        return tuple(sorted(candidates, key=lambda item: (item.color_id, item.center_x_px, item.center_y_px)))

    def _assign_slots(self, regions: tuple[MaterialRegion, ...]) -> tuple[tuple[int, ...] | None, str]:
        # 单区模式下，只需要判断是不是唯一的物料。
        if not regions:
            return None, "区域内无物料"
        if len(regions) > 1:
            return None, "工作区内检测到多件物料，要求单物料"
        return (regions[0].color_id,), "目标明确锁定"

    def _match_speed(self, current: tuple[MaterialRegion, ...], interval_s: float) -> float | None:
        previous = self._previous_regions
        if len(current) != len(previous) or not current:
            return None
        maximum_distance = float(self.config["detection"]["maximum_match_distance_px"])
        best_distances = None
        best_total = math.inf
        for order in itertools.permutations(range(len(current))):
            distances = []
            valid = True
            for old_index, new_index in enumerate(order):
                old, new = previous[old_index], current[new_index]
                if old.color_id != new.color_id:
                    valid = False
                    break
                distance = math.hypot(new.center_x_px - old.center_x_px, new.center_y_px - old.center_y_px)
                if distance > maximum_distance:
                    valid = False
                    break
                distances.append(distance)
            total = sum(distances)
            if valid and total < best_total:
                best_total = total
                best_distances = distances
        if best_distances is None:
            return None
        return max(best_distances, default=0.0) / interval_s

    def _clear_tracking_baseline(self, timestamp_s):
        self._previous_regions = ()
        self._previous_timestamp_s = timestamp_s
        self._previous_slots = None
        self._moving_frames = 0
        self._low_speed_frames = 0

    def _update_motion(self, regions, slots, assignment_detail, timestamp_s):
        motion = self.config["motion"]
        if self._previous_timestamp_s is None:
            self._moving_frames = 0
            self._low_speed_frames = 0
            return 0.0, "WAITING_FOR_MOTION", "已建立跟踪基线，等待运动"

        interval_s = timestamp_s - self._previous_timestamp_s
        baseline_state = "ACQUIRING" if self._motion_confirmed else "WAITING_FOR_MOTION"
        if not (float(motion["minimum_frame_interval_s"]) <= interval_s <= float(motion["maximum_frame_interval_s"])):
            self._moving_frames = 0
            self._low_speed_frames = 0
            return 0.0, baseline_state, f"帧间隔{interval_s:.3f}s无效，已重建基线"

        matched_speed = self._match_speed(regions, interval_s)
        if matched_speed is None:
            self._moving_frames = 0
            self._low_speed_frames = 0
            return 0.0, baseline_state, "物料数量或颜色变化，重建测速基线"

        speed = float(matched_speed)
        moving_threshold = float(motion["moving_speed_px_s"])
        near_stop_threshold = float(motion["near_stop_speed_px_s"])
        if speed >= moving_threshold:
            self._moving_frames += 1
            self._low_speed_frames = 0
            required_moving = int(motion["required_moving_frames"])
            if self._moving_frames >= required_moving:
                self._motion_confirmed = True
                self._moving_frames = required_moving
                return speed, "MOVING", f"已确认移动，速度{speed:.1f}px/s，等待停稳"
            return speed, "CONFIRMING_MOTION", f"运动证据{self._moving_frames}/{required_moving}，速度{speed:.1f}px/s"

        if not self._motion_confirmed:
            self._moving_frames = 0
            self._low_speed_frames = 0
            return speed, "WAITING_FOR_MOTION", f"当前{speed:.1f}px/s；尚未观察到移动"
        if speed > near_stop_threshold:
            self._low_speed_frames = 0
            return speed, "SETTLING", f"正在减速，当前{speed:.1f}px/s"
        if slots is None:
            self._low_speed_frames = 0
            return speed, "POSITIONING", assignment_detail
        if slots != self._previous_slots:
            self._low_speed_frames = 0
            return speed, "POSITIONING", "物料发生突变或更换，重新确认"

        self._low_speed_frames += 1
        return speed, "NEAR_STOP", f"低速证据{self._low_speed_frames}/{int(motion['required_low_speed_frames'])}"

    def detect(self, frame: np.ndarray, timestamp: float | None = None) -> MaterialPickupDetection:
        started_s = time.perf_counter()
        if not bool(self.config.get("calibrated")):
            self.reset()
            return self._empty_result(started_s, "UNCALIBRATED", "请先标定工作区", calibrated=False)
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise MaterialPickupError("物料识别要求BGR三通道图像")
        expected = (int(self.config["image_width"]), int(self.config["image_height"]))
        if (frame.shape[1], frame.shape[0]) != expected:
            raise MaterialPickupError(f"分辨率不匹配，应为 {expected[0]}x{expected[1]}")
        timestamp_s = time.monotonic() if timestamp is None else float(timestamp)

        regions = self._find_regions(frame)
        maximum_count = int(self.config["detection"]["maximum_material_count"])
        if not regions:
            self._clear_tracking_baseline(timestamp_s)
            return self._empty_result(started_s, "SEARCHING", "区内无物料")
        if len(regions) > maximum_count:
            self._clear_tracking_baseline(timestamp_s)
            return self._empty_result(started_s, "AMBIGUOUS", f"检测到{len(regions)}候选物，单点模式仅支持1件", regions=regions)

        slots, assignment_detail = self._assign_slots(regions)
        speed, state, detail = self._update_motion(regions, slots, assignment_detail, timestamp_s)
        motion = self.config["motion"]

        self._previous_regions = regions
        self._previous_timestamp_s = timestamp_s
        self._previous_slots = slots
        required = int(motion["required_low_speed_frames"])
        ready = bool(self._motion_confirmed and slots is not None and self._low_speed_frames >= required)
        
        if ready:
            state = "READY"
            detail = "物料已接近停稳且目标锁定，可上报一次"
        return MaterialPickupDetection(
            detected=True, ready=ready, calibrated=True, state=state,
            material_count=len(regions),
            slot_colors=slots if slots is not None else (0,),
            max_speed_px_s=float(speed),
            motion_confirmed=self._motion_confirmed,
            moving_frames=self._moving_frames,
            required_moving_frames=int(motion["required_moving_frames"]),
            low_speed_frames=self._low_speed_frames,
            required_low_speed_frames=required,
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            detail=detail, regions=regions,
        )


def draw_material_pickup_detection(
    frame: np.ndarray, result: MaterialPickupDetection, config: dict
) -> np.ndarray:
    """在预览中动态绘制标定区域和物料轮廓。"""
    
    if bool(config.get("calibrated")):
        for name in config.get("regions", {}).keys():
            rectangle = config["regions"][name]
            first = (rectangle["x"], rectangle["y"])
            second = (rectangle["x"] + rectangle["width"], rectangle["y"] + rectangle["height"])
            color = (0, 220, 255)
            cv2.rectangle(frame, first, second, color, 2)
            cv2.putText(frame, name.upper(), (first[0] + 4, first[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            
    for region in result.regions:
        cv2.drawContours(frame, [region.contour], -1, (0, 255, 0), 2)
        x_value, y_value, width, height = region.bounding_box
        cv2.rectangle(frame, (x_value, y_value), (x_value + width, y_value + height), (0, 255, 0), 1)
        cv2.putText(frame, f"{region.color_id}:{region.name}",
                    (x_value, max(18, y_value - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 2, cv2.LINE_AA)
                    
    status = (
        f"State: {result.state}  Count={result.material_count}  "
        f"Target={result.slot_colors}  Speed={result.max_speed_px_s:.1f}px/s  "
        f"Move={result.moving_frames}/{result.required_moving_frames}  "
        f"Low={result.low_speed_frames}/{result.required_low_speed_frames}"
    )
    cv2.rectangle(frame, (10, 62), (1060, 100), (0, 0, 0), -1)
    cv2.putText(frame, status, (20, 89), cv2.FONT_HERSHEY_SIMPLEX,
                0.63, (255, 255, 255), 2, cv2.LINE_AA)
    return frame