"""转盘检测封装：加载相机标定、生成畸变映射并计算像素到毫米偏差。

"""

import cv2
from pathlib import Path

# 导入转盘检测模块
from turntable_detector import (
    load_turntable_detector_config,
    TurntableDetector,
    draw_turntable_detection
)

# 转盘实际物理半径 (300mm直径的转盘，半径应为150.0mm；若实际半径为300mm请改回300.0)
REAL_TURNTABLE_RADIUS_MM = 150.0  


def load_calibration(path):
    """加载相机的标定内参和畸变系数"""
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise RuntimeError(f"标定文件不存在：{resolved}")

    try:
        yaml_text = resolved.read_text(encoding="utf-8")
    except Exception as e:
        raise RuntimeError(f"无法读取标定文件文本：{e}")

    storage = cv2.FileStorage(yaml_text, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
    if not storage.isOpened():
        raise RuntimeError(f"无法解析标定文件内容：{resolved}")
        
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
        new_camera_matrix = storage.getNode("new_camera_matrix").mat()
    finally:
        storage.release()
        
    if camera_matrix is None or distortion is None or new_camera_matrix is None:
        raise RuntimeError("标定文件缺少必要的矩阵参数")
        
    return (width, height), camera_matrix, distortion, new_camera_matrix


def init_zhuanpan_detector(calibration_file, turntable_config_file):
    """
    初始化转盘检测相关的参数和对象。
    返回: detector (检测器对象), map1, map2 (畸变映射表)
    """
    # 1. 加载标定参数
    size, camera_matrix, distortion, new_camera_matrix = load_calibration(calibration_file)
    print(f"转盘标定参数加载成功，适用分辨率: {size[0]}x{size[1]}")

    # 2. 预先计算畸变映射表
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, distortion, None, new_camera_matrix, size, cv2.CV_16SC2
    )

    # 3. 加载圆盘检测配置并初始化检测器
    print("正在加载转盘检测配置...")
    turntable_config = load_turntable_detector_config(turntable_config_file)
    detector = TurntableDetector(turntable_config)
    print("圆盘定位器初始化完成。")
    
    return detector, map1, map2


def process_zhuanpan(raw_frame, detector, map1, map2):
    """
    处理单帧画面，进行畸变矫正、转盘检测和计算实际距离。
    返回: 
        is_found (bool): 是否找到转盘
        is_stable (bool): 连续多帧观测是否已稳定
        dx_mm (float): X方向实际偏差(毫米)
        dy_mm (float): Y方向实际偏差(毫米)
        processed_frame (ndarray): 画好检测框的画面
    """
    # 1. 执行畸变矫正
    frame_rect = cv2.remap(raw_frame, map1, map2, cv2.INTER_LINEAR)

    if detector is None:
        return False, False, 0.0, 0.0, frame_rect

    # 2. 执行转盘外圆检测算法
    result = detector.detect(frame_rect)

    # 3. 解析结果并计算距离 (修复了原有的 result.center / result.radius 属性名错误)
    if result is not None and result.detected and result.radius_px > 0:
        height, width = frame_rect.shape[:2]
        cam_center_x = width / 2.0
        cam_center_y = height / 2.0

        obj_center_x = result.center_x_px
        obj_center_y = result.center_y_px

        # 计算像素偏差 (以相机中心为原点，与 yuanhuan.py 坐标系方向保持一致)
        dx_pixel = obj_center_x - cam_center_x
        dy_pixel = cam_center_y - obj_center_y

        # 计算 像素 到 毫米 的比例 (通过 实际半径 / 像素半径 得出)
        mm_per_pixel = REAL_TURNTABLE_RADIUS_MM / result.radius_px

        # 计算实际毫米偏差
        dx_mm = dx_pixel * mm_per_pixel
        dy_mm = dy_pixel * mm_per_pixel
        
        # 在画面上把找到的圆盘画出来
        draw_turntable_detection(frame_rect, result)
        
        return True, result.stable, dx_mm, dy_mm, frame_rect
    else:
        draw_turntable_detection(frame_rect, result)
        return False, False, 0.0, 0.0, frame_rect


# 保留独立运行的入口，用于调试
if __name__ == "__main__":
    device = 0
    calibration_file = "config/realtek_rgb_camera.yaml"
    turntable_config_file = "config/turntable_detector.json"
    fps = 30.0

    try:
        detector, map1, map2 = init_zhuanpan_detector(calibration_file, turntable_config_file)
        size, _, _, _ = load_calibration(calibration_file)
        
        camera = cv2.VideoCapture(device)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        camera.set(cv2.CAP_PROP_FPS, fps)
        
        cv2.namedWindow("Turntable Debug", cv2.WINDOW_NORMAL)

        while True:
            ok, raw_frame = camera.read()
            if not ok or raw_frame is None:
                break

            is_found, is_stable, dx, dy, debug_frame = process_zhuanpan(raw_frame, detector, map1, map2)
            
            if is_found:
                state_str = "STABLE" if is_stable else "WAIT"
                print(f"独立测试 [{state_str}]: dX={dx:.2f}mm, dY={dy:.2f}mm")

            cv2.imshow("Turntable Debug", debug_frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                break
    finally:
        if 'camera' in locals() and camera is not None:
            camera.release()
        cv2.destroyAllWindows()