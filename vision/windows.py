"""无串口离线调试窗口：复用正式视觉模块，用键盘切换任务并叠加中心准星。

"""

import time
import cv2

# 导入各个视觉模块
import yuanhuan as yh
import yuanhuan2 as yh2  # 新增：引入码垛环识别模块
import wuliaopan as zp  # 转盘封装
import yanse as ys      # 引入封装好的颜色模块

def main():
    """离线调试入口：初始化各检测器，用键盘命令切换任务并显示中心准星。"""
    print("Beginning (无串口离线调试模式 + 中心准星)")
    response = b'\x0A'  # 默认启动时进入三圆环模式
    last_response = None
    run_text = ''
    res_text = ''

    yuanhuan_start = b'\x0A'
    yanse_start = b'\x0B'
    zhuanpan_start = b'\x0C'
    yuanhuanerlun_start = b'\x0D'
    shuzi_start = b'\x10'
    xiuxian_start = b'\x11'
    
    color_map = {0: "Empty", 1: "Red", 2: "Yellow", 3: "Blue", 4: "Green", 5: "Black", 6: "L-Blue"}
    
    # 1. 初始化转盘检测器 & 相机畸变矫正表
    calibration_file = "config/realtek_rgb_camera.yaml"
    turntable_config_file = "config/turntable_detector.json"
    
    try:
        zp_detector, zp_map1, zp_map2 = zp.init_zhuanpan_detector(calibration_file, turntable_config_file)
        zp_ready = True
    except Exception as e:
        print(f"转盘及标定初始化失败: {e}")
        zp_ready = False
        zp_detector, zp_map1, zp_map2 = None, None, None

    # 2. 初始化颜色物料检测器
    material_detector = ys.init_yanse_detector("config/material_pickup_detector.json", "config/material_color_thresholds.json")
    material_ready = (material_detector is not None)

    # 3. 初始化三圆环工位检测器
    ring_config_file = "config/ring_station_detector.json"
    ring_config = None
    try:
        ring_config = yh.load_ring_station_config(ring_config_file)
        ring_detector = yh.RingStationDetector(ring_config)
        ring_ready = True
        print("三圆环工位检测器初始化成功。")
    except Exception as e:
        print(f"三圆环检测器初始化失败: {e}")
        ring_ready = False
        ring_detector = None

    # 4. 初始化码垛环工位检测器 (二轮)
    stacked_config_file = "config/stacked_ring_detector.json"
    try:
        if ring_config is not None:
            stacked_config = yh2.load_stacked_ring_config(stacked_config_file)
            stacked_detector = yh2.StackedRingDetector(stacked_config, ring_config)
            stacked_ready = True
            print("码垛环检测器初始化成功。")
        else:
            raise Exception("缺少基础三圆环配置")
    except Exception as e:
        print(f"码垛环检测器初始化失败: {e}")
        stacked_ready = False
        stacked_detector = None

    lst = time.time()

    # 初始化 USB 摄像头 (Windows 环境兼容写法，但补齐了抗延迟参数)
    cap_usb = cv2.VideoCapture(0)
    cap_usb.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 强制清除积压帧
    cap_usb.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # 启用 MJPG 高帧率
    cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap_usb.set(cv2.CAP_PROP_FPS, 30)
    
    print("\n--- 快捷键提示 ---")
    print("按 [1]: 三圆环工位1 | 按 [2]: 颜色识别 | 按 [3]: 转盘检测")
    print("按 [4]: 三圆环工位2 | 按 [5]: 数字识别 | 按 [0]: 闲置模式 | 按 [q]: 退出\n")
    
    while True:
        ret_usb, frame_usb = cap_usb.read()
        
        if not ret_usb or frame_usb is None:
            time.sleep(0.01)
            continue
            
        img_usb = frame_usb.copy()
            
        st = time.time()
        diff = st - lst
        run_t = int(1 / diff) if diff > 0.005 else 0
        lst = st

        # 状态切换时彻底清理所有历史缓存
        if response != last_response:
            print(f"状态切换: {last_response} -> {response}，重置检测器状态")
            if zp_detector is not None: zp_detector.reset()
            if ring_detector is not None: ring_detector.reset()
            if stacked_detector is not None: stacked_detector.reset()
            # 强制清空颜色检测记忆，防止秒锁 Bug
            ys._last_color = 0
            ys._color_start_time = 0.0
            last_response = response
        
        if response == xiuxian_start:
            run_text = "xiuxian_start"
            res_text = "Idle"
                
        # --- 任务一：基础三圆环 ---
        elif response == yuanhuan_start:
            run_text = "yuanhuan_1"
            try:
                if zp_ready and zp_map1 is not None and zp_map2 is not None:
                    img_usb = cv2.remap(img_usb, zp_map1, zp_map2, cv2.INTER_LINEAR)

                if ring_ready and ring_detector is not None:
                    result = ring_detector.detect(img_usb)
                    yh.draw_ring_station_detection(img_usb, result) 
                    
                    if result.detected and result.stable:
                        # 基础三圆环中心坐标提取
                        mid_center_x, mid_center_y = result.centers_px[1]
                        dx_mm, dy_mm = result.error_x_mm, result.error_y_mm
                        angle_val = result.angle_error_deg 
                        
                        dir_x = 1 if dx_mm >= 0 else 0
                        dir_y = 1 if dy_mm >= 0 else 0
                        abs_dx = min(255, int(round(abs(dx_mm))))
                        abs_dy = min(255, int(round(abs(dy_mm))))
                        dir_angle = 1 if angle_val >= 0 else 0
                        abs_angle = min(255, int(round(abs(angle_val * 10))))
                        
                        res_text = f"MidX:{mid_center_x:.0f} MidY:{mid_center_y:.0f} YawErr:{angle_val:.1f}° [STABLE]"
                        
                        # 模拟打包数据包 (定长 8 字节)
                        payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle, 0xEF])
                        
                    elif result.detected:
                        res_text = f"Waiting Stable... [WAIT]"
                    else:
                        res_text = "No Circle"
                else:
                    res_text = "Ring Not Init"
            except Exception as e:
                res_text = "Yuanhuan Error"
                print(f"三圆环处理出错: {e}")

        # --- 任务二：码垛环识别 ---
        elif response == yuanhuanerlun_start:
            run_text = "yuanhuan_2"
            try:
                if zp_ready and zp_map1 is not None and zp_map2 is not None:
                    img_usb = cv2.remap(img_usb, zp_map1, zp_map2, cv2.INTER_LINEAR)

                if stacked_ready and stacked_detector is not None:
                    result = stacked_detector.detect(img_usb)
                    yh2.draw_stacked_ring_detection(img_usb, result) 
                    
                    if result.detected and result.stable:
                        # 码垛环的中心属性名是 middle_center_px
                        mid_center_x, mid_center_y = result.middle_center_px 
                        dx_mm, dy_mm = result.error_x_mm, result.error_y_mm
                        angle_val = result.angle_error_deg 
                        
                        dir_x = 1 if dx_mm >= 0 else 0
                        dir_y = 1 if dy_mm >= 0 else 0
                        abs_dx = min(255, int(round(abs(dx_mm))))
                        abs_dy = min(255, int(round(abs(dy_mm))))
                        dir_angle = 1 if angle_val >= 0 else 0
                        abs_angle = min(255, int(round(abs(angle_val * 10))))
                        
                        res_text = f"MidX:{mid_center_x:.0f} MidY:{mid_center_y:.0f} YawErr:{angle_val:.1f}° [STABLE]"
                        
                        # 模拟打包数据包 (定长 8 字节)
                        payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle, 0xEF])
                        
                    elif result.detected:
                        res_text = f"Waiting Stable... [WAIT]"
                    else:
                        res_text = "No Stacked Ring"
                else:
                    res_text = "Stacked Ring Not Init"
            except Exception as e:
                res_text = "Stacked Ring Error"
                print(f"码垛环处理出错: {e}")
                
        # --- 任务三：转盘外圆识别 ---
        elif response == zhuanpan_start:
            run_text = "zhuanpan_start"
            if zp_ready:
                try:
                    is_found, is_stable, dx_mm, dy_mm, processed_frame = zp.process_zhuanpan(
                        img_usb, zp_detector, zp_map1, zp_map2
                    )
                    
                    if is_found:
                        if is_stable:
                            dir_x = 1 if dx_mm >= 0 else 0
                            dir_y = 1 if dy_mm >= 0 else 0
                            abs_dx = min(255, int(round(abs(dx_mm))))
                            abs_dy = min(255, int(round(abs(dy_mm))))
                            
                            res_text = f"dX:{dx_mm:.0f} dY:{dy_mm:.0f} [STABLE]"
                            # 补齐到 8 字节
                            payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy, 0x00, 0x00, 0xEF])
                        else:
                            res_text = f"dX:{dx_mm:.0f} dY:{dy_mm:.0f} [WAIT]"
                    else:
                        res_text = "No Turntable"
                        
                    img_usb = processed_frame
                except Exception as e:
                    res_text = "Zhuanpan Error"
            else:
                res_text = "zp Not Init"

        # --- 任务四：颜色识别 ---
        elif response == yanse_start:
            run_text = "yanse_start"
            if material_ready and zp_ready:
                try:
                    is_stable, current_color, img_usb = ys.process_yanse(img_usb, material_detector, zp_map1, zp_map2)
                    
                    if is_stable and current_color != 0:
                        # 补齐到 8 字节
                        payload = bytes([0xFE, current_color & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF])
                        name_l = color_map.get(current_color, str(current_color))
                        res_text = f"C:{name_l} [DETECTED]"
                    else:
                        state_str = "Empty" if current_color == 0 else color_map.get(current_color, str(current_color))
                        res_text = f"{state_str} [WAIT]"
                        
                except Exception as e:
                    res_text = "Yanse Error"
                    print(f"处理颜色出错: {e}")
            else:
                res_text = "Yanse/Cam Not Init"

        elif response == shuzi_start:
            run_text = "shuzi_start"
            res_text = "Shuzi Mode (TODO)"
                
        elif response is None:
            run_text = "Wait Command"
            res_text = "Idle"

        # 绘制状态文本
        cv2.putText(img_usb, f"RUN:{run_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(img_usb, f"RES:{res_text}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(img_usb, f"FPS:{run_t}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # === 在图像正中心（1280x720 的中心点 640, 360）绘制准星和红点 ===
        center_x, center_x_y = 1280 // 2, 720 // 2
        cv2.line(img_usb, (center_x - 20, center_x_y), (center_x + 20, center_x_y), (0, 255, 0), 2)
        cv2.line(img_usb, (center_x, center_x_y - 20), (center_x, center_x_y + 20), (0, 255, 0), 2)
        cv2.circle(img_usb, (center_x, center_x_y), 4, (0, 0, 255), -1)
        # ==========================================================

        display_img = cv2.resize(img_usb, (640, 360))
        cv2.imshow('Main Vision Terminal', display_img)

        # 键盘按键捕获（用于切换模式）
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break
        elif key == ord('1'):
            response = yuanhuan_start
            print("手动切换 -> 三圆环工位 1")
        elif key == ord('2'):
            response = yanse_start
            print("手动切换 -> 颜色物料检测")
        elif key == ord('3'):
            response = zhuanpan_start
            print("手动切换 -> 转盘检测")
        elif key == ord('4'):
            response = yuanhuanerlun_start
            print("手动切换 -> 三圆环工位 2 (码垛环)")
        elif key == ord('5'):
            response = shuzi_start
            print("手动切换 -> 数字识别")
        elif key == ord('0'):
            response = xiuxian_start
            print("手动切换 -> 闲置模式")

    cap_usb.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()