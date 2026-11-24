"""视觉任务主程序：接收串口命令并调度圆环、转盘、颜色和数字识别。

"""

import time
import threading
from threading import Thread
import cv2
import serial
import yuanhuan as yh
import yuanhuan2 as yh2  
import wuliaopan as zp  
import yanse as ys      

# 全局变量声明
ser = None
response = None          

# 串口操作的线程锁，防止发送和接收冲突
ser_lock = threading.Lock()


def race():
    """后台线程：重连串口、读取命令字节并更新全局响应。"""
    global ser, response
    valid_commands = [b'\xA1', b'\xB1', b'\xC1', b'\xD1', b'\x10', b'\x11']
    while True:
        try:
            # 1. 检查并重连串口
            need_open = False
            with ser_lock:
                if ser is None or not ser.is_open:
                    need_open = True

            if need_open:
                new_ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.1)
                time.sleep(2)
                with ser_lock:
                    ser = new_ser

            # 2. 非阻塞检查缓冲区并读取指令
            with ser_lock:
                if ser is not None and ser.is_open and ser.in_waiting > 0:
                    data_chunk = ser.read(ser.in_waiting)
                    valid_found = None
                    for byte_val in data_chunk:
                        b = bytes([byte_val])
                        if b in valid_commands:
                            valid_found = b
                    
                    if valid_found:
                        response = valid_found
                        print(f"到达新任务点, 收到新的指令 {valid_found}")

            time.sleep(0.005)  
                        
        except Exception as e:
            print(f"串口异常: {e}")
            with ser_lock:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
            time.sleep(1)


def send_data_to_mcu(data_bytes):
    """在串口锁内发送一帧数据；串口不可用时静默跳过。"""
    global ser
    with ser_lock:
        if ser is not None and ser.is_open:
            try:
                ser.write(data_bytes)
            except Exception as e:
                print(f"发送数据失败: {e}")


def _draw_status_overlay(frame, run_text, res_text, run_t):
    """在主画面左上角叠加运行态、识别结果和帧率文本。"""
    cv2.putText(frame, f"RUN:{run_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(frame, f"RES:{res_text}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(frame, f"FPS:{run_t}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)


def main():
    """初始化各检测器并进入主循环，按当前命令分派视觉任务。"""
    global response
    print("Beginning")
    response = None
    last_response = None
    run_text = ''
    res_text = ''

    last_send_time = 0.0
    SEND_INTERVAL_S = 0.10  

    yuanhuan_start = b'\xA1'
    yanse_start = b'\xB1'
    zhuanpan_start = b'\xC1'
    yuanhuanerlun_start = b'\xD1'
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

    cap_usb = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap_usb.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap_usb.set(cv2.CAP_PROP_FPS, 30)
    
    Thread(target=race, daemon=True).start()
    
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
            ys._last_color = 0
            ys._color_start_time = 0.0
            last_response = response
        
        if response == xiuxian_start:
            run_text = "xiuxian_start"
            res_text = "Idle"
                
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
                        
                        res_text = f"MidX:{mid_center_x:.0f} MidY:{mid_center_y:.0f} YawErr:{angle_val:.1f}° [SEND]"
                        
                        if st - last_send_time >= SEND_INTERVAL_S:
                            payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle, 0xEF])
                            send_data_to_mcu(payload)
                            last_send_time = st
                    elif result.detected:
                        res_text = f"Waiting Stable... [WAIT]"
                    else:
                        res_text = "No Circle"
                else:
                    res_text = "Ring Not Init"
            except Exception as e:
                res_text = "Yuanhuan Error"
                print(f"三圆环处理出错: {e}")

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
                        
                        res_text = f"MidX:{mid_center_x:.0f} MidY:{mid_center_y:.0f} YawErr:{angle_val:.1f}° [SEND]"
                        
                        if st - last_send_time >= SEND_INTERVAL_S:
                            # 8 字节定长指令包 (与一轮圆环结构一致)
                            payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy, dir_angle, abs_angle, 0xEF])
                            send_data_to_mcu(payload)
                            last_send_time = st
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
                            
                            res_text = f"dX:{dx_mm:.0f} dY:{dy_mm:.0f} [SEND]"
                            if st - last_send_time >= SEND_INTERVAL_S:
                                # 填充 0x00 对齐到 8 字节定长数据包
                                payload = bytes([0xFE, dir_x, dir_y, abs_dx, abs_dy,0xEF])
                                send_data_to_mcu(payload)
                                last_send_time = st
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
                        if st - last_send_time >= SEND_INTERVAL_S:
                            # 填充 0x00 对齐到 8 字节定长数据包
                            payload = bytes([0xFE, current_color & 0xFF,0xEF])
                            send_data_to_mcu(payload)
                            last_send_time = st
                            
                        name_l = color_map.get(current_color, str(current_color))
                        res_text = f"C:{name_l} [SEND]"
                    else:
                        state_str = "Empty" if current_color == 0 else color_map.get(current_color, str(current_color))
                        res_text = f"{state_str} [Locking]"
                        
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

        _draw_status_overlay(img_usb, run_text, res_text, run_t)
        
        display_img = cv2.resize(img_usb, (640, 360))
        cv2.imshow('Main Vision Terminal', display_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_usb.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()