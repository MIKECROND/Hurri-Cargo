"""数字识别：基于合成模板与现场自定义模板的混合匹配。

"""

import os
import cv2
import numpy as np

class HybridTemplateRecognizer:
    def __init__(self, template_dir="custom_templates"):
        self.template_dir = template_dir
        os.makedirs(self.template_dir, exist_ok=True)

        self.synth_templates = {i: [] for i in range(10)}
        self.custom_templates = {i: [] for i in range(10)}

        self._build_synth_templates()
        self._load_custom_templates()

    def normalize_roi(self, roi_thresh):
        """把二值 ROI 居中缩放为固定尺寸模板，保持原有留白比例。"""
        coords = cv2.findNonZero(roi_thresh)
        if coords is None:
            return np.zeros((28, 28), dtype=np.uint8)
        x, y, w, h = cv2.boundingRect(coords)
        tight = roi_thresh[y:y+h, x:x+w]

        max_dim = int(max(w, h) * 1.25)
        if max_dim == 0:
            return np.zeros((28, 28), dtype=np.uint8)
        square = np.zeros((max_dim, max_dim), dtype=np.uint8)
        y_off = (max_dim - h) // 2
        x_off = (max_dim - w) // 2
        square[y_off:y_off+h, x_off:x_off+w] = tight

        return cv2.resize(square, (28, 28), interpolation=cv2.INTER_AREA)

    def _build_synth_templates(self):
        fonts = [
            cv2.FONT_HERSHEY_SIMPLEX,
            cv2.FONT_HERSHEY_DUPLEX,
            cv2.FONT_HERSHEY_COMPLEX
        ]
        for digit in range(10):
            for font in fonts:
                for thickness in [3, 5]:
                    canvas = np.zeros((100, 100), dtype=np.uint8)
                    cv2.putText(canvas, str(digit), (18, 78), font, 2.6, 255, thickness, cv2.LINE_AA)
                    _, binary = cv2.threshold(canvas, 127, 255, cv2.THRESH_BINARY)
                    for angle in [-5, 0, 5]:
                        rotation_matrix = cv2.getRotationMatrix2D((50, 50), angle, 1.0)
                        rotated = cv2.warpAffine(binary, rotation_matrix, (100, 100))
                        self.synth_templates[digit].append(self.normalize_roi(rotated))

    def _load_custom_templates(self):
        count = 0
        for filename in os.listdir(self.template_dir):
            if filename.endswith(".png") and "_" in filename:
                try:
                    digit = int(filename.split("_")[0])
                    img_path = os.path.join(self.template_dir, filename)
                    img = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                    if img is not None and img.shape == (28, 28):
                        self.custom_templates[digit].append(img)
                        count += 1
                except Exception:
                    pass
        print(f"已加载现场自定义模板: {count} 个")

    def save_custom_template(self, digit, norm_28):
        """把当前有效数字画面保存为该数字的现场模板。"""
        if np.count_nonzero(norm_28) < 20:
            print("当前没有检测到有效数字画面，无法保存模板！")
            return
        idx = len(self.custom_templates[digit]) + 1
        fname = f"{digit}_{idx}.png"
        save_path = os.path.join(self.template_dir, fname)
        cv2.imencode('.png', norm_28)[1].tofile(save_path)
        self.custom_templates[digit].append(norm_28.copy())
        print(f"[录制成功] 已将当前数字保存为数字 【{digit}】 的专属模板 ({fname})！")

    def _count_holes(self, roi_thresh):
        padded = cv2.copyMakeBorder(roi_thresh, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=0)
        contours, hierarchy = cv2.findContours(padded, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return 0
        holes = 0
        for i in range(len(contours)):
            if hierarchy[0][i][3] != -1 and cv2.contourArea(contours[i]) > 15:
                holes += 1
        return holes

    def predict(self, roi_thresh, orig_w, orig_h):
        """按宽高比、模板匹配得分与孔洞数给出数字和置信度。"""
        norm_img = self.normalize_roi(roi_thresh)
        aspect_ratio = orig_w / float(orig_h)

        if aspect_ratio < 0.32:
            return 1, 0.99, norm_img

        scores = np.zeros(10, dtype=np.float32)

        for digit in range(10):
            best_synth = 0.0
            for tmpl in self.synth_templates[digit]:
                tmpl_crop = tmpl[2:26, 2:26]
                res = cv2.matchTemplate(norm_img, tmpl_crop, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(res)
                if max_val > best_synth:
                    best_synth = max_val

            best_custom = 0.0
            for tmpl in self.custom_templates[digit]:
                tmpl_crop = tmpl[2:26, 2:26]
                res = cv2.matchTemplate(norm_img, tmpl_crop, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(res)
                if max_val > best_custom:
                    best_custom = max_val

            if best_custom > 0:
                scores[digit] = max(best_synth, best_custom * 1.25)
            else:
                scores[digit] = best_synth

        holes = self._count_holes(roi_thresh)
        if holes == 2:
            scores[8] += 0.18
        elif holes == 1:
            for candidate_digit in [0, 4, 6, 9]:
                scores[candidate_digit] += 0.08
        elif holes == 0:
            for candidate_digit in [1, 2, 3, 5, 7]:
                scores[candidate_digit] += 0.08

        best_digit = int(np.argmax(scores))
        confidence = float(min(0.99, max(0.0, scores[best_digit])))
        return best_digit, confidence, norm_img

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    template_dir = os.path.join(base_dir, "custom_templates")
    recognizer = HybridTemplateRecognizer(template_dir)
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("无法打开摄像头！")
        return

    ret, init_frame = cap.read()
    if not ret:
        print("无法获取摄像头画面进行标定！")
        return

    print("\n" + "="*50)
    print("【单区域自主标定模式】")
    print("请用鼠标在画面上框出你想要识别的目标数字区域：")
    print("（操作方法：按住鼠标左键拖动画框，选好后按键盘 【Enter】 或 【Space】 确认）")
    print("="*50 + "\n")

    bx, by, bw, bh = cv2.selectROI("Select Target Box", init_frame, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Select Target Box")

    if bw == 0 or bh == 0:
        print("标定已取消或无效！")
        return

    print(f"标定完成！筛选框坐标: x={bx}, y={by}, w={bw}, h={bh}")
    print("进入实时识别主循环... 按 'q' 键退出，按键盘数字键 0-9 可为当前帧录制模板。")

    last_preview_28 = np.zeros((28, 28), dtype=np.uint8)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh_global = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh_global = cv2.morphologyEx(thresh_global, cv2.MORPH_OPEN, kernel)
        thresh_global = cv2.morphologyEx(thresh_global, cv2.MORPH_CLOSE, kernel)

        # 绘制用户标定的单筛选框
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (255, 150, 0), 2)
        cv2.putText(frame, "Target ROI", (bx, by - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 0), 2)

        box_thresh = thresh_global[by:by+bh, bx:bx+bw]
        contours, _ = cv2.findContours(box_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        recognized_str = ""
        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_cnt)
            
            if w * h > 150 and h > 20:
                digit_roi = box_thresh[y:y+h, x:x+w]
                digit, conf, norm_28 = recognizer.predict(digit_roi, w, h)
                
                if conf > 0.40:
                    recognized_str = str(digit)
                    last_preview_28 = norm_28
                    cv2.rectangle(frame, (bx + x, by + y), (bx + x + w, by + y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"{digit} ({conf:.2f})", (bx + x, by + y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        # 左上角显示特征图
        preview_large = cv2.resize(last_preview_28, (112, 112), interpolation=cv2.INTER_NEAREST)
        frame[10:122, 10:122] = cv2.cvtColor(preview_large, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(frame, (10, 10), (122, 122), (0, 255, 255), 1)
        cv2.putText(frame, "Current ROI", (16, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        if recognized_str:
            cv2.putText(frame, f"Result: {recognized_str}", (150, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.imshow("Single-Box Digit Recognition", frame)
        cv2.imshow("Global Otsu Threshold", thresh_global)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif ord('0') <= key <= ord('9'):
            target_digit = key - ord('0')
            recognizer.save_custom_template(target_digit, last_preview_28)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()