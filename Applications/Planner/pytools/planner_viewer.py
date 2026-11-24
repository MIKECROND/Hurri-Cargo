import re
import tkinter as tk
from tkinter import scrolledtext, messagebox
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def parse_coordinates(text):
    pattern = r"x=\s*([-\d.]+)[,\s]+y=\s*([-\d.]+)"
    matches = re.findall(pattern, text)
    return [(float(x), float(y)) for x, y in matches]


def plot_path_with_boxes_and_circles(points, title="Path with Boxes and Circles"):
    if not points:
        print("没有找到有效的坐标点！")
        return

    xs, ys = zip(*points)
    fig, ax = plt.subplots(figsize=(12, 10))

    ax.plot(xs, ys, marker='', linestyle='-', color='b', linewidth=1.5, label='Path')

    circle_radius = 15
    for (x, y) in points:
        circle = patches.Circle((x, y), radius=circle_radius,
                                edgecolor='gold', facecolor='yellow',
                                alpha=0.3, linewidth=1)
        ax.add_patch(circle)

    SCALE = 100
    obstacle_vertices = [
        (0.40, 0.40), (0.85, 0.40), (0.40, 0.85), (0.85, 0.85),
        (0.40, 1.25), (0.40, 1.70), (0.85, 1.25), (0.85, 1.70),
        (1.25, 0.40), (1.25, 0.85), (1.70, 0.40), (1.70, 0.85),
        (1.25, 1.25), (1.25, 1.70), (1.70, 1.25), (1.70, 1.70),
        (0.00, 0.76), (0.00, 1.34), (-0.15, 0.76), (-0.15, 1.34),
        (0.76, 2.10), (0.76, 2.25), (1.34, 2.10), (1.34, 2.25),
        (-0.15, -0.15), (-0.15, 2.25), (2.25, -0.15), (2.25, 2.25),
    ]

    box_bounds = []
    for i in range(0, len(obstacle_vertices), 4):
        group = obstacle_vertices[i:i + 4]
        gx = [p[0] * SCALE for p in group]
        gy = [p[1] * SCALE for p in group]
        x0, x1 = min(gx), max(gx)
        y0, y1 = min(gy), max(gy)
        box_bounds.append((x0, y0, x1, y1))
        rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                 linewidth=2, edgecolor='yellow', facecolor='none')
        ax.add_patch(rect)

    all_x = list(xs)
    all_y = list(ys)
    for (x0, y0, x1, y1) in box_bounds:
        all_x += [x0, x1]
        all_y += [y0, y1]

    x_min = min(all_x) - circle_radius * 1.2
    x_max = max(all_x) + circle_radius * 1.2
    y_min = min(all_y) - circle_radius * 1.2
    y_max = max(all_y) + circle_radius * 1.2
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    ax.set_aspect('equal', adjustable='box')
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    ax.legend()

    plt.show()


class LogInputApp:
    """常驻的 Tkinter 输入窗口，可以多次提交"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("日志输入（可多次提交）")
        self.root.geometry("760x560")

        # 顶部提示
        tk.Label(self.root,
                 text="粘贴日志后点“绘制”（Ctrl+Enter）。窗口会保留，可继续输入下一段。",
                 anchor="w", fg="#333").pack(fill="x", padx=10, pady=(10, 0))

        # 多行文本框
        self.text_area = scrolledtext.ScrolledText(
            self.root, wrap="word", font=("Consolas", 11))
        self.text_area.pack(fill="both", expand=True, padx=10, pady=10)
        self.text_area.focus_set()

        # 选项：是否清空输入框
        self.clear_var = tk.BooleanVar(value=False)
        opt_frame = tk.Frame(self.root)
        opt_frame.pack(fill="x", padx=10)
        tk.Checkbutton(opt_frame, text="绘制后清空输入框",
                       variable=self.clear_var).pack(side="left")

        # 按钮
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill="x", padx=10, pady=(5, 12))

        tk.Button(btn_frame, text="清空", width=10,
                  command=self.on_clear).pack(side="left", padx=5)

        tk.Button(btn_frame, text="退出", width=10,
                  command=self.on_quit).pack(side="right", padx=5)

        tk.Button(btn_frame, text="绘制", width=12, bg="#a5d6a7",
                  command=self.on_plot).pack(side="right", padx=5)

        # 快捷键
        self.root.bind("<Control-Return>", lambda e: self.on_plot())
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

        # 记录窗口位置，供 matplotlib 图错开
        self.fig_offset = 0

    def on_clear(self):
        self.text_area.delete("1.0", "end")

    def on_quit(self):
        self.root.destroy()
        plt.close('all')  # 一起关掉所有图

    def on_plot(self):
        log_text = self.text_area.get("1.0", "end")
        points = parse_coordinates(log_text)

        if not points:
            messagebox.showwarning("提示", "没有解析到有效坐标，请检查格式。")
            return

        print(f"解析到 {len(points)} 个点：")
        for i, p in enumerate(points):
            print(f"  [{i}] {p}")

        title = f"Path with Boxes and Circles  ({len(points)} pts)"
        plot_path_with_boxes_and_circles(points, title=title)

        # 让下一个图错开一点，避免完全重叠
        self.fig_offset = (self.fig_offset + 1) % 6

        if self.clear_var.get():
            self.on_clear()

        # 绘图后把焦点还给输入框，方便继续粘贴
        self.root.lift()
        self.root.focus_force()
        self.text_area.focus_set()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    LogInputApp().run()