#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘旋转中心标定计算器（支持两种方法）

两种方法（互斥，见模式选择）
----------------------------
A. 默认：OPS 两点法（位置单位 **mm**，输入取 `ops view`，或按注释换算后的 `nav pose`）
   把车原地转一个角度（不需要精确 180°），记录转前/转后位姿，计算旋转中心/导航位姿点
   相对四轮几何中心的偏移，即 chassis_config.h 中的：

       CHASSIS_ROTATION_CENTER_OFFSET_X / _Y   （旋转中心 C 相对轮组中心 O）

   如果输入取的是原始 `ops view`（OPS 安装点 P），而车体参考点 G 已配置为导航位姿点
   （CHASSIS_BODY_REF_OFFSET 非零），再用 --body-ref-x-cm/--body-ref-y-cm 传入 G-P，
   工具会换算成 C=G 需要的 (P-O)+(G-P)。

   **前提：标定时 CHASSIS_ROTATION_CENTER_OFFSET 必须是 0**。本方法的数学结果是
   "被测点相对【实际旋转中心】的偏移"，偏移非 0 时会得到 P-C（少了当前偏移），不能直接填。

B. --drift：导航位姿漂移修正法（位置单位 **cm**，输入取 `nav pose` 读数）
   直接测量"纯 yaw 旋转时导航位姿点漂移了多少"，反推旋转中心需要补的修正量：

       CHASSIS_ROTATION_CENTER_OFFSET += 修正量

   这是最终验收"旋转不画圆/不整体平移"最直接的方法：它不依赖 P-O、G-P 的分别测量，
   也不要求先把偏移置 0（增量式迭代收敛）。**标定旋转必须在导航空闲时开环执行**
   （`car vel 0 0 <w>` 或 `car key`）；导航闭环会把位姿点按在目标点上，测出来接近 0、无效。

原理
----
A. 两点法：设实际旋转中心 A（世界系固定），被观测点 X 与车体固连，则
       p(t) = A + R(yaw(t)) . (X - A)
   因此由转前 p0,yaw0 与转后 p1,yaw1：

       X - A = (R(yaw1) - R(yaw0))^{-1} . (p1 - p0)

   只要实际转角不接近 0°/360°（建议 >= 90°）即可。当且仅当 A = O（偏移为 0）时，
   结果才等于 X - O。

B. 漂移修正：导航位姿点 G 与固连车体，纯旋转时
       G(t) = A + R(yaw(t)) . R(yaw0)^T . (G0 - A)
   故 G1-G0 = (R(yaw1)-R(yaw0)) . R(yaw0)^T . (G0-A)，反解得

       e_body = R(-yaw0) . (R(yaw1)-R(yaw0))^{-1} . (G1-G0)   (机体系)

   e_body 即"导航位姿点相对实际旋转中心"的机体系偏移。把旋转中心从 A 挪到 G，需要
   CHASSIS_ROTATION_CENTER_OFFSET += e_body。180° 特例：e_body = -0.5 * R(-yaw0) * dG。

用法
----
    # A. 两点法（mm/deg，OPS 安装点）
    py Applications/chassis/pytools/pivot_cal.py x0 y0 yaw0 x1 y1 yaw1
    py ... --body-ref-x-cm -2.0 --body-ref-y-cm 1.0 x0 y0 yaw0 x1 y1 yaw1

    # B. 漂移修正法（cm/deg，导航位姿点，先开环转 ~180°，记录转前/转后 nav pose）
    py Applications/chassis/pytools/pivot_cal.py --drift x0 y0 yaw0 x1 y1 yaw1
    py ... --drift --rot-x-cm -3.80 --rot-y-cm -7.45 x0 y0 yaw0 x1 y1 yaw1

也可以不传参交互输入。A 法输出可直接抄进 Applications/chassis/chassis_config.h；
B 法输出为"在当前值上加的修正量"（给了 --rot-x/y 时同时打印新值）。
"""

import argparse
import math
import sys


def normalize_deg(angle):
    """把角度归一化到 (-180, 180]。"""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def rotation_point_offset(x0_mm, y0_mm, yaw0_deg, x1_mm, y1_mm, yaw1_deg):
    """计算被观测点相对【实际旋转中心】的机体系偏移 (mm)。

    A. 两点法。返回 (offset_x_mm, offset_y_mm, delta_yaw_deg)。
    偏移宏置 0 时实际旋转中心 = 四轮几何中心 O，结果即 (被测点 - O)。
    """
    t0 = math.radians(yaw0_deg)
    t1 = math.radians(yaw1_deg)

    # R(t1) - R(t0) = [[a, -b], [b, a]]
    a = math.cos(t1) - math.cos(t0)
    b = math.sin(t1) - math.sin(t0)
    k = a * a + b * b

    delta_deg = normalize_deg(yaw1_deg - yaw0_deg)
    if abs(delta_deg) < 20.0 or k < 1e-3:
        raise ValueError(
            "实际转角太小（%.2f deg），两点法病态：请在开阔场地把车原地转 >= 90 deg 后重测"
            % delta_deg
        )

    dx = x1_mm - x0_mm
    dy = y1_mm - y0_mm

    # d = (R1 - R0)^-1 * dp = 1/k * [[a, b], [-b, a]] * dp
    offset_x_mm = (a * dx + b * dy) / k
    offset_y_mm = (-b * dx + a * dy) / k
    return offset_x_mm, offset_y_mm, delta_deg


def drift_correction(x0_cm, y0_cm, yaw0_deg, x1_cm, y1_cm, yaw1_deg):
    """B. 漂移修正法：由纯 yaw 旋转前后 nav pose 计算旋转中心修正量 (cm, 机体系)。

    返回 (correction_x_cm, correction_y_cm, delta_yaw_deg)。
    修正量 = 导航位姿点相对实际旋转中心的机体系偏移，需加到当前
    CHASSIS_ROTATION_CENTER_OFFSET 上。
    """
    delta_deg = normalize_deg(yaw1_deg - yaw0_deg)
    if abs(delta_deg) < 60.0:
        raise ValueError(
            "实际转角太小（%.2f deg），漂移修正病态：请开环把车原地转 >= 120 deg（建议 180 deg）后重测"
            % delta_deg
        )

    d = math.radians(delta_deg)
    c = math.cos(d)
    s = math.sin(d)
    # M = R(delta) - I = [[c-1, -s], [s, c-1]]
    det = (c - 1.0) * (c - 1.0) + s * s
    if det < 1e-6:
        raise ValueError("实际转角太小，矩阵不可逆")

    dx = x1_cm - x0_cm
    dy = y1_cm - y0_cm
    # e_world = M^-1 * dG = 1/det * [[c-1, s], [-s, c-1]] * dG
    ewx = ((c - 1.0) * dx + s * dy) / det
    ewy = (-s * dx + (c - 1.0) * dy) / det

    # 世界系 -> 机体系（转前 yaw0 对应的机体系）
    t0 = math.radians(yaw0_deg)
    c0 = math.cos(t0)
    s0 = math.sin(t0)
    ex = c0 * ewx + s0 * ewy
    ey = -s0 * ewx + c0 * ewy
    return ex, ey, delta_deg


def _read_interactive():
    try:
        return [float(input(prompt)) for prompt in (
            "x0: ", "y0: ", "yaw0 (deg): ",
            "x1: ", "y1: ", "yaw1 (deg): ",
        )]
    except (EOFError, KeyboardInterrupt):
        print("\n已取消")
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="底盘旋转中心标定计算器（A 两点法 / B 漂移修正法）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  A. 两点法（位置 mm，取 ops view，偏移宏先置 0）：\n"
            "     py pivot_cal.py -12.5 4.0 0.0 8.5 3.0 155.0\n"
            "  A. 两点法 + 车体参考点换算（输入 ops view，BODY_REF 已配置）：\n"
            "     py pivot_cal.py --body-ref-x-cm -3.1 --body-ref-y-cm -6.782 -12.5 4.0 0.0 8.5 3.0 155.0\n"
            "  B. 漂移修正法（位置 cm，取 nav pose，开环转 ~180 度）：\n"
            "     py pivot_cal.py --drift 110.0 20.0 0.0 106.9 13.2 179.0\n"
            "     py pivot_cal.py --drift --rot-x-cm -3.80 --rot-y-cm -7.45 ...\n"
            "\n"
            "注意：\n"
            "  - A 法必须先把两个偏移宏置 0，否则结果是相对当前实际旋转中心的偏移，不能直接填；\n"
            "  - B 法必须在导航空闲时用速度模式开环旋转（导航闭环会主动按住位姿点，测量无效）；\n"
            "  - 若已把 G 配置为导航位姿点且输入取自 nav pose（输出已是 G），A 法不要再加 --body-ref。"
        ),
    )
    parser.add_argument("x0", nargs="?", type=float, help="转前 X（A 法 mm / B 法 cm）")
    parser.add_argument("y0", nargs="?", type=float, help="转前 Y（A 法 mm / B 法 cm）")
    parser.add_argument("yaw0", nargs="?", type=float, help="转前 yaw (deg)")
    parser.add_argument("x1", nargs="?", type=float, help="转后 X（A 法 mm / B 法 cm）")
    parser.add_argument("y1", nargs="?", type=float, help="转后 Y（A 法 mm / B 法 cm）")
    parser.add_argument("yaw1", nargs="?", type=float, help="转后 yaw (deg)")
    parser.add_argument(
        "--drift",
        action="store_true",
        help="使用 B 漂移修正法（输入取 nav pose，单位 cm；输出为旋转中心修正量）",
    )
    parser.add_argument(
        "--rot-x-cm",
        type=float,
        default=None,
        help="(B 法可选) 当前 CHASSIS_ROTATION_CENTER_OFFSET_X，用于直接输出修正后的新值",
    )
    parser.add_argument(
        "--rot-y-cm",
        type=float,
        default=None,
        help="(B 法可选) 当前 CHASSIS_ROTATION_CENTER_OFFSET_Y，用于直接输出修正后的新值",
    )
    parser.add_argument(
        "--body-ref-x-cm",
        type=float,
        default=0.0,
        help="(A 法) 车体参考点 G 相对 OPS 点 P 的机体系 X 偏移 (cm)，即 CHASSIS_BODY_REF_OFFSET_X",
    )
    parser.add_argument(
        "--body-ref-y-cm",
        type=float,
        default=0.0,
        help="(A 法) 车体参考点 G 相对 OPS 点 P 的机体系 Y 偏移 (cm)，即 CHASSIS_BODY_REF_OFFSET_Y",
    )
    parser.add_argument(
        "--digits", type=int, default=2, help="输出小数位（默认 2 位，A 法即 0.01cm）"
    )
    args = parser.parse_args(argv)

    values = [args.x0, args.y0, args.yaw0, args.x1, args.y1, args.yaw1]
    if any(v is None for v in values):
        unit = "cm" if args.drift else "mm"
        print("请依次输入转前/转后的位姿（位置 %s，角度 deg）：" % unit)
        values = _read_interactive()
        if values is None:
            return 1

    x0, y0, yaw0, x1, y1, yaw1 = values
    fmt = "%%.%df" % args.digits

    if args.drift:
        if args.body_ref_x_cm != 0.0 or args.body_ref_y_cm != 0.0:
            print("提示：--drift 模式不使用 --body-ref（漂移法直接测导航位姿点），已忽略。")
        try:
            ex, ey, delta_deg = drift_correction(x0, y0, yaw0, x1, y1, yaw1)
        except ValueError as err:
            print("ERROR: %s" % err, file=sys.stderr)
            return 1

        mag = math.hypot(ex, ey)
        print("实际转角 deltayaw = %s deg（指令角度不必是 180°）" % (fmt % delta_deg))
        print("导航位姿点相对实际旋转中心的机体系偏移（= 需要加的修正量）：")
        print("  e = (%s, %s) cm, |e| = %s cm"
              % (fmt % ex, fmt % ey, fmt % mag))
        print("")
        if args.rot_x_cm is not None and args.rot_y_cm is not None:
            new_x = args.rot_x_cm + ex
            new_y = args.rot_y_cm + ey
            print("填入 Applications/chassis/chassis_config.h（= 当前值 + 修正量）：")
            print("  #define CHASSIS_ROTATION_CENTER_OFFSET_X (%sf)" % (fmt % new_x))
            print("  #define CHASSIS_ROTATION_CENTER_OFFSET_Y (%sf)" % (fmt % new_y))
        else:
            print("把下面修正量加到当前 CHASSIS_ROTATION_CENTER_OFFSET 上（可用 --rot-x-cm/--rot-y-cm 直接输出新值）：")
            print("  修正量 = (%s, %s) cm" % (fmt % ex, fmt % ey))
        print("")
        if mag > 15.0:
            print("警告：修正量 %.1fcm 偏大：先核对 BODY_REF 是否把导航位姿点定在车体几何中心、"
                  "以及 nav pose 位置单位是 cm（ops view 是 mm，别混用）。" % mag)
        print("验证：烧录后重复本测量，180° 时 nav pose 的 x/y 变化应 < 1~2cm（可迭代 2~3 次）。")
        print("      车体几何中心也应基本不动；若 nav pose 不动但车体仍整体平移/画圆，")
        print("      说明 CHASSIS_BODY_REF_OFFSET 没指在真实车体几何中心（见 README §4.5）。")
        print("")
        print("注意：必须在导航空闲时用速度模式开环旋转（car vel 0 0 30 / car key）；")
        print("      nav 闭环会把位姿点按在目标点上，测到的漂移接近 0、标定无效。")
        return 0

    # ---------- A. 两点法 ----------
    try:
        ox_mm, oy_mm, delta_deg = rotation_point_offset(
            x0, y0, yaw0, x1, y1, yaw1
        )
    except ValueError as err:
        print("ERROR: %s" % err, file=sys.stderr)
        return 1

    ox_cm = ox_mm / 10.0
    oy_cm = oy_mm / 10.0
    ref_x_cm = args.body_ref_x_cm
    ref_y_cm = args.body_ref_y_cm
    # 旋转中心/导航位姿点 G 相对四轮几何中心 O：G-O = (P-O) + (G-P)
    gx_cm = ox_cm + ref_x_cm
    gy_cm = oy_cm + ref_y_cm

    print("实际转角 deltayaw = %s deg（指令角度不必是 180°）" % (fmt % delta_deg))
    print("被测点相对【实际旋转中心】的机体系偏移：(%s, %s) cm [+X 前, +Y 左]"
          % (fmt % ox_cm, fmt % oy_cm))
    print("  （标定前已把 CHASSIS_ROTATION_CENTER_OFFSET 置 0 时，这就是它相对四轮几何中心 O 的偏移）")
    if ref_x_cm != 0.0 or ref_y_cm != 0.0:
        print("车体参考点相对 OPS 点：   G-P = (%s, %s) cm"
              % (fmt % ref_x_cm, fmt % ref_y_cm))
        print("旋转中心/导航位姿点：     C=G-O = (%s, %s) cm" % (fmt % gx_cm, fmt % gy_cm))
    print("")
    print("填入 Applications/chassis/chassis_config.h：")
    if ref_x_cm != 0.0 or ref_y_cm != 0.0:
        print("  #define CHASSIS_BODY_REF_OFFSET_X (%sf)" % (fmt % ref_x_cm))
        print("  #define CHASSIS_BODY_REF_OFFSET_Y (%sf)" % (fmt % ref_y_cm))
    print("  #define CHASSIS_ROTATION_CENTER_OFFSET_X (%sf)" % (fmt % gx_cm))
    print("  #define CHASSIS_ROTATION_CENTER_OFFSET_Y (%sf)" % (fmt % gy_cm))
    print("")
    if ref_x_cm == 0.0 and ref_y_cm == 0.0:
        print("提示：若 OPS 安装点不在车体几何中心，应先把“车体参考点相对 OPS 点”的偏移")
        print("      填入 CHASSIS_BODY_REF_OFFSET（见 README §4.3/§4.5），并用 --body-ref")
        print("      重算本结果，或改用 `nav pose`（输出已是车体参考点）重新两点法标定。")
    print("")
    print("验证（注意看对参考点）：")
    print("  - 若 G=P（BODY_REF=0，C=P）：纯 yaw 时 `ops view` 的 x/y 应基本不变；")
    print("  - 若已按 §4.3/§4.5 把 G 设为车体参考点（C=G）：纯 yaw 时 `ops view` 会绕圈（正常），")
    print("    应看 `nav pose` 的 x/y 基本不变，且车体几何中心不整体平移。")
    print("  - 不要用 `car pos`（位置模式）验证偏移非 0 后的“绕 C 转”，四轮脉冲不等、轨迹有瞬态偏差。")
    print("  - 残余漂移直接修正法见 --drift（README §4.5「漂移修正法」）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
