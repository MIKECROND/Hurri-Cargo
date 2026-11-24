#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把 baseline_runner（或后续重构版 runner）输出的 CSV 汇总成 markdown 评测报告。

输入（--results-dir 目录下）：
    <prefix>_zone1.csv / <prefix>_zone2.csv       每条点对的汇总指标
    <prefix>_rects_zone1.csv / ..._zone2.csv      独立几何真值用的障碍矩形
    <prefix>_points_zone1.csv / ..._zone2.csv     点位名称与坐标

输出：
    markdown 报告（--out 指定文件；省略则打印到 stdout）

用法：
    py summarize.py --results-dir %TEMP%\\planner_harness\\out --label "HEAD 6413fac（重构前基线）"
    py summarize.py --results-dir ... --out report.md

设计说明：
    - 报告里的“严格碰撞”严格按 tests/README.md 的定义：min_clearance_cm < 15cm；
    - 由于 30cm 宽的通道恰好等于车宽（30cm），大量路径的名义清障正好是 15.0cm，
      浮点计算会把它们压到 14.99999x，因此额外分类：
        * 明确侵入：min_clearance < 14.5cm（有 >0.5cm 的确定侵入，扣除采样/浮点误差）
        * 零余量擦边：14.5cm <= min_clearance < 15.0cm
    - 该脚本与规划器版本无关，重构后同一套指标可直接复用做对比。
"""

import argparse
import csv
import os
import sys

# 分类阈值（cm）：严格碰撞 = min_clearance < ROBOT_RADIUS
ROBOT_RADIUS_CM = 15.0
# 明确侵入阈值：比半径小 0.5cm 以上，排除 2cm 采样步长与浮点舍入带来的边界噪声
HARD_INTRUSION_CM = 14.5
# 零余量上界：名义清障正好 15.0cm 时，浮点可能落在 15.0 的两侧，统一按“零余量”归类
ZERO_MARGIN_HI_CM = 15.05
# “直线化额外损失”报告阈值（cm）：简化后清障比 A* 原始路径还小 0.4cm 以上才列出
SIMPLIFY_LOSS_CM = 0.4


def read_csv(path):
    """读 CSV，返回 dict 列表；文件不存在返回 None。"""
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def fnum(row, key):
    """取数值字段；失败行（ok=0）里的 -1 也照常返回。"""
    return float(row[key])


def load_zone(results_dir, prefix, zone):
    """加载单个 zone 的所有输入文件。"""
    z = str(zone)
    pairs = read_csv(os.path.join(results_dir, "%s_zone%s.csv" % (prefix, z)))
    if pairs is None:
        raise SystemExit("[FAIL] 找不到 %s_zone%s.csv（--results-dir 是否指对？）" % (prefix, z))
    rects = read_csv(os.path.join(results_dir, "%s_rects_zone%s.csv" % (prefix, z))) or []
    points = read_csv(os.path.join(results_dir, "%s_points_zone%s.csv" % (prefix, z))) or []
    return {"zone": zone, "pairs": pairs, "rects": rects, "points": points}


def classify(row):
    """给一条点对分类：safe / zero_margin / hard / fail。

    注意：严格碰撞（是否 < 15.0cm）必须用 CSV 的 `collision` 列判断，而不是用四舍五入后的
    min_clearance_cm 重新比较——CSV 只保留 4 位小数，14.99999x 会被写成 15.0000。
    但“分类”用 15.05cm 作为零余量上界：名义清障正好 15.0cm（车宽=30cm 通道）的路径无论
    浮点落在 15.0 哪一侧，物理上都是零余量，统一归为 zero_margin。
    """
    if str(row.get("ok", "0")).strip() != "1":
        return "fail"
    clr = fnum(row, "min_clearance_cm")
    if clr < HARD_INTRUSION_CM:
        return "hard"
    if clr < ZERO_MARGIN_HI_CM:
        return "zero_margin"
    return "safe"


def penetration(row):
    """侵入量（cm）：正数表示越过机器人半径的深度。"""
    return ROBOT_RADIUS_CM - fnum(row, "min_clearance_cm")


def rect_label(rects, idx):
    """把最近矩形下标格式化成 [x0,y0]~[x1,y1]cm；越界返回 '-'。"""
    try:
        i = int(float(idx))
    except (TypeError, ValueError):
        return "-"
    if i < 0 or i >= len(rects):
        return "-"
    r = rects[i]
    return "[%s,%s]~[%s,%s]" % (r["x0_cm"], r["y0_cm"], r["x1_cm"], r["y1_cm"])


def zone_stats(zdata):
    """单个 zone 的统计量。"""
    pairs = zdata["pairs"]
    ok = [r for r in pairs if str(r.get("ok", "0")).strip() == "1"]
    fail = len(pairs) - len(ok)
    kinds = [classify(r) for r in ok]
    safe = kinds.count("safe")
    zero_margin = kinds.count("zero_margin")
    hard = kinds.count("hard")
    collision = sum(1 for r in ok if str(r.get("collision", "0")).strip() == "1")
    raw_coll = sum(1 for r in ok if str(r.get("raw_collision", "0")).strip() == "1")
    wps = [int(float(r["waypoints"])) for r in ok]
    # 新版 runner 拿不到 A* 原始逐格路径时 astar_pts 填 -1，此时相关列显示 "-"
    astar = [int(float(r["astar_pts"])) for r in ok if int(float(r["astar_pts"])) >= 0]
    has_astar = len(astar) > 0
    has_raw = any(fnum(r, "raw_min_clearance_cm") > 0 for r in ok)
    raw_coll = raw_coll if has_raw else 0
    lens = [fnum(r, "length_cm") for r in ok]
    turns = [fnum(r, "sum_turn_deg") for r in ok]
    goal_errs = [abs(fnum(r, "goal_error_cm")) for r in ok]
    worst = min(ok, key=lambda r: fnum(r, "min_clearance_cm")) if ok else None
    return {
        "total": len(pairs),
        "ok": len(ok),
        "fail": fail,
        "safe": safe,
        "zero_margin": zero_margin,
        "hard": hard,
        "collision": collision,
        "strict_safe": len(ok) - collision,
        "raw_collision": raw_coll,
        "has_raw": has_raw,
        "has_astar": has_astar,
        "avg_wp": sum(wps) / len(wps) if wps else 0.0,
        "max_wp": max(wps) if wps else 0,
        "avg_astar": sum(astar) / len(astar) if astar else 0.0,
        "avg_len": sum(lens) / len(lens) if lens else 0.0,
        "avg_turn": sum(turns) / len(turns) if turns else 0.0,
        "max_goal_err": max(goal_errs) if goal_errs else 0.0,
        "worst": worst,
    }


def aggregate(stats_list):
    """两个 zone 合计。"""
    ok_rows = [r for z in stats_list for r in z["zdata"]["pairs"] if str(r.get("ok", "0")).strip() == "1"]
    kinds = [classify(r) for r in ok_rows]
    collision = sum(1 for r in ok_rows if str(r.get("collision", "0")).strip() == "1")
    return {
        "total": sum(z["total"] for z in stats_list),
        "ok": sum(z["ok"] for z in stats_list),
        "fail": sum(z["fail"] for z in stats_list),
        "safe": kinds.count("safe"),
        "zero_margin": kinds.count("zero_margin"),
        "hard": kinds.count("hard"),
        "collision": collision,
        "strict_safe": len(ok_rows) - collision,
        "raw_collision": sum(
            1
            for z in stats_list
            for r in z["zdata"]["pairs"]
            if str(r.get("ok", "0")).strip() == "1" and str(r.get("raw_collision", "0")).strip() == "1"
        ),
        "avg_wp": sum(z["avg_wp"] * z["ok"] for z in stats_list) / max(1, sum(z["ok"] for z in stats_list)),
        "max_wp": max((z["max_wp"] for z in stats_list), default=0),
        "avg_astar": sum(z["avg_astar"] * z["ok"] for z in stats_list) / max(1, sum(z["ok"] for z in stats_list)),
        "has_raw": all(z["has_raw"] for z in stats_list),
        "has_astar": all(z["has_astar"] for z in stats_list),
        "avg_len": sum(z["avg_len"] * z["ok"] for z in stats_list) / max(1, sum(z["ok"] for z in stats_list)),
        "max_goal_err": max((z["max_goal_err"] for z in stats_list), default=0.0),
    }


def pair_label(row):
    return "%s -> %s" % (row["start"], row["goal"])


def fmt(x, nd=2):
    return ("%." + str(nd) + "f") % x


def md_point_list(zdata):
    items = ["%s(%s,%s)" % (p["name"], fmt(float(p["x_cm"]), 0), fmt(float(p["y_cm"]), 0)) for p in zdata["points"]]
    return "、".join(items) if items else "-"


def generate_report(results_dir, prefix, label):
    zones = [load_zone(results_dir, prefix, 1), load_zone(results_dir, prefix, 2)]
    stats_list = [dict(zone_stats(z), zdata=z) for z in zones]
    total = aggregate(stats_list)

    lines = []
    add = lines.append

    add("# 规划器路径评测报告：%s" % label)
    add("")
    add("> 本报告由 `tests/harness/summarize.py` 从 `%s` 下的 CSV 自动生成，可直接重跑复现。" % results_dir)
    add("> 几何真值、指标定义、主机浮点精度要求见 `tests/README.md`。")
    add("")
    add("## 1. 结论摘要")
    add("")
    add("- 规划成功：**%d/%d** 条有向点对（zone1 %d/%d，zone2 %d/%d）。"
        % (total["ok"], total["total"], stats_list[0]["ok"], stats_list[0]["total"],
           stats_list[1]["ok"], stats_list[1]["total"]))
    add("- 严格安全（全程最小清障 ≥ %.1fcm）：**%d/%d**；物理碰撞（最小清障 < %.1fcm）：**%d/%d**。"
        % (ROBOT_RADIUS_CM, total["strict_safe"], total["ok"], ROBOT_RADIUS_CM, total["collision"], total["ok"]))
    zm_strict = total["collision"] - total["hard"]  # 零余量里因浮点落 <15cm 被算成严格碰撞的条数
    zm_nominal = total["zero_margin"] - zm_strict  # 名义正好 15cm、严格判定算安全的条数
    add("- 碰撞中明确侵入（< %.1fcm）**%d** 条；名义零余量（%.2f~%.2fcm）**%d** 条。零余量这组里，"
        "%d 条因浮点略低于 15cm 被计入严格碰撞，%d 条名义清障正好 15cm（30cm 通道 = 30cm 车宽），"
        "物理上安全余量为 0。"
        % (HARD_INTRUSION_CM, total["hard"], HARD_INTRUSION_CM, ZERO_MARGIN_HI_CM, total["zero_margin"],
           zm_strict, zm_nominal))
    if total["hard"] == 0 and total["collision"] > 0:
        add("- 说明：这 %d 条“严格碰撞”全部是名义清障恰为 15.0cm 的零余量路径（30cm 通道 = 30cm 车宽 / "
            "端点贴墙），world↔map 浮点换算使其测得 14.99999cm；**没有 < %.1fcm 的明确侵入**。"
            % (total["collision"], HARD_INTRUSION_CM))
    add("- 若把零余量路径也算作高风险，则真正留有余量的路径只有 **%d/%d**。" % (total["safe"], total["ok"]))
    if total["has_raw"]:
        add("- A* 原始逐格路径的最小清障已 < %.1fcm 的条数：**%d/%d**。它与“物理碰撞”条数基本相同，"
            "说明碰撞不是直线化（simplify）后才出现的，搜索用的碰撞判据本身就不满足 %.1fcm 车体半径。"
            % (ROBOT_RADIUS_CM, total["raw_collision"], total["ok"], ROBOT_RADIUS_CM))
    else:
        add("- A* 原始逐格路径指标：本版 runner 不暴露 A* 原始逐格路径（astar_pts=-1），"
            "“原始路径已碰撞/直线化损失”等列不适用。")

    # 全局最差点
    worst_row = None
    worst_zone = None
    for z in stats_list:
        w = z["worst"]
        if w is None:
            continue
        if worst_row is None or fnum(w, "min_clearance_cm") < fnum(worst_row, "min_clearance_cm"):
            worst_row = w
            worst_zone = z
    if worst_row is not None:
        add("- 全局最小清障：**%.2fcm**（侵入 %.2fcm），点对 **%s**（zone%d），位于世界坐标 **(%.1f, %.1f)cm**，"
            "最近障碍矩形 %s。"
            % (fnum(worst_row, "min_clearance_cm"), penetration(worst_row), pair_label(worst_row),
               worst_zone["zdata"]["zone"], fnum(worst_row, "min_clr_x_cm"), fnum(worst_row, "min_clr_y_cm"),
               rect_label(worst_zone["zdata"]["rects"], worst_row.get("min_clr_rect"))))

    astar_text = ("%.1f" % total["avg_astar"]) if total["has_astar"] else "-"
    add("- 平均航点 **%.2f** 个、最大 **%d** 个；平均 A* 原始格点 **%s** 个；平均路径长 **%.1fcm**；"
        "终点误差最大 %.4fcm（终点被精确替换为目标点）。"
        % (total["avg_wp"], total["max_wp"], astar_text, total["avg_len"], total["max_goal_err"]))
    add("")

    # 汇总统计
    add("## 2. 汇总统计")
    add("")
    add("| zone | 点对 | 规划成功 | 严格安全(≥15cm) | 严格碰撞(<15cm) | 明确侵入(<14.5cm) | "
        "零余量(14.5~15.05cm) | 有余量(>15.05cm) | 原始路径已碰撞 | 平均航点 | 最大航点 | 平均A*格点 | "
        "平均路径长cm | 最小清障cm |")
    add("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for z in stats_list:
        w = z["worst"]
        add("| %d | %d | %d | %d | %d | %d | %d | %d | %s | %.2f | %d | %s | %.1f | %s |"
            % (z["zdata"]["zone"], z["total"], z["ok"], z["strict_safe"], z["collision"], z["hard"],
               z["zero_margin"], z["safe"], z["raw_collision"] if z["has_raw"] else "-", z["avg_wp"],
               z["max_wp"], ("%.1f" % z["avg_astar"]) if z["has_astar"] else "-", z["avg_len"],
               fmt(fnum(w, "min_clearance_cm")) if w else "-"))
    add("| **合计** | %d | %d | %d | %d | %d | %d | %d | %s | %.2f | %d | %s | %.1f | %s |"
        % (total["total"], total["ok"], total["strict_safe"], total["collision"], total["hard"],
           total["zero_margin"], total["safe"], total["raw_collision"] if total["has_raw"] else "-",
           total["avg_wp"], total["max_wp"], astar_text, total["avg_len"],
           fmt(fnum(worst_row, "min_clearance_cm")) if worst_row is not None else "-"))
    add("")
    add("> 注：严格安全/严格碰撞按 `min_clearance_cm < 15cm` 严格判定（CSV 的 `collision` 列）；"
        "分类列（明确侵入/零余量/有余量）用 14.5cm、15.05cm 两个门限，避免 2cm 采样与浮点噪声把"
        "“名义 15cm”的路径分到两侧。")
    add("")
    add("各 zone 点位（世界坐标，cm）：")
    for z in stats_list:
        add("- zone%d：%s" % (z["zdata"]["zone"], md_point_list(z["zdata"])))
    add("")

    # Top5 最差
    all_ok = []
    for z in stats_list:
        for r in z["zdata"]["pairs"]:
            if str(r.get("ok", "0")).strip() == "1":
                all_ok.append((z, r))
    all_ok.sort(key=lambda item: fnum(item[1], "min_clearance_cm"))
    add("## 3. 最差清障 Top%d" % min(5, len(all_ok)))
    add("")
    raw_hdr = "A*原始清障cm | " if total["has_raw"] else ""
    raw_sep = "---: | " if total["has_raw"] else ""
    add("| # | zone | 点对 | 最小清障cm | 侵入cm | 最近点世界坐标cm | 最近障碍矩形cm | %s航点数 |"
        % raw_hdr)
    add("|---:|---:|---|---:|---:|---|---|%s---: |" % raw_sep)
    for i, (z, r) in enumerate(all_ok[:5], 1):
        raw_cell = ("%.2f | " % fnum(r, "raw_min_clearance_cm")) if total["has_raw"] else ""
        add("| %d | %d | %s | %.2f | %.2f | (%.1f, %.1f) | %s | %s%d |"
            % (i, z["zdata"]["zone"], pair_label(r), fnum(r, "min_clearance_cm"), penetration(r),
               fnum(r, "min_clr_x_cm"), fnum(r, "min_clr_y_cm"),
               rect_label(z["zdata"]["rects"], r.get("min_clr_rect")), raw_cell,
               int(float(r["waypoints"]))))
    add("")

    # 明确侵入清单
    hard_rows = [(z, r) for (z, r) in all_ok if classify(r) == "hard"]
    hard_rows.sort(key=lambda item: fnum(item[1], "min_clearance_cm"))
    add("## 4. 明确侵入清单（最小清障 < %.1fcm，共 %d 条）" % (HARD_INTRUSION_CM, len(hard_rows)))
    add("")
    if hard_rows:
        add("| zone | 点对 | 最小清障cm | 侵入cm | 最近点世界坐标cm | 最近障碍矩形cm | A*原始清障cm |")
        add("|---:|---|---:|---:|---|---|---:|")
        for z, r in hard_rows:
            add("| %d | %s | %.2f | %.2f | (%.1f, %.1f) | %s | %.2f |"
                % (z["zdata"]["zone"], pair_label(r), fnum(r, "min_clearance_cm"), penetration(r),
                   fnum(r, "min_clr_x_cm"), fnum(r, "min_clr_y_cm"),
                   rect_label(z["zdata"]["rects"], r.get("min_clr_rect")), fnum(r, "raw_min_clearance_cm")))
    else:
        add("无。")
    add("")

    # 零余量擦边
    zero_rows = [(z, r) for (z, r) in all_ok if classify(r) == "zero_margin"]
    zero_rows.sort(key=lambda item: fnum(item[1], "min_clearance_cm"))
    add("## 5. 零余量擦边清单（%.2fcm ≤ 最小清障 < %.2fcm，共 %d 条）"
        % (HARD_INTRUSION_CM, ZERO_MARGIN_HI_CM, len(zero_rows)))
    add("")
    add("说明：这类路径的名义清障正好等于车体半径（本场地多条 30cm 通道恰好等于车宽 30cm），"
        "浮点误差使其落在 15.0cm 两侧；物理上没有可测的侵入深度，但安全余量为 0，属于高风险路径。"
        "表中“严格碰撞”列按 < 15.0cm 严格判定。")
    add("")
    if zero_rows:
        add("| zone | 点对 | 最小清障cm | 严格碰撞 | 最近点世界坐标cm | 最近障碍矩形cm |")
        add("|---:|---|---:|---|---|---|")
        for z, r in zero_rows:
            add("| %d | %s | %.4f | %s | (%.1f, %.1f) | %s |"
                % (z["zdata"]["zone"], pair_label(r), fnum(r, "min_clearance_cm"),
                   "是" if str(r.get("collision", "0")).strip() == "1" else "否",
                   fnum(r, "min_clr_x_cm"), fnum(r, "min_clr_y_cm"),
                   rect_label(z["zdata"]["rects"], r.get("min_clr_rect"))))
    else:
        add("无。")
    add("")

    # 直线化带来的额外损失
    add("## 6. 直线化（simplify/string-pulling）额外损失")
    add("")
    if total["has_raw"]:
        loss_rows = [(z, r) for (z, r) in all_ok
                     if fnum(r, "raw_min_clearance_cm") - fnum(r, "min_clearance_cm") > SIMPLIFY_LOSS_CM]
        loss_rows.sort(key=lambda item: fnum(item[1], "min_clearance_cm") - fnum(item[1], "raw_min_clearance_cm"))
        add("判定：`A* 原始路径最小清障 - 直线化后最小清障 > %.1fcm`，说明直线化为了少航点把路径拉得更贴障碍。"
            % SIMPLIFY_LOSS_CM)
        add("")
        if loss_rows:
            add("| zone | 点对 | A*原始清障cm | 直线化后清障cm | 额外损失cm |")
            add("|---:|---|---:|---:|---:|")
            for z, r in loss_rows:
                raw = fnum(r, "raw_min_clearance_cm")
                cur = fnum(r, "min_clearance_cm")
                add("| %d | %s | %.2f | %.2f | %.2f |" % (z["zdata"]["zone"], pair_label(r), raw, cur, raw - cur))
        else:
            add("无。")
    else:
        add("本版 runner 不暴露 A* 原始逐格路径（`astar_pts=-1`），无法计算“直线化损失”；"
            "直线化后的最终路径已直接按独立几何真值测量，见第 3~5、7 节。")
    add("")

    # 逐点对明细
    add("## 7. 按点对明细")
    add("")
    add("> 余量cm = 最小清障 - 15cm，负值表示侵入；分类门限：明确侵入 < 14.5cm、零余量 14.5~15.05cm、"
        "有余量 > 15.05cm。")
    add("")
    for z in stats_list:
        zd = z["zdata"]
        add("### zone%d" % zd["zone"])
        add("")
        add("| start → goal | ok | A*格点 | 航点 | 路径长cm | 最小清障cm | 余量cm | 分类 | "
            "最近点世界坐标cm | 最近障碍矩形cm | 最大转角° | 转角总和° |")
        add("|---|---:|---:|---:|---:|---:|---:|---|---|---|---:|---:|")
        for r in zd["pairs"]:
            if str(r.get("ok", "0")).strip() != "1":
                add("| %s | ✗ | - | - | - | - | - | 规划失败 | - | - | - | - |" % pair_label(r))
                continue
            kind = classify(r)
            kind_text = {"safe": "有余量", "zero_margin": "零余量", "hard": "明确侵入"}[kind]
            margin = fnum(r, "min_clearance_cm") - ROBOT_RADIUS_CM
            astar_pts = int(float(r["astar_pts"]))
            astar_text = str(astar_pts) if astar_pts >= 0 else "-"
            add("| %s | ✓ | %s | %d | %.1f | %.2f | %+.2f | %s | (%.1f, %.1f) | %s | %.1f | %.1f |"
                % (pair_label(r), astar_text, int(float(r["waypoints"])), fnum(r, "length_cm"),
                   fnum(r, "min_clearance_cm"), margin, kind_text,
                   fnum(r, "min_clr_x_cm"), fnum(r, "min_clr_y_cm"),
                   rect_label(zd["rects"], r.get("min_clr_rect")),
                   fnum(r, "max_turn_deg"), fnum(r, "sum_turn_deg")))
        add("")

    # 复现命令
    add("## 8. 复现")
    add("")
    add("```powershell")
    add("# ===== 重构前基线（HEAD 6413fac，git 快照） =====")
    add("# 1) 导出旧源码快照（SHA-256 校验）")
    add("py Applications\\Planner\\tests\\harness\\baseline\\snapshot_baseline.py")
    add("")
    add("# 2) 编译基线驱动（-msse2 -mfpmath=sse 对齐 Cortex-M4 单精度语义）")
    add("D:\\devtool\\MinGW\\bin\\gcc.exe -std=c99 -O2 -Wall -Wextra -msse2 -mfpmath=sse `")
    add("    -I $env:TEMP\\planner_harness\\baseline_src -I Applications\\Planner\\tests\\harness `")
    add("    $env:TEMP\\planner_harness\\baseline_src\\planner_algo.c `")
    add("    $env:TEMP\\planner_harness\\baseline_src\\Cstar_map_core.c `")
    add("    $env:TEMP\\planner_harness\\baseline_src\\priority_queue.c `")
    add("    Applications\\Planner\\tests\\harness\\reference_metrics.c `")
    add("    Applications\\Planner\\tests\\harness\\baseline\\baseline_runner.c `")
    add("    -o $env:TEMP\\planner_harness\\build\\baseline_runner.exe")
    add("& $env:TEMP\\planner_harness\\build\\baseline_runner.exe --out-dir $env:TEMP\\planner_harness\\out")
    add("")
    add("# ===== 重构版（工作区源码） =====")
    add("# 一键脚本：编译 new_runner/edge_runner + 跑 40 条 + 边界用例 + 生成 new 报告与对比报告")
    add("powershell -NoProfile -ExecutionPolicy Bypass -File Applications\\Planner\\tests\\harness\\run_new.ps1")
    add("")
    add("# 或手动：")
    add("D:\\devtool\\MinGW\\bin\\gcc.exe -std=c99 -O2 -Wall -Wextra -DPLANNER_HOST_TEST -msse2 -mfpmath=sse `")
    add("    -I Applications\\Planner -I Applications\\Planner\\utils -I Applications\\Planner\\tests\\harness `")
    add("    Applications\\Planner\\utils\\Cstar_map_core.c Applications\\Planner\\utils\\planner_algo.c `")
    add("    Applications\\Planner\\utils\\priority_queue.c `")
    add("    Applications\\Planner\\tests\\harness\\reference_metrics.c `")
    add("    Applications\\Planner\\tests\\harness\\new_runner.c `")
    add("    -o $env:TEMP\\planner_harness\\new_build\\new_runner.exe -lm")
    add("& $env:TEMP\\planner_harness\\new_build\\new_runner.exe --out-dir $env:TEMP\\planner_harness\\new_out")
    add("py Applications\\Planner\\tests\\harness\\summarize.py --results-dir $env:TEMP\\planner_harness\\new_out `")
    add("    --prefix new --label '重构版' --out new_report.md")
    add("py Applications\\Planner\\tests\\harness\\compare.py `")
    add("    --baseline-dir Applications\\Planner\\tests\\results `")
    add("    --new-dir $env:TEMP\\planner_harness\\new_out --out after_report.md")
    add("```")
    add("")
    add("## 附录：CSV 字段说明")
    add("")
    add("`*_zone<z>.csv`（每条点对一行）：")
    add("")
    add("- `zone,start,goal`：启动区、起点名、终点名")
    add("- `ok`：A* 是否成功（0/1）")
    add("- `astar_pts`：A* 原始逐格路径点数；`waypoints`：直线化后航点数")
    add("- `length_cm`：直线化后路径总长")
    add("- `min_clearance_cm`：直线化后路径全程最小清障距离（点到障碍矩形精确欧氏距离）")
    add("- `collision`：是否 `min_clearance_cm < 15cm`（0/1）")
    add("- `min_clr_wp_idx`：清障距离最小的航点下标（辅助参考）")
    add("- `min_clr_seg`：最小清障所在路径段下标（0 基）")
    add("- `min_clr_rect`：最小清障处最近障碍矩形下标（对应 `*_rects_zone<z>.csv`）")
    add("- `min_clr_x_cm, min_clr_y_cm`：最小清障发生点世界坐标（cm）")
    add("- `segs_over_radius`：含“清障 < 15cm”采样点的路径段数")
    add("- `wp_over_radius`：清障 < 15cm 的航点数")
    add("- `goal_error_cm`：路径末点与目标点距离（应≈0）")
    add("- `max_turn_deg, sum_turn_deg`：内部航点最大转角、转角绝对值之和")
    add("- `raw_min_clearance_cm, raw_collision`：A* 原始逐格路径的同口径指标")
    add("- `raw_min_clr_x_cm, raw_min_clr_y_cm, raw_min_clr_rect`：原始路径最小清障位置与最近矩形")
    add("")
    add("`*_paths_zone<z>.csv`（每个航点一行）：`wp_idx, world_x_cm, world_y_cm, clearance_cm, "
        "seg_len_cm, turn_deg`。")
    add("")
    add("`*_rects_zone<z>.csv`：独立几何真值使用的障碍矩形（由旧 config.h 每 4 个障碍顶点一组构造）。")
    add("")

    return "\n".join(lines)


def summarize_v2(new_dir, out_path):
    """v2（方形车体）摘要：读 newv2_zone*.csv，输出点对表 + 汇总 markdown。"""
    rows = []
    for z in (1, 2):
        p = os.path.join(new_dir, "newv2_zone%d.csv" % z)
        if not os.path.exists(p):
            continue
        with open(p, "r", encoding="utf-8-sig", newline="") as fh:
            rows.extend(csv.DictReader(fh))

    def fnum(row, key, default=0.0):
        try:
            return float(row.get(key, default))
        except (TypeError, ValueError):
            return default

    def inum(row, key, default=0):
        try:
            return int(float(row.get(key, default)))
        except (TypeError, ValueError):
            return default

    ok_rows = [r for r in rows if inum(r, "ok") == 1]
    fail_rows = [r for r in rows if inum(r, "ok") != 1]
    soft = sum(1 for r in ok_rows if inum(r, "tier") == 0)
    tight = len(ok_rows) - soft
    rots = [r for r in ok_rows if inum(r, "rotation_index", 65535) != 65535]
    min_rot = min([fnum(r, "sat_rot_clear_cm") for r in rots] + [0.0]) if rots else -1.0
    avg_len = sum(fnum(r, "length_cm") for r in ok_rows) / len(ok_rows) if ok_rows else 0.0
    max_len = max([fnum(r, "length_cm") for r in ok_rows] + [0.0])
    avg_ms = sum(fnum(r, "plan_ms") for r in ok_rows) / len(ok_rows) if ok_rows else 0.0
    max_ms = max([fnum(r, "plan_ms") for r in ok_rows] + [0.0])
    sat_coll = sum(1 for r in ok_rows if inum(r, "sat_general_collision_samples") > 0)
    pen_viol = sum(1 for r in ok_rows if inum(r, "sat_endpoint_pen_violations") > 0)

    lines = []
    lines.append("# v2（30×30 方形车体）评测摘要\n")
    lines.append("> 独立精确 SAT 真值；执行语义航向；端点豁免区=到起终点欧氏 ≤25cm。\n")
    lines.append("- 点对：**%d**，成功 **%d**，失败 **%d**。" % (len(rows), len(ok_rows), len(fail_rows)))
    lines.append("- tier：soft **%d** / tight **%d**；旋转方案 **%d**，最小独立旋转清障 **%.2fcm**。" % (
        soft, tight, len(rots), min_rot))
    lines.append("- 方形 SAT 碰撞路径：**%d**；端点豁免区侵入超限：**%d**。" % (sat_coll, pen_viol))
    lines.append("- 平均长度 **%.1fcm**（max %.1f）；规划耗时 avg **%.2fms** / max **%.2fms**。" % (
        avg_len, max_len, avg_ms, max_ms))
    lines.append("")
    lines.append("| zone | 点对 | yaw | 航点 | 长度cm | rot | tier | SAT最小signed cm | 最大穿透cm | 耗时ms |")
    lines.append("|---:|---|---:|---:|---:|---:|---|---:|---:|---:|")
    for r in ok_rows:
        rot = inum(r, "rotation_index", 65535)
        rot_txt = "无" if rot == 65535 else str(rot)
        lines.append("| %d | %s -> %s | %.0f→%.0f | %d | %.1f | %s | %s | %.2f | %.2f | %.2f |" % (
            inum(r, "zone"), r["start"], r["goal"],
            fnum(r, "start_yaw_deg"), fnum(r, "goal_yaw_deg"),
            inum(r, "waypoints"), fnum(r, "length_cm"), rot_txt,
            "soft" if inum(r, "tier") == 0 else "tight",
            fnum(r, "sat_min_signed_cm"), fnum(r, "sat_max_penetration_cm"), fnum(r, "plan_ms")))
    for r in fail_rows:
        lines.append("| %d | %s -> %s | %.0f→%.0f | FAIL(fail_reason=%s) | - | - | - | - | - | - |" % (
            inum(r, "zone"), r["start"], r["goal"],
            fnum(r, "start_yaw_deg"), fnum(r, "goal_yaw_deg"), r.get("fail_reason", "")))
    text = "\n".join(lines) + "\n"
    if out_path:
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(text)
        print("[OK] v2 摘要已写入 %s" % os.path.abspath(out_path))
    else:
        sys.stdout.write(text)
    return 0


def main():
    """CLI 入口：分派 v2 摘要与旧版通用报告生成。"""
    # v2 模式：summarize.py --v2 --new-dir DIR [--out PATH]
    if "--v2" in sys.argv:
        argv = sys.argv[1:]
        new_dir = None
        out_path = None
        i = 0
        while i < len(argv):
            if argv[i] == "--v2":
                i += 1
            elif argv[i] == "--new-dir" and i + 1 < len(argv):
                new_dir = argv[i + 1]
                i += 2
            elif argv[i] == "--out" and i + 1 < len(argv):
                out_path = argv[i + 1]
                i += 2
            else:
                i += 1
        if not new_dir:
            print("usage: summarize.py --v2 --new-dir DIR [--out PATH]", file=sys.stderr)
            return 2
        return summarize_v2(os.path.abspath(new_dir), out_path)

    parser = argparse.ArgumentParser(description="CSV -> markdown 规划器评测报告")
    parser.add_argument("--results-dir", required=True, help="baseline_runner 输出目录")
    parser.add_argument("--label", default="HEAD 6413fac（重构前基线）", help="报告标题里的被测版本名")
    parser.add_argument("--prefix", default="baseline", help="CSV 文件名前缀（默认 baseline）")
    parser.add_argument("--out", default=None, help="输出 markdown 路径；省略则打印到 stdout")
    args = parser.parse_args()

    report = generate_report(os.path.abspath(args.results_dir), args.prefix, args.label)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(report)
        print("[OK] 报告已写入 %s" % os.path.abspath(args.out))
    else:
        sys.stdout.write(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
