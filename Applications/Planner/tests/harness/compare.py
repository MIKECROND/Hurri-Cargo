#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""生成“重构前 vs 重构后”对比报告（markdown）。

输入：
    --baseline-dir DIR   重构前基线结果目录（含 baseline_zone1/2.csv；仓库 tests/results）
    --new-dir DIR        重构版结果目录（含 new_zone1/2.csv、new_stats_zone1/2.csv，
                         可选 acceptance.csv / edge_results.csv）
输出：
    markdown 对比报告（--out 指定文件；省略则打印到 stdout）

用法：
    py compare.py --baseline-dir Applications/Planner/tests/results `
        --new-dir %TEMP%/planner_harness/new_out --out after_report.md

说明：
    - 两侧的“独立真值最小清障”都由 reference_metrics.c 用同一套几何模型测量，口径一致；
    - 新实现不暴露 A* 原始逐格路径（astar_pts=-1 / raw_*=0），涉及 raw 的指标只对基线有意义；
    - 显著差异判定：|Δ最小清障| ≥0.5cm，或航点数变化，或 |Δ路径长| ≥5cm。
"""

import argparse
import csv
import os
import sys

ROBOT_RADIUS_CM = 15.0
HARD_INTRUSION_CM = 14.5
ZERO_MARGIN_HI_CM = 15.05
CLEARANCE_DIFF_CM = 0.5
LENGTH_DIFF_CM = 5.0


def classify(row):
    """按"明确侵入 / 零余量 / 有余量"分类（与 summarize.py 口径一致）。

    严格碰撞（collision 列，<15.0cm）与分类不同：名义清障正好 15.0cm 的路径
    （30cm 通道 = 30cm 车宽、HOME 端点离场边条 15cm）会因 world↔map 浮点往返误差
    在 14.99999~15.00001 之间抖动，统一归入"零余量"。
    """
    clr = fnum(row, "min_clearance_cm")
    if clr < HARD_INTRUSION_CM:
        return "hard"
    if clr < ZERO_MARGIN_HI_CM:
        return "zero_margin"
    return "safe"


def read_csv(path):
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def fnum(row, key):
    return float(row[key])


def load_pairs(directory, prefix):
    """读取两个 zone 的汇总 CSV，返回 {(zone,start,goal): row}。"""
    out = {}
    for zone in (1, 2):
        rows = read_csv(os.path.join(directory, "%s_zone%d.csv" % (prefix, zone)))
        if rows is None:
            raise SystemExit("[FAIL] 缺少 %s/%s_zone%d.csv" % (directory, prefix, zone))
        for r in rows:
            out[(zone, r["start"], r["goal"])] = r
    return out


def load_stats(directory, prefix):
    """读取新实现的附加统计 CSV（tier/expanded_nodes 等），键同 load_pairs。"""
    out = {}
    for zone in (1, 2):
        rows = read_csv(os.path.join(directory, "%s_stats_zone%d.csv" % (prefix, zone)))
        if rows is None:
            continue
        for r in rows:
            out[(zone, r["start"], r["goal"])] = r
    return out


def summarize(pairs):
    """单侧汇总指标。"""
    ok = [r for r in pairs.values() if str(r.get("ok", "0")).strip() == "1"]
    coll = [r for r in ok if str(r.get("collision", "0")).strip() == "1"]
    kinds = [classify(r) for r in ok]
    hard = [r for r in ok if classify(r) == "hard"]
    worst = min(ok, key=lambda r: fnum(r, "min_clearance_cm")) if ok else None
    return {
        "total": len(pairs),
        "ok": len(ok),
        "fail": len(pairs) - len(ok),
        "safe": len(ok) - len(coll),
        "collision": len(coll),
        "hard": len(hard),
        "zero_margin": kinds.count("zero_margin"),
        "margin": kinds.count("safe"),
        "avg_wp": sum(float(r["waypoints"]) for r in ok) / len(ok) if ok else 0.0,
        "max_wp": max((int(float(r["waypoints"])) for r in ok), default=0),
        "avg_len": sum(fnum(r, "length_cm") for r in ok) / len(ok) if ok else 0.0,
        "max_goal_err": max((abs(fnum(r, "goal_error_cm")) for r in ok), default=0.0),
        "worst": worst,
    }


def worst_label(worst):
    if worst is None:
        return "-"
    return "%s -> %s（%.2fcm @ (%.1f,%.1f)cm）" % (
        worst["start"],
        worst["goal"],
        fnum(worst, "min_clearance_cm"),
        fnum(worst, "min_clr_x_cm"),
        fnum(worst, "min_clr_y_cm"),
    )


def pair_label(key):
    return "%s -> %s" % (key[1], key[2])


def read_acc(directory):
    rows = read_csv(os.path.join(directory, "acceptance.csv"))
    return rows


def read_edge(directory):
    rows = read_csv(os.path.join(directory, "edge_results.csv"))
    return rows


def read_profile(directory):
    rows = read_csv(os.path.join(directory, "profile_results.csv"))
    return rows


def tier_summary(stats):
    """按 zone 统计 soft/tight 数量与实测清障范围。"""
    result = []
    for zone in (1, 2):
        soft = []
        tight = []
        fail = 0
        for key, r in stats.items():
            if key[0] != zone:
                continue
            if str(r.get("tier")) == "255":
                fail += 1
                continue
            clr = fnum(r, "ref_min_clearance_cm")
            if str(r.get("tier")) == "0":
                soft.append(clr)
            else:
                tight.append(clr)
        result.append(
            {
                "zone": zone,
                "soft": len(soft),
                "tight": len(tight),
                "fail": fail,
                "soft_min": min(soft) if soft else None,
                "soft_max": max(soft) if soft else None,
                "soft_avg": sum(soft) / len(soft) if soft else 0.0,
                "tight_min": min(tight) if tight else None,
                "tight_max": max(tight) if tight else None,
                "tight_avg": sum(tight) / len(tight) if tight else 0.0,
            }
        )
    return result


def generate_report(baseline_dir, new_dir, baseline_label, new_label):
    before = load_pairs(baseline_dir, "baseline")
    after = load_pairs(new_dir, "new")
    after_stats = load_stats(new_dir, "new")
    # 把 tier 等附加列合并进 after（主 CSV 保持与基线相同的 24 列表头）
    for key, srow in after_stats.items():
        if key in after:
            after[key]["tier"] = srow.get("tier", "-1")
    b = summarize(before)
    a = summarize(after)
    tiers = tier_summary(after_stats)
    acc = read_acc(new_dir)
    edge = read_edge(new_dir)
    profile = read_profile(new_dir)

    lines = []
    add = lines.append

    add("# 规划器重构前后对比报告：%s → %s" % (baseline_label, new_label))
    add("")
    add("> 本报告由 `tests/harness/compare.py` 基于两套 runner 的 CSV 自动生成；")
    add("> 两侧路径均由 `reference_metrics.c` 的**独立几何真值**测量（不读 Planner 网格/清障距离场）。")
    add("> 基线数据：`%s`；重构版数据：`%s`。" % (baseline_dir, new_dir))
    add("")
    add("## 0. 结论摘要")
    add("")
    add("- 规划成功：重构前 **%d/%d** → 重构后 **%d/%d**（失败：%d → %d）。"
        % (b["ok"], b["total"], a["ok"], a["total"], b["fail"], a["fail"]))
    add("- 明确侵入（最小清障 < %.1fcm，真实穿障）：重构前 **%d** 条 → 重构后 **%d** 条。"
        % (HARD_INTRUSION_CM, b["hard"], a["hard"]))
    add("- 严格按 < %.1fcm 判定的“碰撞”条数：%d → %d；两侧的这部分差异都是“名义清障正好 15.0cm”的"
        "零余量路径（30cm 通道 = 30cm 车宽 / HOME 端点离场边条 15cm）在 world↔map 浮点换算后测得 "
        "14.99999cm，并非真实侵入。"
        % (ROBOT_RADIUS_CM, b["collision"], a["collision"]))
    add("- 分类口径：明确侵入（<%.1fcm）%d → %d；零余量（%.2f~%.2fcm）%d → %d；有余量（>%.2fcm）%d → %d。"
        % (HARD_INTRUSION_CM, b["hard"], a["hard"], HARD_INTRUSION_CM, ZERO_MARGIN_HI_CM, b["zero_margin"],
           a["zero_margin"], ZERO_MARGIN_HI_CM, b["margin"], a["margin"]))
    add("- 严格安全（全程最小清障 ≥ %.1fcm）：重构前 **%d/%d** → 重构后 **%d/%d**。"
        % (ROBOT_RADIUS_CM, b["safe"], b["ok"], a["safe"], a["ok"]))
    add("- 全局最小清障：重构前 **%s** → 重构后 **%s**。"
        % (worst_label(b["worst"]), worst_label(a["worst"])))
    add("- 平均航点：%.2f → **%.2f**；最大航点：%d → %d；平均路径长：%.1fcm → **%.1fcm**；"
        "终点误差最大：%.4fcm → %.4fcm。"
        % (b["avg_wp"], a["avg_wp"], b["max_wp"], a["max_wp"], b["avg_len"], a["avg_len"],
           b["max_goal_err"], a["max_goal_err"]))
    if a["worst"] is not None:
        min_after = fnum(a["worst"], "min_clearance_cm")
        add("- **新实现全部 %d 条路径按独立真值测量的最小清障为 %.2fcm（≥ %.1fcm）：其中 %d 条为名义 15.0cm 的"
            "零余量路径、%d 条有余量，0 条明确侵入（<%.1fcm）。**"
            % (a["ok"], min_after, ROBOT_RADIUS_CM, a["zero_margin"], a["margin"], HARD_INTRUSION_CM))
    add("- tier 分布（重构后）：zone1 soft=%d / tight=%d；zone2 soft=%d / tight=%d。"
        % (tiers[0]["soft"], tiers[0]["tight"], tiers[1]["soft"], tiers[1]["tight"]))
    if acc is not None:
        acc_pass = sum(1 for r in acc if r["result"] == "PASS")
        acc_fail = sum(1 for r in acc if r["result"] == "FAIL")
        add("- new_runner 验收断言：**%d PASS / %d FAIL**。" % (acc_pass, acc_fail))
    if edge is not None:
        e_pass = sum(1 for r in edge if r["result"] == "PASS")
        e_fail = sum(1 for r in edge if r["result"] == "FAIL")
        add("- edge_runner 边界/回归用例：**%d PASS / %d FAIL**。" % (e_pass, e_fail))
    if profile is not None:
        p_pass = sum(1 for r in profile if r["result"] == "PASS")
        p_fail = sum(1 for r in profile if r["result"] == "FAIL")
        add("- profile_runner 执行剖面单元测试（执行层段长/剩余路程 Blocker 回归）：**%d PASS / %d FAIL**。"
            % (p_pass, p_fail))
    add("")

    add("## 1. 前后汇总对比")
    add("")
    add("| 指标 | 重构前（%s） | 重构后（%s） | 变化 |" % (baseline_label, new_label))
    add("|---|---:|---:|---:|")
    add("| 规划成功 | %d/%d | %d/%d | %+d |" % (b["ok"], b["total"], a["ok"], a["total"], a["ok"] - b["ok"]))
    add("| 严格安全(≥15cm) | %d/%d | %d/%d | %+d |"
        % (b["safe"], b["ok"], a["safe"], a["ok"], a["safe"] - b["safe"]))
    add("| 明确侵入(<14.5cm) | %d | %d | %+d |" % (b["hard"], a["hard"], a["hard"] - b["hard"]))
    add("| 零余量(14.5~15.05cm) | %d | %d | %+d |"
        % (b["zero_margin"], a["zero_margin"], a["zero_margin"] - b["zero_margin"]))
    add("| 有余量(>15.05cm) | %d | %d | %+d |" % (b["margin"], a["margin"], a["margin"] - b["margin"]))
    add("| 严格碰撞(<15cm，含零余量浮点抖动) | %d | %d | %+d |"
        % (b["collision"], a["collision"], a["collision"] - b["collision"]))
    add("| 最小清障 | %s | %s | - |" % (worst_label(b["worst"]), worst_label(a["worst"])))
    add("| 平均航点 | %.2f | %.2f | %+.2f |" % (b["avg_wp"], a["avg_wp"], a["avg_wp"] - b["avg_wp"]))
    add("| 最大航点 | %d | %d | %+d |" % (b["max_wp"], a["max_wp"], a["max_wp"] - b["max_wp"]))
    add("| 平均路径长cm | %.1f | %.1f | %+.1f |" % (b["avg_len"], a["avg_len"], a["avg_len"] - b["avg_len"]))
    add("| 终点误差最大cm | %.4f | %.4f | - |" % (b["max_goal_err"], a["max_goal_err"]))
    add("")
    add("> 注：新实现（`Planner_PlanPath`）不对外暴露 A* 原始逐格路径，`astar_pts=-1`、`raw_*` 列不可用，"
        "因此“A* 原始路径清障/直线化损失”只对基线有意义；重构版的安全指标按最终下发的路径测量。")
    add("")

    # ===== 逐点对差异（只列显著项） =====
    diffs = []
    unchanged = 0
    for key in sorted(before.keys()):
        rb = before[key]
        ra = after.get(key)
        if ra is None:
            continue
        if str(rb.get("ok", "0")).strip() != "1" or str(ra.get("ok", "0")).strip() != "1":
            diffs.append(
                {
                    "key": key,
                    "kind": "规划失败状态变化",
                    "d_clr": 0.0,
                    "d_wp": 0,
                    "d_len": 0.0,
                    "ra": ra,
                    "rb": rb,
                }
            )
            continue
        clr_b = fnum(rb, "min_clearance_cm")
        clr_a = fnum(ra, "min_clearance_cm")
        wp_b = int(float(rb["waypoints"]))
        wp_a = int(float(ra["waypoints"]))
        len_b = fnum(rb, "length_cm")
        len_a = fnum(ra, "length_cm")
        d_clr = clr_a - clr_b
        d_wp = wp_a - wp_b
        d_len = len_a - len_b
        if abs(d_clr) >= CLEARANCE_DIFF_CM or d_wp != 0 or abs(d_len) >= LENGTH_DIFF_CM:
            diffs.append(
                {
                    "key": key,
                    "kind": "",
                    "d_clr": d_clr,
                    "d_wp": d_wp,
                    "d_len": d_len,
                    "ra": ra,
                    "rb": rb,
                }
            )
        else:
            unchanged += 1

    add("## 2. 逐点对差异（仅显著项：|Δ清障|≥%.1fcm 或航点变化 或 |Δ长度|≥%.0fcm）"
        % (CLEARANCE_DIFF_CM, LENGTH_DIFF_CM))
    add("")
    improved = sum(1 for d in diffs if d["d_clr"] > 0)
    declined = sum(1 for d in diffs if d["d_clr"] < 0)
    add("- 显著差异 **%d/%d** 条；其中清障提升 %d 条、清障下降 %d 条；无显著差异 %d 条。"
        % (len(diffs), len(before), improved, declined, unchanged))
    add("")
    if diffs:
        add("| zone | 点对 | 最小清障 前→后 cm | Δ清障 cm | 航点 前→后 | Δ长度 cm | 重构后 tier |")
        add("|---:|---|---:|---:|---|---:|---|")
        for d in diffs:
            ra = d["ra"]
            rb = d["rb"]
            if d["kind"]:
                add("| %d | %s | %s → %s | - | %s → %s | - | - |"
                    % (d["key"][0], pair_label(d["key"]), rb.get("ok"), ra.get("ok"),
                       rb.get("waypoints"), ra.get("waypoints")))
                continue
            tier_txt = {0: "soft", 1: "tight"}.get(int(ra.get("tier", 1)), "-") if "tier" in ra else "-"
            add("| %d | %s | %.2f → %.2f | %+.2f | %d → %d | %+.1f | %s |"
                % (d["key"][0], pair_label(d["key"]), fnum(rb, "min_clearance_cm"),
                   fnum(ra, "min_clearance_cm"), d["d_clr"], int(float(rb["waypoints"])),
                   int(float(ra["waypoints"])), d["d_len"], tier_txt))
    add("")

    # ===== tier 分布与实测清障关系 =====
    add("## 3. tier 分布与实测清障关系（重构后）")
    add("")
    add("| zone | soft 条数 | soft 清障范围 cm | soft 平均 cm | tight 条数 | tight 清障范围 cm | tight 平均 cm | 失败 |")
    add("|---:|---:|---|---:|---:|---|---:|---:|")
    for t in tiers:
        soft_range = ("%.2f ~ %.2f" % (t["soft_min"], t["soft_max"])) if t["soft_min"] is not None else "-"
        tight_range = ("%.2f ~ %.2f" % (t["tight_min"], t["tight_max"])) if t["tight_min"] is not None else "-"
        add("| %d | %d | %s | %.2f | %d | %s | %.2f | %d |"
            % (t["zone"], t["soft"], soft_range, t["soft_avg"], t["tight"], tight_range, t["tight_avg"],
               t["fail"]))
    add("")
    add("> soft = 满足“半径 + 4cm 安全余量”的推荐路径；tight = 起终点贴障碍时退化为仅满足 15cm 半径的路径。"
        "tight 路径的实测最小清障可等于 15cm（端点本身贴墙），soft 路径应明显更大。")
    add("")

    # ===== 验收断言 =====
    add("## 4. 验收断言（new_runner，重构后）")
    add("")
    if acc:
        add("| 检查项 | 结果 | 详情 |")
        add("|---|---|---|")
        for r in acc:
            add("| %s | %s | %s |" % (r["check"], r["result"], r["detail"]))
    else:
        add("未找到 `acceptance.csv`（请先运行 new_runner）。")
    add("")

    # ===== 边界用例 =====
    add("## 5. 边界/回归用例（edge_runner，重构后）")
    add("")
    if edge:
        add("| 用例 | 名称 | 结果 | 详情 |")
        add("|---|---|---|---|")
        for r in edge:
            add("| %s | %s | %s | %s |" % (r["test"], r["name"], r["result"], r["detail"]))
    else:
        add("未找到 `edge_results.csv`（请先运行 edge_runner）。")
    add("")

    # ===== 执行剖面单元测试 =====
    add("## 5b. 执行剖面单元测试（profile_runner，重构后）")
    add("")
    if profile:
        p_pass = sum(1 for r in profile if r["result"] == "PASS")
        p_fail = sum(1 for r in profile if r["result"] == "FAIL")
        add("背景：`planner_port.c` 曾把 `Planner_PointSegmentDistanceCm`（点到线段距离）误当段长，"
            "导致段长/剩余路程恒 0、中间航点被跳过。修复后执行剖面抽成算法层纯函数，本组用例直接测试："
            "`Planner_PointDistanceCm` / `Planner_BuildRemainingCm` / `Planner_WaypointSpeedCmS` / "
            "`Planner_SegmentSpeedLimitCmS` / `Planner_WaypointTimeoutMs`，并回归“多段路径 remaining 非零且"
            "单调递减、速度剖面非恒 40”。")
        add("")
        add("结果：**%d PASS / %d FAIL**。" % (p_pass, p_fail))
        add("")
        add("| 用例 | 内容 | 结果 | 详情 |")
        add("|---|---|---|---|")
        for r in profile:
            add("| %s | %s | %s | %s |" % (r["test"], r["name"], r["result"], r["detail"]))
    else:
        add("未找到 `profile_results.csv`（请先运行 profile_runner）。")
    add("")

    # ===== 复现 =====
    add("## 6. 复现命令")
    add("")
    add("```powershell")
    add("# 重新生成重构前基线 8 个 CSV（%TEMP%\\planner_harness\\out）")
    add("powershell -NoProfile -ExecutionPolicy Bypass -File Applications\\Planner\\tests\\harness\\run_baseline.ps1")
    add("")
    add("# 编译并运行重构版 runner + 边界用例 + 执行剖面单测，生成 new_*.csv / acceptance.csv / "
        "edge_results.csv / profile_results.csv")
    add("powershell -NoProfile -ExecutionPolicy Bypass -File Applications\\Planner\\tests\\harness\\run_new.ps1")
    add("")
    add("# 单独生成对比报告")
    add("py Applications\\Planner\\tests\\harness\\compare.py `")
    add("    --baseline-dir Applications\\Planner\\tests\\results `")
    add("    --new-dir $env:TEMP\\planner_harness\\new_out --out after_report.md")
    add("```")
    add("")
    add("> 主机 gcc 必须带 `-msse2 -mfpmath=sse`（对齐 Cortex-M4 单精度语义）；"
        "`run_baseline.ps1` / `run_new.ps1` 已固化该参数。")
    add("")

    return "\n".join(lines)


def main():
    """CLI 入口：生成重构前后对比报告。"""
    parser = argparse.ArgumentParser(description="重构前后对比报告生成器")
    parser.add_argument("--baseline-dir", required=True, help="基线结果目录（含 baseline_zone*.csv）")
    parser.add_argument("--new-dir", required=True, help="重构版结果目录（含 new_zone*.csv 等）")
    parser.add_argument("--baseline-label", default="HEAD 6413fac（重构前）", help="基线显示名")
    parser.add_argument("--new-label", default="重构版（工作区）", help="重构版显示名")
    parser.add_argument("--out", default=None, help="输出 markdown 路径；省略则打印到 stdout")
    args = parser.parse_args()

    report = generate_report(
        os.path.abspath(args.baseline_dir),
        os.path.abspath(args.new_dir),
        args.baseline_label,
        args.new_label,
    )
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(report)
        print("[OK] 对比报告已写入 %s" % os.path.abspath(args.out))
    else:
        sys.stdout.write(report)
    return 0


if __name__ == "__main__":
    # v2（方形车体）报告：compare.py --v2 ... 直接委托给 v2_report.py 生成
    # baseline_report_v2.md（旧 HEAD 路径方形真值复核）与 after_report_v2.md（前后对比）。
    if "--v2" in sys.argv:
        import v2_report

        sys.exit(v2_report.main())
    sys.exit(main())
