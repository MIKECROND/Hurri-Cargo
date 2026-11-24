#!/usr/bin/env python3
"""Generate the host-side nav simulator Markdown report from runs_*.csv.

Input : <out-dir>/runs_*.csv written by Applications/nav/tests/harness/sim_runner.c
Output: <out-path> Markdown report (Chinese), plus a short summary on stdout.

Only the Python standard library is used (py 3.x).
"""

import argparse
import csv
import glob
import math
import os
import sys

SCENARIO_ORDER = [
    "line100_30", "line100_50", "line100_70", "line100_100", "short5",
    "line100_yaw90", "diag100", "rot90", "rot180", "pass_straight",
    "pass_turn", "seq_3pt", "line30_100",
]
DELAY_ORDER = [0, 10, 20, 50, 100]
QUICK_DELAYS = [0, 50]

FIELDS = [
    "tag", "suite", "scenario", "n_legs", "delay_ms", "tau_ms", "slip",
    "ops_rate_hz", "ops_delay_ms", "rpm_quant", "motor_acc", "jitter_max_ms",
    "seed", "nav_error", "arrive_total_ms", "leg_arrive_ms", "leg_pos_err_cm",
    "leg_yaw_err_deg", "leg_true_dist_cm", "leg_settle_tail_ms",
    "leg_overshoot_cm", "leg_yaw_over_deg", "leg_lat_max_cm", "leg_osc",
    "leg_cmd_rev", "leg_max_speed", "leg_path_len_cm", "final_pos_err_cm",
    "final_yaw_err_deg", "end_pos_err_cm", "end_yaw_err_deg",
    "max_overshoot_cm", "overshoot_ratio", "max_yaw_overshoot_deg",
    "max_yaw_dev_deg", "lat_max_cm", "lat_rms_cm", "near_zone_ms",
    "near_osc_count", "cmd_reversals", "max_speed_cm_s", "path_len_cm",
    "mean_speed_cm_s", "pass_min_speed_cm_s", "nav_dt_min_ms",
    "nav_dt_max_ms", "nav_dt_mean_ms", "sim_time_ms",
]


def _float(v, default=float("nan")):
    try:
        if v is None or v == "":
            return default
        return float(v)
    except (TypeError, ValueError):
        return default


def _int(v, default=-1):
    try:
        if v is None or v == "":
            return default
        return int(float(v))
    except (TypeError, ValueError):
        return default


def _last_num(v, default=float("nan")):
    """For joined fields like '1:1234;2:800' return the last numeric token."""
    if v is None or v == "":
        return default
    s = str(v)
    token = s.split(";")[-1]
    if ":" in token:
        token = token.split(":")[-1]
    return _float(token, default)


def load_rows(out_dir):
    rows = []
    for path in sorted(glob.glob(os.path.join(out_dir, "runs_*.csv"))):
        with open(path, newline="", encoding="utf-8", errors="replace") as f:
            reader = csv.DictReader(f)
            for r in reader:
                r["_file"] = os.path.basename(path)
                rows.append(r)
    return rows


def by_tag(rows):
    tags = {}
    for r in rows:
        tags.setdefault(r.get("tag", ""), []).append(r)
    return tags


def index_by(tag_rows):
    """(scenario, delay) -> row"""
    out = {}
    for r in tag_rows:
        key = (r.get("scenario", ""), _int(r.get("delay_ms"), 0))
        out[key] = r
    return out


def scenario_rows(tag_rows, scenario):
    return [r for r in tag_rows if r.get("scenario") == scenario]


def get(tag_rows, scenario, delay):
    for r in tag_rows:
        if r.get("scenario") == scenario and _int(r.get("delay_ms"), -999) == delay:
            return r
    return None


def fmt(v, nd=2, dash="-"):
    if v is None:
        return dash
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return dash
        return ("%%.%df" % nd) % v
    return str(v)


def md_table(headers, rows):
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("|" + "|".join(["---"] * len(headers)) + "|")
    for row in rows:
        lines.append("| " + " | ".join(str(c) for c in row) + " |")
    return "\n".join(lines)


def metric(tag_rows, scenario, delay, field, last=False):
    r = get(tag_rows, scenario, delay)
    if r is None:
        return None
    if last:
        v = _last_num(r.get(field), float("nan"))
        return None if math.isnan(v) else v
    return _float(r.get(field), float("nan"))


def mean(vals):
    vals = [v for v in vals if v is not None and not math.isnan(v)]
    if not vals:
        return None
    return sum(vals) / len(vals)


def mean_metric(tag_rows, delay, field, scenarios=None, last=False):
    scenarios = scenarios or SCENARIO_ORDER
    vals = []
    for sc in scenarios:
        v = metric(tag_rows, sc, delay, field, last=last)
        if v is not None:
            vals.append(v)
    return mean(vals)


def cost(row):
    """Composite cost used to rank parameter variants (time + accuracy + smoothness)."""
    if row is None:
        return None
    t = _float(row.get("arrive_total_ms"), 1e6) / 1000.0
    e = _float(row.get("final_pos_err_cm"), 5.0)
    y = _float(row.get("final_yaw_err_deg"), 20.0)
    ov = max(0.0, _float(row.get("max_overshoot_cm"), 5.0))
    osc = max(0.0, _float(row.get("near_osc_count"), 5.0))
    conv = 1.0 if _int(row.get("nav_error"), 1) == 0 else 10.0
    return conv * (t + 2.0 * e + 0.05 * y + 0.3 * ov + 0.1 * osc)


def tag_has(rows, tag):
    return tag in rows and len(rows[tag]) > 0


def section_delay_tables(tags, variants):
    """Per-metric tables: rows = scenario, columns = latest delay series, one block per variant."""
    out = []
    out.append("### 2.1 到位时间 arrive_total_ms（ms，越小越快）\n")
    for tag, label in variants:
        if not tag_has(tags, tag):
            continue
        rows = []
        for sc in SCENARIO_ORDER:
            cells = [fmt(metric(tags[tag], sc, d, "arrive_total_ms")) for d in DELAY_ORDER]
            if all(c == "-" for c in cells):
                continue
            rows.append([sc] + cells)
        out.append("**%s**\n" % label)
        out.append(md_table(["场景"] + ["d=%dms" % d for d in DELAY_ORDER], rows))
        out.append("")

    out.append("### 2.2 最终位置误差 final_pos_err_cm（cm，越小越准）\n")
    for tag, label in variants:
        if not tag_has(tags, tag):
            continue
        rows = []
        for sc in SCENARIO_ORDER:
            cells = [fmt(metric(tags[tag], sc, d, "final_pos_err_cm")) for d in DELAY_ORDER]
            if all(c == "-" for c in cells):
                continue
            rows.append([sc] + cells)
        out.append("**%s**\n" % label)
        out.append(md_table(["场景"] + ["d=%dms" % d for d in DELAY_ORDER], rows))
        out.append("")

    out.append("### 2.3 最终航向误差 final_yaw_err_deg（deg）\n")
    for tag, label in variants:
        if not tag_has(tags, tag):
            continue
        rows = []
        for sc in SCENARIO_ORDER:
            cells = [fmt(metric(tags[tag], sc, d, "final_yaw_err_deg")) for d in DELAY_ORDER]
            if all(c == "-" for c in cells):
                continue
            rows.append([sc] + cells)
        out.append("**%s**\n" % label)
        out.append(md_table(["场景"] + ["d=%dms" % d for d in DELAY_ORDER], rows))
        out.append("")
    return "\n".join(out)


def section_delay_summary(tags, variants):
    out = []
    out.append("### 2.4 延迟-性能汇总（所有场景平均）\n")
    header = ["延迟"] + [h for _, label in variants for h in (
        "%s时间s" % label, "%s位置cm" % label, "%s超调cm" % label,
        "%s振荡" % label, "%s误差数" % label)]
    rows = []
    for d in DELAY_ORDER:
        row = ["%dms" % d]
        for tag, label in variants:
            if not tag_has(tags, tag):
                row += ["-", "-", "-", "-", "-"]
                continue
            row += [
                fmt((mean_metric(tags[tag], d, "arrive_total_ms") or float("nan")) / 1000.0, 2),
                fmt(mean_metric(tags[tag], d, "final_pos_err_cm"), 2),
                fmt(mean_metric(tags[tag], d, "max_overshoot_cm"), 2),
                fmt(mean_metric(tags[tag], d, "near_osc_count"), 1),
                "%d" % sum(1 for sc in SCENARIO_ORDER
                           if (get(tags[tag], sc, d) or {}).get("nav_error", "1") not in ("0", "0.0")),
            ]
        rows.append(row)
    out.append(md_table(header, rows))
    out.append("")
    out.append("*误差数 = 该延迟下未正常到达（超时/ERROR/未完成）的场景数。*\n")

    # stability threshold
    out.append("### 2.5 失稳起点（按场景平均值判定）\n")
    out.append(md_table(
        ["版本", "首次出现超调>1cm", "首次出现位置误差>1cm", "首次出现振荡>0.5次", "首次出现未到达"],
        [[label,
          _first_bad(tags, tag, "max_overshoot_cm", 1.0),
          _first_bad(tags, tag, "final_pos_err_cm", 1.0),
          _first_bad(tags, tag, "near_osc_count", 0.5),
          _first_bad_err(tags, tag)] for tag, label in variants if tag_has(tags, tag)]))
    out.append("")
    out.append("*“-”表示到 100ms 延迟仍未出现该现象。*\n")
    return "\n".join(out)


def _first_bad(tags, tag, field, threshold):
    for d in DELAY_ORDER:
        v = mean_metric(tags[tag], d, field)
        if v is not None and v > threshold:
            return "%dms" % d
    return "-"


def _first_bad_err(tags, tag):
    for d in DELAY_ORDER:
        bad = 0
        for sc in SCENARIO_ORDER:
            r = get(tags[tag], sc, d)
            if r is not None and _int(r.get("nav_error"), 1) != 0:
                bad += 1
        if bad:
            return "%dms(%d个)" % (d, bad)
    return "-"


def section_scenario_notes(tags, variants):
    """Highlight per-scenario worst offenders at high delay."""
    if not tag_has(tags, "current"):
        return ""
    out = ["### 2.6 高延迟下表现最差的场景（current, d=100ms）\n"]
    rows = []
    for sc in SCENARIO_ORDER:
        r = get(tags["current"], sc, 100)
        if r is None:
            continue
        rows.append([
            sc,
            fmt(_float(r.get("arrive_total_ms"), float("nan")), 0),
            fmt(_float(r.get("final_pos_err_cm"), float("nan")), 2),
            fmt(_float(r.get("final_yaw_err_deg"), float("nan")), 2),
            fmt(_float(r.get("max_overshoot_cm"), float("nan")), 2),
            "%d" % _int(r.get("near_osc_count"), 0),
            "%s" % ("OK" if _int(r.get("nav_error"), 1) == 0 else "FAIL"),
        ])
    rows.sort(key=lambda x: x[4] if x[4] != "-" else "-1", reverse=True)
    out.append(md_table(["场景", "时间ms", "位置cm", "yaw°", "超调cm", "振荡", "结果"], rows))
    out.append("")
    return "\n".join(out)


def parse_variant_tag(tag, prefix):
    """sw_<name>_<value> / mx_<NAME>_<value> -> (name, value_str)."""
    if not tag.startswith(prefix):
        return None
    body = tag[len(prefix):]
    if "_" not in body:
        return None
    name, value = body.rsplit("_", 1)
    return name, value


def section_tunable_sweeps(tags):
    if not tag_has(tags, "current_quick"):
        return ""
    default = index_by(tags["current_quick"])
    groups = {}
    for tag, rows in tags.items():
        parsed = parse_variant_tag(tag, "sw_")
        if parsed is None or parsed[0] == "stop_settle_ms":
            continue
        groups.setdefault(parsed[0], []).append((parsed[1], tag))

    out = ["### 3.1 运行时整定参数敏感度（--tune，默认=current_quick，目标：综合代价最小）\n"]
    out.append("*综合代价 = 到位时间(s) + 2×位置误差(cm) + 0.05×yaw误差(deg) + 0.3×超调(cm) + 0.1×振荡次数；"
               "比值 = 该取值代价 / 默认值代价（<1 更好）。扫描用 quick 套件（9 场景×延迟{0,50}）。*\n")
    ranking = []
    for name, variants in groups.items():
        ratios = []
        for v, tag in variants:
            rows = tags[tag]
            idx = index_by(rows)
            vals = []
            times = []
            errs = []
            for key, drow in default.items():
                vrow = idx.get(key)
                if vrow is None:
                    continue
                c_default = cost(drow)
                c_var = cost(vrow)
                if c_default and c_var is not None and c_default > 1e-6:
                    vals.append(c_var / c_default)
                times.append(_float(vrow.get("arrive_total_ms")) / 1000.0)
                errs.append(_float(vrow.get("final_pos_err_cm")))
            if not vals:
                continue
            ratios.append((v, mean(vals), mean(times), mean(errs)))
        if not ratios:
            continue
        base_mean = ratios[0][1] if ratios else None
        best = min(ratios, key=lambda x: x[1] if x[1] is not None else 9e9)
        worst = max(ratios, key=lambda x: x[1] if x[1] is not None else 9e9)
        spread = None
        if best[1] is not None and worst[1] is not None and best[1] > 1e-9:
            spread = (worst[1] - best[1]) / best[1] * 100.0
        ranking.append((spread if spread is not None else 0.0, name, ratios, best, worst))
    ranking.sort(key=lambda x: -x[0])

    rows = []
    for spread, name, ratios, best, worst in ranking:
        cells = []
        for v, ratio, t, e in ratios:
            mark = " **" if v == best[0] else ""
            cells.append("%s=%.3f%s" % (v, ratio, mark))
        rows.append([name, " / ".join(cells), "%.1f%%" % spread])
    out.append(md_table(["参数", "取值=代价比（**=最优）", "取值间差异"], rows))
    out.append("")

    out.append("### 3.2 推荐值（按综合代价比）\n")
    rec_rows = []
    for spread, name, ratios, best, worst in ranking:
        if spread < 1.0:
            continue
        rec_rows.append([name, best[0], "%.3f" % best[1], "%.1f%%" % spread])
    out.append(md_table(["参数", "推荐取值", "代价比", "敏感度(极差/最优)"], rec_rows))
    out.append("")
    out.append("*注：代价比接近 1 表示该参数在仿真模型下不敏感，不建议改默认值。*\n")
    return "\n".join(out)


def section_macro_sweeps(tags):
    if not tag_has(tags, "current_quick"):
        return ""
    default = index_by(tags["current_quick"])
    groups = {}
    for tag, rows in tags.items():
        if not tag.startswith("mx_"):
            continue
        body = tag[len("mx_"):]
        if "_" not in body:
            continue
        name, value = body.rsplit("_", 1)
        groups.setdefault(name, []).append((value.replace("p", "."), tag))

    out = ["### 3.3 编译期宏扫描（不能运行时改的 NAV_* 宏）\n"]
    if not groups:
        out.append("*未运行宏扫描（-NoSweep）。*\n")
        return "\n".join(out)
    rows = []
    for name, variants in groups.items():
        cells = []
        for v, tag in variants:
            idx = index_by(tags[tag])
            vals = []
            for key, drow in default.items():
                vrow = idx.get(key)
                if vrow is None:
                    continue
                c_default = cost(drow)
                c_var = cost(vrow)
                if c_default and c_var is not None and c_default > 1e-6:
                    vals.append(c_var / c_default)
            if vals:
                cells.append("%s=%.3f" % (v, mean(vals)))
        rows.append([name, " / ".join(cells)])
    out.append(md_table(["宏", "取值=代价比"], rows))
    out.append("")
    return "\n".join(out)


def section_stop_settle(tags):
    out = []
    groups = []
    for tag in tags:
        parsed = parse_variant_tag(tag, "sw_")
        if parsed is not None and parsed[0] == "stop_settle_ms":
            groups.append((int(float(parsed[1])), tag))
    if not groups:
        out.append("### 4 STOPPING 最少等待 stop_settle_ms 对比\n")
        out.append("*未运行（-NoSweep）。*\n")
        return "\n".join(out)
    groups.sort()

    def agg(tag):
        rows = tags[tag]
        return {
            "time": mean([_float(r.get("arrive_total_ms")) / 1000.0 for r in rows]),
            "pos": mean([_float(r.get("final_pos_err_cm")) for r in rows]),
            "pos_max": max([_float(r.get("final_pos_err_cm")) for r in rows] or [float("nan")]),
            "yaw": mean([_float(r.get("final_yaw_err_deg")) for r in rows]),
            "yaw_max": max([_float(r.get("final_yaw_err_deg")) for r in rows] or [float("nan")]),
            "tail": mean([_last_num(r.get("leg_settle_tail_ms")) for r in rows]),
            "osc": mean([_float(r.get("near_osc_count")) for r in rows]),
            "fail": sum(1 for r in rows if _int(r.get("nav_error"), 1) != 0),
        }

    default = agg("current_quick") if tag_has(tags, "current_quick") else None
    rows = []
    for value, tag in groups:
        a = agg(tag)
        rows.append([
            "%d (默认150)" % value if value == 150 else "%d" % value,
            fmt(a["time"], 2), fmt(a["pos"], 3), fmt(a["pos_max"], 2),
            fmt(a["yaw"], 2), fmt(a["yaw_max"], 2),
            fmt(a["tail"], 0) if a["tail"] is not None else "-",
            fmt(a["osc"], 1), "%d" % a["fail"],
        ])
    out.append("### 4 STOPPING 最少等待 stop_settle_ms 对比（当前代码，quick 套件均值）\n")
    out.append(md_table(["stop_settle_ms", "平均时间s", "平均位置误差cm", "最大位置误差cm",
                         "平均yaw误差deg", "最大yaw误差deg", "STOPPING尾延迟ms", "平均振荡", "未到达数"], rows))
    out.append("")
    if default is not None:
        out.append("默认 150ms 对比：位置误差 %.3f→%.3fcm，yaw %.2f→%.2f°，平均时间 %.2f→%.2fs。"
                   % (default["pos"], agg(groups[0][1])["pos"], default["yaw"], agg(groups[0][1])["yaw"],
                      default["time"], agg(groups[0][1])["time"]))
        out.append("")
    return "\n".join(out)


def section_uncertainty(tags):
    out = ["### 5 模型不确定性（tau / 打滑 / OPS 帧率 / 控制周期抖动）\n"]
    if not tag_has(tags, "current_unc"):
        out.append("*未运行（-NoUncertainty）。*\n")
        return "\n".join(out)
    rows = tags["current_unc"]
    base = "line100_70"

    def agg(sel, field):
        vals = [_float(r.get(field)) for r in sel]
        return mean(vals)

    groups = {}
    for r in rows:
        key = (_float(r.get("tau_ms")), _float(r.get("slip")), _float(r.get("ops_rate_hz")),
               _int(r.get("delay_ms"), 0), _int(r.get("jitter_max_ms"), 0))
        groups.setdefault(key, []).append(r)

    out.append("#### 5.1 轮速一阶滞后 tau（其余参数默认）\n")
    tau_rows = []
    for tau in sorted(set(k[0] for k in groups)):
        for d in (0, 50):
            sel = []
            for k, v in groups.items():
                if k[0] == tau and k[1] == 0.0 and k[2] == 50.0 and k[3] == d and k[4] == 0:
                    sel.extend(v)
            tau_rows.append([fmt(tau, 0), "%d" % d,
                             fmt(mean([_float(r.get("arrive_total_ms")) / 1000.0 for r in sel]), 2),
                             fmt(agg(sel, "final_pos_err_cm"), 2),
                             fmt(agg(sel, "max_overshoot_cm"), 2),
                             fmt(agg(sel, "near_osc_count"), 1)])
    out.append(md_table(["tau_ms", "延迟ms", "avg时间s", "avg位置cm", "avg超调cm", "avg振荡"], tau_rows))
    out.append("")

    def dim_table(title, key_index, keys, ops_rate=None, slip=None, tau=None):
        lines = ["#### %s\n" % title]
        rows = []
        for val in keys:
            for d in (0, 50):
                sel = []
                for k, v in groups.items():
                    if k[key_index] != val:
                        continue
                    if k[3] != d or k[4] != 0:
                        continue
                    if ops_rate is not None and k[2] != ops_rate:
                        continue
                    if slip is not None and k[1] != slip:
                        continue
                    if tau is not None and k[0] != tau:
                        continue
                    sel.extend(v)
                if not sel:
                    continue
                rows.append([fmt(val, 2), "%d" % d,
                             fmt(mean([_float(r.get("arrive_total_ms")) / 1000.0 for r in sel]), 2),
                             fmt(agg(sel, "final_pos_err_cm"), 2),
                             fmt(agg(sel, "max_overshoot_cm"), 2),
                             fmt(agg(sel, "max_yaw_dev_deg"), 2)])
        lines.append(md_table(["取值", "延迟ms", "avg时间s", "avg位置cm", "avg超调cm", "yaw偏差deg"], rows))
        lines.append("")
        return "\n".join(lines)

    out.append(dim_table("5.2 轮地打滑 slip（tau=60ms, OPS=50Hz）", 1, [0.0, 0.05, 0.10], tau=60.0))
    out.append(dim_table("5.3 OPS 更新率 ops_rate_hz（tau=60ms, slip=0）", 2, [20.0, 50.0, 100.0], tau=60.0))
    out.append(dim_table("5.4 打滑 10% + tau=120ms 组合", 2, [20.0, 50.0, 100.0], tau=120.0, slip=0.10))

    out.append("#### 5.5 控制周期抖动（模拟命令队列阻塞 Nav_Task，jitter 0~8ms@35%）\n")
    jrows = []
    for d in (0, 20, 50):
        for bit in (0, 8):
            sel = []
            for k, v in groups.items():
                if k[3] == d and k[4] == bit and k[0] == 60.0 and k[1] == 0.0 and k[2] == 50.0:
                    sel.extend(v)
            if not sel:
                continue
            jrows.append(["%dms" % d, "%dms" % bit,
                          fmt(mean([_float(r.get("arrive_total_ms")) / 1000.0 for r in sel]), 2),
                          fmt(agg(sel, "final_pos_err_cm"), 2),
                          fmt(agg(sel, "max_overshoot_cm"), 2),
                          fmt(mean([_float(r.get("nav_dt_max_ms")) for r in sel]), 1)])
    out.append(md_table(["命令延迟", "周期抖动上限", "avg时间s", "avg位置cm", "avg超调cm", "实测最大周期ms"], jrows))
    out.append("")
    return "\n".join(out)


def section_acceptance(tags):
    out = ["### 6 验收断言\n"]
    rows = []

    def add(name, ok, detail):
        rows.append([name, "PASS" if ok else "FAIL/WARN", detail])

    if not tag_has(tags, "current"):
        add("current 基线套件存在", False, "缺少 runs_current.csv")
        out.append(md_table(["检查", "结果", "说明"], rows))
        return "\n".join(out)
    cur = tags["current"]

    d0 = [r for r in cur if _int(r.get("delay_ms"), -1) == 0]
    fails = [r for r in d0 if _int(r.get("nav_error"), 1) != 0]
    add("延迟 0ms：全部场景正常到达", len(fails) == 0, "%d/%d 场景失败" % (len(fails), len(d0)))

    if d0:
        worst = max(d0, key=lambda r: _float(r.get("final_pos_err_cm"), 0))
        add("延迟 0ms：最大位置误差 < 2cm", _float(worst.get("final_pos_err_cm"), 9) < 2.0,
            "worst=%s %.2fcm" % (worst.get("scenario"), _float(worst.get("final_pos_err_cm"), 0)))
        worstd = [r for r in d0 if r.get("scenario", "").startswith("line100_")]
        if worstd:
            worst2 = max(worstd, key=lambda r: _float(r.get("final_pos_err_cm"), 0))
            add("延迟 0ms：直线场景位置误差 < 1cm", _float(worst2.get("final_pos_err_cm"), 9) < 1.0,
                "worst=%s %.2fcm" % (worst2.get("scenario"), _float(worst2.get("final_pos_err_cm"), 0)))

    if tag_has(tags, "baseline"):
        miss = [sc for sc in SCENARIO_ORDER if get(tags["baseline"], sc, 0) is None]
        add("基线快照可复现（场景齐全）", len(miss) == 0,
            "缺失: %s" % ",".join(miss) if miss else "13/13 场景")
    out.append(md_table(["检查", "结果", "说明"], rows))
    out.append("")
    return "\n".join(out)


def section_conclusions(tags, variants):
    out = ["### 7 数据驱动结论\n"]
    bullets = []

    def agg_tag(tag, d, field):
        if not tag_has(tags, tag):
            return None
        return mean_metric(tags[tag], d, field)

    for tag, label in variants:
        if not tag_has(tags, tag):
            continue
        t0 = agg_tag(tag, 0, "arrive_total_ms")
        t100 = agg_tag(tag, 100, "arrive_total_ms")
        e0 = agg_tag(tag, 0, "final_pos_err_cm")
        e100 = agg_tag(tag, 100, "final_pos_err_cm")
        o0 = agg_tag(tag, 0, "max_overshoot_cm")
        o100 = agg_tag(tag, 100, "max_overshoot_cm")
        if t0 is not None and t100 is not None:
            bullets.append("- **%s**：命令延迟 0→100ms，平均到位时间 %.2fs→%.2fs（%+.0f%%），"
                           "平均位置误差 %.2f→%.2fcm，平均超调 %.2f→%.2fcm。"
                           % (label, t0 / 1000.0, t100 / 1000.0, (t100 - t0) / max(t0, 1) * 100,
                              e0, e100, o0, o100))
    if tag_has(tags, "baseline") and tag_has(tags, "current"):
        for d in DELAY_ORDER:
            tb = agg_tag("baseline", d, "arrive_total_ms")
            tc = agg_tag("current", d, "arrive_total_ms")
            eb = agg_tag("baseline", d, "final_pos_err_cm")
            ec = agg_tag("current", d, "final_pos_err_cm")
            if None in (tb, tc, eb, ec):
                continue
            bullets.append("- 改动前后对比 d=%dms：时间 %.2fs→%.2fs（%+.0f%%），位置误差 %.2f→%.2fcm。"
                           % (d, tb / 1000.0, tc / 1000.0, (tc - tb) / max(tb, 1) * 100, eb, ec))
    out.append("\n".join(bullets) if bullets else "*无足够数据。*")
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rows = load_rows(args.out_dir)
    if not rows:
        print("ERROR: no runs_*.csv found in %s" % args.out_dir, file=sys.stderr)
        return 2
    tags = by_tag(rows)

    variants = [("baseline", "基线(冻结)"), ("current", "当前改动版")]

    parts = []
    parts.append("# nav 控制主机端仿真报告（自动生成）\n")
    parts.append("本报告由 `Applications/nav/tests/harness/report.py` 从 `runs_*.csv` 自动生成。"
                 "仿真链路 = 真实 `nav_core.c` 控制 + 主机端被控对象模型（真实麦轮逆运动学 → 整数 RPM 量化 → "
                 "电机加速度斜坡 → 轮速一阶滞后 → 打滑 → 位姿积分 → OPS 测量/OPS 帧龄 → Loc_Get）。"
                 "详细方法与模型假设见 `Applications/nav/tests/README.md`。\n")
    parts.append("> 模型是理想地面 + 一阶轮速滞后 + 纯延迟，**不是实车标定**；结论用于比较方案/找趋势，"
                 "绝对值需实车校核。\n")

    parts.append(section_acceptance(tags))
    parts.append(section_delay_tables(tags, variants))
    parts.append(section_delay_summary(tags, variants))
    parts.append(section_scenario_notes(tags, variants))
    parts.append(section_tunable_sweeps(tags))
    parts.append(section_macro_sweeps(tags))
    parts.append(section_stop_settle(tags))
    parts.append(section_uncertainty(tags))
    parts.append(section_conclusions(tags, variants))

    text = "\n".join(p for p in parts if p)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(text)

    # short stdout summary
    print("[report] %s (%d rows, %d tags)" % (args.out, len(rows), len(tags)))
    for tag in sorted(tags):
        print("  %-24s %5d rows" % (tag, len(tags[tag])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
