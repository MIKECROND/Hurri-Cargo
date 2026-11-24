# 旧基线（HEAD 6413fac）方形车体可达性复核报告（v2 真值）

> 本报告用 v2 的**独立精确 SAT 真值**复核 v1 重构前 HEAD 的路径在 30×30cm 方形车体模型下的表现。
> 旧规划器按“半径 15cm 圆”建模（`PLANNER_ROBOT_RADIUS_CM=15`），对旋转/拐角过于乐观；
> 本报告按方形车体执行语义（除最后一段外保持起点 yaw，最后一段用目标 yaw）逐段 1cm 采样做 SAT。
> 输入：`tests/results/baseline_paths_zone*.csv`（v1 直线化航点，世界坐标 cm）。

## 0. 摘要

- 复核路径：**40** 条（zone1+zone2 全部有向任务点对）。
- 方形车体下穿障（一般路段 SAT 相交或端点区侵入 >1cm）的路径：**26/40** 条；
  其中一般路段相交采样 **504/7247**、端点区侵入超限采样 **0**。
- 全局最大穿透深度：**6.52cm**（MATL_PROCESSING -> QrCode @ (98.5, 98.0)cm）。
- 全局最小方形 signed clearance：**-6.52cm**（<0 即侵入）。
- 需要终点原地补转（start_yaw != goal_yaw）的路径：14 条，其中终点不满足旋转安全（欧氏 <23.22cm）**10** 条。
- 结论：v1 的“半径 15cm 圆”模型无法覆盖 30×30cm 方形车体，旧路径在方形真值下大量穿障/切角，
  必须使用 v2 方形车体规划器（本报告同时作为 v2 after 报告的对照基线）。

## 1. 逐路径方形真值明细

| zone | 点对 | yaw 前→后 | 航点 | 长度cm | v1圆最小清障 | v1判定 | SAT相交采样(一般) | 端点侵入超限采样 | 最大穿透cm | SAT最小signed cm | 终点补转安全 |
|---:|---|---:|---:|---:|---:|---|---:|---:|---:|---:|---|
| 1 | HOME -> MATL_GRAB | 0→0 | 2 | 105.9 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 1 | HOME -> MATL_PROCESSING | 0→180 | 6 | 267.7 | 13.44 | 碰撞 | 31 | 0 | 4.78 | -4.78 | **不安全(20.0)** |
| 1 | HOME -> MATL_STORAGE | 0→90 | 6 | 267.7 | 13.44 | 碰撞 | 31 | 0 | 4.78 | -4.78 | **不安全(20.0)** |
| 1 | HOME -> QrCode | 0→0 | 2 | 105.5 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 1 | MATL_GRAB -> HOME | 0→0 | 2 | 105.9 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 1 | MATL_GRAB -> MATL_PROCESSING | 0→180 | 2 | 176.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | **不安全(20.0)** |
| 1 | MATL_GRAB -> MATL_STORAGE | 0→90 | 4 | 170.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | **不安全(20.0)** |
| 1 | MATL_GRAB -> QrCode | 0→0 | 4 | 172.5 | 17.89 | 通过 | 6 | 0 | 1.52 | -1.52 | - |
| 1 | MATL_PROCESSING -> HOME | 180→0 | 6 | 270.4 | 14.14 | 碰撞 | 23 | 0 | 4.67 | -4.67 | **不安全(15.0)** |
| 1 | MATL_PROCESSING -> MATL_GRAB | 180→0 | 2 | 176.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | 安全(25.8) |
| 1 | MATL_PROCESSING -> MATL_STORAGE | 180→90 | 4 | 164.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | **不安全(20.0)** |
| 1 | MATL_PROCESSING -> QrCode | 180→0 | 4 | 172.7 | 15.91 | 通过 | 6 | 0 | 1.52 | -1.52 | 安全(25.0) |
| 1 | MATL_STORAGE -> HOME | 90→0 | 6 | 270.4 | 14.14 | 碰撞 | 23 | 0 | 4.67 | -4.67 | **不安全(15.0)** |
| 1 | MATL_STORAGE -> MATL_GRAB | 90→0 | 4 | 170.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | 安全(25.8) |
| 1 | MATL_STORAGE -> MATL_PROCESSING | 90→180 | 4 | 164.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | **不安全(20.0)** |
| 1 | MATL_STORAGE -> QrCode | 90→0 | 2 | 180.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | 安全(25.0) |
| 1 | QrCode -> HOME | 0→0 | 2 | 105.5 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 1 | QrCode -> MATL_GRAB | 0→0 | 4 | 172.5 | 17.89 | 通过 | 6 | 0 | 1.52 | -1.52 | - |
| 1 | QrCode -> MATL_PROCESSING | 0→180 | 4 | 172.7 | 16.03 | 通过 | 6 | 0 | 1.52 | -1.52 | **不安全(20.0)** |
| 1 | QrCode -> MATL_STORAGE | 0→90 | 2 | 180.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | **不安全(20.0)** |
| 2 | HOME -> MATL_GRAB | 0→0 | 6 | 273.7 | 13.44 | 碰撞 | 31 | 0 | 4.78 | -4.78 | - |
| 2 | HOME -> MATL_PROCESSING | 0→0 | 2 | 106.9 | 14.23 | 碰撞 | 18 | 0 | 2.79 | -2.79 | - |
| 2 | HOME -> MATL_STORAGE | 0→0 | 6 | 267.8 | 13.21 | 碰撞 | 49 | 0 | 4.67 | -4.67 | - |
| 2 | HOME -> QrCode | 0→0 | 2 | 105.5 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 2 | MATL_GRAB -> HOME | 0→0 | 6 | 276.4 | 14.14 | 碰撞 | 23 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_GRAB -> MATL_PROCESSING | 0→0 | 2 | 176.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | - |
| 2 | MATL_GRAB -> MATL_STORAGE | 0→0 | 4 | 170.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_GRAB -> QrCode | 0→0 | 4 | 180.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_PROCESSING -> HOME | 0→0 | 2 | 106.9 | 14.23 | 碰撞 | 18 | 0 | 2.79 | -2.79 | - |
| 2 | MATL_PROCESSING -> MATL_GRAB | 0→0 | 2 | 176.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | - |
| 2 | MATL_PROCESSING -> MATL_STORAGE | 0→0 | 4 | 164.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_PROCESSING -> QrCode | 0→0 | 4 | 172.4 | 11.18 | 碰撞 | 22 | 0 | 6.52 | -6.52 | - |
| 2 | MATL_STORAGE -> HOME | 0→0 | 6 | 267.8 | 13.21 | 碰撞 | 49 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_STORAGE -> MATL_GRAB | 0→0 | 4 | 170.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_STORAGE -> MATL_PROCESSING | 0→0 | 4 | 164.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | MATL_STORAGE -> QrCode | 0→0 | 2 | 180.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | - |
| 2 | QrCode -> HOME | 0→0 | 2 | 105.5 | 15.00 | 通过 | 0 | 0 | 0.00 | 0.00 | - |
| 2 | QrCode -> MATL_GRAB | 0→0 | 4 | 180.1 | 14.14 | 碰撞 | 14 | 0 | 4.67 | -4.67 | - |
| 2 | QrCode -> MATL_PROCESSING | 0→0 | 4 | 172.4 | 11.18 | 碰撞 | 22 | 0 | 6.52 | -6.52 | - |
| 2 | QrCode -> MATL_STORAGE | 0→0 | 2 | 180.0 | 15.00 | 碰撞 | 0 | 0 | 0.00 | 0.00 | - |

## 2. 最严重碰撞（按穿透深度）

| zone | 点对 | 最大穿透cm | 位置（世界 cm） |
|---:|---|---:|---|
| 2 | MATL_PROCESSING -> QrCode | 6.52 | (98.5, 98.0) |
| 2 | QrCode -> MATL_PROCESSING | 6.52 | (98.0, 98.5) |
| 1 | HOME -> MATL_PROCESSING | 4.78 | (25.4, 100.2) |
| 1 | HOME -> MATL_STORAGE | 4.78 | (100.2, 25.4) |
| 2 | HOME -> MATL_GRAB | 4.78 | (100.2, 25.4) |
| 1 | MATL_GRAB -> MATL_STORAGE | 4.67 | (109.7, 99.7) |
| 1 | MATL_PROCESSING -> HOME | 4.67 | (100.3, 110.3) |
| 1 | MATL_PROCESSING -> MATL_STORAGE | 4.67 | (109.7, 110.3) |
| 1 | MATL_STORAGE -> HOME | 4.67 | (110.3, 100.3) |
| 1 | MATL_STORAGE -> MATL_GRAB | 4.67 | (110.3, 100.3) |

## 3. 复现

```powershell
# 1) 重建 v1 基线路径（HEAD 6413fac 快照）
powershell -NoProfile -ExecutionPolicy Bypass -File Applications\Planner\tests\harness\run_baseline.ps1
# 2) 生成 v2 真值复核报告
py Applications\Planner\tests\harness\v2_report.py --v2 --baseline-dir Applications\Planner\tests\results --out-baseline tests\baseline_report_v2.md
```
