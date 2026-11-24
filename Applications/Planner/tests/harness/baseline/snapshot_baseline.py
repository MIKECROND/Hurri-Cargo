#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""从 git 提交导出“重构前”规划器源码快照，供 baseline_runner 编译复现。

为什么需要快照：
    baseline_runner 必须编译【重构前 HEAD 版本】的 planner_algo.c / Cstar_map_core.c /
    priority_queue.c 以及它们依赖的头文件；重构开始后工作区源码会变化，直接 -I 工作区
    会失去“基线”意义。因此固定从 git 提交 6413fac 导出到临时目录，并用 SHA-256 校验
    内容，保证任何一次复现的输入字节一致。

用法：
    py snapshot_baseline.py [--repo D:\\project\\T-Car] [--dest DIR] [--commit SHA]

默认：
    --repo    = D:\\project\\T-Car
    --dest    = %TEMP%\\planner_harness\\baseline_src
    --commit  = 6413facd5c62eb9f7bd5f50cc5852df9e02f2e95（重构前 main 的 HEAD）

退出码：0 = 全部文件导出且校验通过；1 = git 读取/哈希校验失败。
"""

import argparse
import hashlib
import os
import subprocess
import sys
import tempfile

# 重构前基线提交（tests/baseline_report.md 中记录的就是这个版本）
BASELINE_COMMIT = "6413facd5c62eb9f7bd5f50cc5852df9e02f2e95"

# (git 仓库内路径, 导出后的文件名, SHA-256)
# 说明：baseline_runner.c 里 include 的都是平铺头文件名，所以导出到同一个平面目录。
FILES = [
    ("Applications/Planner/planner_config.h", "planner_config.h",
     "c258d16b7e987366fd66b54849e04ac70c504430bd534d499b8dc9277f26b0b8"),
    ("Applications/Planner/planner_switch.h", "planner_switch.h",
     "bd731233d5030bc8203d3f78b3c6fa3a6cc6becb8e00d0f2a91516d636d0c3a0"),
    ("Applications/Planner/planner_port.h", "planner_port.h",
     "b833d059c5b8a96d1b6233ff9f8c7fca5f2d82f979847497c9280b94836df96a"),
    ("Applications/Planner/planner_port.c", "planner_port.c",
     "2ab0f5cfc92c5a3163dd30bad5f262f41569b7fcd17d685d3d145b1f112115e0"),
    ("Applications/Planner/utils/planner_algo.h", "planner_algo.h",
     "cc1cbd262fca127a574152d3917e58f76c3ed6982e377fabc6b110d8cff2af0a"),
    ("Applications/Planner/utils/planner_algo.c", "planner_algo.c",
     "3970aa147a7e5d5103afef8ca44c3af2d0a4d6230c236bcadfcdae9dc836c0fb"),
    ("Applications/Planner/utils/Cstar_map_core.h", "Cstar_map_core.h",
     "f1f951e287ba7c4b219b6d3377b0442a631391b6eedb44c5af123871bec936f2"),
    ("Applications/Planner/utils/Cstar_map_core.c", "Cstar_map_core.c",
     "ce5585002fd2eac1d28c0b6491de705b767a893058c762374283cf77bc1d4e7d"),
    ("Applications/Planner/utils/priority_queue.h", "priority_queue.h",
     "aa20d5585f1666ea898cd4a70e289721fea850c3616c29d2816f8630127b9c00"),
    ("Applications/Planner/utils/priority_queue.c", "priority_queue.c",
     "685794696c976605d219e8c021c5a7c0eedaf35f10b8b0c1e461e8cdeddda571"),
    ("Applications/nav/nav_map_data.c", "nav_map_data.c",
     "80d90a16c94c4bacd6b588120390f12fe4f7d72283a464f0953b93bd26e7d728"),
]


def git_show(repo, commit, path):
    """读取 git 对象内容（原始字节，不做编码转换）。"""
    proc = subprocess.run(
        ["git", "-C", repo, "show", "%s:%s" % (commit, path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "git show 失败（%s:%s）：%s"
            % (commit, path, proc.stderr.decode("utf-8", errors="replace").strip())
        )
    return proc.stdout


def main():
    parser = argparse.ArgumentParser(description="导出重构前规划器源码快照（供 baseline_runner 编译）")
    parser.add_argument("--repo", default=r"D:\project\T-Car", help="T-Car 仓库根目录")
    parser.add_argument(
        "--dest",
        default=os.path.join(tempfile.gettempdir(), "planner_harness", "baseline_src"),
        help="快照输出目录（默认 %%TEMP%%\\planner_harness\\baseline_src）",
    )
    parser.add_argument("--commit", default=BASELINE_COMMIT, help="基线提交 SHA（默认重构前 HEAD）")
    parser.add_argument("--no-verify", action="store_true", help="跳过 SHA-256 校验（仅调试用）")
    args = parser.parse_args()

    repo = os.path.abspath(args.repo)
    dest = os.path.abspath(args.dest)
    if not os.path.isdir(os.path.join(repo, ".git")):
        print("[FAIL] %s 不是 git 仓库根目录" % repo, file=sys.stderr)
        return 1

    os.makedirs(dest, exist_ok=True)
    verify = (not args.no_verify) and (args.commit == BASELINE_COMMIT)

    print("快照来源: %s" % repo)
    print("基线提交: %s%s" % (args.commit, "" if args.commit == BASELINE_COMMIT else "（非默认基线，跳过哈希校验）"))
    print("输出目录: %s" % dest)
    print()
    print("%-26s %-10s %s" % ("文件", "大小(B)", "SHA-256"))

    failed = 0
    for git_path, out_name, expected_sha in FILES:
        try:
            data = git_show(repo, args.commit, git_path)
        except RuntimeError as exc:
            print("[FAIL] %s" % exc, file=sys.stderr)
            failed += 1
            continue

        sha = hashlib.sha256(data).hexdigest()
        status = "OK"
        if verify and sha != expected_sha:
            status = "HASH-MISMATCH"
            failed += 1

        with open(os.path.join(dest, out_name), "wb") as fh:
            fh.write(data)
        print("%-26s %-10d %s  %s" % (out_name, len(data), sha, status))

    print()
    if failed:
        print("[FAIL] %d 个文件导出/校验失败" % failed, file=sys.stderr)
        return 1
    print("[OK] 基线快照已就绪：%s" % dest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
