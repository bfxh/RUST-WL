#!/usr/bin/env python3
"""（已撤为指路牌）本仓「一条命令跑全量验证」是 `bash scripts/gate_all.sh`——不要两个入口。

这里曾有一份我自用的 local_gate.py（快/全两档 + 机器级锁），写完才发现仓库里**已经有**
gate_all.sh（bd484d7 引入：主仓五项 + 行为门 + 金样门）。两个入口并存正是「陈旧入口陷阱」
本身，故撤回，把增量并进 gate_all.sh：

  · 补三步此前只在 CI 跑的静态门：discipline_scan / deps_lock / ci_shape
    （缺了就会「本地绿、CI 红」）；
  · clippy 对齐 CI 强化档那 6 个额外的 `-D`（此前本地只 `-D warnings`，判据比 CI 弱）；
  · 计时类门（determinism / m0_gates / m1_islands / 金样门）加**机器级独占锁**，
    把原来那条「别并发其它 cargo 构建」的注释变成执行；拿不到锁 = exit 8「未能判定」。

用法：
    bash scripts/gate_all.sh                  # 全量（含金样门，约 2-3 分钟）
    SKIP_GOLD=1 bash scripts/gate_all.sh      # 跳过金样门
"""
import sys

print("本仓用 bash scripts/gate_all.sh（一条命令跑全量验证）；本文件已撤为指路牌，"
      "内容见其 docstring。")
sys.exit(0)
