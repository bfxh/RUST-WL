//! loom 并发原语模型（施工令 T4：`cfg(loom)` 特性测试）。
//!
//! 现状（M0）：库内**没有手写同步原语**——唯一跨线程共享状态是
//! `ScopedPool::for_each_range` 的 `AtomicUsize::fetch_add` 分块领取
//! （`vxl-phys-core/src/schedule.rs`）。本文件以 loom 原子逐字镜像该**领取协议**
//! （fetch_add × chunk 取块 → 越界即止），并验证其核心不变量：**每个块恰好被
//! 领取一次**（无重复、无遗漏——重复 = 并行写同槽位，是确定性契约的底线）。
//!
//! **触发条件**：任何手写同步原语（自研原子/unsafe 队列/持久工作窃取池，§5 与
//! `schedule.rs` 注释的 M2/M3 升级路径）进入库内时，必须在同一 PR 内为此文件
//! 增加对应原语的真实 loom 模型——本文件的协议镜像随即升级为代码级建模。
//!
//! 本地/CI 运行（务必带抢占上界，否则状态空间爆炸）：
//! `RUSTFLAGS="--cfg loom" LOOM_MAX_PREEMPTIONS=3 cargo test -p vxl-phys-core --test loom_primitives`

#![cfg(loom)]

use loom::sync::atomic::{AtomicUsize, Ordering};
use loom::sync::Arc;
use loom::thread;

/// 镜像 ScopedPool 的领取式分块：`next.fetch_add(1)` × chunk = 起始下标。
#[test]
fn chunk_claim_is_exactly_once() {
    loom::model(|| {
        const N: usize = 6; // 元素数（小规模让 loom 状态空间可穷举）
        const CHUNK: usize = 2;
        const THREADS: usize = 3;

        let next = Arc::new(AtomicUsize::new(0));
        let claimed = Arc::new(loom::sync::Mutex::new(Vec::<usize>::new()));

        let mut handles = Vec::new();
        for _ in 0..THREADS {
            let next = next.clone();
            let claimed = claimed.clone();
            handles.push(thread::spawn(move || loop {
                let s = next.fetch_add(1, Ordering::Relaxed) * CHUNK;
                if s >= N {
                    break;
                }
                claimed.lock().unwrap().push(s);
            }));
        }
        for h in handles {
            h.join().unwrap();
        }

        let mut got = claimed.lock().unwrap().clone();
        got.sort_unstable();
        // 恰好领满 [0, CHUNK, ...] 全部起始点，且无重复。
        let expect: Vec<usize> = (0..N).step_by(CHUNK).collect();
        assert_eq!(got, expect, "块领取必须恰好一次（无重复/无遗漏）");
    });
}
