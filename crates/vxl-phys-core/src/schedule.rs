//! 任务调度（§6：单一 JobSystem；禁 Rayon 与自研池混用）。
//!
//! ## 现状与架构结论（M1 实测，2026-09-12）
//!
//! 实现 = **作用域分块并行**（`std::thread::scope`，每并行区域建立 worker）：
//! - 物理各相的并行段全是「借用调用方状态 + 按离散下标写槽位」——借用闭包
//!   进不了跨相持久的 `'static` 作业队列。**已系统验证**：把队列/worker
//!   做成「每步一次、各相共享」的持久池（crossbeam-deque 无锁工作窃取，
//!   借用作业按 `'scope`/`'env` 双生命周期装盒）在 Safe Rust 下**不可行**：
//!   任务必须按「步」装盒，而各相数据按「调用」借用（后相 `&mut` 与前相
//!   共享借用冲突，NLL 无法把借用延长到整步）；绕开该约束需要 unsafe
//!   （生命周期擦除）或把全管线改成 SPMD 分片 + 屏障（每条并行循环改为
//!   按 worker 分片的数据视图）。
//! - 代价（实测，Windows，spawn ≈ 90µs）：每帧 ≤4 个并行区域 × (threads−1)
//!   次 spawn ≈ 1.9ms @8 线程；已用 `min_parallel` 门槛（工作 < 门槛走
//!   inline）避免小区域净亏。
//! - 升级路径（M2/M3 二选一）：
//!   (a) SPEC §6 允许「Unsafe 并发过 loom」——在独立调度 crate 内放宽
//!       `forbid(unsafe_code)`、做生命周期擦除 + loom 验证，得真·持久池；
//!   (b) SPMD 重构：管线各相改为「按 worker 分片 + 屏障」的固定循环
//!       （零 unsafe，但引擎各相 API 变成分片视图）。
//! - 确定性契约（§5）：`f(start,end)` 只写自己区间的槽位；跨块归并按索引
//!   有序（`chunks_mut` 保序）；块内纯函数——结果与串行 bit 级一致
//!   （`parallel_matches_serial_bitwise` 测试守门）。

//!
//! M1 实现 = **作用域分块并行**（`std::thread::scope`）：
//! - 物理各相的并行段全是「借用调用方状态 + 按离散下标写槽位」——借用闭包
//!   进不了持久 worker 池的 `'static` 作业队列；作用域线程零 unsafe（本仓
//!   `#![forbid(unsafe_code)]`），线程启动开销 ≈ 10µs/个，每帧 ≤4 相 ×
//!   (threads−1) 个，占比 <1%；
//! - 确定性契约（§5）：`f(start,end)` 只写自己区间的槽位；跨块归并按索引
//!   有序（`chunks_mut` 保序）；块内纯函数——结果与串行 bit 级一致；
//! - 真正的工作窃取无锁队列（crossbeam-deque）留给 M2 的 `'static` 作业
//!   （GPU 提交/资产）或经 loom 审查的 unsafe 池（§6），本版不引入。

/// 调度抽象（依赖注入，§1：核心不绑定具体实现）。
pub trait JobSystem: Send + Sync {
    /// worker 数（含主线程参与）。
    fn threads(&self) -> usize;

    /// 并行分块遍历 `[0, n)`。`f(start, end)` 只允许写自己区间的槽位。
    /// `n` 低于并行门槛或 `threads ≤ 1` 时 inline 串行（结果 bit 级一致）。
    fn for_each_range(&self, n: usize, f: &(dyn Fn(usize, usize) + Send + Sync)) {
        if n > 0 {
            f(0, n);
        }
    }
}

/// 串行实现（默认；行为与并行实现 bit 级一致——契约见模块注释）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SerialJobSystem;

impl JobSystem for SerialJobSystem {
    fn threads(&self) -> usize {
        1
    }
}

/// 作用域分块并行实现。无状态；每次调用临时开 `threads−1` 个 scoped worker，
/// 主线程同时参与消费，块边界由 `next` 原子领取（负载均衡）。
#[derive(Clone, Copy, Debug)]
pub struct ScopedPool {
    threads: usize,
    /// 低于此规模不开线程（线程启动开销 > 收益）。
    pub min_parallel: usize,
}

impl ScopedPool {
    pub fn new(threads: usize) -> Self {
        Self {
            threads: threads.max(1),
            min_parallel: 2048,
        }
    }
}

impl JobSystem for ScopedPool {
    fn threads(&self) -> usize {
        self.threads
    }

    fn for_each_range(&self, n: usize, f: &(dyn Fn(usize, usize) + Send + Sync)) {
        if n == 0 {
            return;
        }
        if self.threads <= 1 || n < self.min_parallel {
            f(0, n);
            return;
        }
        let chunk = (n / (self.threads * 4)).max(1);
        let next = std::sync::atomic::AtomicUsize::new(0);
        let grab = |next: &std::sync::atomic::AtomicUsize| -> Option<(usize, usize)> {
            let s = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed) * chunk;
            if s >= n {
                None
            } else {
                Some((s, (s + chunk).min(n)))
            }
        };
        std::thread::scope(|s| {
            for _ in 1..self.threads {
                s.spawn(|| {
                    while let Some((a, b)) = grab(&next) {
                        f(a, b);
                    }
                });
            }
            while let Some((a, b)) = grab(&next) {
                f(a, b);
            }
        });
    }
}

/// 并行执行互不相交的**可变**段（岛级求解 gather→solve→scatter 用）：
/// 第 i 段独享 `&mut outs[i]`，共享只读 `&inputs[i]`。段间无浮点交互（§5）。
/// 借用安全（`thread::scope`），无 unsafe；`threads ≤ 1` 或单段时主线程自做。
pub fn run_mut_sections<T, U, F>(inputs: &[T], outs: &mut [U], threads: usize, f: F)
where
    T: Sync,
    U: Send,
    F: Fn(usize, &T, &mut U) + Sync + Send,
{
    if threads <= 1 || inputs.len() <= 1 {
        for (i, (t, u)) in inputs.iter().zip(outs.iter_mut()).enumerate() {
            f(i, t, u);
        }
        return;
    }
    std::thread::scope(|s| {
        let f = &f;
        // 前面各段派发，最后一段留主线程；段数 > threads 时主线程做满后继续派发。
        let spawned = std::cell::Cell::new(0usize);
        let total = inputs.len();
        for (i, (t, u)) in inputs.iter().zip(outs.iter_mut()).enumerate() {
            let do_spawn = i + 1 != total && spawned.get() + 1 < threads;
            if do_spawn {
                spawned.set(spawned.get() + 1);
                let (t, u) = (&*t, u);
                s.spawn(move || f(i, t, u));
            } else {
                f(i, t, u);
            }
        }
    });
}

/// 按 `threads` 分段并行遍历 `&mut [U]`（AABB 计算/接触输出等按槽位写）。
/// **spawn 数被钳制在 threads−1**（每次 spawn ≈ 100µs 固定开销，禁止按
/// chunk 数爆炸式开线程）；`len < min_parallel` 时主线程串行（开销 > 收益）。
pub fn for_each_chunk_mut<U, F>(items: &mut [U], threads: usize, min_parallel: usize, f: F)
where
    U: Send,
    F: Fn(usize, usize, &mut [U]) + Sync + Send,
{
    let n = items.len();
    if n == 0 {
        return;
    }
    if threads <= 1 || n < min_parallel {
        f(0, n, items);
        return;
    }
    let chunk = n.div_ceil(threads);
    let f = &f;
    std::thread::scope(|s| {
        let n_chunks = items.len().div_ceil(chunk);
        for (ci, slice) in items.chunks_mut(chunk).enumerate() {
            let start = ci * chunk;
            // 最后一段留给主线程（其余派发，spawn 总数 ≤ threads−1）。
            if ci + 1 == n_chunks {
                f(start, slice.len(), slice);
            } else {
                s.spawn(move || f(start, slice.len(), slice));
            }
        }
    });
}
