//! 任务调度抽象（§6）。
//!
//! M0 为串行实现；并行 JobSystem（工作窃取 + 无锁队列，禁 Rayon 混用）在 M1 落地。
//! 确定性要求：并行分块时 f 只写自己的索引区间，跨块归并按索引有序。

pub trait JobSystem {
    /// 对 `[start, end)` 两个闭区间参数调用 f（区间划分由实现决定）。
    fn for_each_range(&self, n: usize, f: &dyn Fn(usize, usize));
}

/// 串行实现（M0 默认；行为与未来并行实现 bit 级一致——归约都按索引有序）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SerialJobSystem;

impl JobSystem for SerialJobSystem {
    fn for_each_range(&self, n: usize, f: &dyn Fn(usize, usize)) {
        if n > 0 {
            f(0, n);
        }
    }
}
