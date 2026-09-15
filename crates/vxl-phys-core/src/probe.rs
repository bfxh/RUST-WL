//! # 跨目标计时探针
//!
//! 相位计时（`PhaseTimings`）是纯诊断量，不参与物理与状态哈希。原生目标用
//! `std::time::Instant`；**wasm32-unknown-unknown 没有时钟实现**
//! （`Instant::now()` 会 panic，经 wasm 桥跑引擎时表现为 `unreachable` 陷阱），
//! 因此该目标下探针退化为零时长常量。
//!
//! 用法：
//! ```
//! let t0 = vxl_phys_core::probe::start();
//! // …被测代码…
//! let us = vxl_phys_core::probe::us(t0);
//! ```

#[cfg(not(target_arch = "wasm32"))]
mod imp {
    /// 计时起点（原生：真实 `Instant`）。
    pub type Stamp = std::time::Instant;

    /// 取计时起点。
    #[inline]
    pub fn start() -> Stamp {
        std::time::Instant::now()
    }

    /// 起点到现在的微秒数。
    #[inline]
    pub fn us(s: Stamp) -> u64 {
        s.elapsed().as_micros() as u64
    }

    /// 起点到现在的纳秒数。**逐次计时亚微秒区段必须用它**：`us` 会把每次
    /// 亚微秒区间截断成 0（实测：盒对裁剪单次 ≈200 ns，逐次取 µs 后累计
    /// 只剩"2.7 µs / 1361 次 = 窄相 1%"的假象；真值要用 ns 累计）。
    #[inline]
    pub fn ns(s: Stamp) -> u64 {
        s.elapsed().as_nanos() as u64
    }
}

#[cfg(target_arch = "wasm32")]
mod imp {
    /// 计时起点（wasm：零尺寸占位，无时钟可读）。
    #[derive(Clone, Copy, Debug, Default)]
    pub struct Stamp;

    /// 取计时起点（wasm：常量）。
    #[inline]
    pub fn start() -> Stamp {
        Stamp
    }

    /// 起点到现在的微秒数（wasm：恒为 0）。
    #[inline]
    pub fn us(_s: Stamp) -> u64 {
        0
    }

    /// 起点到现在的纳秒数（wasm：恒为 0）。
    #[inline]
    pub fn ns(_s: Stamp) -> u64 {
        0
    }
}

pub use imp::{ns, start, us, Stamp};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_measures_nonnegative() {
        let t0 = start();
        // 空区间允许 0；只断言不 panic 且无符号回绕。
        let _ = us(t0);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_probe_advances() {
        let t0 = start();
        std::thread::sleep(std::time::Duration::from_micros(50));
        assert!(us(t0) >= 50, "原生探针应读到真实时长");
    }

    #[cfg(target_arch = "wasm32")]
    #[test]
    fn wasm_probe_is_zero() {
        assert_eq!(us(start()), 0);
    }
}
