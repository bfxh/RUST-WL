//! # vxl-phys-gpu
//!
//! GPU 后端（§8，纯 Rust 路线）—— M4。统一入口 trait；主实现 = wgpu
//! （Vulkan/Metal/DX12/WebGPU），着色器 = rust-gpu → SPIR-V；NVIDIA 极致路径 =
//! cuda-oxide SIMT + C ABI 主机桥（唯一允许的独立后端）。
//!
//! 硬性要求：
//! - **必须 CPU 回退且结果一致**（同档参数下位姿哈希一致，容差 = 严格模式量化容差）；
//! - 必须出 GPU vs CPU 对比数据；禁止单一厂商绑定。
//!
//! 依赖（wgpu / cuda-oxide）在 M4 引入，避免骨架期拖慢编译。

#![forbid(unsafe_code)]

pub mod bbox;
pub mod grid;
pub mod pipeline;
pub mod probe;

/// GPU 后端统一入口。
pub trait PhysGpuBackend: Send + Sync {
    fn name(&self) -> &'static str;
    /// 是否可用（设备枚举 + 特性检测）。
    fn available(&self) -> bool;
    /// 批量任务提交入口（粒子/流体/布料/批量碰撞/批量 memfind）。
    /// M4 实现；M0 返回 false 表示未就绪。
    fn submit(&self, _tag: &'static str) -> bool {
        false
    }
}

/// CPU 回退占位：任何 GPU 路径都必须能在无 GPU 环境退化到与 CPU 一致的路径。
pub struct CpuFallback;

impl PhysGpuBackend for CpuFallback {
    fn name(&self) -> &'static str {
        "cpu-fallback"
    }
    fn available(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fallback_always_available() {
        assert!(CpuFallback.available());
    }
}
