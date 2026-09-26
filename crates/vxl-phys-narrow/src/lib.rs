//! # vxl-phys-narrow
//!
//! 窄相（§2.4）：
//! - 凸-凸：SAT（面法线 + 棱叉积轴）+ 参考面 Sutherland–Hodgman 裁剪 → ≤4 点流形；
//! - 球：解析（球-球 / 球-凸体最近点）；
//! - 高度场：列采样特化（§2.4「列裁剪 + 局部采样」的 M0 版）；
//! - GJK/EPA 通用凸路径：**凸体外壳 × {盒|球|外壳}**（`gjk.rs`）；**{盒|球|外壳|胶囊} × 提供者**见 `provider.rs`。
//!
//! 流形法线约定：`normal` 从 a 指向 b；求解器把 +n 冲量施加给 b、−n 施加给 a。
//! skin = speculative margin（§4.3）：分离距离 ≤ skin 仍生成「预期接触」。

// 注：本 crate 不再是 `forbid`——SAT 扫描有 SSE2 内核（`simd.rs`，用户已批准
// unsafe；SAFETY 注释见该模块），其余代码仍 `deny`（模块级 `allow` 仅限那里）。
#![deny(unsafe_code)]

pub mod gjk;
pub mod heightfield;
pub mod polytope;

use std::collections::HashMap;

use heightfield::HeightField;
use polytope::ConvexPolytope;
mod simd;

use vxl_phys_core::{JobSystem, Mat3, Quat, Shape, Vec3, CYLINDER_SEGMENTS};

// ── 按域拆出的子模块（子目录 src/）
mod entry;
mod geom;
mod hf;
mod pair;
mod pair_shaped;
mod phase;
mod prims;
mod provider;
mod sat;
mod store;
mod support;
mod types;
pub(crate) use self::{entry::*, geom::*};
pub use self::{phase::*, store::*, types::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests;
