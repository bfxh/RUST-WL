//! # vxl-phys-soft
//!
//! 软体/布料（`SPEC.md` §4.6/§4.7，XPBD）—— M3 落地。
//!
//! - [`params`]：参数骨架（刚度档 α / 撕裂阈值 / 自碰撞），数值全部来自规格书；
//! - [`rope`]：**绳索最小闭环**（1D 粒子链 + XPBD 距离约束 + 点-形状接触）——
//!   判据在 `tests/rope_minimal.rs`（悬垂形状对**同长度解析悬链线** / 二阶收敛 / 落在真实三角网上）。
//!
//! 待落地（各自是后续切片）：布料三组约束 + Bridson 面元气动、体积约束、自碰撞、
//! 与刚体的双向耦合（Akinci 边界）、GPU 档、以及**门面接线**（`World::add_rope` + `world_step` 相位）。

#![forbid(unsafe_code)]

pub mod params;
pub mod rope;

pub use params::{ClothConstraints, SelfCollision, Stiffness, TearStrain};
pub use rope::Rope;
