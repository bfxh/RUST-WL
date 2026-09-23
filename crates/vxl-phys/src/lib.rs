//! # vxl-phys
//!
//! `vxl_phys` 引擎门面（§1 依赖注入）：组装默认管线
//! core + broad + narrow + solver + integrate + island，游戏引擎（Bevy）只通过
//! 本 crate 的 `World` 消费物理（§0「纯 Rust 物理，引擎只做渲染壳」）。
//!
//! 步进序（每子步）：
//! 1. 力场 → 2. 速度积分 → 3. 宽相 → 4. 窄相 → 5. 求解 + 岛级休眠
//!    → 6. 位置积分。
//!
//! 液体域（切片1，单向耦合）：每 tick 体子步全部完成后，`fluid_pass` 以流体
//! 自身的固定子步数推进全部流体系统（`World::add_fluid` 注册）。
//!
//! 确定性（§5）：固定步长、严格 f32、有序归约；`state_hash()` 每 60 tick 比对。

#![forbid(unsafe_code)]

pub use vxl_phys_broad::{Aabb, BroadPhase, BvhBroadPhase, GridBroadPhase};
pub use vxl_phys_core as core;
pub use vxl_phys_core::{
    BodyId, BodySet, BodyType, FrictionModel, JobSystem, Material, PhaseArena, PhysConfig, Preset,
    Quat, ScopedPool, SerialJobSystem, Shape, Vec3,
};
pub use vxl_phys_field::{FieldRegistry, ForceField, GravityField};
pub use vxl_phys_integrate::Integrator;
pub use vxl_phys_narrow::heightfield::HeightField;
pub use vxl_phys_narrow::{CompoundChild, ContactPoint, DefaultNarrowPhase, Manifold, NarrowPhase};
pub use vxl_phys_replay::{Recorder, StateHash, Xxh3Hash};
pub use vxl_phys_solver::joints::{Joint, JointKind, JointSet};
pub use vxl_phys_solver::{ccd, warm_match_stats_take, ImpulseSolver};
pub use vxl_phys_terrain::TerrainSet;

// ── 按域拆出的子模块（子目录 src/）
mod arenas;
mod impact;
mod props;
mod providers;
mod types;
mod world_body;
mod world_build;
mod world_ccd;
mod world_health;
mod world_step;
mod world_struct;
pub(crate) use self::props::*;
pub use self::{arenas::*, impact::*, providers::*, types::*, world_struct::*};
// ↑ 子模块顶层条目再导出（impl-only 模块不入 glob，避免 unused）

#[cfg(test)]
mod tests;
