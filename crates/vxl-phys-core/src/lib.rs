//! # vxl-phys-core
//!
//! 数学（严格 f32）/ 质量属性 / 配置 / SoA 体数据 / 任务调度抽象。
//! 规格：`docs/SPEC.md` §1 / §2.1 / §4 / §5。
//!
//! 硬性原则（§0）：
//! - 零外部重依赖；
//! - 确定性优先：表达式求值顺序固定、无 fast-math、无平台 intrinsic、
//!   归约一律按索引有序进行；
//! - `#![forbid(unsafe_code)]`。

#![forbid(unsafe_code)]

pub mod aabb;
pub mod body;
pub mod config;
pub mod grid;
pub mod interop;
pub mod mass;
pub mod material;
pub mod math;
pub mod mem;
pub mod probe;
pub mod schedule;
pub mod shape;

pub use aabb::{union_aabb, Aabb};
pub use body::{BodyId, BodySet, BodyType};
pub use config::{FrictionModel, PhysConfig, Preset};
pub use mass::{mass_props, MassProps};
pub use material::{Material, MaterialId};
pub use math::{Mat3, Quat, Vec3};
pub use mem::{PhaseArena, Pose32, PoseArray, Vel32, VelArray};
pub use schedule::{run_mut_sections, JobSystem, ScopedPool, SerialJobSystem};
pub use shape::{HeightFieldId, Shape, CYLINDER_SEGMENTS};
