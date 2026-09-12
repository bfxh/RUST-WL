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

pub mod body;
pub mod config;
pub mod mass;
pub mod material;
pub mod math;
pub mod schedule;
pub mod shape;

pub use body::{BodyId, BodySet, BodyType};
pub use config::{FrictionModel, PhysConfig, Preset};
pub use mass::{mass_props, MassProps};
pub use material::{Material, MaterialId};
pub use math::{Mat3, Quat, Vec3};
pub use schedule::{JobSystem, SerialJobSystem};
pub use shape::{HeightFieldId, Shape, CYLINDER_SEGMENTS};
