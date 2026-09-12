//! # vxl-phys-field
//!
//! 力场注册表（§1 vxl-phys-field）：重力/风/爆炸/吸引/排斥/涡流/自定义。
//! 每帧把力写入 `BodySet::force / torque` 累加器，由积分器消费并清零。
//! 注册表顺序固定（插入序），保证确定性。

#![forbid(unsafe_code)]

use vxl_phys_core::{BodySet, Vec3};

pub trait ForceField: Send + Sync {
    /// 对全体动体累加力/力矩。实现必须按体索引有序遍历。
    fn apply(&self, bodies: &mut BodySet);
}

/// 均匀重力场。
#[derive(Clone, Copy, Debug)]
pub struct GravityField {
    pub g: Vec3,
}

impl ForceField for GravityField {
    fn apply(&self, bodies: &mut BodySet) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let mass = 1.0 / bodies.inv_mass[i];
            bodies.force[i] += self.g * mass;
        }
    }
}

/// 均匀风场（M0 演示用；面元气动力版在 vxl-phys-aero / 布料 §4.7）。
#[derive(Clone, Copy, Debug)]
pub struct WindField {
    pub velocity: Vec3,
    /// F = k·|v_rel|·v_rel（线性化阻力）。
    pub k: f32,
}

impl ForceField for WindField {
    fn apply(&self, bodies: &mut BodySet) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let v_rel = self.velocity - bodies.linvel[i];
            bodies.force[i] += v_rel * (self.k * v_rel.length());
        }
    }
}

/// 球形吸引/排斥场（正=吸引）。
#[derive(Clone, Copy, Debug)]
pub struct AttractField {
    pub center: Vec3,
    pub strength: f32,
}

impl ForceField for AttractField {
    fn apply(&self, bodies: &mut BodySet) {
        for i in 0..bodies.len() {
            if !bodies.is_dynamic(i) {
                continue;
            }
            let d = self.center - bodies.position[i];
            let dist = d.length().max(0.5);
            bodies.force[i] += d.normalize() * (self.strength / (dist * dist));
        }
    }
}

/// 力场注册表：按插入序依次生效。
#[derive(Default)]
pub struct FieldRegistry {
    fields: Vec<Box<dyn ForceField>>,
}

impl FieldRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(&mut self, field: Box<dyn ForceField>) {
        self.fields.push(field);
    }

    pub fn apply(&self, bodies: &mut BodySet) {
        for f in &self.fields {
            f.apply(bodies);
        }
    }
}
