//! SoA 刚体存储（§3 内存目标：热数据 SoA，100 万刚体 ≤ 2 GB 的基线布局）。
//!
//! 确定性约束（§5）：所有遍历按体索引升序；写入顺序固定。

use crate::mass::mass_props;
use crate::math::{Quat, Vec3};
use crate::shape::Shape;

pub type BodyId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyType {
    Static,
    Dynamic,
}

/// 结构化数组（SoA）刚体集合。
#[derive(Clone, Debug, Default)]
pub struct BodySet {
    pub shape: Vec<Shape>,
    pub body_type: Vec<BodyType>,
    pub position: Vec<Vec3>,
    pub rotation: Vec<Quat>,
    pub linvel: Vec<Vec3>,
    pub angvel: Vec<Vec3>,
    pub inv_mass: Vec<f32>,
    pub local_inv_inertia: Vec<Vec3>,
    /// 外力/外力矩累加器（每子步被积分器消费并清零）。
    pub force: Vec<Vec3>,
    pub torque: Vec<Vec3>,
    pub awake: Vec<bool>,
    pub sleep_timer: Vec<f32>,
}

impl BodySet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.shape.len()
    }

    pub fn is_empty(&self) -> bool {
        self.shape.is_empty()
    }

    #[inline]
    pub fn is_dynamic(&self, i: usize) -> bool {
        self.inv_mass[i] > 0.0
    }

    pub fn push_static(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId {
        let id = self.shape.len() as BodyId;
        self.shape.push(shape);
        self.body_type.push(BodyType::Static);
        self.position.push(position);
        self.rotation.push(rotation);
        self.linvel.push(Vec3::ZERO);
        self.angvel.push(Vec3::ZERO);
        self.inv_mass.push(0.0);
        self.local_inv_inertia.push(Vec3::ZERO);
        self.force.push(Vec3::ZERO);
        self.torque.push(Vec3::ZERO);
        self.awake.push(true);
        self.sleep_timer.push(0.0);
        id
    }

    pub fn push_dynamic(
        &mut self,
        shape: Shape,
        position: Vec3,
        rotation: Quat,
        density: f32,
    ) -> BodyId {
        let mp = mass_props(&shape, density);
        let id = self.shape.len() as BodyId;
        self.shape.push(shape);
        self.body_type.push(BodyType::Dynamic);
        self.position.push(position);
        self.rotation.push(rotation);
        self.linvel.push(Vec3::ZERO);
        self.angvel.push(Vec3::ZERO);
        self.inv_mass.push(mp.inv_mass);
        self.local_inv_inertia.push(mp.local_inv_inertia);
        self.force.push(Vec3::ZERO);
        self.torque.push(Vec3::ZERO);
        self.awake.push(true);
        self.sleep_timer.push(0.0);
        id
    }

    #[inline]
    pub fn wake(&mut self, i: usize) {
        if self.is_dynamic(i) {
            self.awake[i] = true;
            self.sleep_timer[i] = 0.0;
        }
    }

    pub fn set_linvel(&mut self, i: usize, v: Vec3) {
        self.wake(i);
        self.linvel[i] = v;
    }

    pub fn set_angvel(&mut self, i: usize, w: Vec3) {
        self.wake(i);
        self.angvel[i] = w;
    }

    /// 世界系逆惯性作用：I⁻¹_world·L = R·diag(local_I⁻¹)·Rᵀ·L。
    #[inline]
    pub fn apply_world_inv_inertia(&self, i: usize, l: Vec3) -> Vec3 {
        let r = crate::math::Mat3::from_quat(self.rotation[i]);
        let lt = r.transpose_mul_vec3(l);
        r.mul_vec3(lt.mul_per_elem(self.local_inv_inertia[i]))
    }

    /// 点接触速度 v + ω×r（r：COM → 作用点）。
    #[inline]
    pub fn velocity_at(&self, i: usize, r: Vec3) -> Vec3 {
        self.linvel[i] + self.angvel[i].cross(r)
    }

    /// 施加冲量（世界系，作用于世界点；自动唤醒）。
    pub fn apply_impulse(&mut self, i: usize, impulse: Vec3, world_point: Vec3) {
        if !self.is_dynamic(i) {
            return;
        }
        self.wake(i);
        self.linvel[i] = self.linvel[i] + impulse * self.inv_mass[i];
        let r = world_point - self.position[i];
        let l = r.cross(impulse);
        let dw = self.apply_world_inv_inertia(i, l);
        self.angvel[i] += dw;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_and_query() {
        let mut b = BodySet::new();
        let s = b.push_static(
            Shape::Box {
                half: Vec3::splat(0.5),
            },
            Vec3::ZERO,
            Quat::IDENTITY,
        );
        let d = b.push_dynamic(Shape::Sphere { radius: 0.5 }, Vec3::Y, Quat::IDENTITY, 1.0);
        assert!(!b.is_dynamic(s as usize));
        assert!(b.is_dynamic(d as usize));
        b.set_linvel(d as usize, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(b.linvel[d as usize], Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn impulse_changes_velocity() {
        let mut b = BodySet::new();
        let d = b.push_dynamic(
            Shape::Sphere { radius: 1.0 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
        );
        let m = 4.0 / 3.0 * core::f32::consts::PI;
        b.apply_impulse(d as usize, Vec3::new(m, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        // 线速度 Δv = J/m = 1；角速度 = I⁻¹(r×J)。
        assert!((b.linvel[d as usize].x - 1.0).abs() < 1e-4);
        assert!(b.angvel[d as usize].z < 0.0);
    }
}
