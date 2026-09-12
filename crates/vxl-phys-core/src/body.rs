//! SoA 刚体存储（§3 内存目标：热数据 SoA，100 万刚体 ≤ 2 GB 的基线布局）。
//!
//! 热/冷分离（§0.1 #10）：**热组** = `position`（`PoseArray`，pos+rot 同 32B 记录）
//! 与 `linvel`（`VelArray`，lin+ang 同 32B 记录）——宽/窄/积分每帧全量扫的部分；
//! **冷组** = 形状/类型/质量/攒力/休眠/材质（低频或相外访问），索引与热组同序。
//! 热组字段名保留 `position[i]` / `linvel[i]` 写法（数组实现 `Index/IndexMut`，
//! 索引语义与旧 `Vec<Vec3>` 一致）；旋转与角速度经 `rot(i)`/`set_rot`、
//! `angvel(i)`/`set_angvel`（同一 32B 记录的第二分量）。
//!
//! 确定性约束（§5）：所有遍历按体索引升序；写入顺序固定。

use crate::mass::mass_props;
use crate::math::{Quat, Vec3};
use crate::mem::{PoseArray, VelArray};
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
    // ---- 热组（32B 记录、缓冲 32B 对齐、逐体连续）----
    /// 位姿（pos 分量经 `position[i]`；rot 分量经 `rot(i)`/`set_rot`）。
    pub position: PoseArray,
    /// 速度（lin 分量经 `linvel[i]`；ang 分量经 `angvel(i)`/`set_angvel`）。
    pub linvel: VelArray,
    // ---- 冷组（低频 / 相外访问）----
    pub shape: Vec<Shape>,
    pub body_type: Vec<BodyType>,
    pub inv_mass: Vec<f32>,
    pub local_inv_inertia: Vec<Vec3>,
    /// 外力/外力矩累加器（每子步被积分器消费并清零）。
    pub force: Vec<Vec3>,
    pub torque: Vec<Vec3>,
    pub awake: Vec<bool>,
    pub sleep_timer: Vec<f32>,
    /// 材质表（§4.4/§4.5）：槽 0 = 默认材质；体通过 `material[i]` 引用。
    pub materials: Vec<crate::material::Material>,
    pub material: Vec<crate::material::MaterialId>,
}

impl BodySet {
    pub fn new() -> Self {
        let mut s = Self::default();
        s.materials.push(crate::material::Material::default());
        s
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

    /// 注册材质，返回材质 id。
    pub fn add_material(
        &mut self,
        material: crate::material::Material,
    ) -> crate::material::MaterialId {
        let id = self.materials.len() as crate::material::MaterialId;
        self.materials.push(material);
        id
    }

    /// 设置体材质（默认 0）。
    pub fn set_material(&mut self, i: usize, material: crate::material::MaterialId) {
        self.material[i] = material;
    }

    pub fn push_static(&mut self, shape: Shape, position: Vec3, rotation: Quat) -> BodyId {
        let id = self.shape.len() as BodyId;
        self.shape.push(shape);
        self.body_type.push(BodyType::Static);
        self.position.push(position, rotation);
        self.linvel.push(Vec3::ZERO, Vec3::ZERO);
        self.inv_mass.push(0.0);
        self.local_inv_inertia.push(Vec3::ZERO);
        self.force.push(Vec3::ZERO);
        self.torque.push(Vec3::ZERO);
        self.awake.push(true);
        self.sleep_timer.push(0.0);
        self.material.push(0);
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
        self.position.push(position, rotation);
        self.linvel.push(Vec3::ZERO, Vec3::ZERO);
        self.inv_mass.push(mp.inv_mass);
        self.local_inv_inertia.push(mp.local_inv_inertia);
        self.force.push(Vec3::ZERO);
        self.torque.push(Vec3::ZERO);
        self.awake.push(true);
        self.sleep_timer.push(0.0);
        self.material.push(0);
        id
    }

    /// 姿态（位置 + 旋转）只读快照。
    #[inline]
    pub fn pose(&self, i: usize) -> (Vec3, Quat) {
        (self.position[i], self.position.rot(i))
    }

    /// 旋转（热组 32B 记录的第二分量；位置用 `position[i]`）。
    #[inline]
    pub fn rot(&self, i: usize) -> Quat {
        self.position.rot(i)
    }

    #[inline]
    pub fn set_rot(&mut self, i: usize, q: Quat) {
        self.position.set_rot(i, q);
    }

    /// 角速度（热组 32B 记录的第二分量；线速度用 `linvel[i]`）。
    #[inline]
    pub fn angvel(&self, i: usize) -> Vec3 {
        self.linvel.ang(i)
    }

    /// 角速度原始写入（引擎热路径；**不触碰唤醒状态**）。
    #[inline]
    pub fn set_angvel_raw(&mut self, i: usize, w: Vec3) {
        self.linvel.set_ang(i, w);
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

    /// 公共设置语义：唤醒后设置（镜像 `set_linvel`）。
    pub fn set_angvel(&mut self, i: usize, w: Vec3) {
        self.wake(i);
        self.linvel.set_ang(i, w);
    }

    /// 世界系逆惯性作用：I⁻¹_world·L = R·diag(local_I⁻¹)·Rᵀ·L。
    #[inline]
    pub fn apply_world_inv_inertia(&self, i: usize, l: Vec3) -> Vec3 {
        let r = crate::math::Mat3::from_quat(self.rot(i));
        let lt = r.transpose_mul_vec3(l);
        r.mul_vec3(lt.mul_per_elem(self.local_inv_inertia[i]))
    }

    /// 点接触速度 v + ω×r（r：COM → 作用点）。
    #[inline]
    pub fn velocity_at(&self, i: usize, r: Vec3) -> Vec3 {
        self.linvel[i] + self.angvel(i).cross(r)
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
        self.set_angvel_raw(i, self.angvel(i) + dw);
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
        assert!(b.angvel(d as usize).z < 0.0);
    }
}
