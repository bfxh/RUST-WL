//! 互操作层（**兼容轴**的最小前置；路线见 `docs/ROUTE.md` §2.1/§5）。
//!
//! 本模块只放**跨域唯一通道**的 trait 与最小共享类型：任何物理域（刚体/体素/软体/
//! 布/液体/喷溅/风…）都只允许通过这四个接口与其它域交互——**禁止域内特判**。
//!
//! | 接口 | 职责 | 谁实现 |
//! |---|---|---|
//! | [`CollisionProvider`] | 接触查询（最近点/流形） | 体素 SDF、trimesh、高度场、喷溅场… |
//! | [`MediumField`] | 介质采样 / 沉积（浮力/阻力/风） | 液体、气体、颗粒… |
//! | [`StateBridge`] | 表示转换 / 导出导入（渲染、持久化） | 每域一份 |
//! | [`ConstraintElement`] | 统一约束元素（求解器唯一消费形态） | 刚体接触、关节、XPBD、耦合约束… |
//!
//! 设计纪律（ROUTE §5）：接口只依赖 `vxl-phys-core` 类型（本模块即宿主）；
//! 域的**具体类型不得出现在此**（否则 DAG 反向依赖）。实现放在各域 crate 内。

use crate::{Aabb, Quat, Vec3};

/// 表面查询结果（有向距离 ≥ 0 = 在表面外侧）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceHit {
    /// 表面上最近点（世界系）。
    pub point: Vec3,
    /// 表面法线（世界系，单位向量，指向外侧）。
    pub normal: Vec3,
    /// 有向距离（正 = 外侧、负 = 内部）。
    pub signed_dist: f32,
}

impl SurfaceHit {
    /// 接触深度（本引擎约定：**正 = 穿透**）。
    #[inline]
    pub fn depth(&self) -> f32 {
        -self.signed_dist
    }
}

/// 规范接触点（窄相 `Manifold` 可无损转换；跨域 provider 的输出形态）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InteropContact {
    /// 世界系接触点。
    pub point: Vec3,
    /// 法线（a→b，单位）。
    pub normal: Vec3,
    /// 深度（正 = 穿透；负 = 预期接触，见 SPEC §4.3）。
    pub depth: f32,
    /// 接触特征 ID（跨帧稳定；0 = 无特征）。
    pub feature: u32,
}

/// 介质采样（浮力/阻力/风的统一输入）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MediumSample {
    /// 密度（kg/m³）。
    pub density: f32,
    /// 流速（m/s）。
    pub velocity: Vec3,
    /// 动力黏度（Pa·s）。
    pub viscosity: f32,
    /// 温度（K；仅热耦合域消费）。
    pub temperature: f32,
    /// 占用率（0..1；自由表面/体积分数）。
    pub occupied: f32,
}

impl MediumSample {
    /// 空介质（真空；用于无耦合时的默认返回）。
    pub const VACUUM: MediumSample = MediumSample {
        density: 0.0,
        velocity: Vec3::ZERO,
        viscosity: 0.0,
        temperature: 0.0,
        occupied: 0.0,
    };
}

/// 表示种类（`StateBridge` 的导出/导入形态）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BridgeKind {
    /// 刚体（位姿 + 形状）。
    Rigid,
    /// 体素（稀疏体素/密度场）。
    Voxel,
    /// 粒子（颗粒/流体/软体节点）。
    Particle,
    /// 高斯喷溅（3DGS 集合；见 ROUTE §3.1）。
    Splat,
    /// 三角网（布料/软体表面/静态网格）。
    Mesh,
}

/// 约束元素种类（统一求解器按此分派；新增域 = 新增种类）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ElementKind {
    /// 刚体接触（法向 + 摩擦锥）。
    Contact,
    /// 关节（铰链/球窝/滑块/固定/马达…）。
    Joint,
    /// XPBD 约束（软体/布：距离/体积/弯曲…）。
    Xpbd,
    /// 介质耦合约束（浮力/阻力/风；由 `MediumField` 采样生成）。
    Medium,
    /// CCD/TOI 约束（扫掠式）。
    Ccd,
}

/// 接触提供者：任何域都可作为碰撞体（体素 SDF / trimesh / 高度场 / 喷溅场…）。
///
/// 宽相用 [`bounds`](CollisionProvider::bounds) 取包围盒；窄相通过
/// [`closest_point`](CollisionProvider::closest_point) 做点查询，或由
/// [`contacts_box`](CollisionProvider::contacts_box) 直接产出流形
/// （默认实现：以盒的 8 角点走 `closest_point` 采样——够用但不最优，
/// 高度场/体素/trimesh 应各自覆写为专用解法）。
pub trait CollisionProvider {
    /// 世界系包围盒（宽相用）。
    fn bounds(&self) -> Aabb;

    /// 表面上离 `p` 最近的点；`None` = 查询范围外（视为无碰撞）。
    fn closest_point(&self, p: Vec3) -> Option<SurfaceHit>;

    /// 为「盒形包络」（half + 位姿）生成接触点；返回是否产出了至少一个接触。
    /// 默认实现 = 8 角点采样（`skin` 为预期接触带，`depth ≥ −skin` 的点才保留）。
    fn contacts_box(
        &self,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let m = crate::Mat3::from_quat(rot);
        let mut any = false;
        for k in 0..8 {
            let local = Vec3::new(
                if k & 1 == 0 { -half.x } else { half.x },
                if k & 2 == 0 { -half.y } else { half.y },
                if k & 4 == 0 { -half.z } else { half.z },
            );
            let p = pos + m.mul_vec3(local);
            if let Some(hit) = self.closest_point(p) {
                if hit.signed_dist < skin {
                    out.push(InteropContact {
                        point: hit.point,
                        normal: hit.normal,
                        depth: hit.depth(),
                        feature: k as u32 + 1,
                    });
                    any = true;
                }
            }
        }
        any
    }
}

/// 外部碰撞提供者集合（体素/网格/喷溅场…）：**窄相的查询面**。
///
/// 窄相（`vxl-phys-narrow`）只依赖 `vxl-phys-core`，因此不能直接持有域类型；
/// 由门面/域 crate 实现本 trait 并在 `collide` 时传入（见 ROUTE §5 依赖纪律）。
pub trait ProviderColliders: Send + Sync {
    /// `id` 的世界包围盒（宽相 AABB 用；`None` = 该 id 未注册）。
    fn bounds(&self, id: u32) -> Option<Aabb>;

    /// 「盒形包络 vs provider(id)」的接触（世界系；追加进 `out`；返回是否产出）。
    /// 约定与 [`CollisionProvider::contacts_box`] 相同：只保留 `depth ≥ −skin` 的点，
    /// 法线为 provider 表面**外向**法线。
    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool;

    /// 「球 vs provider(id)」的接触（世界系）。SDF 类提供者可解析求解
    /// （`depth = r − sdf(center)`、法线取 SDF 梯度）；默认实现返回 false
    /// （未支持 ⇒ 该形状对不产生接触）。
    fn contacts_sphere(
        &self,
        _id: u32,
        _center: Vec3,
        _radius: f32,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
}

/// 空提供者集合（未注册任何 provider 时的默认）。
pub struct NoProviders;

impl ProviderColliders for NoProviders {
    fn bounds(&self, _id: u32) -> Option<Aabb> {
        None
    }

    fn contacts_box(
        &self,
        _id: u32,
        _half: Vec3,
        _pos: Vec3,
        _rot: Quat,
        _skin: f32,
        _out: &mut Vec<InteropContact>,
    ) -> bool {
        false
    }
}

/// 介质场：域之间的**唯一**双向通道（液体/气体/颗粒的采样与反作用沉积）。
pub trait MediumField {
    /// 采样 `x` 处的介质状态（只读；无介质处返回 [`MediumSample::VACUUM`]）。
    fn sample(&self, x: Vec3) -> MediumSample;

    /// 反作用沉积：把 `momentum`/`mass` 从（或向）介质注入——双向耦合的一半。
    /// 约定：`pressure_work` 用于能量审计（正 = 介质对物体做功）。
    fn deposit(&mut self, x: Vec3, momentum: Vec3, mass: f32, pressure_work: f32);
}

/// 状态桥：表示转换与导出/导入（渲染、持久化、回放、扫描起步）。
pub trait StateBridge {
    /// 本桥的表示种类。
    fn kind(&self) -> BridgeKind;

    /// 导出位置（渲染/持久化用；顺序稳定，供哈希与回放）。
    fn export_positions(&self, out: &mut Vec<Vec3>);

    /// 反向导入（可选；默认不支持）。返回是否成功。
    fn import_positions(&mut self, _src: &[Vec3]) -> bool {
        false
    }
}

/// 约束元素：**统一求解器唯一消费的形态**（ROUTE §5 结构决定①）。
///
/// 元素自带「涉及哪些体/粒子」（用于分岛、着色与并行划分）；求解期由求解器
/// 提供状态视图并驱动 `warm_start` / `solve`。
///
/// **注意**：本 trait 是 M2 的定案草案——状态视图（`&mut` 索引空间）与
/// 求解器接口在 M2 一并定稿；M1 内不接线（现有 `ContactConstraint` 保持私有）。
pub trait ConstraintElement {
    /// 元素种类。
    fn kind(&self) -> ElementKind;

    /// 涉及的可解实体索引（升序，去重；分岛/着色/并行的依据）。
    fn bodies(&self) -> &[u32];

    /// 该元素当前是否有效（失效元素不参与求解，见 M1 的 `active` 门）。
    fn is_active(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 盒角点采样的默认实现：平面（y=0）上半空盒应产出预期接触。
    struct PlaneField;

    impl CollisionProvider for PlaneField {
        fn bounds(&self) -> Aabb {
            Aabb {
                min: Vec3::new(-100.0, -1.0, -100.0),
                max: Vec3::new(100.0, 0.0, 100.0),
            }
        }

        fn closest_point(&self, p: Vec3) -> Option<SurfaceHit> {
            Some(SurfaceHit {
                point: Vec3::new(p.x, 0.0, p.z),
                normal: Vec3::new(0.0, 1.0, 0.0),
                signed_dist: p.y,
            })
        }
    }

    #[test]
    fn default_contacts_box_samples_corners() {
        let f = PlaneField;
        let mut out = Vec::new();
        // 盒心 y = 0.4、半高 0.5 ⇒ 底面 4 角穿透 0.1
        let any = f.contacts_box(
            Vec3::splat(0.5),
            Vec3::new(0.0, 0.4, 0.0),
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any);
        assert_eq!(out.len(), 4, "只有底面 4 角进入 skin 带");
        for c in &out {
            assert!(
                (c.depth - 0.1).abs() < 1e-5,
                "穿透深度应为 0.1，实际 {}",
                c.depth
            );
            assert!((c.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6);
        }
    }

    #[test]
    fn speculative_contact_kept_within_skin() {
        let f = PlaneField;
        let mut out = Vec::new();
        // 底面在 y=0.01（未接触，缝 0.01 ≤ skin 0.02）⇒ 保留为预期接触（负深度）
        let any = f.contacts_box(
            Vec3::splat(0.5),
            Vec3::new(0.0, 0.51, 0.0),
            Quat::IDENTITY,
            0.02,
            &mut out,
        );
        assert!(any);
        assert_eq!(out.len(), 4);
        for c in &out {
            assert!(
                (c.depth + 0.01).abs() < 1e-5,
                "预期接触深度应 −0.01，实际 {}",
                c.depth
            );
        }
    }
}
