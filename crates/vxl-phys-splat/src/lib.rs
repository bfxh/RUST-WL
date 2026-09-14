//! # vxl-phys-splat
//!
//! **高斯喷溅（Gaussian Splatting）域**——ROUTE §3.1 三层用法的最小落地：
//!
//! 1. **渲染桥**：`Splat` 直存渲染器要的字段（中心/各向异性尺度/姿态/不透明度/颜色），
//!    导出经 `export_splats` 一次拷贝（`StateBridge` 的喷溅侧形态）。
//! 2. **物理代理**：`GaussianSplatField` 实现 `ProviderColliders`——刚体（盒/球/外壳）
//!    与喷溅体经 `contacts_point`/`contacts_box`/`contacts_sphere` 接触，**不引入新形状**。
//! 3. **隐式场**：密度 σ(p) = Σ wᵢ·exp(−½·αᵢ(p))（各向异性二次型），等值面 σ = τ；
//!    距离用一阶隐式近似 `f = (σ − τ)/|∇σ|`（f < 0 = 内部），法线 = −∇σ/|∇σ|。
//!    —— 与 SPH 核同形（都是各向同性/异性核叠加），故该场可直接复用作介质采样源。
//!
//! 确定性：全部遍历按 splat 注册序；无 HashMap/无浮点归约顺序变化。
//! 性能边界：点查询 O(#splats·cut)（cut = 3σ 外截断）；加速结构（均匀网格/BVH）见
//! CATALOG「待办」——当前档位面向演示与中等规模（≤ 数千颗）。

#![deny(unsafe_code)]

use vxl_phys_core::interop::{InteropContact, ProviderColliders};
use vxl_phys_core::{Aabb, Mat3, Vec3};

/// 单颗高斯喷溅（渲染参数即物理参数，无第二套表示）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Splat {
    /// 世界中心。
    pub center: Vec3,
    /// 局部各向异性尺度（σ，米；0 = 退化为刚性方向）。
    pub scale: Vec3,
    /// 姿态（各向异性主轴；单位阵 = 轴对齐）。
    pub rot: Mat3,
    /// 不透明度 / 权重（密度和中的系数；物理上 = 该核的“质量”）。
    pub opacity: f32,
    /// 渲染颜色（RGB，0..1）；物理不消费，只随导出走。
    pub color: [f32; 3],
}

impl Splat {
    /// 各向同性核。
    pub fn isotropic(center: Vec3, radius: f32, opacity: f32) -> Self {
        Self {
            center,
            scale: Vec3::splat(radius),
            rot: Mat3::IDENTITY,
            opacity,
            color: [0.8, 0.85, 0.95],
        }
    }

    /// 局部坐标 → 世界（仅旋转；尺度在求值时逐轴除）。
    #[inline]
    fn axes(&self) -> [Vec3; 3] {
        [
            self.rot.mul_vec3(Vec3::X),
            self.rot.mul_vec3(Vec3::Y),
            self.rot.mul_vec3(Vec3::Z),
        ]
    }
}

/// 均匀网格（密集 `Vec<Vec<u32>>`；按注册序登记 ⇒ 格内索引自然升序。
/// **格内序 = 注册序** ⇒ 查询求和顺序与全扫一致 ⇒ 逐位一致，见模组测试）。
#[derive(Clone, Debug)]
struct Grid {
    origin: Vec3,
    inv_cell: f32,
    dims: (u32, u32, u32),
    bins: Vec<Vec<u32>>,
}

/// 高斯喷溅场（隐式场；实现提供者接口）。
#[derive(Clone, Debug)]
pub struct GaussianSplatField {
    splats: Vec<Splat>,
    /// 等值面阈值：σ > iso ⇒ 内部。
    pub iso: f32,
    /// 截断（α 超过该值不再计入；9 = 3σ）。
    pub cut: f32,
    /// 加速结构（`rebuild_grid` 建；`push` 置脏为 None ⇒ 不建即为全扫）。
    grid: Option<Grid>,
    /// 建网格的门槛（核数少时全扫更快；0 = 总是建）。
    pub grid_min_splats: usize,
    /// **介质参数**（`MediumField` 通道；ROUTE §3.1 第三层）：每单位 σ 的质量密度
    /// （kg/m³）。0 = 该场只作碰撞提供者、不作介质（零成本，压缩为不采样）。
    pub medium_density: f32,
    /// 介质黏性系数（Pa·s / σ 单位；单侧耦合用不到，供下游求解器消费）。
    pub medium_viscosity: f32,
    /// 介质自身的流速（风/水流；阻力按相对速度算）。
    pub medium_velocity: Vec3,
}

/// 建网格的格数上限（超限不建，退回全扫；防内存灾难）。
const GRID_MAX_CELLS: u64 = 4 << 20;

impl Default for GaussianSplatField {
    fn default() -> Self {
        Self::new(0.5)
    }
}

impl GaussianSplatField {
    pub fn new(iso: f32) -> Self {
        Self {
            splats: Vec::new(),
            iso,
            cut: 9.0,
            grid: None,
            grid_min_splats: 64,
            medium_density: 0.0,
            medium_viscosity: 0.0,
            medium_velocity: Vec3::ZERO,
        }
    }

    pub fn push(&mut self, s: Splat) {
        self.splats.push(s);
        self.grid = None; // 脏（下次查询退回全扫；需要时重建）
    }

    /// 建/重建均匀网格（格边长 = 3·max(尺度)；超限或核数不足则建空）。
    /// 幂等：重复调用结果相同（确定性）。
    pub fn rebuild_grid(&mut self) {
        self.grid = None;
        let n = self.splats.len();
        if n < self.grid_min_splats {
            return;
        }
        let mut max_s = 0.0f32;
        for s in &self.splats {
            max_s = max_s.max(s.scale.x).max(s.scale.y).max(s.scale.z);
        }
        let cell = max_s * 3.0;
        if cell <= 1e-6 {
            return;
        }
        let b = self.world_bounds();
        let dims = (
            ((b.max.x - b.min.x) / cell).ceil() as u32 + 1,
            ((b.max.y - b.min.y) / cell).ceil() as u32 + 1,
            ((b.max.z - b.min.z) / cell).ceil() as u32 + 1,
        );
        let cells = dims.0 as u64 * dims.1 as u64 * dims.2 as u64;
        if cells == 0 || cells > GRID_MAX_CELLS {
            return;
        }
        let mut bins: Vec<Vec<u32>> = vec![Vec::new(); cells as usize];
        let inv_cell = 1.0 / cell;
        for (i, s) in self.splats.iter().enumerate() {
            let r = max_s * 3.0;
            let lo = s.center - Vec3::splat(r);
            let hi = s.center + Vec3::splat(r);
            let c0 = (
                (((lo.x - b.min.x) * inv_cell).floor().max(0.0)) as u32,
                (((lo.y - b.min.y) * inv_cell).floor().max(0.0)) as u32,
                (((lo.z - b.min.z) * inv_cell).floor().max(0.0)) as u32,
            );
            let c1 = (
                (((hi.x - b.min.x) * inv_cell).ceil() as u32).min(dims.0 - 1),
                (((hi.y - b.min.y) * inv_cell).ceil() as u32).min(dims.1 - 1),
                (((hi.z - b.min.z) * inv_cell).ceil() as u32).min(dims.2 - 1),
            );
            for cx in c0.0..=c1.0 {
                for cy in c0.1..=c1.1 {
                    for cz in c0.2..=c1.2 {
                        let idx = ((cx * dims.1 + cy) * dims.2 + cz) as usize;
                        bins[idx].push(i as u32);
                    }
                }
            }
        }
        self.grid = Some(Grid {
            origin: b.min,
            inv_cell,
            dims,
            bins,
        });
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.splats.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.splats.is_empty()
    }

    #[inline]
    pub fn splats(&self) -> &[Splat] {
        &self.splats
    }

    /// 单核二次型 α = Σⱼ((p−c)·uⱼ/sⱼ)²（各向异性轴对齐到 uⱼ）。
    #[inline]
    fn alpha(s: &Splat, ax: &[Vec3; 3], p: Vec3) -> f32 {
        let d = p - s.center;
        let mut a = 0.0;
        for (j, aj) in ax.iter().enumerate() {
            let sj = match j {
                0 => s.scale.x,
                1 => s.scale.y,
                _ => s.scale.z,
            }
            .max(1e-4);
            let t = d.dot(*aj) / sj;
            a += t * t;
        }
        a
    }

    /// 候选核迭代（顺序 = 注册序；有网格时只取所在格的登记表 ⇒ 与全扫逐位一致）。
    #[inline]
    fn candidates(&self, p: Vec3) -> impl Iterator<Item = &Splat> {
        let empty: &[u32] = &[];
        let (list, all) = match &self.grid {
            Some(g) => {
                let cx = ((p.x - g.origin.x) * g.inv_cell).floor();
                let cy = ((p.y - g.origin.y) * g.inv_cell).floor();
                let cz = ((p.z - g.origin.z) * g.inv_cell).floor();
                if cx < 0.0 || cy < 0.0 || cz < 0.0 {
                    (empty, false)
                } else {
                    let (cx, cy, cz) = (cx as u32, cy as u32, cz as u32);
                    if cx >= g.dims.0 || cy >= g.dims.1 || cz >= g.dims.2 {
                        (empty, false)
                    } else {
                        let i = ((cx * g.dims.1 + cy) * g.dims.2 + cz) as usize;
                        (g.bins[i].as_slice(), false)
                    }
                }
            }
            None => (empty, true),
        };
        let splats = &self.splats;
        let len = if all { splats.len() } else { list.len() };
        (0..len).map(move |k| if all { &splats[k] } else { &splats[list[k] as usize] })
    }

    /// 密度 σ(p) 与梯度 ∇σ(p)（解析；截断外核不计）。
    pub fn density_grad(&self, p: Vec3) -> (f32, Vec3) {
        let mut sum = 0.0;
        let mut g = Vec3::ZERO;
        for s in self.candidates(p) {
            let ax = s.axes();
            let a = Self::alpha(s, &ax, p);
            if a > self.cut {
                continue;
            }
            let e = (-0.5 * a).exp() * s.opacity;
            sum += e;
            // dσ/dp = w·exp(−α/2)·(−Σⱼ ((p−c)·uⱼ/sⱼ²)·uⱼ)
            let d = p - s.center;
            let mut inner = Vec3::ZERO;
            for (j, aj) in ax.iter().enumerate() {
                let sj = match j {
                    0 => s.scale.x,
                    1 => s.scale.y,
                    _ => s.scale.z,
                }
                .max(1e-4);
                let t = d.dot(*aj) / (sj * sj);
                inner += *aj * t;
            }
            g += inner * (-e);
        }
        (sum, g)
    }

    /// 隐式距离（f < 0 = 场内部）。梯度为零处返回 `(σ − iso)·大数` 的符号距离近似。
    pub fn sdf(&self, p: Vec3) -> f32 {
        let (s, g) = self.density_grad(p);
        let gl = g.length();
        if gl < 1e-6 {
            // 极值区（峰/远场）：用带符号常数近似，方向信息已不可用
            if s > self.iso {
                return -(s - self.iso).sqrt().min(1.0);
            }
            return (self.iso - s).sqrt().min(1.0);
        }
        (self.iso - s) / gl // 内部 σ > iso ⇒ 负 ✓（外层负号在 caller 展开）
    }

    /// 场外包盒（k·σ + margin；空场 = 退化盒）。
    /// 注意与 `ProviderColliders::bounds(id)` 同名不同签名 ⇒ 这里用独立名避免遮蔽。
    pub fn world_bounds(&self) -> Aabb {
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for s in &self.splats {
            let r = s.scale.x.max(s.scale.y).max(s.scale.z) * 3.0;
            lo = lo.min(s.center - Vec3::splat(r));
            hi = hi.max(s.center + Vec3::splat(r));
        }
        if self.splats.is_empty() {
            return Aabb {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            };
        }
        Aabb {
            min: lo - Vec3::splat(0.1),
            max: hi + Vec3::splat(0.1),
        }
    }
}

impl ProviderColliders for GaussianSplatField {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        let _ = id;
        Some(self.world_bounds())
    }

    /// 点查询（探针半径 = skin）：`depth = skin − sdf(p)`；法线 = −∇σ/|∇σ|（外向）。
    fn contacts_point(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let _ = id;
        let (s, g) = self.density_grad(p);
        let gl = g.length();
        let f = if gl > 1e-6 {
            (self.iso - s) / gl
        } else if s > self.iso {
            -(s - self.iso).sqrt().min(1.0)
        } else {
            (self.iso - s).sqrt().min(1.0)
        };
        let depth = skin - f;
        if depth < -skin {
            return true; // 支持查询；该点不在带内
        }
        let gl = g.length();
        let n = if gl > 1e-6 {
            (-g) * (1.0 / gl)
        } else {
            Vec3::Y
        };
        out.push(InteropContact {
            point: p,
            normal: n,
            depth,
            feature: 0,
        });
        true
    }

    /// 盒查询：盒表面采样（6 面 × 中心 + 4 角，与体素盒路径同构）。
    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: vxl_phys_core::Quat,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let r = Mat3::from_quat(rot);
        let axes = [r.mul_vec3(Vec3::X), r.mul_vec3(Vec3::Y), r.mul_vec3(Vec3::Z)];
        let hs = [half.x, half.y, half.z];
        for i in 0..3 {
            let (u, hu) = (axes[(i + 1) % 3], hs[(i + 1) % 3]);
            let (v, hv) = (axes[(i + 2) % 3], hs[(i + 2) % 3]);
            for &side in &[-1.0f32, 1.0] {
                let face_center = pos + axes[i] * (side * hs[i]);
                self.contacts_point(id, face_center, skin, out);
                for &(su, sv) in &[(-1.0f32, -1.0f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                    let corner = face_center + u * (su * hu) + v * (sv * hv);
                    self.contacts_point(id, corner, skin, out);
                }
            }
        }
        true
    }

    /// 球查询：中心 SDF 解析深度（球面最近点方向 ≈ SDF 梯度）。
    fn contacts_sphere(
        &self,
        id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<InteropContact>,
    ) -> bool {
        let _ = id;
        let f = self.sdf(center);
        let depth = radius - f;
        if depth < -skin {
            return true;
        }
        let (_, g) = self.density_grad(center);
        let gl = g.length();
        let n = if gl > 1e-6 {
            (-g) * (1.0 / gl)
        } else {
            Vec3::Y
        };
        out.push(InteropContact {
            point: center - n * (radius.min(f.max(0.0))),
            normal: n,
            depth,
            feature: 1,
        });
        true
    }
}

impl vxl_phys_core::interop::MediumField for GaussianSplatField {
    /// 采样：密度 ∝ σ(p)（每单位 σ 的 `medium_density`）、速度 = 场流速、
    /// 占用率 = σ/iso 钳到 0..1（iso 外为自由空间）。
    /// `medium_density == 0` 时直接返回真空（零成本短路）。
    fn sample(&self, x: Vec3) -> vxl_phys_core::interop::MediumSample {
        use vxl_phys_core::interop::MediumSample;
        if self.medium_density <= 0.0 || self.splats.is_empty() {
            return MediumSample::VACUUM;
        }
        let (sigma, _) = self.density_grad(x);
        if sigma <= 0.0 {
            return MediumSample::VACUUM;
        }
        MediumSample {
            density: sigma * self.medium_density,
            velocity: self.medium_velocity,
            viscosity: sigma * self.medium_viscosity,
            temperature: 0.0,
            occupied: if self.iso > 0.0 {
                (sigma / self.iso).clamp(0.0, 1.0)
            } else {
                0.0
            },
        }
    }

    /// 沉积：**当前为单向耦合**（介质不因受力而改变——喷溅场是静态隐式场）。
    /// 双向耦合（把动量沉积回核、驱动场演化）见 ROUTE §3.1 待办。
    fn deposit(&mut self, _x: Vec3, _momentum: Vec3, _mass: f32, _pressure_work: f32) {}
}

/// **渲染桥**：导出喷溅参数（中心/尺度/姿态/透明度/颜色）——不经物理表示往返。
pub fn export_splats(field: &GaussianSplatField, out: &mut Vec<(Vec3, Vec3, Mat3, f32, [f32; 3])>) {
    out.clear();
    for s in field.splats() {
        out.push((s.center, s.scale, s.rot, s.opacity, s.color));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_splat() -> GaussianSplatField {
        let mut f = GaussianSplatField::new(0.5);
        f.push(Splat::isotropic(Vec3::ZERO, 0.5, 1.0));
        f
    }

    #[test]
    fn density_peak_and_decay_match_analytic() {
        let f = one_splat();
        let (s0, _) = f.density_grad(Vec3::ZERO);
        assert!((s0 - 1.0).abs() < 1e-6, "σ(0)={s0}");
        // r = 0.5 → α = 1 → e^-0.5
        let (s1, _) = f.density_grad(Vec3::new(0.5, 0.0, 0.0));
        assert!((s1 - (-0.5f32).exp()).abs() < 1e-6, "σ(0.5)={s1}");
        // 截断（3σ = 1.5）
        let (s2, _) = f.density_grad(Vec3::new(2.0, 0.0, 0.0));
        assert_eq!(s2, 0.0);
    }

    #[test]
    fn sdf_sign_and_surface() {
        let f = one_splat();
        assert!(f.sdf(Vec3::ZERO) < 0.0, "内部应为负");
        assert!(f.sdf(Vec3::new(1.4, 0.0, 0.0)) > 0.0, "外部应为正");
        // 等值面在 α ≈ 1.177（σ = 0.5 ⇒ exp(−α/2) = 0.5）⇒ r ≈ 0.5·√1.177 ≈ 0.542
        let r_iso = 0.5 * (2.0f32 * (1.0f32 / 0.5).ln()).sqrt();
        let s = f.sdf(Vec3::new(r_iso, 0.0, 0.0));
        assert!(s.abs() < 0.05, "等值面处 sdf={s}（应 ≈ 0）");
    }

    #[test]
    fn contact_normal_points_outward() {
        let f = one_splat();
        let mut out = Vec::new();
        // 右侧靠内：法线应指 +X（向外推）
        let p = Vec3::new(0.4, 0.0, 0.0);
        assert!(f.contacts_point(0, p, 0.05, &mut out));
        assert_eq!(out.len(), 1);
        assert!(out[0].normal.x > 0.9, "n={:?}", out[0].normal);
        assert!(out[0].depth > 0.0, "深内点 depth 应为正: {}", out[0].depth);
        // 远处无接触
        out.clear();
        f.contacts_point(0, Vec3::new(3.0, 0.0, 0.0), 0.05, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn anisotropic_splat_elongates_along_axis() {
        let mut f = GaussianSplatField::new(0.5);
        f.push(Splat {
            center: Vec3::ZERO,
            scale: Vec3::new(0.15, 0.15, 1.0), // Z 向拉长
            rot: Mat3::IDENTITY,
            opacity: 1.0,
            color: [1.0, 1.0, 1.0],
        });
        // 沿 Z 到 0.8 仍是内部/近场；沿 X 到 0.8 已远
        let fz = f.sdf(Vec3::new(0.0, 0.0, 0.8));
        let fx = f.sdf(Vec3::new(0.8, 0.0, 0.0));
        assert!(fz < fx, "fz={fz} fx={fx}（Z 向应更“实”）");
    }

    /// **L3 验收主判据**：网格加速与全扫**逐位一致**（格内索引升序 = 注册序）。
    #[test]
    fn grid_matches_brute_force_bitwise() {
        let mut f = GaussianSplatField::new(0.5);
        let mut st: u32 = 0x1234_5678;
        let mut next = move || {
            st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (st >> 8) as f32 / (1u32 << 24) as f32
        };
        for _ in 0..200 {
            let c = Vec3::new(next() * 4.0 - 2.0, next() * 4.0 - 2.0, next() * 4.0 - 2.0);
            let r = 0.15 + next() * 0.3;
            f.push(Splat::isotropic(c, r, 0.4 + next()));
        }
        let mut pts = Vec::new();
        let mut st2: u32 = 0x9e37_79b9;
        let mut next2 = move || {
            st2 = st2.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (st2 >> 8) as f32 / (1u32 << 24) as f32
        };
        for _ in 0..500 {
            pts.push(Vec3::new(
                next2() * 6.0 - 3.0,
                next2() * 6.0 - 3.0,
                next2() * 6.0 - 3.0,
            ));
        }
        // 全扫基线（未建网格）
        let brute: Vec<(f32, Vec3)> = pts.iter().map(|&p| f.density_grad(p)).collect();
        f.rebuild_grid();
        for (i, &p) in pts.iter().enumerate() {
            let g = f.density_grad(p);
            assert_eq!(g.0.to_bits(), brute[i].0.to_bits(), "σ 不一致 @{i}");
            assert_eq!(g.1.x.to_bits(), brute[i].1.x.to_bits(), "gx @{i}");
            assert_eq!(g.1.y.to_bits(), brute[i].1.y.to_bits(), "gy @{i}");
            assert_eq!(g.1.z.to_bits(), brute[i].1.z.to_bits(), "gz @{i}");
        }
    }

    /// L3 微基准（`cargo test -- --nocapture` 看数字）：全扫 vs 网格的查询耗时。
    /// 只报数不设阈值（时间类断言在 CI 上不稳）；正确性由上面的逐位一致测试负责。
    #[test]
    fn grid_speedup_microbench() {
        let mut f = GaussianSplatField::new(0.5);
        let mut st: u32 = 0x51ed_2701;
        let mut next = move || {
            st = st.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (st >> 8) as f32 / (1u32 << 24) as f32
        };
        for _ in 0..2000 {
            let c = Vec3::new(next() * 20.0 - 10.0, next() * 8.0, next() * 20.0 - 10.0);
            f.push(Splat::isotropic(c, 0.3, 1.0));
        }
        let mut pts = Vec::new();
        for _ in 0..2000 {
            pts.push(Vec3::new(
                next() * 20.0 - 10.0,
                next() * 8.0,
                next() * 20.0 - 10.0,
            ));
        }
        let t0 = std::time::Instant::now();
        let mut acc = 0.0f32;
        for _ in 0..10 {
            for &p in &pts {
                acc += f.density_grad(p).0;
            }
        }
        let brute = t0.elapsed().as_secs_f64();
        f.rebuild_grid();
        let t1 = std::time::Instant::now();
        let mut acc2 = 0.0f32;
        for _ in 0..10 {
            for &p in &pts {
                acc2 += f.density_grad(p).0;
            }
        }
        let grid = t1.elapsed().as_secs_f64();
        println!(
            "L3 微基准（2000 核 × 2000 点 × 10 轮）：全扫 {:.1} ms | 网格 {:.1} ms | 加速 {:.1}×（acc {acc:.3} vs {acc2:.3}）",
            brute * 1e3,
            grid * 1e3,
            brute / grid
        );
    }

    #[test]
    fn export_roundtrip_is_bitwise() {
        let f = one_splat();
        let mut out = Vec::new();
        export_splats(&f, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, Vec3::ZERO);
        assert_eq!(out[0].4, [0.8, 0.85, 0.95]);
    }
}
