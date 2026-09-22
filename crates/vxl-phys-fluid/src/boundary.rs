//! **Akinci 两层边界粒子**（SPEC §4.8；`M1-EXIT.md` §4 的 2b）——纯几何半边：
//! 体的表面采样 + 两层内移 + 每粒体积。**不含任何 SPH 公式**（核/压力仍走
//! `FluidSystem` 里既有的那几式，见 `lib.rs` 的 `density_pass`/`pressure_pass`）。
//!
//! 形态（写死，不发明）：
//! - 表面采样间距 = 流体晶格间距 `s`（[`super::FluidSystem::particle_spacing`]）；
//! - **两层**：沿**内**法线各内移 `0.5·s`（近层）与 `1.5·s`（远层）；
//! - 每粒体积 `V_b = s³` ⇒ 质量 `m_b = ρ0·V_b`（与流体单粒质量同源口径）。
//!
//! **为什么两层、为什么 0.5s/1.5s**（不是拍的档）：既有壁邻补偿（`density_pass`
//! 的镜像鬼影）等价于把静置晶格在壁后的镜像层补回核内质量；静置首层落在
//! `sdf = skin = 0.15h`，h = 2s 时镜像层落在 −0.3s/−1.3s 一带，且**只有前两层
//! 落在核半径 h 内**（第三层距离已 > h）⇒ 两层是这个口径下的**离散最小配置**。
//! 层位取 0.5s/1.5s（相对壁面的整数半格），与镜像层同量级。
//! ⚠️ 两者**不是**等价的：鬼影随流体实际分布逐粒同步（故能与沉降后的分层对齐），
//! 边界粒子是**分布无关的固定两层**——这正是它便宜、能反作用的原因，也是它
//! 近壁密度不如鬼影精确的原因（实测差量见 `tests/`）。
//!
//! 支持的形状 = 解析族（Box/Sphere/Cylinder/Capsule/Cone）；复合体/高度场/
//! provider/凸壳**不生成**（0 粒）⇒ facade 侧对这类体回退 2a 粗档（`M1-EXIT.md` §4）。

use vxl_phys_core::{Shape, Vec3};

/// 局部空间表面采样：点 + **外**法线（单位向量）。序 = 生成序（确定性）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceLattice {
    pub points: Vec<Vec3>,
    pub normals: Vec<Vec3>,
}

/// 两层边界粒子的**局部**位置与体积。
/// 序 = 层 0 全部点 → 层 1 全部点（层内序 = [`SurfaceLattice`] 序）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoundaryLattice {
    pub pts: Vec<Vec3>,
    /// 层 0 的粒子数（层 1 同数 ⇒ 总数 = `2·n0`）。
    pub n0: usize,
    /// 每粒体积 `s³`（质量 = `ρ0·V_b`，由 `FluidSystem` 乘）。
    pub volume: f32,
}

impl BoundaryLattice {
    pub fn len(&self) -> usize {
        self.pts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }
}

/// 按流体晶格间距 `s`、核半径 `h` 生成两层边界粒子。
/// **体积按 Akinci 自洽标定** `V_b = 1 / Σ_b W(r_bb)`（对边界粒子**自己**的邻域求和，
/// 含自身项、同一 poly6 核；见 `self_sum_volume`）——即"边界介质自己在静止密度下压力中性"
/// （p_b = Tait(ρ0) = 0），边界对流体只贡献核质量。
/// ⚠️ 本仓 2026-09-22 之前取**无限介质理想化**的 `s³`：两层是**截断**的薄壳，其
/// `Σ_b W` 比无限介质小 ⇒ `1/Σ_bW > s³`（实测平面两层 ≈1.4×）⇒ 旧口径**低估**了近壁补偿。
/// 形状不受支持 ⇒ 空（facade 回退粗档，不静默造粒）。
///
/// **层深按最薄半厚钳制**（`d_k = min((k+0.5)·s, (0.45+0.45k)·half_min)`）：
/// 薄体（某向半厚 < 1.5s）若老实内移 1.5s，两侧层的粒子会**穿过体心互相穿透**
/// ⇒ 体内堆积的边界粒子互相供密度 ⇒ ρ_b 虚高 ⇒ 体积力/反作用整片失真（实测
/// 0.12 m 盒：假侧向力 51 N、浮力 5.9×ρVg，2026-09-22）。钳制是**几何约束**
/// （粒子必须留在体内、不许互穿），不是新物理式；厚体（half_min ≥ 1.5s）钳制不生效，
/// 层位仍是 0.5s/1.5s。
pub fn lattice(shape: &Shape, s: f32, h: f32) -> BoundaryLattice {
    let s = s.max(1e-6);
    let mut surf = SurfaceLattice::default();
    surface(shape, s, &mut surf);
    let n0 = surf.points.len();
    let hm = min_half_extent(shape);
    let mut pts = Vec::with_capacity(n0 * 2);
    for k in 0..2 {
        let want = (k as f32 + 0.5) * s;
        let cap = (0.45 + 0.45 * k as f32) * hm;
        let d = want.min(cap).max(1e-6);
        for (p, n) in surf.points.iter().zip(surf.normals.iter()) {
            pts.push(*p - *n * d);
        }
    }
    let volume = volume_from_self_sum(&pts, h.max(s));
    BoundaryLattice { pts, n0, volume }
}

/// **Akinci 自洽体积标定**：`V_b = 1 / Σ_b W`——对每个边界粒子，用它**自己那套**
/// 边界粒子（同体同批）在半径 `h` 内的 poly6 求和（含自身项 `W(0)`，与密度轮口径一致），
/// 取平均后求倒数。含义：`ρ_b = ρ0·V_b·Σ_b W = ρ0` ⇒ 边界介质"自己"处在静止密度
/// （压力中性），它对流体的贡献就是干净的一份核质量。
/// 确定性：按索引序求和（同体同批、位置固定 ⇒ 每次构建同值）。
fn volume_from_self_sum(pts: &[Vec3], h: f32) -> f32 {
    let sum = boundary_self_sum(pts, h);
    if sum > 1e-12 {
        1.0 / sum
    } else {
        1.0
    }
}

/// 边界粒子**自己那套**的核和 `Σ_b W`（含自身项、半径 `h`；索引序求和 ⇒ 确定性）。
/// `volume_from_self_sum` 取它的倒数当 `V_b`。测试用它验"标定自洽"。
fn boundary_self_sum(pts: &[Vec3], h: f32) -> f32 {
    if pts.is_empty() {
        return 0.0;
    }
    let h2 = h * h;
    let k6 = 315.0 / (64.0 * std::f32::consts::PI * h.powi(9));
    let w0 = k6 * h2 * h2 * h2;
    let mut acc = 0.0f32;
    for (i, pi) in pts.iter().enumerate() {
        let mut sum = w0; // 自身项
        for (j, pj) in pts.iter().enumerate() {
            if i == j {
                continue;
            }
            let r2 = (*pi - *pj).length_squared();
            if r2 <= h2 {
                let t = h2 - r2;
                sum += k6 * t * t * t;
            }
        }
        acc += sum;
    }
    acc / pts.len() as f32
}

/// 该形状是否支持边界粒子生成。**facade 的粗档回退判据**：不支持 ⇒ 该体不产生
/// 边界粒子、也不进覆盖集 ⇒ 仍走 2a 介质场（既不叠加、也不留空）。
pub fn supports(shape: &Shape) -> bool {
    matches!(
        shape,
        Shape::Box { .. }
            | Shape::Sphere { .. }
            | Shape::Cylinder { .. }
            | Shape::Capsule { .. }
            | Shape::Cone { .. }
    )
}

/// 形状的**最薄半厚**（层深钳制用；不支持的形状返回 0 ⇒ `lattice` 也不出点）。
fn min_half_extent(shape: &Shape) -> f32 {
    match *shape {
        Shape::Box { half } => half.x.min(half.y).min(half.z),
        Shape::Sphere { radius } => radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => radius.min(half_height),
        Shape::Capsule {
            half_height,
            radius,
        } => radius.min(half_height),
        // 锥的内切量取底半径与半高的小者（锥尖那侧更薄，但沿法线内移是朝轴心，
        // 半高是保守估计）。
        Shape::Cone {
            half_height,
            radius,
        } => radius.min(half_height),
        _ => 0.0,
    }
}

/// 表面采样入口（按形状族分派；不支持的形状不出点）。
fn surface(shape: &Shape, s: f32, out: &mut SurfaceLattice) {
    match *shape {
        Shape::Box { half } => box_faces(half, s, out),
        Shape::Sphere { radius } => sphere_shell(Vec3::ZERO, radius, s, 0, out),
        Shape::Cylinder {
            half_height,
            radius,
        } => cylinder(half_height, radius, s, out),
        Shape::Capsule {
            half_height,
            radius,
        } => capsule(half_height, radius, s, out),
        Shape::Cone {
            half_height,
            radius,
        } => cone(half_height, radius, s, out),
        // 复合体（子形状在窄相 store 里，本 crate 不可达）、高度场、provider、凸壳：
        // 不生成。**返回 0 粒而不是近似**——近似面片会给错体积，比没有更坏。
        _ => {}
    }
}

/// 按间距 `s` 覆盖长度 `len` 的格数（≥1；四舍五入到最近整数）。
#[inline]
fn count(len: f32, s: f32) -> usize {
    ((len / s).round() as i64).max(1) as usize
}

/// 轴向读写（栅格生成用；`Vec3` 只有 x/y/z 三个字段，故用下标转写）。
#[inline]
fn set_axis(v: &mut Vec3, a: usize, val: f32) {
    match a {
        0 => v.x = val,
        1 => v.y = val,
        _ => v.z = val,
    }
}

/// 长方体：6 面 × 二维栅格（面内取格心 ⇒ 面内无重合，计数按 `边长/s` 四舍五入）。
/// 相邻面在棱上的最近点距 ≈ `s/2`（各自格心内缩），且两层内移方向不同 ⇒ 不重合。
fn box_faces(half: Vec3, s: f32, out: &mut SurfaceLattice) {
    let h = [half.x, half.y, half.z];
    for a in 0..3 {
        let (b, c) = ((a + 1) % 3, (a + 2) % 3);
        let (hb, hc) = (h[b], h[c]);
        let nb = count(2.0 * hb, s);
        let nc = count(2.0 * hc, s);
        let db = 2.0 * hb / nb as f32;
        let dc = 2.0 * hc / nc as f32;
        for sign in [1.0f32, -1.0] {
            for i in 0..nb {
                for j in 0..nc {
                    let mut p = Vec3::ZERO;
                    let mut n = Vec3::ZERO;
                    set_axis(&mut p, a, sign * h[a]);
                    set_axis(&mut n, a, sign);
                    set_axis(&mut p, b, -hb + (i as f32 + 0.5) * db);
                    set_axis(&mut p, c, -hc + (j as f32 + 0.5) * dc);
                    out.points.push(p);
                    out.normals.push(n);
                }
            }
        }
    }
}

/// 球面采样（黄金角分层）：`y` 均匀 ⇒ 面积元均匀（无极点聚集）。
/// `hemi`：0 = 整球；+1 = 上半球（`y ≥ 0`）；−1 = 下半球（`y < 0`，赤道只归 +1 侧
/// ⇒ 两半球拼起来不重复）。点数按 `面积/s²` ⇒ 半球密度与整球一致。
fn sphere_shell(center: Vec3, radius: f32, s: f32, hemi: i8, out: &mut SurfaceLattice) {
    let area = if hemi == 0 { 4.0 } else { 2.0 } * std::f32::consts::PI * radius * radius;
    let n = count(area, s * s);
    let ga = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
    for k in 0..n {
        let y = 1.0 - 2.0 * (k as f32 + 0.5) / n as f32;
        let keep = match hemi {
            0 => true,
            1 => y >= 0.0,
            _ => y < 0.0,
        };
        if !keep {
            continue;
        }
        let r = (1.0 - y * y).max(0.0).sqrt();
        let th = ga * k as f32;
        let (sin, cos) = th.sin_cos();
        let nrm = Vec3::new(r * cos, y, r * sin);
        out.points.push(center + nrm * radius);
        out.normals.push(nrm);
    }
}

/// 水平圆盘采样（法线 ±Y）：同心环栅格，环半径 `(i+½)·dr`、环上点数按该环周长
/// ⇒ 面密度 ≈ `1/s²`（半径 ≪ s 时退化为少量点，这是正确的"面积上放不下更多"）。
fn disk_y(cy: f32, sign: f32, radius: f32, s: f32, out: &mut SurfaceLattice) {
    let rings = count(radius, s);
    let dr = radius / rings as f32;
    for i in 0..rings {
        let r = (i as f32 + 0.5) * dr;
        let nt = count(2.0 * std::f32::consts::PI * r, s);
        for k in 0..nt {
            let th = (k as f32 + 0.5) * (2.0 * std::f32::consts::PI / nt as f32);
            let (sin, cos) = th.sin_cos();
            out.points.push(Vec3::new(r * cos, cy, r * sin));
            out.normals.push(Vec3::new(0.0, sign, 0.0));
        }
    }
}

/// 圆柱：侧面（周向 × 轴向栅格，法线径向）+ 两端圆盘。
/// ⚠️ 侧面用**栅格**而非精确圆弧：棱面近似与本仓碰撞侧的多面化口径一致
/// （`TECH-SURVEY.md` A9）；边界粒子间距 ≪ 半径时误差 ≪ 一间格。
fn cylinder(half_height: f32, radius: f32, s: f32, out: &mut SurfaceLattice) {
    let nt = count(2.0 * std::f32::consts::PI * radius, s);
    let ny = count(2.0 * half_height, s);
    for i in 0..nt {
        let th = (i as f32 + 0.5) * (2.0 * std::f32::consts::PI / nt as f32);
        let (sin, cos) = th.sin_cos();
        let nrm = Vec3::new(cos, 0.0, sin);
        for j in 0..ny {
            let y = -half_height + (j as f32 + 0.5) * (2.0 * half_height / ny as f32);
            out.points
                .push(Vec3::new(nrm.x * radius, y, nrm.z * radius));
            out.normals.push(nrm);
        }
    }
    disk_y(half_height, 1.0, radius, s, out);
    disk_y(-half_height, -1.0, radius, s, out);
}

/// 胶囊（本地方向 = +Y）：中段侧面 + 两半球帽（复用球面采样，密度口径一致）。
fn capsule(half_height: f32, radius: f32, s: f32, out: &mut SurfaceLattice) {
    let nt = count(2.0 * std::f32::consts::PI * radius, s);
    let ny = count(2.0 * half_height, s);
    for i in 0..nt {
        let th = (i as f32 + 0.5) * (2.0 * std::f32::consts::PI / nt as f32);
        let (sin, cos) = th.sin_cos();
        let nrm = Vec3::new(cos, 0.0, sin);
        for j in 0..ny {
            let y = -half_height + (j as f32 + 0.5) * (2.0 * half_height / ny as f32);
            out.points
                .push(Vec3::new(nrm.x * radius, y, nrm.z * radius));
            out.normals.push(nrm);
        }
    }
    sphere_shell(Vec3::new(0.0, half_height, 0.0), radius, s, 1, out);
    sphere_shell(Vec3::new(0.0, -half_height, 0.0), radius, s, -1, out);
}

/// 圆锥（本仓口径：底面半径 `radius` 在 `y = −half_height`，顶点在 `y = +half_height`）：
/// 侧面（半径随 y 线性收）+ 底面圆盘。
/// 侧面外法线 = `normalize((cosθ, k, sinθ))`、`k = radius/(2·half_height)`（侧面斜率）
/// ——锥侧面是**斜**面，法线不是径向，这一步不近似。
fn cone(half_height: f32, radius: f32, s: f32, out: &mut SurfaceLattice) {
    let h = half_height.max(1e-6);
    let k = radius / (2.0 * h);
    let slant = ((2.0 * h) * (2.0 * h) + radius * radius).sqrt();
    let ny = count(slant, s);
    for j in 0..ny {
        let y = -h + (j as f32 + 0.5) * (2.0 * h / ny as f32);
        let r = (radius * (h - y) / (2.0 * h)).max(0.0);
        let nt = count(2.0 * std::f32::consts::PI * r, s);
        for i in 0..nt {
            let th = (i as f32 + 0.5) * (2.0 * std::f32::consts::PI / nt as f32);
            let (sin, cos) = th.sin_cos();
            out.points.push(Vec3::new(r * cos, y, r * sin));
            let nrm = Vec3::new(cos, k, sin);
            out.normals.push(nrm * (1.0 / nrm.length()));
        }
    }
    disk_y(-h, -1.0, radius, s, out);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boxed(h: f32) -> Shape {
        Shape::Box {
            half: Vec3::splat(h),
        }
    }

    /// 方盒：6 面 × 栅格，两层；粒子全部**在体内**（内移量 > 0 ⇒ 严格小于面）。
    #[test]
    fn box_lattice_two_layers_inside() {
        let half = 0.06f32;
        let s = 0.05f32;
        let l = lattice(&boxed(half), s, 2.0 * s);
        // 面内格数 = round(0.12/0.05) = 2 ⇒ 6 面 × 2×2 = 24 表面点。
        assert_eq!(l.n0, 24, "表面点数应 = 6×2×2");
        assert_eq!(l.len(), 48, "两层应翻倍");
        // 标定的**性质**（而不是某个魔数）：`V_b·Σ_bW = 1` ⇔ `ρ0·V_b·Σ_bW = ρ0`
        // ——边界介质自己在静止密度下压力中性。
        // ⚠️ 注意 `V_b` 与 `s³` 的关系**随体形变化**（实测，2026-09-22）：本用例这个
        // 0.12 盒的面采样只有 2×2 点、两层又被薄体钳制贴得很近 ⇒ 层间核重叠大 ⇒ Σ_bW 大
        // ⇒ `V_b ≈ 0.41·s³`；而大体（面采样密、两层相距 1.0s）≈ `0.95·s³`。
        // 旧口径固定 `s³` 恰好对**稀疏小体**过量注入 2.4× 质量 ⇒ 近壁 ρ_i 虚增 ⇒ p_i 失真
        // （实测：h=0.1/半长 0.12 从 −6.13×ρVg 反号回到 1.00×）。
        let sum = boundary_self_sum(&l.pts, 2.0 * s);
        assert!(
            (l.volume * sum - 1.0).abs() < 1e-4,
            "标定应自洽：V_b·Σ_bW = {} （应 = 1）",
            l.volume * sum
        );
        for p in &l.pts {
            assert!(
                p.x.abs() <= half && p.y.abs() <= half && p.z.abs() <= half,
                "边界粒子必须在体内：{p:?}"
            );
            assert!(
                p.x.abs() < half || p.y.abs() < half || p.z.abs() < half,
                "不得落在表面上（两层都已内移）：{p:?}"
            );
        }
    }

    /// 球：全部点在球内且距球心 ≥ R − 1.5s（最远层内移量 ≤ 1.5s + 采样误差）。
    #[test]
    fn sphere_lattice_inside_shell() {
        let r = 0.2f32;
        let s = 0.05f32;
        let l = lattice(&Shape::Sphere { radius: r }, s, 2.0 * s);
        assert!(!l.is_empty());
        assert_eq!(l.len() % 2, 0);
        for p in &l.pts {
            let d = p.length();
            assert!(d <= r, "超出球面：{d} > {r}");
            assert!(d >= r - 1.5 * s - 1e-3, "内移过多（应 ≤ 1.5s）：{d}");
        }
    }

    /// 圆柱/胶囊/圆锥：只查"非空 + 全在局部 AABB 内 + 两层计数为偶"。
    #[test]
    fn primitive_families_nonempty_and_contained() {
        let s = 0.05f32;
        let shapes = [
            Shape::Cylinder {
                half_height: 0.15,
                radius: 0.1,
            },
            Shape::Capsule {
                half_height: 0.1,
                radius: 0.08,
            },
            Shape::Cone {
                half_height: 0.15,
                radius: 0.12,
            },
        ];
        for sh in shapes {
            let l = lattice(&sh, s, 2.0 * s);
            assert!(!l.is_empty(), "应生成粒子：{sh:?}");
            assert_eq!(l.len() % 2, 0, "两层 ⇒ 偶数");
            let rr = sh.bounding_sphere_radius() + 1e-4;
            for p in &l.pts {
                assert!(p.length() <= rr, "超出包围球：{p:?} ({sh:?})");
            }
        }
    }

    /// 不支持的形状 ⇒ **零粒**（回退粗档的信号，不是静默近似）。
    #[test]
    fn unsupported_shapes_yield_nothing() {
        let s = 0.05f32;
        for sh in [
            Shape::Provider(0),
            Shape::HeightField(0),
            Shape::ConvexHull {
                hull: 0,
                half: Vec3::splat(0.1),
            },
            Shape::Compound {
                compound: 0,
                half: Vec3::splat(0.1),
            },
        ] {
            assert_eq!(lattice(&sh, s, 2.0 * s).len(), 0, "不支持：{sh:?}");
        }
    }

    /// 确定性：同形状同间距两次生成**逐位一致**（含浮点运算序）。
    #[test]
    fn lattice_is_deterministic() {
        let s = 0.05f32;
        for sh in [
            boxed(0.07),
            Shape::Sphere { radius: 0.13 },
            Shape::Cone {
                half_height: 0.1,
                radius: 0.09,
            },
        ] {
            let a = lattice(&sh, s, 2.0 * s);
            let b = lattice(&sh, s, 2.0 * s);
            assert_eq!(a, b, "两次生成应逐位一致：{sh:?}");
        }
    }
}
