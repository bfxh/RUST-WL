//! **提供者（体素 / 三角网 / 平面集 / 喷溅）参与的对：体形状 → 接触候选**的分发与采样。
//!
//! 为什么单独一个文件：`pair_shaped.rs` 是 876 行 / 最长函数 112 行的**受门禁棘轮**文件
//! （`god-baseline.json`，只准减不许胖）⇒ 采样这类"还会继续长"的代码放这里，那边只留
//! 一行分发（净减行数，见该文件 `provider_pair`）。
//!
//! 语义：`Box` / `Sphere` / `ConvexHull` 三臂是从 `pair_shaped.rs` **纯搬移**（逐条不变）；
//! `Capsule` 是新增（见 `capsule_provider_contacts` 的注）。
use super::*;

impl DefaultNarrowPhase {
    /// 把「体形状 vs 提供者」的候选接触压进 `buf`；返回 `false` = 该形状暂不受理。
    /// 调用方（`pair_shaped.rs::provider_pair`）拿到 `buf` 后再做**主导面选择 + 取点**。
    #[allow(clippy::too_many_arguments)] // 形状/位姿/提供者/出参 + 带符号朝向
    pub(crate) fn provider_shape_contacts(
        &mut self,
        body_shape: &Shape,
        body: u32,
        bpos: Vec3,
        brot: Quat,
        id: u32,
        pr_is_a: bool,
        band: f32,
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        buf: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match *body_shape {
            Shape::Box { half } => providers.contacts_box(id, half, bpos, brot, band, buf),
            // 球：SDF 类提供者解析求解（`depth = r − sdf(center)`）
            Shape::Sphere { radius } => providers.contacts_sphere(id, bpos, radius, band, buf),
            // 外壳 vs 提供者：**顶点采样**（逐顶点按 SDF 解析求深度/法线；多点 ⇒ 面接触稳定）。
            Shape::ConvexHull { .. } => self.hull_provider_contacts(
                body_shape, body, bpos, brot, id, pr_is_a, band, providers, buf,
            ),
            // 胶囊 vs 提供者：**沿轴 N 球采样**（胶囊 = 沿轴球族的并集）。
            Shape::Capsule {
                half_height,
                radius,
            } => {
                capsule_provider_contacts(bpos, brot, half_height, radius, id, band, providers, buf)
            }
            // 圆柱 / 圆锥 vs 提供者：**端面圆 + 母线采样**（逐点查询；见下方函数注）。
            Shape::Cylinder {
                half_height,
                radius,
            } => {
                let rings = [(-half_height, radius), (0.0, radius), (half_height, radius)];
                ring_provider_contacts(bpos, brot, rings, None, id, band, providers, buf)
            }
            Shape::Cone {
                half_height,
                radius,
            } => {
                // 圆锥半径沿母线线性收缩：ρ(y) = radius·(half_height − y)/(2·half_height)；
                // 顶点（+端）是几何极值点，单独补一个样本。
                let rings = [
                    (-half_height, radius),
                    (-half_height * 0.5, radius * 0.75),
                    (0.0, radius * 0.5),
                ];
                ring_provider_contacts(
                    bpos,
                    brot,
                    rings,
                    Some(half_height),
                    id,
                    band,
                    providers,
                    buf,
                )
            }
            _ => false, // 其余形状 vs 提供者：待专用查询
        }
    }

    /// 外壳 vs 提供者（原 `pair_shaped.rs` 的 hull 臂，纯搬移）。
    ///
    /// 顶点序即特征序；顶点世界点走缓存（同体同帧多对时只做一次 O(n) 变换）。
    /// 返回 `true` = "本通路受理"（含**外壳数据缺失**的情形——搬移前那里是 `return true`，
    /// 语义是"整对结束"，调用方对 `true`/`false` 的处理相同，这里原样保留）。
    #[allow(clippy::too_many_arguments)]
    fn hull_provider_contacts(
        &mut self,
        body_shape: &Shape,
        body: u32,
        bpos: Vec3,
        brot: Quat,
        id: u32,
        pr_is_a: bool,
        band: f32,
        providers: &dyn vxl_phys_core::interop::ProviderColliders,
        buf: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        let side = if pr_is_a { 1 } else { 0 };
        if !self.fill_hull_world(side, body, body_shape, bpos, brot) {
            return true;
        }
        let mut supported = false;
        for k in 0..self.hull_pts[side].len() {
            supported |= providers.contacts_point(id, self.hull_pts[side][k], band, buf);
        }
        supported
    }
}

/// **胶囊 vs 提供者：沿轴 N 球采样**（`k = 0` = `−axis·half_height` 端，`k = N−1` =
/// `+axis·half_height` 端，均布 `t = k/(N−1)` ⇒ 采样点与速度/帧率无关）。
///
/// 为什么是球采样：提供者接口只有**点 / 球 / 盒**三个原语（`vxl-phys-core/src/interop.rs`），
/// `Capsule` 不在其中；而胶囊恰是"沿轴球族的并集" ⇒ 逐球心问一次 `contacts_sphere`
/// （解析、带半径），并集就是胶囊的接触集合——**不必给 trait 加 `contacts_capsule`**
/// （那要四个提供者各实现一遍，且 SDF 类本就直接支持球）。这条口径与**高度场**路径的
/// `hf.rs::capsule_heightfield`（同样沿轴 N=5 球采样）一致 ⇒ 同一形状在两条地形路上的
/// 行为可对齐比较。
///
/// **去重的范围**（写清不藏）：只剔除**球心重合**的样本——`half_height ≈ 0` 的退化胶囊会让
/// 5 个球心落在同一点，同一点给 4 槽重复接触 = **4 倍刚度**。**不按"同一张面只留最深"去重**：
/// 侧躺的胶囊在同一张面上给的是一整条线上的**不同点**（这正是它需要 ≥2 点支撑的原因），
/// 按面收敛到最深一点会让它退化成"单点支撑 ⇒ 来回摇"；点数上限由下游的 4 槽负责。
///
/// 已知取舍：相邻球心相距 `d = 2·half_height/(N−1)` ⇒ 两球包络在轴中段比真胶囊最薄处浅
/// `≈ d²/(8·radius)`（本仓测试档 0.35/0.35 ⇒ ≈ 1 cm）⇒ 一条**恰好卡在两样本正中间**的尖脊
/// 会晚一个子步建接触。要更密就抬 `N`（或按轴长/半径自适应），本期先与高度场同档。
#[allow(clippy::too_many_arguments)] // 端点/朝向/半径 ×2 + 提供者 + 出参
fn capsule_provider_contacts(
    bpos: Vec3,
    brot: Quat,
    half_height: f32,
    radius: f32,
    id: u32,
    band: f32,
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
    buf: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    const SAMPLES: usize = 5;
    let axis = Mat3::from_quat(brot).mul_vec3(Vec3::Y);
    let denom = (SAMPLES - 1) as f32;
    // 球心去重阈值取**相对半径**（形状自带的尺度）：绝对阈值在小尺度下失效。
    let min2 = (radius * 1e-3) * (radius * 1e-3);
    let mut centers = [Vec3::ZERO; SAMPLES];
    let mut n = 0usize;
    let mut supported = false;
    for k in 0..SAMPLES {
        let t = k as f32 / denom;
        let c = bpos + axis * (half_height * (2.0 * t - 1.0));
        if centers[..n]
            .iter()
            .any(|p| (*p - c).length_squared() <= min2)
        {
            continue;
        }
        centers[n] = c;
        n += 1;
        supported |= providers.contacts_sphere(id, c, radius, band, buf);
    }
    supported
}

/// **圆柱 / 圆锥 vs 提供者：端面圆 + 母线采样**（逐点 `contacts_point`，与盒/外壳的顶点采样同族）。
///
/// **为什么不能像胶囊那样走"沿轴球采样"**：球的表面**绕轴旋转对称**，沿轴球族的并集恰好就是胶囊；
/// 但同一族球对圆柱的并集**永远是"端部也收成球"的胶囊**——球在端面之外还伸出 `radius` ⇒ 正立圆柱
/// 会被那个**不存在的圆角托起 `radius`**（0.35 m，不是小量）。圆柱/圆锥的极值点在**端面圆周**与
/// **母线**上 ⇒ 只能采点。
///
/// **每环恰好 4 点、且从"世界最低方向"起算 90° 均布** —— 这条是**判据的要求**，不是省算力：
/// 下游（`pick_dominant_normal` + `four_corner_points`）**只有 4 槽**，候选 >4 时按提供者给的
/// `feature` 排序取前 4 ⇒ **可能丢掉一个角**。实测（本文件首版 = 16 固定角 + 解析最低点，17 候选）：
/// 直立圆柱的端面圆在 5 个候选里被丢 1 个 ⇒ 接触补丁**不对称** ⇒ 每 tick 注入净力矩 ⇒ 圆柱
/// **倾倒并"拧"进地板**（逐 tick 轨迹：`y 0.36 → −0.44`、`ay 1.00 → −0.93`，而流形深度一直 ≈ 0）。
/// 候选**恰好 4 个**时 `four_corner_points` 不排序、不丢弃（它的分支是 `len() > 4`）⇒ 补丁必然对称。
///
/// **从最低方向起算**同样是为精确：固定角（0/90/180/270）在侧躺时没有一个落在环的**最低方向**上
/// （最坏差半个角隙）⇒ 圆柱会陷进地板 `ρ(1−cos45°) ≈ 0.29ρ`（本档半径 0.35 ⇒ **10 cm**）。
/// 环平面水平时（圆柱/圆锥正立）"最低方向"退化 ⇒ 退回固定角（此时整环等高，四角本就对称贴住支撑面）。
///
/// **采样集合**（局部系，轴 = +Y）：`rings` 给（轴向位置, 环半径）三元组，每环 4 点；`tip` 给
/// **圆锥顶点**（圆柱 `None`）。**端面圆心不采**——圆盘的支撑永远在圆周上，补圆心只会多一个候选
/// 而**破坏上面那条对称性**（盒的"面心补位"能成立是因为它由提供者用 `feature` 标了面心优先级，
/// 本通道的 `feature` 由提供者给、我们改不动）。合计：圆柱 3×4 = 12 点、圆锥 13 点（比盒的 30 点省）。
///
/// **侧躺的接触与稳定位形**（实测，免得读判据时误判）：**圆柱**侧躺 = **线接触**（母线平行于地面，
/// 三个环各出 1 点、共线）⇒ 停在质心高 `radius`；**圆锥**侧躺时**轴正水平的那一刻只有底圈最低点
/// 一个接触点（不稳定）**，随后自己滚到**母线贴地**（顶点与底圈最低点同时贴地）才稳定
/// —— 所以侧躺圆锥的判据只判"贴地"，不判"位形仍是水平轴"。
#[allow(clippy::too_many_arguments)] // 位姿 + 采样环 + 顶点 + 提供者 + 出参
fn ring_provider_contacts(
    bpos: Vec3,
    brot: Quat,
    rings: [(f32, f32); 3],
    tip: Option<f32>,
    id: u32,
    band: f32,
    providers: &dyn vxl_phys_core::interop::ProviderColliders,
    buf: &mut Vec<vxl_phys_core::interop::InteropContact>,
) -> bool {
    let m = Mat3::from_quat(brot);
    let axis = m.mul_vec3(Vec3::Y);
    // 世界 −Y 在环平面内的分量（归一化即"环上最低"方向）：e = a·(a·Y) − Y；其模 = |sin(轴与 Y 的角)|。
    let e = axis * axis.y - Vec3::Y;
    // 映回局部系取起始角（环平面 = 局部 XZ 平面 ⇒ φ = atan2(z, x)）；水平环没有最低方向 ⇒ 退回 0°。
    let phi = if e.length_squared() > 1e-8 {
        let l = brot.conjugate().rotate_vec3(e);
        l.z.atan2(l.x)
    } else {
        0.0
    };
    let mut supported = false;
    for (cy, rho) in rings {
        for k in 0..4 {
            let th = phi + std::f32::consts::FRAC_PI_2 * k as f32;
            let local = Vec3::new(rho * th.cos(), cy, rho * th.sin());
            supported |= providers.contacts_point(id, bpos + m.mul_vec3(local), band, buf);
        }
    }
    if let Some(ty) = tip {
        supported |= providers.contacts_point(id, bpos + axis * ty, band, buf);
    }
    supported
}
