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
