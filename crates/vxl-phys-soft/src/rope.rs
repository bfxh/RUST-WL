//! **绳索最小闭环**（T1，见 `docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §T1）：
//! 1D 粒子链 + **XPBD 距离约束** + **点-形状接触**（走 `interop::ProviderColliders`，与窄相同一个提供者通道）。
//!
//! **为什么先做绳索**（调研档的口径）：布料那条线最硬的缺口不是 XPBD 本身，而是"三角形作为一等几何"
//! （`Shape` 无三角网/无自碰撞/无面元气动）。而绳索只需要"距离约束 + 点-形状接触"⇒ 能在**不碰三角网
//! 自碰撞**的前提下把 XPBD 的核心循环（子步 × 约束投影 × 拉格朗日乘子/柔度）打通，并立起机器无关判据。
//!
//! **口径**（对齐 `SPEC.md` §4.6 与 Müller et al. 2020 XPBD）：
//! - **子步 × 每子步 1 次迭代**（"Small Steps"口径）⇒ 子步数是**刚度**旋钮，不是精度旋钮；
//! - 距离约束：`C = |Δx| − L`、`α̃ = α/h²`、`Δλ = (−C − α̃·λ)/(w₁+w₂+α̃)`、`Δx = ±w·Δλ·∇C`
//!   （`λ` **每子步清零** —— 它是子步内的量）；`α = 0` ⇒ 不可伸长；
//! - 接触：**位置级投影**（= XPBD 接触 `α = 0` 的特例）⇒ **无恢复系数**（XPBD 接触口的天然行为），
//!   且只有**真穿透**（`depth > 0`）才推、带内（预判接触）不推；
//! - 速度由 `v = (x − x_prev)/h` 反推 ⇒ 位置级修正自动进入速度，**不需要冲量求解器**。
//!
//! **本片不做**（写清边界，各自是后续切片）：摩擦（切向）、体积/弯曲约束、自碰撞、与刚体的双向耦合
//! （Akinci 边界）、GPU 档、以及**门面接线**（`World::add_rope` + `world_step` 相位 + 档位开关）。
//! 本片目标 = **最小闭环 + 判据**（判据在 `tests/rope_minimal.rs`）。
use vxl_phys_core::{interop::ProviderColliders, Vec3};

/// 绳索：等距粒子链 + XPBD 距离约束 + 点-形状接触。
pub struct Rope {
    /// 粒子位置（世界系）。
    pub pos: Vec<Vec3>,
    /// 子步起点位置：速度由 `(pos − prev)/h` 反推。
    prev: Vec<Vec3>,
    /// 粒子速度（`m/s`）。
    pub vel: Vec<Vec3>,
    /// 逆质量（`0` = **钉住**：约束与接触都不会移动它，速度恒为 0）。
    pub inv_mass: Vec<f32>,
    /// 每条距离约束的拉格朗日乘子（每子步清零）。
    lambda: Vec<f32>,
    /// 段长（rest length，`m`）。
    pub rest_len: f32,
    /// compliance α（`m/N`）：`0` = 不可伸长；档位见 [`crate::Stiffness::alpha`]。
    pub compliance: f32,
    /// 粒子半径（接触用，`m`）：`0` = 质点（贴在面上）。
    pub radius: f32,
    /// 子步数（每子步 1 次约束迭代）。
    pub substeps: u32,
    /// 接触皮肤带（预判接触宽度，`m`）。
    pub skin: f32,
    /// 速度阻尼（每子步乘一次）：`1.0` = 无阻尼。**不是** XPBD 的组成部分，只为把"悬垂形状"
    /// 做成**稳态读数**（否则绳永远在摆，读数只能取窗口均值，见测量协议 §5）。
    pub damping: f32,
    /// 接触查询缓冲（复用，免每粒子每子步一次堆分配）。
    buf: Vec<vxl_phys_core::interop::InteropContact>,
}

impl Rope {
    /// **直线投放**：`nodes` 个粒子均布在 `a → b` 上，**两端钉住**；段长 = 弦长/(nodes−1)（紧绳）。
    pub fn line(a: Vec3, b: Vec3, nodes: usize, radius: f32) -> Self {
        Self::span(a, b, nodes, (b - a).length(), radius)
    }

    /// 同上，但**段长按给定总长**：`total_len > 弦长` ⇒ 松绳（初始被压缩，会自己垂下来）。
    pub fn span(a: Vec3, b: Vec3, nodes: usize, total_len: f32, radius: f32) -> Self {
        let n = nodes.max(2);
        let denom = n as f32 - 1.0;
        let mut pos = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as f32 / denom;
            pos.push(a + (b - a) * t);
        }
        let mut inv_mass = vec![1.0f32; n];
        inv_mass[0] = 0.0;
        inv_mass[n - 1] = 0.0;
        Rope {
            prev: pos.clone(),
            vel: vec![Vec3::ZERO; n],
            lambda: vec![0.0f32; n - 1],
            pos,
            inv_mass,
            rest_len: total_len / denom,
            compliance: 0.0,
            radius,
            substeps: 8,
            skin: 0.01,
            damping: 1.0,
            buf: Vec::new(),
        }
    }

    /// 粒子数。
    pub fn nodes(&self) -> usize {
        self.pos.len()
    }

    /// 钉住 / 松开第 `i` 个粒子。
    pub fn set_pinned(&mut self, i: usize, pinned: bool) {
        if i < self.inv_mass.len() {
            self.inv_mass[i] = if pinned { 0.0 } else { 1.0 };
        }
    }

    /// 第 `k` 段的当前长度（判据用：不可伸长性）。
    pub fn segment_len(&self, k: usize) -> f32 {
        (self.pos[k + 1] - self.pos[k]).length()
    }

    /// 推进一个 `dt`（内部再切 `substeps` 个子步）；接触走 `providers` 里 `ids` 指定的提供者。
    pub fn step(&mut self, dt: f32, gravity: Vec3, providers: &dyn ProviderColliders, ids: &[u32]) {
        let h = dt / self.substeps.max(1) as f32;
        for _ in 0..self.substeps {
            self.substep(h, gravity, providers, ids);
        }
    }

    /// 单个子步：预测 → 距离约束 → 接触 → 速度回写。
    fn substep(&mut self, h: f32, gravity: Vec3, providers: &dyn ProviderColliders, ids: &[u32]) {
        let n = self.pos.len();
        // ① 预测：`v ← v + g·h`、`x_prev ← x`、`x ← x + v·h`（钉住粒子原地不动、速度清零）。
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                self.prev[i] = self.pos[i];
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] += gravity * h;
            self.prev[i] = self.pos[i];
            self.pos[i] += self.vel[i] * h;
        }
        // ② 距离约束（Gauss-Seidel 顺序推进；`λ` 每子步清零）。
        for l in self.lambda.iter_mut() {
            *l = 0.0;
        }
        let a_tilde = self.compliance / (h * h);
        for k in 0..n.saturating_sub(1) {
            let (i, j) = (k, k + 1);
            let w = self.inv_mass[i] + self.inv_mass[j];
            if w <= 0.0 {
                continue; // 两粒子都钉住 ⇒ 该约束无自由度
            }
            let d = self.pos[j] - self.pos[i];
            let len = d.length();
            if len < 1e-9 {
                continue; // 退化（两端重合）：方向无定义，跳过（下一子步自会分开）
            }
            let dir = d * (1.0 / len);
            let c = len - self.rest_len;
            let dl = (-c - a_tilde * self.lambda[k]) / (w + a_tilde);
            self.lambda[k] += dl;
            self.pos[i] -= dir * (self.inv_mass[i] * dl);
            self.pos[j] += dir * (self.inv_mass[j] * dl);
        }
        // ③ 接触：位置级投影（只有真穿透才推 ⇒ 无恢复系数）。
        if self.radius >= 0.0 && !ids.is_empty() {
            self.project_contacts(providers, ids);
        }
        // ④ 速度回写 + 阻尼。
        let inv_h = 1.0 / h;
        for i in 0..n {
            if self.inv_mass[i] == 0.0 {
                self.vel[i] = Vec3::ZERO;
                continue;
            }
            self.vel[i] = (self.pos[i] - self.prev[i]) * inv_h * self.damping;
        }
    }

    /// 逐粒子把穿透推到面上（多接触按 Gauss-Seidel 顺序逐个推；带内预判不推）。
    fn project_contacts(&mut self, providers: &dyn ProviderColliders, ids: &[u32]) {
        // `buf` 借出去才能再借 `self.pos`（同窄相 `prims.rs` 的 `mem::take` 手法）。
        let mut buf = std::mem::take(&mut self.buf);
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            for &id in ids {
                buf.clear();
                if !providers.contacts_sphere(id, self.pos[i], self.radius, self.skin, &mut buf) {
                    continue;
                }
                for c in &buf {
                    if c.depth > 0.0 {
                        self.pos[i] += c.normal * c.depth;
                    }
                }
            }
        }
        self.buf = buf;
    }
}
