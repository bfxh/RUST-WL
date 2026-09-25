//! narrow_tier：窄相**卡上档**的门面侧（`PLAN-gpu.md` §17.9）。
//!
//! 单独成档的理由（与 `world_step/fluid_stepper.rs` 同款）：接线代码集中一处，`world_step.rs` 只留
//! 调用点——既有文件加行要配"函数变短"（尺寸门），新文件只判阈值。
//!
//! **一趟的形状**：主机打包逐体表 + 对表 → 后端（卡上）出**对序**槽表 → 主机把"卡上不接手的对"
//! 交给 `DefaultNarrowPhase` 回填 → 按**对序**装配成 `manifolds`（槽有流形就用槽、是哨兵就取回填段）。
//! ⇒ 流形表的**结构与顺序**与纯 CPU 路径逐条对应，差别只在浮点末位（口径 B）。
//!
//! **两条前提**（`PLAN-gpu.md` §17.7）：① 本档**不实现 `predict_inflate`** ⇒ `predict_dt > 0` 的
//! 那一趟直接让位（回退 CPU），绝不在两套口径之间混；② 容差是"地板 ⊕ 尺度"的混合口径 ⇒ 卡上档
//! 的读数与默认档**必然不同**，只能当显式档用（默认档一行都不走这里）。

use super::*;

/// **宿主窄相槽**：`host` = `DefaultNarrowPhase`（它持有外壳/复合体仓库 ⇒ 卡上档也靠它做回填），
/// `tier` = 可选的**卡上档**后端。
///
/// 为什么包一层而不是给 `World` 加一个字段：`world_struct.rs` 是**零函数档**（加行即红）且
/// `World` 的成员数正卡在棘轮上（23）⇒ 换字段类型不动行数与成员数。`Deref`/`DerefMut` 让
/// 既有的 `self.narrow.xxx(...)` 调用点（16 处）**一行都不用改**。
pub struct NarrowSlot {
    pub host: DefaultNarrowPhase,
    pub(crate) tier: Option<Box<dyn vxl_phys_core::narrow_tier::NarrowTierBackend>>,
}

impl From<DefaultNarrowPhase> for NarrowSlot {
    fn from(host: DefaultNarrowPhase) -> Self {
        Self { host, tier: None }
    }
}

impl std::ops::Deref for NarrowSlot {
    type Target = DefaultNarrowPhase;
    fn deref(&self) -> &DefaultNarrowPhase {
        &self.host
    }
}

impl std::ops::DerefMut for NarrowSlot {
    fn deref_mut(&mut self) -> &mut DefaultNarrowPhase {
        &mut self.host
    }
}

impl World {
    /// **注册卡上窄相档**：此后每个"检测趟"都先试卡上路径（`NarrowTierBackend`，如 GPU 档）；
    /// 未注册 / 后端报错 / 超容量 / `predict_dt > 0` 时**整趟回退 CPU**（不半途混用）。
    ///
    /// **逐位不变性**：未注册时一行都不走新路径 ⇒ 默认档（含金样门与三哈希）逐位不变。
    pub fn set_narrow_tier(
        &mut self,
        tier: Box<dyn vxl_phys_core::narrow_tier::NarrowTierBackend>,
    ) {
        self.narrow.tier = Some(tier);
    }

    /// 是否已注册卡上窄相档（诊断/探针用）。
    pub fn has_narrow_tier(&self) -> bool {
        self.narrow.tier.is_some()
    }

    /// 本 tick 的对表（诊断用：卡上档那一趟的输入就是这个列表）。
    pub fn pairs(&self) -> &[(u32, u32)] {
        &self.pairs
    }

    /// **卡上窄相一趟**：返回 `true` = `self.manifolds` 已由卡上档填好；`false` = 调用方走 CPU 路径。
    ///
    /// `predict_dt` = 本次检测趟本要传给 CPU 窄相的预测时长（见 `set_predict_dt`）：非 0 就让位
    /// ——本档没有 `predict_inflate`，照跑会与 CPU 差一项。
    pub(crate) fn narrow_tier_pass(&mut self, pairs: &[(u32, u32)], predict_dt: f32) -> bool {
        if predict_dt > 0.0 {
            return false; // 前提①：本档不实现 predict_inflate ⇒ 那一趟必须整趟走 CPU
        }
        // `take` 出来再放回（同 `fluid_stepper_pass` 的理由）：`narrow_run` 借后端，而随后要可变借
        // 宿主窄相与 `self.manifolds`，直接 `as_ref()` 会把 `self` 整份借住。
        let Some(tier) = self.narrow.tier.take() else {
            return false;
        };
        let packed = vxl_phys_core::narrow_tier::pack_bodies(&self.bodies);
        dump_words_if_asked(&packed, pairs, self.tick);
        let flat = vxl_phys_core::narrow_tier::flat_pairs(pairs);
        let slots = match tier.narrow_run(&packed, &flat) {
            Ok(s) => s,
            Err(_) => {
                self.narrow.tier = Some(tier); // 卡上跑不通（无适配器/超容量）⇒ 回退 CPU，档还留着
                return false;
            }
        };
        // 回填段：只把「卡上不接手」的对交给 CPU 窄相（**一次调用** ⇒ 它的缓存/并行照旧生效）。
        let sub: Vec<(u32, u32)> = pairs
            .iter()
            .enumerate()
            .filter(|(i, _)| slots.count(*i) == vxl_phys_core::narrow_tier::NOT_HANDLED)
            .map(|(_, p)| *p)
            .collect();
        let mut host: Vec<Manifold> = Vec::new();
        if !sub.is_empty() {
            self.narrow.host.collide(
                &self.bodies,
                &sub,
                self.terrain.slice(),
                &self.providers,
                &mut host,
                self.jobs.as_ref(),
            );
        }
        // 装配 = **对序**：逐对按升序取「卡上槽」或「回填段」。回填段是 `pairs` 的子序列 ⇒ 顺序走
        // 下去时 `(a, b)` 相配的流形必然相邻（不需要查表）；复合体一对多流形自然落在这一段里。
        self.manifolds.clear();
        let mut cur = 0usize;
        for (i, &(a, b)) in pairs.iter().enumerate() {
            match slots.count(i) {
                vxl_phys_core::narrow_tier::NOT_HANDLED => {
                    while cur < host.len() && (host[cur].a, host[cur].b) == (a, b) {
                        self.manifolds.push(host[cur].clone());
                        cur += 1;
                    }
                }
                0 => {}
                _ => self.manifolds.push(manifold_of_slot(&slots, i)),
            }
        }
        self.narrow.tier = Some(tier);
        true
    }
}

/// 槽 → 流形（`space` 恒 0：复合体一律走回填 ⇒ 卡上不会出现"子形状流形"）。
fn manifold_of_slot(sl: &vxl_phys_core::narrow_tier::NarrowSlots, i: usize) -> Manifold {
    let (a, b) = sl.pair(i);
    let n = sl.normal(i);
    let cnt = sl.count(i) as usize;
    let mut buf = [ContactPoint::default(); 4];
    for (k, slot) in buf.iter_mut().enumerate().take(cnt) {
        let (p, d, f) = sl.point(i, k);
        *slot = ContactPoint {
            point: Vec3::new(p[0], p[1], p[2]),
            depth: d,
            feature: f,
        };
    }
    Manifold {
        a,
        b,
        normal: Vec3::new(n[0], n[1], n[2]),
        points: vxl_phys_narrow::ContactPoints::from_slice(&buf[..cnt]),
    }
}

/// **一次性诊断**（默认关，生产路径零开销）：`VXL_NARROW_DUMP=<a>:<b>:<tick>` ⇒ 在那一趟里，
/// 把**本趟真正打包给卡上**的那两个体的位姿/半长打出来。用途（§17.9 补记六的下一步）：与探针
/// **按快照重建**的那份**逐字对拍** ⇒ 区分"问题在核读表"还是"问题在打包那一刻的体态（帧）"。
fn dump_words_if_asked(packed: &[u32], pairs: &[(u32, u32)], tick: u64) {
    const W: usize = vxl_phys_core::narrow_tier::BODY_WORDS;
    let Ok(v) = std::env::var("VXL_NARROW_DUMP") else {
        return;
    };
    let mut it = v.split(':');
    let (x, y, t) = (it.next(), it.next(), it.next());
    let (Some(a), Some(b), Some(t0)) = (
        x.and_then(|s| s.parse::<u32>().ok()),
        y.and_then(|s| s.parse::<u32>().ok()),
        t.and_then(|s| s.parse::<u64>().ok()),
    ) else {
        return;
    };
    if tick != t0 || !pairs.contains(&(a, b)) {
        return;
    }
    let f = |i: usize, k: usize| f32::from_bits(packed[i * W + k]);
    let p3 = |i: usize, k: usize| [f(i, k), f(i, k + 1), f(i, k + 2)];
    let q4 = |i: usize, k: usize| [f(i, k), f(i, k + 1), f(i, k + 2), f(i, k + 3)];
    eprintln!(
        "NARROW_DUMP tick={tick} 对({a},{b})：a.pos={:?} a.rot={:?} a.half={:?} | b.pos={:?} b.rot={:?} b.half={:?}",
        p3(a as usize, 0),
        q4(a as usize, 3),
        p3(a as usize, 8),
        p3(b as usize, 0),
        q4(b as usize, 3),
        p3(b as usize, 8),
    );
}
