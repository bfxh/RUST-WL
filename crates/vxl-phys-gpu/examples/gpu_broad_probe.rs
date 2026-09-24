//! **宽相上卡的对拍**（`PLAN-gpu.md` §17.2 第一片）：卡上 `vxl_phys_gpu::broad` vs CPU
//! `GridBroadPhase`（M0 的精确宽相，为"大世界"保留的那份）。
//!
//! **为什么参考选 `GridBroadPhase` 而不是默认的 `BvhBroadPhase`**：后者的配对集合**不是当前 AABB
//! 的纯函数**——它用速度相关的 **fat margin**（`fat_margin_for(linvel)`）、只纳入**清醒**的动体、
//! 且树状态**跨 tick 增量**（`bvh_phase.rs`：`fat_margin_for` / `bodies.awake[i]` / 增量维护）
//! ⇒ 与它逐位对齐等于把"历史 + 睡眠 + 树"一起搬上卡。`GridBroadPhase` 的配对集合是纯函数
//! （"全部相交的 AABB 对，静态-静态除外"）⇒ 这才是**与加速结构无关**的物理量，可以做硬判据。
//!
//! **判据**：两边的配对表 `sort_unstable + dedup` 之后**逐元素相同**（长度 + 每一项）。
//! **金丝雀**：把**卡上那一份输入**里某个体的 AABB 挪一下 ⇒ 判据**必须变红**（证明比较有分辨力）。
//! **自证**：同一输入在卡上连跑两次 ⇒ 输出逐位相同（原子序不定不该漏到结果里）。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_broad_probe [--adapter K]`

use vxl_phys::{BroadPhase, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_broad::{shape_aabb, GridBroadPhase};
use vxl_phys_gpu::broad::broad_on_adapter;

/// 场景：一层静态盒地板 + 一批动态盒（与 `m1_profile` 同族，规模缩小到可秒级复现）。
fn scene(n_side: usize, n_dyn: usize) -> World {
    let mut w = World::new(PhysConfig::default());
    let half = 0.5f32;
    for k in 0..n_side * n_side {
        let x = (k % n_side) as f32 - n_side as f32 * 0.5;
        let z = (k / n_side) as f32 - n_side as f32 * 0.5;
        w.add_static(
            Shape::Box {
                half: Vec3::splat(half),
            },
            Vec3::new(x, 0.0, z),
            Quat::IDENTITY,
        );
    }
    for k in 0..n_dyn {
        // ⚠️ 必须**贴着地板**（静态盒顶面 y = 0.5）且**彼此够近** ⇒ 配对非空；否则判据空转
        //（首版把动体放到 y≥0.6 的高度、x/z 铺开 ±10 ⇒ 两个方向都不相交 ⇒ 0 对，金丝雀如期喊
        // "判据无分辨力"，正是它的用处）。
        let x = ((k * 37) % 97) as f32 / 97.0 * 10.0 - 5.0;
        let z = ((k * 53) % 89) as f32 / 89.0 * 10.0 - 5.0;
        let y = 0.5 + ((k * 29) % 7) as f32 * 0.08;
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.4),
            },
            Vec3::new(x, y, z),
            Quat::IDENTITY,
            1000.0,
        );
    }
    w
}

/// 逐体 AABB（**与 CPU 宽相同一份 `shape_aabb`**；本场景无高度场/provider ⇒ 两个切片为空）。
fn aabbs_of(w: &World, skin: f32) -> Vec<vxl_phys_core::Aabb> {
    (0..w.bodies.len())
        .map(|i| {
            shape_aabb(
                &w.bodies.shape[i],
                w.bodies.position[i],
                w.bodies.rot(i),
                skin,
                &[],
                &[],
            )
        })
        .collect()
}

/// AABB 表 → 卡上布局（8 f32/体：`min.xyz | pad | max.xyz | dyn`）。
fn pack(w: &World, aabbs: &[vxl_phys_core::Aabb]) -> Vec<f32> {
    let mut out = Vec::with_capacity(aabbs.len() * 8);
    for (i, a) in aabbs.iter().enumerate() {
        let dyn_bit = if w.bodies.is_dynamic(i) { 1.0f32 } else { 0.0 };
        out.extend_from_slice(&[a.min.x, a.min.y, a.min.z, 0.0]);
        out.extend_from_slice(&[a.max.x, a.max.y, a.max.z, dyn_bit]);
    }
    out
}

/// 比较两份配对表（都已 sort+dedup）⇒ `(相等?, 第一条不同项)`。
fn cmp(a: &[(u32, u32)], b: &[(u32, u32)]) -> (bool, Option<(u32, u32)>) {
    if a.len() != b.len() {
        return (false, b.get(a.len()).copied().or_else(|| a.last().copied()));
    }
    for (x, y) in a.iter().zip(b.iter()) {
        if x != y {
            return (false, Some(*y));
        }
    }
    (true, None)
}

fn main() {
    let mut adapter = 0usize;
    let rest: Vec<String> = std::env::args().skip(1).collect();
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    // ⚠️ 名字不能叫 `CELL`：本仓词汇门把独立单词 `cell` 列为禁词（只豁免 `std::cell`）。
    const CELL_SIZE: f32 = 2.0;
    const SKIN: f32 = 0.01;
    const CAP_PAIRS: u32 = 1 << 20;
    println!("== 宽相上卡 vs CPU `GridBroadPhase`：**配对集合逐位相同** ==");
    for (n_side, n_dyn) in [(20usize, 40usize), (40, 200), (60, 600)] {
        let w = scene(n_side, n_dyn);
        let n = w.bodies.len() as u32;
        let aabbs = aabbs_of(&w, SKIN);
        // CPU 参考（同一份 AABB：`shape_aabb` 是共享实现）
        let mut bp = GridBroadPhase::new(CELL_SIZE, SKIN);
        let jobs = vxl_phys_core::schedule::SerialJobSystem;
        let cpu_pairs = bp.compute_pairs(&w.bodies, &[], &[], &jobs).to_vec();
        // 卡上
        let boxes = pack(&w, &aabbs);
        let out = broad_on_adapter(adapter, &boxes, n, CELL_SIZE, CAP_PAIRS);
        if let Some(e) = &out.error {
            println!("  ⚠️ 卡上宽相不可用：{e}");
            return;
        }
        let (eq, first) = cmp(&cpu_pairs, &out.pairs);
        println!(
            "  n={n:>5}（静态 {} + 动态 {n_dyn}）| 格 {cells} | CPU {cp} 对 / 卡上 {gp} 对 | overflow {ov} ⇒ {verdict}",
            n as usize - n_dyn,
            cp = cpu_pairs.len(),
            gp = out.pairs.len(),
            ov = out.overflow,
            cells = out.cells,
            verdict = if eq {
                "**逐位相同 ✓**".to_string()
            } else {
                format!("**不同 ✗**（首条差异：卡上 {first:?}）")
            }
        );
        // 自证：卡上连跑两次（原子序不定不该漏到结果里）
        let again = broad_on_adapter(adapter, &boxes, n, CELL_SIZE, CAP_PAIRS);
        println!(
            "     自证：卡上连跑两次 ⇒ {}",
            if again.pairs == out.pairs {
                "逐位相同 ✓".to_string()
            } else {
                format!(
                    "**不同 ✗**（{} vs {} 对）",
                    again.pairs.len(),
                    out.pairs.len()
                )
            }
        );
        // 金丝雀：只挪**卡上那一份**输入 ⇒ 判据必须变红。
        // ⚠️ 扰动要**真的改变相交关系**：抬一个静态盒 0.6 m 不算（x/z 不变 ⇒ 它与哪些动体相交没变，
        // 配对集合原样 ⇒ 首版金丝雀报"无分辨力"）。这里取**最后一个动态体**横移 3 m（跨出接触范围）。
        let mut probe = boxes.clone();
        let k = (n as usize - 1) * 8;
        probe[k] += 3.0;
        probe[k + 4] += 3.0; // min.x 与 max.x 同移 ⇒ 整块平移
        let canary = broad_on_adapter(adapter, &probe, n, CELL_SIZE, CAP_PAIRS);
        let (ceq, _) = cmp(&cpu_pairs, &canary.pairs);
        println!(
            "     金丝雀（只挪卡上输入的一个 AABB）⇒ {n2} 对 vs CPU {cp} 对 ⇒ {verdict}",
            n2 = canary.pairs.len(),
            cp = cpu_pairs.len(),
            verdict = if ceq {
                "**仍然相同 ✗（判据无分辨力！）**"
            } else {
                "如期变红 ✓"
            }
        );
        if n > 2000 {
            break; // 再大的规模交给 §17.1 的相位账（本探针只要判据成立）
        }
    }
}
