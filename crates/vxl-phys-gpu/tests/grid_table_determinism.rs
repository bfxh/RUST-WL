//! **【定位探针·收口】2b 路径不可复现的源头：格表（爆格 ⇒ `canon` 跳过规范化）**（§24 / §24.4）
//!
//! ## 已实测（2026-09-26）
//!
//! `vxl_phys_gpu::grid::grid_on_adapter` 喂同一份输入（**流体 + 地板边界**）跑 6 遍，**盒照管线的取法**
//! （借引擎那张表 `neighbor_grid()` 的 `min/inv/dims`）：
//!
//! | 量 | 结果 | 说明 |
//! |---|---|---|
//! | `items` 哈希 | **5 / 6 遍不同** | 不可复现 ⇒ `place` 的 `atomicAdd` 槽位序**没被规范化掉** |
//! | `start` 哈希 | 0 / 6 遍不同 | 计数/前缀和确定（整数、顺序无关）✅ |
//! | `overflow` | **4** | 有 4 个格超过 `cap = 512` ⇒ **`canon` 对它们直接放弃规范化** |
//!
//! ⇒ **链条**：粒子落在格盒之外（地板那层）⇒ 被 `axis_bin` 钳位**压进边缘格** ⇒ 那几个格**爆满** ⇒
//! `canon` 按其护栏**跳过规范化** ⇒ 那些格的 `items` 留在**原子占位序** ⇒ 密度核按 `items` 求和 ⇒
//! **求和序随运行变** ⇒ 1 ulp 种子 ⇒ 被混沌放大。纯流体没有盒外粒子、不爆格 ⇒ 全程规范化 ⇒ **逐位可复现**
//! ——与"不可复现是 2b 特有"完全对上。
//!
//! ⚠️ **我先前把 `overflow` 判成 0 是量错了对象**：那个"最大格占用 8"是在**引擎那张表**上量的，而它是
//! `set_boundary_particles` **之前**建的（不含地板、盒也不含）⇒ 拿它当"格子不爆"的依据是错的。
//! **教训（本会话第三条同族）**：**先确认量的是不是同一张表/同一个时刻**（前两条见 §24.1、§24.3）。
//!
//! ## 本文件跑两遍盒子取法，用来**分辨"盒子陈旧"与"别的原因"**
//!
//! - **借引擎的表**（管线的现行取法）：2b 场景 —— 盒外粒子 > 0、`overflow` > 0、`items` 不可复现；
//! - **覆盖全部粒子的新盒**（`vxl_phys_core::grid::grid_box` 按**当前全量位置**算）：若三者都归零
//!   ⇒ **根因就是"盒子没覆盖粒子"**（修法 = 把盒的来源改成全量位置）；若不归零 ⇒ 另有原因（继续查）。
//!
//! ## 修法（两档，**未实施**，都属**换代级**，别静默做）
//!
//! 1. **根因档**：格盒按**全量**粒子算（含边界），别借可能在 `set_boundary_particles` 之前建的表。
//! 2. **护栏档**：抬 `cap`（512 → 4096）让爆满的格也被规范化——**没治根**，盒外粒子仍在错格里，
//!    只是把"不可复现"换成"错但可复现"。
//!
//! CI 无适配器 ⇒ 与其它 GPU 探针同口径跳过（不构成 CI 门禁）。
//!
//! ⚠️ 本文件**只放一个 `#[test]`**：两个 GPU 重的测试会被 `cargo test` 并行跑 ⇒ 并行建多个设备实例
//! 会**卡住**（实测 >300 s 超时）。

use vxl_phys_core::grid::{grid_box, GRID_MAX_BINS};
use vxl_phys_core::{Quat, Shape, Vec3};
use vxl_phys_fluid::{BodyPose, FluidConfig, FluidSystem};
use vxl_phys_gpu::grid::{grid_on_adapter, GridInputs, GridParams};

const N: usize = 16;
const SPACING: f32 = 0.05;
const RUNS: usize = 6;

/// 场景：晶格 + 5 趟静置 + 剪切初速；`floor` 为真时再加一块地板（提供 2b 边界粒子）。
fn scene(floor: bool) -> FluidSystem {
    let cfg = FluidConfig::default();
    let h = cfg.smoothing_radius;
    let mut f = FluidSystem::new(
        cfg,
        Vec3::new(
            -(N as f32) * SPACING * 0.5,
            0.5,
            -(N as f32) * SPACING * 0.5,
        ),
        [N, N, N],
        SPACING,
    );
    for _ in 0..5 {
        f.step(1.0 / 60.0, &vxl_phys_core::interop::NoProviders);
    }
    let mut vs = f.velocities().to_vec();
    for (i, v) in vs.iter_mut().enumerate() {
        let p = f.positions()[i];
        v.x += 0.6 * (p.y * 12.0).sin();
        v.z += 0.4 * (p.y * 8.0).cos();
    }
    f.set_velocities(&vs);
    if floor {
        let half = N as f32 * SPACING * 0.5 + 4.0 * h;
        let bodies = vec![(
            0u32,
            Shape::Box {
                half: Vec3::new(half, 2.0 * SPACING, half),
            },
            BodyPose {
                pos: Vec3::new(0.0, -2.0 * h, 0.0),
                rot: Quat::IDENTITY,
                linvel: Vec3::ZERO,
                angvel: Vec3::ZERO,
            },
        )];
        assert!(f.set_boundary_particles(&bodies) > 0, "地板没造出边界粒子");
    }
    f
}

/// FNV-1a（对 u32 序列）——只比"两次是不是同一个表"。
fn fnv(xs: &[u32]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for &x in xs {
        h ^= x as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// 一条取法下的读数：`(items 哈希不同的遍数, overflow, 盒外粒子数)`。
fn probe(f: &FluidSystem, fresh_box: bool, tag: &str) -> (usize, u32, usize) {
    let (apos, _, _, _) = f.raw_particles();
    let np = apos.len();
    let mut pos_flat: Vec<f32> = Vec::with_capacity(np * 3);
    for p in apos {
        pos_flat.extend_from_slice(&[p.x, p.y, p.z]);
    }
    let gd = f.neighbor_grid();
    // 两种盒子取法：**借引擎那张表**（管线现行）/ **按当前全量位置新算**（覆盖所有粒子）。
    let (min, inv, dims) = if fresh_box {
        let (lo, hi) = apos
            .iter()
            .fold((apos[0], apos[0]), |(l, h), p| (l.min(*p), h.max(*p)));
        let (m, bin, d) = grid_box(lo, hi, f.config().smoothing_radius, GRID_MAX_BINS);
        (m, 1.0 / bin, d)
    } else {
        (gd.min, gd.inv, [gd.dims.0, gd.dims.1, gd.dims.2])
    };
    let (nx, ny, nz) = (dims[0], dims[1], dims[2]);
    let fl = |o: f32, v: f32| -> f32 { ((v - o) * inv).floor() };
    let oob = apos
        .iter()
        .filter(|p| {
            fl(min.x, p.x) < 0.0
                || fl(min.x, p.x) >= nx as f32
                || fl(min.y, p.y) < 0.0
                || fl(min.y, p.y) >= ny as f32
                || fl(min.z, p.z) < 0.0
                || fl(min.z, p.z) >= nz as f32
        })
        .count();
    let params = GridParams {
        gmin: [min.x, min.y, min.z],
        inv,
        nx,
        ny,
        nz,
        n: np as u32,
        total: nx * ny * nz,
        cap: 512,
        _pad0: 0,
        _pad1: 0,
    };
    let mut hs: Vec<u64> = Vec::new();
    let mut ovf = 0u32;
    for _ in 0..RUNS {
        let out = grid_on_adapter(
            0,
            &GridInputs {
                pos_flat: &pos_flat,
            },
            params,
            1,
        );
        assert!(out.error.is_none(), "格表探针出错：{:?}", out.error);
        ovf = ovf.max(out.overflow);
        hs.push(fnv(&out.items));
    }
    let bad = hs.iter().filter(|&&h| h != hs[0]).count();
    let cells = total_size(nx, ny, nz);
    println!(
        "  · [{tag}] items 哈希不同 {bad}/{RUNS}｜overflow {ovf}｜盒外粒子 {oob}｜格数 {cells}"
    );
    (bad, ovf, oob)
}

fn total_size(nx: u32, ny: u32, nz: u32) -> u32 {
    nx * ny * nz
}

#[test]
fn grid_table_box_source_decides_reproducibility() {
    if vxl_phys_gpu::probe::adapters().is_empty() {
        println!("（本机无可用适配器 ⇒ 跳过；与其它 GPU 探针同口径）");
        return;
    }
    println!("== 盒取法 vs 格表可复现性（每档同输入跑 {RUNS} 遍）==");
    // ① 纯流体：两种取法都应当干净（控制组）。
    let pf = scene(false);
    let (pb, po, pe) = probe(&pf, false, "纯流体·借引擎表");
    // ⚠️ 控制组只钉"可复现性"与"不爆格"：**借引擎那张表**本来就会有一批盒外粒子（那张表是
    // `set_boundary_particles`/最后一次 `step()` 之前建的 ⇒ 盒略小），实测 304 粒 —— **它无害**，
    // 只要不把格挤爆（`overflow == 0`）就仍然可复现。这正是"2b 为什么不同"的对照面。
    assert_eq!(
        (pb, po),
        (0, 0),
        "纯流体在借引擎表时也必须可复现且不爆格（实得 哈希不同 {pb}、overflow {po}、盒外 {pe}）"
    );
    assert!(
        pe > 0,
        "借引擎表这一档预期应当有盒外粒子（这是对照面的前提）"
    );
    drop(pf);
    // ② 2b：借引擎表 vs 新算覆盖盒。
    let bf = scene(true);
    let (b1, o1, e1) = probe(&bf, false, "2b·借引擎表（管线现行）");
    let (b2, o2, e2) = probe(&bf, true, "2b·新算覆盖盒");
    println!(
        "⇒ 判读：借引擎表那档 {}；新算覆盖盒那档 {}",
        if b1 == 0 {
            "可复现"
        } else {
            "**不可复现**"
        },
        if b2 == 0 {
            "可复现 ⇒ 根因 = **盒子没覆盖粒子**（修法：盒按全量位置算）"
        } else {
            "仍不可复现 ⇒ 另有原因（继续查）"
        }
    );
    println!("  （2b 借表档 overflow={o1} 盒外={e1}；新盒档 overflow={o2} 盒外={e2}）");
}
