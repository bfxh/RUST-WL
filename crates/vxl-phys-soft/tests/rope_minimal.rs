//! **绳索最小闭环的判据**（T1，`docs/SURVEY-SOFT-CLOTH-AND-CONVERSION.md` §T1）：
//! 把"XPBD 核心循环（子步 × 约束投影 × 拉格朗日乘子）+ 点-形状接触"钉成可复跑的读数。
//!
//! 四条判据，全部**机器无关**（无计时、无随机、纯 CPU ⇒ CI 会真跑）：
//! ① **悬垂形状**：两端钉住的松绳稳定后，垂度对**同长度解析悬链线**的值；
//! ② **不可伸长**（α = 0）：段长剩余拉伸有界 **且** 子步翻倍至少降 3×（二阶收敛）；
//! ③ **落在真实三角网地形上**：绳落到网格地板上后**每个粒子都就位于半径高度**且已静止
//!    —— 这条走引擎真正那条提供者通道（`TriMesh` 的 `ProviderColliders`），不是自造 collider；
//! ④ **确定性**：同构造两次跑末态逐位相同 + 末态哈希**冻结基线**。
//!
//! 边界：本片不含摩擦/自碰撞/刚体耦合（见 `rope.rs` 头的"本片不做"）。
//!
//! ⚠️ **换代级**：两条哈希是冻结值 ⇒ 改绳索数值/接触口径必须重冻并在此登记
//! （同 `default_tier_stability` 的惯例）。

use vxl_phys_core::{interop::NoProviders, Vec3};
use vxl_phys_soft::Rope;
use vxl_phys_terrain::mesh::TriMesh;

const DT: f32 = 1.0 / 60.0;
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// 平地板（y = 0）：8×8 格、±4 m（与门面里那条提供者判据同几何）。
fn flat_mesh() -> TriMesh {
    const N: usize = 8;
    const S: f32 = 4.0;
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for iz in 0..=N {
        for ix in 0..=N {
            let x = -S + (2.0 * S * ix as f32) / N as f32;
            let z = -S + (2.0 * S * iz as f32) / N as f32;
            verts.push(Vec3::new(x, 0.0, z));
        }
    }
    for iz in 0..N as u32 {
        for ix in 0..N as u32 {
            let a = iz * (N as u32 + 1) + ix;
            let c = a + 1;
            let d = a + N as u32 + 1;
            let e = d + 1;
            tris.push([a, d, c]);
            tris.push([c, d, e]);
        }
    }
    TriMesh::new(verts, tris)
}

/// **解析悬链线垂度**：绳长 `len`、水平跨距 `span`、两端等高 ⇒ 解 `len = 2a·sinh(span/2a)` 的 `a`，
/// 垂度 `= a·cosh(span/2a) − a`。（左端随 `a` 单调递减：`a→∞` 时 `len→span`、`a→0` 时 `len→∞` ⇒ 二分。）
fn catenary_sag(len: f32, span: f32) -> f32 {
    let (mut lo, mut hi) = (1e-3f32, 1e3f32);
    for _ in 0..80 {
        let a = 0.5 * (lo + hi);
        if 2.0 * a * (span / (2.0 * a)).sinh() > len {
            lo = a;
        } else {
            hi = a;
        }
    }
    let a = 0.5 * (lo + hi);
    a * (span / (2.0 * a)).cosh() - a
}

/// 全程最大的**段长相对误差**（不可伸长性读数）。
fn worst_len_err(r: &Rope) -> f32 {
    (0..r.nodes() - 1)
        .map(|k| (r.segment_len(k) - r.rest_len).abs() / r.rest_len)
        .fold(0.0f32, f32::max)
}

/// 最大速度（稳态读数）。
fn max_speed(r: &Rope) -> f32 {
    r.vel.iter().map(|v| v.length()).fold(0.0f32, f32::max)
}

/// 末态位置的 **FNV-1a**（按 f32 位模式逐字节）：确定性判据（不引外部依赖）。
fn state_hash(r: &Rope) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for p in &r.pos {
        for v in [p.x, p.y, p.z] {
            for b in v.to_bits().to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

/// 两端钉住的松绳：读垂度 + 不可伸长 + 收敛阶 + 确定性，四条一起判。
///
/// **为什么"不可伸长"判的是残差而不是 0**：`α = 0` 只表示"约束本身无柔度"，而 XPBD 每子步只做
/// **1 次 Gauss-Seidel 投影**（Small Steps 口径）⇒ 先投影过的约束会被后面的再次带偏，残差随**张力**
/// 上升。实测（本仓，绳长 1.4 / 跨距 1.0 / 33 节点）：子步 4 / 8 / 16 / 32 / 64 ⇒ 段长最大相对误差
/// **1.154e-1 / 2.944e-2 / 7.094e-3 / 1.269e-3 / 2.830e-4** —— 每翻一倍降 ≈4×。所以判据写成
/// **机制级**：残差有界 **且** 子步翻倍至少降 3×（比"冻一个数"更能抓住回归）。
///
/// **垂度对的是"同长度悬链线"**（把**实测有效长** `Σ|段|` 代入解析式）⇒ 把上面那份拉伸残差从形状
/// 判据里剔掉，只剩离散/形状误差：实测 8 子步差 **0.081%**、4 子步 0.148% ⇒ 这条链**就是**悬链线
/// （判据给 0.5%，留 6× 余量）。
#[test]
fn rope_sags_to_catenary_stays_inextensible_and_deterministic() {
    const NODES: usize = 33;
    const SPAN: f32 = 1.0;
    const LEN: f32 = 1.4;
    let a = Vec3::new(-0.5 * SPAN, 1.0, 0.0);
    let b = Vec3::new(0.5 * SPAN, 1.0, 0.0);

    // 阻尼只为把"悬垂形状"做成**稳态读数**（否则绳永远在摆 ⇒ 读数只能取窗口均值）。
    let run = |substeps: u32| {
        let mut r = Rope::span(a, b, NODES, LEN, 0.0);
        r.substeps = substeps;
        r.damping = 0.999;
        let mut worst = 0.0f32;
        for t in 0..6000 {
            r.step(DT, GRAVITY, &NoProviders, &[]);
            if t >= 100 {
                worst = worst.max(worst_len_err(&r));
            }
        }
        (r, worst)
    };
    let (r8, worst8) = run(8);
    let (_, worst16) = run(16);
    let (r8_again, _) = run(8);

    let eff8: f32 = (0..NODES - 1).map(|k| r8.segment_len(k)).sum();
    let sag8 = a.y - r8.pos[NODES / 2].y;
    let sag_ref = catenary_sag(eff8, SPAN);
    let shape_err = (sag8 - sag_ref).abs() / sag_ref;
    println!(
        "垂度 实测={sag8:.6} 同长悬链线={sag_ref:.6}（差 {:.4}%）| 有效长={eff8:.6}（静止长 {LEN}）| 段长残差 8子步={worst8:.3e} 16子步={worst16:.3e}（降 {:.2}×）| 末速max={:.3e} | 哈希={:016x}",
        shape_err * 100.0,
        worst8 / worst16,
        max_speed(&r8),
        state_hash(&r8)
    );

    assert!(
        shape_err < 5e-3,
        "悬垂形状该是**同长度悬链线**（实测差 {:.4}%）——大了说明形状不对，不是拉伸的问题",
        shape_err * 100.0
    );
    assert!(
        worst8 < 4e-2,
        "8 子步下段长最大相对误差该有界（实测 {worst8:.3e}）——大了说明约束投影没生效"
    );
    assert!(
        worst8 / worst16 >= 3.0,
        "子步翻倍 ⇒ 段长残差至少降 3×（实测 {:.2}×）——降不动说明二阶收敛没了（子步没真切、或 λ 没归零）",
        worst8 / worst16
    );
    assert!(
        max_speed(&r8) < 1e-3,
        "松绳该已跑稳（末速 {:.3e}）——没稳说明阻尼/子步不足，垂度读数不可用",
        max_speed(&r8)
    );
    assert_eq!(
        state_hash(&r8),
        state_hash(&r8_again),
        "同构造两次跑末态必须逐位相同（绳索是顺序量，任何并行/未初始化都会在这里露出来）"
    );
    assert_eq!(
        state_hash(&r8),
        0xfaf4_d9e7_443d_dbef,
        "末态哈希是**冻结基线**（换代级：改绳索数值必须重冻并登记）——变了说明物理被改动了"
    );
}

/// 自由下落的绳落到**真实三角网地板**上：每个粒子就位于半径高度、且已静止。
///
/// 这条走的是引擎真正那条**提供者通道**（`TriMesh` 的 `ProviderColliders`）——不是自造 collider。
#[test]
fn rope_rests_on_real_trimesh_provider() {
    const NODES: usize = 33;
    const RADIUS: f32 = 0.02;
    let mesh = flat_mesh();
    let mut r = Rope::line(
        Vec3::new(-0.4, 1.0, 0.0),
        Vec3::new(0.4, 1.0, 0.0),
        NODES,
        RADIUS,
    );
    // 两端也松开 ⇒ 整条绳落到地板上（这一条判的是**接触就位**，不是悬挂）。
    r.set_pinned(0, false);
    r.set_pinned(NODES - 1, false);
    r.damping = 0.999;
    for _ in 0..4000 {
        r.step(DT, GRAVITY, &mesh, &[0]);
    }

    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for i in 0..r.nodes() {
        let clearance = r.pos[i].y - RADIUS;
        lo = lo.min(clearance);
        hi = hi.max(clearance);
    }
    println!(
        "就位间隙 最低={lo:.5} 最高={hi:.5} | 末速max={:.3e} | 哈希={:016x}",
        max_speed(&r),
        state_hash(&r)
    );

    assert!(
        lo > -0.01,
        "没有粒子该陷进地形（最低间隙 {lo:.5}）——红了说明点-形状接触没接上或推得不够"
    );
    assert!(
        hi < 0.05,
        "绳该整体贴在地板上（最高间隙 {hi:.5}）——大了说明还有粒子悬在空中"
    );
    assert!(
        max_speed(&r) < 1e-2,
        "绳该已静止（末速 {:.3e}）",
        max_speed(&r)
    );
    assert_eq!(
        state_hash(&r),
        0xdf50_e584_d880_fb9f,
        "末态哈希是**冻结基线**（换代级：改接触口径必须重冻并登记）"
    );
}
