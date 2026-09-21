//! T5 金样对拍（提前做）：同场景双引擎行为对照——首要问题：25 层留缝塔的
//! 坍塌是物理还是数值？（Rapier 0.35 默认 = TGS-Soft 软接触 4 迭代）。
//!
//! 用法：cargo run --release -- [scene] [ticks] [vxl_iters] [skin] [inner] [maxcorr]
//!        [freq] [substeps] [dump.bin] [stabilization]
//!   dump.bin（可选）：逐帧（每 2 tick）写两侧引擎位姿 + 各自 step 墙钟；`-` ＝ 不转储
//!   stabilization：无偏置末趟迭代数（0 = 关闭，默认；Rapier TGS-Soft 末趟同义）
//!   ⇒ `scripts/render_compare.py` 生成同屏对照 GIF（docs/demo/compare_full.gif）。
//!   「活跃 tick 口径」：入睡后 step 近似空转 ⇒ 全期均值会被稀释（教训：首版得出
//!   rapier 0.01 ms/tick 的假数据），故同时报「活跃 tick 均值」。
//!   scene: tower25 = 25 层 × 10×10 | pile5 = 5 层 × 20×20 | col45 = 5 层 × 3×3
//!   （均：盒 0.5、密度 1000、μ 0.5、e 0、缝 2cm、盒地板）
//! 输出：逐 50 tick 双引擎汇总（|v|max / KE / 入睡数 / 最深穿透 / 高度带）
//!   + 参照体检查点位姿表（容差表雏形：末态 max |Δpos|）。

use rapier3d::prelude::*;
use vxl_phys::{FrictionModel, Material, PhysConfig, Quat, Shape, Vec3, World};

struct Scene {
    layers: usize,
    side: usize,
    spacing: f32,
}

fn scene_of(name: &str) -> Scene {
    match name {
        "pile5" => Scene {
            layers: 5,
            side: 20,
            spacing: 0.52,
        },
        "col45" => Scene {
            layers: 5,
            side: 3,
            spacing: 0.52,
        },
        _ => Scene {
            layers: 25,
            side: 10,
            spacing: 0.52,
        },
    }
}

fn spawn_positions(s: &Scene) -> Vec<Vec3> {
    let off = (s.side as f32 - 1.0) * 0.5 * s.spacing;
    let mut out = Vec::new();
    for layer in 0..s.layers {
        let y = 0.25 + layer as f32 * s.spacing;
        for row in 0..s.side {
            for col in 0..s.side {
                out.push(Vec3::new(
                    col as f32 * s.spacing - off,
                    y,
                    row as f32 * s.spacing - off,
                ));
            }
        }
    }
    out
}

/// 建 vxl 侧世界。8 个参数＝一条配方（与 `RECIPES.md` 的位置参数一一对应）⇒ 显式豁免
/// `too_many_arguments`（与主仓 `solve_constraint` 同先例）；金样门要求本 workspace clippy 全绿。
#[allow(clippy::too_many_arguments)]
fn build_vxl(
    s: &Scene,
    iters: u32,
    skin: f32,
    inner: u32,
    maxcorr: f32,
    freq: f32,
    substeps: u32,
    stabilization: u32,
) -> (World, Vec<usize>) {
    let cfg = PhysConfig {
        velocity_iterations: iters,
        threads: 8,
        contact_skin: skin,
        normal_inner: inner.max(1),
        max_corrective_velocity: maxcorr,
        contact_freq_hz: freq,
        substeps: substeps.max(1),
        stabilization_iterations: stabilization,
        ..PhysConfig::default()
    };
    let mut w = World::new(cfg);
    w.add_static(
        Shape::Box {
            half: Vec3::new(20.0, 0.5, 20.0),
        },
        Vec3::new(0.0, -0.5, 0.0),
        Quat::IDENTITY,
    );
    for p in spawn_positions(s) {
        w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.25),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
    }
    // 摩擦系数可用环境变量 `VXL_MU` 覆盖（默认 0.5 = 引擎默认）：睡眠/角向机制实验的
    // **剂量-响应**用（见 `OPEN-PROBLEMS.md` P1 的角向线索）。
    let mu = std::env::var("VXL_MU")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(0.5);
    if (mu - 0.5).abs() > 1e-6 {
        let m = w.add_material(Material {
            friction: FrictionModel::Coulomb { mu },
            restitution: 0.0,
        });
        for b in 0..w.bodies.len() {
            w.bodies.set_material(b, m);
        }
    }
    let ids: Vec<usize> = (0..w.bodies.len())
        .filter(|&i| w.bodies.is_dynamic(i))
        .collect();
    (w, ids)
}

fn build_rapier(s: &Scene) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
    let mut w = PhysicsWorld::new();
    // 地板：与 vxl 同几何（半 20×0.5×20，顶面 y=0）。
    let ground = w.bodies.insert(
        RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, -0.5, 0.0))
            .build(),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0).build(),
        ground,
        &mut w.bodies,
    );
    let mut handles = Vec::new();
    for p in spawn_positions(s) {
        let body = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(p.x, p.y, p.z))
                .build(),
        );
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.25, 0.25, 0.25)
                .density(1000.0)
                .friction(0.5)
                .restitution(0.0)
                .build(),
            body,
            &mut w.bodies,
        );
        handles.push(body);
    }
    (w, handles)
}

struct Sum {
    vmax: f32,
    ke: f32,
    sleeping: usize,
    deep: f32,
    ymin: f32,
    ymax: f32,
}

fn summary_vxl(w: &World, ids: &[usize]) -> Sum {
    let (mut vmax, mut ke, mut sleeping) = (0.0f32, 0.0f32, 0usize);
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for &i in ids {
        let m = 1.0 / w.bodies.inv_mass[i];
        let v = w.bodies.linvel[i];
        vmax = vmax.max(v.length());
        ke += 0.5 * m * v.length_squared();
        if !w.bodies.awake[i] {
            sleeping += 1;
        }
        let y = w.bodies.position[i].y;
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    let mut deep = 0.0f32;
    for m in w.manifolds() {
        for p in &m.points {
            deep = deep.max(p.depth);
        }
    }
    Sum {
        vmax,
        ke,
        sleeping,
        deep,
        ymin,
        ymax,
    }
}

/// 超阈分解（仅线性 / 仅角速 / 双超 / 总数；阈值 4 cm/s、0.05 rad/s）。
///
/// **为什么要按 50 tick 采样**（2026-09-20 教训）：塔的 KE 在 tick ~150 后就进入
/// 平台（均值 102.5 J、σ 9.2 J ⇒ 自然波动带 ±18%），"超阈体数"同属一个平台；
/// 只在**末态**打一次，会把波动读成趋势——档里"300→600 不降反增 ⇒ 稳态增长"
/// 与复测 1200 tick 的"181 ⇒ 衰减"都是这么读出来的。趋势与波动只能靠时程分。
fn over_threshold(w: &World, ids: &[usize]) -> (usize, usize, usize, usize) {
    let (mut only_lin, mut only_ang, mut both) = (0usize, 0usize, 0usize);
    for &i in ids {
        let l = w.bodies.linvel[i].length() >= 0.04;
        let a = w.bodies.angvel(i).length() >= 0.05;
        match (l, a) {
            (true, true) => both += 1,
            (true, false) => only_lin += 1,
            (false, true) => only_ang += 1,
            (false, false) => {}
        }
    }
    (only_lin, only_ang, both, ids.len())
}

fn summary_rapier(w: &PhysicsWorld, hs: &[RigidBodyHandle]) -> Sum {
    let (mut vmax, mut ke, mut sleeping) = (0.0f32, 0.0f32, 0usize);
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for &h in hs {
        let b = w.bodies.get(h).expect("body");
        let v = b.linvel();
        let m = b.mass();
        vmax = vmax.max(v.length());
        ke += 0.5 * m * v.length_squared();
        if b.is_sleeping() {
            sleeping += 1;
        }
        let y = b.translation().y;
        ymin = ymin.min(y);
        ymax = ymax.max(y);
    }
    let mut deep = 0.0f32;
    for pair in w.narrow_phase.contact_pairs() {
        for man in &pair.manifolds {
            for p in &man.points {
                deep = deep.max(-p.dist);
            }
        }
    }
    Sum {
        vmax,
        ke,
        sleeping,
        deep,
        ymin,
        ymax,
    }
}

/// 参照体：按索引均匀取 16 个（跨层/跨位置采样）。
fn ref_indices(n: usize) -> Vec<usize> {
    let step = (n / 16).max(1);
    (0..n).step_by(step).take(16).collect()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let scene_name = args.next().unwrap_or_else(|| "tower25".into());
    let ticks: usize = args.next().and_then(|s| s.parse().ok()).unwrap_or(600);
    let vxl_iters: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16);
    let skin: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.01);
    let inner: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(4);
    let maxcorr: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(3.0);
    let freq: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(30.0);
    let substeps: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1);
    // 可选：逐帧转储路径（同屏可视化对比用）。**保持在原位**——INDEX/RECIPES 里的
    // render_compare 配方按第 9 个位置参数传它；传 `-` ＝ 不转储（让后面的实验旋钮
    // 可用而不必编造文件名）。
    let dump_path = args.next().filter(|p| p != "-");
    // 其后的实验旋钮（都排在 dump 之后，避免破坏既有配方）：
    let stabilization: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0);
    let s = scene_of(&scene_name);

    let (mut vw, vids) = build_vxl(
        &s,
        vxl_iters,
        skin,
        inner,
        maxcorr,
        freq,
        substeps,
        stabilization,
    );
    let (mut rw, rhs) = build_rapier(&s);
    let refs = ref_indices(vids.len());
    println!(
        "场景 {scene_name}：{} 层 × {}×{} = {} 盒 | vxl {vxl_iters}/{inner}/{substeps}（iters/inner/substeps；skin {skin}、maxcorr {maxcorr}、freq {freq}）| rapier 默认（TGS-Soft 4 迭代 + 软接触）| {ticks} tick",
        s.layers,
        s.side,
        s.side,
        vids.len()
    );
    println!("tick | 引擎 | |v|max | KE(J) | 入睡 | 最深 | y 带 | 超阈(仅 vxl：线/角/双)");
    // 转储头：magic + 每帧 tick 数 + 盒半长（场景统一 0.5）
    let mut dump: Option<std::io::BufWriter<std::fs::File>> = match &dump_path {
        Some(dp) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(dp).expect("dump 打开失败"));
            use std::io::Write as _;
            f.write_all(b"VXLC").unwrap();
            f.write_all(&2u32.to_le_bytes()).unwrap(); // 每帧 2 tick
            f.write_all(&0.5f32.to_le_bytes()).unwrap(); // 半长
            Some(f)
        }
        None => None,
    };
    // SPEC §3 稳定性判据实测：「休眠体被重复唤醒 < 1 次/秒/体」——统计每体的
    // asleep→awake 反转次数（本跑 ticks tick ⇒ 合规线 = ticks/60 次/体）。
    let mut prev_awake: Vec<bool> = vids.iter().map(|&i| vw.bodies.awake[i]).collect();
    let mut flips: Vec<u32> = vec![0; vids.len()];
    // 速度对比：分别累计两个引擎的 step 墙钟（预热 20 tick 后再计时）。
    let mut vxl_ns: u128 = 0;
    let mut rapier_ns: u128 = 0;
    let mut vxl_active: usize = 0;
    let mut rapier_active: usize = 0;
    for t in 1..=ticks {
        let t0 = std::time::Instant::now();
        vw.step();
        let dt_vxl = t0.elapsed().as_nanos();
        let t1 = std::time::Instant::now();
        rw.step();
        let dt_rap = t1.elapsed().as_nanos();
        vxl_ns += dt_vxl;
        rapier_ns += dt_rap;
        // 「活跃 tick」= 至少一个体清醒（入睡后 step 近乎空转 ⇒ 只有活跃期可比）
        if vids.iter().any(|&i| vw.bodies.awake[i]) {
            vxl_active += 1;
        }
        if rhs.iter().any(|h| !rw.bodies[*h].is_sleeping()) {
            rapier_active += 1;
        }
        for (k, &i) in vids.iter().enumerate() {
            if !prev_awake[k] && vw.bodies.awake[i] {
                flips[k] += 1;
            }
            prev_awake[k] = vw.bodies.awake[i];
        }
        // 逐帧转储（每 2 tick 一帧；两侧引擎位姿 + 各自 step 墙钟）
        if dump_path.is_some() && t % 2 == 0 {
            {
                if let Some(f) = dump.as_mut() {
                    use std::io::Write as _;
                    f.write_all(&(t as u32).to_le_bytes()).unwrap();
                    f.write_all(&((dt_vxl as f32) / 1e6).to_le_bytes()).unwrap();
                    f.write_all(&((dt_rap as f32) / 1e6).to_le_bytes()).unwrap();
                    let n = vids.len() as u32;
                    f.write_all(&n.to_le_bytes()).unwrap();
                    for &i in &vids {
                        let p = vw.bodies.position[i];
                        let q = vw.bodies.rot(i);
                        for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w] {
                            f.write_all(&v.to_le_bytes()).unwrap();
                        }
                    }
                    for &h in &rhs {
                        let b = rw.bodies.get(h).expect("body");
                        let p = b.translation();
                        let q = b.rotation();
                        for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w] {
                            f.write_all(&v.to_le_bytes()).unwrap();
                        }
                    }
                }
            }
        }
        if t % 50 == 0 || t == ticks {
            let a = summary_vxl(&vw, &vids);
            let b = summary_rapier(&rw, &rhs);
            // 超阈分解随行打印 ⇒ 趋势/波动一眼可分（见 `over_threshold` 注）。
            let (ol, oa, ob, _) = over_threshold(&vw, &vids);
            println!(
                "{t:4} | vxl    | {:7.3} | {:9.1} | {:5} | {:5.3} | [{:.2},{:.2}] | 超阈 线{ol} 角{oa} 双{ob}",
                a.vmax, a.ke, a.sleeping, a.deep, a.ymin, a.ymax
            );
            println!(
                "{t:4} | rapier | {:7.3} | {:9.1} | {:5} | {:5.3} | [{:.2},{:.2}]",
                b.vmax, b.ke, b.sleeping, b.deep, b.ymin, b.ymax
            );
        }
    }
    {
        // 流形特征稳定性（EXPERIMENTS 末节 K）：精确特征命中率 = warm 配对跨帧是否稳定。
        // 用途：判定 §9/§10 的"裁剪产物 ⇒ 配对漂移"归因是否成立（85%+ ⇒ 不成立）。
        let (we, wf, wm) = vxl_phys_solver::warm_match_stats_take();
        let wt = (we + wf + wm).max(1) as f64;
        println!(
            "== warm 匹配分支（全程累计）：精确特征 {}（{:.1}%） 近邻回退 {}（{:.1}%） 未匹配 {}（{:.1}%）",
            we,
            100.0 * we as f64 / wt,
            wf,
            100.0 * wf as f64 / wt,
            wm,
            100.0 * wm as f64 / wt,
        );
        // 睡眠诊断（只计数）：分辨"岛为什么没睡"——有人快（`all_slow=false`）
        // vs `all_slow=true` 但 `min_timer` 攒不满 `sleep_time`（成员 churn 拖低）。
        let (s_fast, s_wait, s_slept, s_wait_max) = vxl_phys_solver::sleep_diag_take();
        println!(
            "== 睡眠诊断（全程累计）：有人快而拒 {s_fast} | 全慢但未满 {s_wait} | 入睡 {s_slept} | 等待中 min_timer 峰值 {s_wait_max} ms"
        );
        // 深档（默认关）：逐体超阈的**比例**——判定"逐体判据（路 A）的前提是否成立"。
        let (d_fast, d_all) = vxl_phys_solver::sleep_diag_deep_take();
        if d_all > 0 {
            println!(
                "== 睡眠诊断（深档）：逐体超阈 {d_fast}/{d_all} = {:.2}%",
                100.0 * d_fast as f64 / d_all as f64
            );
        }
        let (bs, bc, bh, bsame) = vxl_phys_solver::warm_fallback_kind_take();
        let bt = (bs + bc + bh + bsame).max(1) as f64;
        println!(
            "== 回退命中的成因（占回退）：侧别翻转 {}（{:.1}%） 裁剪路变 {}（{:.1}%） 哈希变 {}（{:.1}%） 特征同 {}（{:.1}%）",
            bs,
            100.0 * bs as f64 / bt,
            bc,
            100.0 * bc as f64 / bt,
            bh,
            100.0 * bh as f64 / bt,
            bsame,
            100.0 * bsame as f64 / bt,
        );
        let (nf, ns) = vxl_phys_solver::warm_normal_flip_take();
        let nt = (nf + ns).max(1) as f64;
        println!(
            "== 参考面代理（占回退命中）：接触法向变化 {}（{:.1}%） 法向不变 {}（{:.1}%）——变化⇒换参考面，不变⇒同面换裁剪侧平面",
            nf,
            100.0 * nf as f64 / nt,
            ns,
            100.0 * ns as f64 / nt,
        );
        let (pts, sep, spec, bias) = vxl_phys_solver::solve_accounting_take();
        println!(
            "== 解算记账（累计）：点 {:.0} 万，其中分离点 {:.1}%（吃 spec），spec 和 {:.0}、bias 和 {:.0}（×1000，均值 {:.2}/{:.2}）",
            pts as f64 / 1e4,
            100.0 * sep as f64 / pts.max(1) as f64,
            spec as f64 / 1e3,
            bias as f64 / 1e3,
            spec as f64 / 1e3 / pts.max(1) as f64,
            bias as f64 / 1e3 / pts.max(1) as f64,
        );
    }
    {
        let worst = flips.iter().copied().max().unwrap_or(0);
        let flippers = flips.iter().filter(|&&f| f > 0).count();
        println!(
            "== 重复唤醒（SPEC §3 判据：< 1 次/秒/体；本跑 {ticks} tick ≈ {:.1}s ⇒ 合规线 {:.1} 次）：\
             最差体 {worst} 次 | 有过唤醒的体 {flippers}/{} | {}",
            ticks as f64 / 60.0,
            ticks as f64 / 60.0,
            vids.len(),
            if (worst as f64) < ticks as f64 / 60.0 {
                "✅ 合规"
            } else {
                "❌ 超线"
            }
        );
    }
    // 末态参照体位姿对照（发散是预期：双求解器数值路径不同；看量级）。
    // **分布口径**（2026-09-21）：`max` 是最大范数、由**单个体**决定（离群脆弱——`|v|max`
    // 已吃过同类教训：子步加到 32 时它反飙而超阈体数继续降）⇒ 同报 `p95/mean`，
    // 用于判别"整体系统性偏移"还是"一个离群体"。
    let mut max_d = 0.0f32;
    let mut ds: Vec<f32> = Vec::new();
    println!("== 末态参照体位姿（vxl vs rapier）：");
    for &k in &refs {
        let i = vids[k];
        let vp = vw.bodies.position[i];
        let rp = rw.bodies.get(rhs[k]).expect("body").translation();
        let d = (vp - Vec3::new(rp.x, rp.y, rp.z)).length();
        max_d = max_d.max(d);
        ds.push(d);
        println!(
            "  体{k:5} vxl({:6.3},{:6.3},{:6.3}) rapier({:6.3},{:6.3},{:6.3}) |Δ| {d:.4}",
            vp.x, vp.y, vp.z, rp.x, rp.y, rp.z
        );
    }
    {
        if let Some(f) = dump.as_mut() {
            use std::io::Write as _;
            f.flush().unwrap();
        }
        let timed = ticks as f64;
        let v_ms = vxl_ns as f64 / timed / 1e6;
        let r_ms = rapier_ns as f64 / timed / 1e6;
        let v_act = vxl_active.max(1) as f64;
        let r_act = rapier_active.max(1) as f64;
        let v_ms_act = vxl_ns as f64 / v_act / 1e6;
        let r_ms_act = rapier_ns as f64 / r_act / 1e6;
        let l1 = format!(
            "   vxl-phys  全期 {:.2} ms/tick（{:.0} FPS） | 活跃 {}/{} tick ⇒ {:.2} ms/活跃tick（{:.0} FPS）",
            v_ms, 1000.0 / v_ms, vxl_active, ticks, v_ms_act, 1000.0 / v_ms_act
        );
        let l2 = format!(
            "   rapier    全期 {:.2} ms/tick（{:.0} FPS） | 活跃 {}/{} tick ⇒ {:.2} ms/活跃tick（{:.0} FPS）",
            r_ms, 1000.0 / r_ms, rapier_active, ticks, r_ms_act, 1000.0 / r_ms_act
        );
        println!("{l1}");
        println!("{l2}");
        println!(
            "   活跃期相对代价 vxl/rapier = {:.2}× | 入睡面：vxl 全程清醒 {} 体，rapier 第 ~50 tick 全体入睡",
            v_ms_act / r_ms_act,
            vids.len()
        );
    }
    ds.sort_by(f32::total_cmp);
    let mean_d = if ds.is_empty() {
        0.0
    } else {
        ds.iter().sum::<f32>() / ds.len() as f32
    };
    let p95_d = if ds.is_empty() {
        0.0
    } else {
        ds[((ds.len() as f32 * 0.95) as usize).min(ds.len() - 1)]
    };
    println!(
        "== 末态 |Δpos|（参照体，n={}）：max {max_d:.4} | p95 {p95_d:.4} | mean {mean_d:.4} m",
        ds.len()
    );
    // 嗡振画像：超阈分解（线性/角速分别）+ 最活跃体明细（含层高）。
    // 计数复用 `over_threshold`（与 50 tick 随行打印**同源** ⇒ 两处不会漂开）。
    let (only_lin, only_ang, both, ntot) = over_threshold(&vw, &vids);
    let clean = ntot - only_lin - only_ang - both;
    let mut top: Vec<(usize, f32, f32, f32)> = Vec::new();
    for &i in &vids {
        let v = vw.bodies.linvel[i].length();
        let w = vw.bodies.angvel(i).length();
        top.push((i, v, w, vw.bodies.position[i].y));
    }
    println!(
        "== 阈值分解：仅线性超阈 {only_lin} | 仅角速超阈 {only_ang} | 双超 {both} | 阈值下 {clean}（共 {}）",
        vids.len()
    );
    top.sort_by(|a, b| b.1.total_cmp(&a.1));
    println!("== |v| 前 8（体, |v|, |ω|, y）：");
    for &(i, v, w, y) in top.iter().take(8) {
        println!("   #{i} |v| {v:.3} |w| {w:.3} y {y:.3}");
    }
    top.sort_by(|a, b| b.2.total_cmp(&a.2));
    println!("== |ω| 前 8（体, |v|, |ω|, y）：");
    for &(i, v, w, y) in top.iter().take(8) {
        println!("   #{i} |v| {v:.3} |w| {w:.3} y {y:.3}");
    }

    // ── **冻结基线自检**（门禁用；只在与基线**完全同配方**时判定）────────────────────
    //
    // 为什么需要它：金样读数此前**只写在 `OPEN-PROBLEMS.md` 的 T5 行里**、没有任何门看着
    // ⇒ 那行曾陈旧到与实际差一倍（入睡写 35/45，实测 45/45）都没人发现（2026-09-21 复测）。
    // 基线可信度：全部是**确定性读数**——塔的 Δpos 已在 600/1200/2400/4800 tick 四点同值
    // （0.0950 / 0.0572）核过；入睡与超阈是计数。
    // 配方与基线见 `docs/RECIPES.md` §金样门；换基线须按 ADR-0004 记换代理由。
    if let Some(fz) = frozen_baseline(scene_name.as_str(), ticks, vxl_iters, skin, inner, substeps)
    {
        let a = summary_vxl(&vw, &vids);
        let (ol, oa, ob, _) = over_threshold(&vw, &vids);
        let mut bad: Vec<String> = Vec::new();
        if a.sleeping != fz.sleeping {
            bad.push(format!("入睡 {} ≠ 基线 {}", a.sleeping, fz.sleeping));
        }
        if (ol, oa, ob) != (fz.ol, fz.oa, fz.ob) {
            bad.push(format!(
                "超阈 {ol}/{oa}/{ob} ≠ 基线 {}/{}/{}",
                fz.ol, fz.oa, fz.ob
            ));
        }
        if (max_d - fz.max_dpos).abs() > 1e-4 {
            bad.push(format!("Δpos max {max_d:.4} ≠ 基线 {:.4}", fz.max_dpos));
        }
        if (mean_d - fz.mean_dpos).abs() > 1e-4 {
            bad.push(format!("Δpos mean {mean_d:.4} ≠ 基线 {:.4}", fz.mean_dpos));
        }
        if bad.is_empty() {
            println!(
                "✅ 金样基线 PASS（{scene_name} / {ticks} tick）：入睡 {}/{}, 超阈 {ol}/{oa}/{ob}, Δpos max {max_d:.4} / mean {mean_d:.4}",
                a.sleeping,
                vids.len()
            );
        } else {
            println!(
                "❌ 金样基线 FAIL（{scene_name} / {ticks} tick）：{}",
                bad.join("；")
            );
            std::process::exit(1);
        }
    } else {
        println!("（本配方无冻结基线 ⇒ 跳过基线判定；基线配方见 `docs/RECIPES.md`）");
    }
}

/// 冻结基线（配方 → 读数）：四项都要逐计数/逐值相符，`Δpos` 用 1e-4 容差。
struct Frozen {
    sleeping: usize,
    ol: usize,
    oa: usize,
    ob: usize,
    max_dpos: f32,
    mean_dpos: f32,
}

/// 只认**文档里那条门禁配方**（`docs/RECIPES.md` §金样门）；其余配方返回 `None` ⇒ 跳过判定
/// （保证本二进制仍可自由做实验：改任一旋钮即不再与基线比较）。
#[allow(clippy::too_many_arguments)]
fn frozen_baseline(
    scene: &str,
    ticks: usize,
    iters: u32,
    skin: f32,
    inner: u32,
    substeps: u32,
) -> Option<Frozen> {
    let same = |t: usize, i: u32, s: f32, inn: u32, sub: u32| {
        ticks == t && iters == i && (skin - s).abs() < 1e-9 && inner == inn && substeps == sub
    };
    match scene {
        // col45 / pile5：`600 16 0.01 4 3.0 30 4`
        "col45" if same(600, 16, 0.01, 4, 4) => Some(Frozen {
            sleeping: 45,
            ol: 0,
            oa: 0,
            ob: 0,
            max_dpos: 0.0034,
            mean_dpos: 0.0016,
        }),
        "pile5" if same(600, 16, 0.01, 4, 4) => Some(Frozen {
            sleeping: 2000,
            ol: 0,
            oa: 0,
            ob: 0,
            max_dpos: 0.0041,
            mean_dpos: 0.0020,
        }),
        // tower25：`2400 16 0.01 1 3.0 30 16`（塔要长跑才收敛）
        "tower25" if same(2400, 16, 0.01, 1, 16) => Some(Frozen {
            sleeping: 2396,
            ol: 0,
            oa: 1,
            ob: 0,
            max_dpos: 0.0950,
            mean_dpos: 0.0572,
        }),
        _ => None,
    }
}
