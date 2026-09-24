//! **刚体前缀的积分**上卡验收（`PLAN-gpu.md` §13.12）。
//!
//! 场景：两个动态体（盒 2 kg + 球 1 kg），每步吃**同一组**脚本化 `(F, τ)` + 重力 + 限速；
//! CPU 侧走 `Integrator::integrate_velocities` + `integrate_positions`，卡上跑 `BodyStage`
//! （`body_integrate.wgsl`：半隐式欧拉 + 世界系逆惯量 + 限速 + 四元数积分）。
//!
//! 判据（**口径 B**：浮点相位不逐位）：N 步后 `pos` / `linvel` / `angvel` 的偏差与**姿态夹角**。
//!
//! 运行：`cargo run --release -p vxl-phys-gpu --example gpu_body_integrate_probe -- [ticks] [--adapter K]`

use std::f32::consts::PI;

use vxl_phys::{Integrator, Quat, Shape, Vec3};
use vxl_phys_core::BodySet;
use vxl_phys_gpu::pipeline::{BodyStage, BodyState, StepCfg};

/// 造两个体：质量与惯量都由 `BodySet::push_dynamic` 按形状算好（与卡上无关，纯初值）。
fn make_bodies() -> BodySet {
    let mut b = BodySet::new();
    b.push_dynamic(
        Shape::Box {
            half: Vec3::new(0.25, 0.15, 0.35),
        },
        Vec3::new(0.0, 2.0, 0.0),
        Quat::from_axis_angle(Vec3::Z, 0.3),
        2.0,
    );
    b.push_dynamic(
        Shape::Sphere { radius: 0.2 },
        Vec3::new(1.0, 2.5, -0.5),
        Quat::from_axis_angle(Vec3::X, 1.1),
        1.0,
    );
    b
}

/// 从 `BodySet` 抽卡上状态（**力/力矩由调用方按脚本填**，这里给 0）。
fn states_of(b: &BodySet) -> Vec<BodyState> {
    (0..b.len())
        .map(|i| BodyState {
            pos: b.position[i],
            inv_mass: b.inv_mass[i],
            rot: b.rot(i),
            linvel: b.linvel[i],
            angvel: b.angvel(i),
            loc_inv_i: b.local_inv_inertia[i],
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
        })
        .collect()
}

/// 脚本力/力矩（确定性；两条链吃同一组）。
fn script(t: f32) -> (Vec3, Vec3) {
    let s = (2.0 * PI * 1.5 * t).sin();
    let c = (2.0 * PI * 1.5 * t).cos();
    (
        Vec3::new(0.9 * s, 14.0, 0.4 * c),
        Vec3::new(0.8, 1.1 * c, -0.7 * s),
    )
}

fn main() {
    let mut it = std::env::args().skip(1);
    let ticks: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(240);
    let rest: Vec<String> = it.collect();
    let mut adapter = 0usize;
    if let Some(k) = rest.iter().position(|x| x == "--adapter") {
        adapter = rest.get(k + 1).and_then(|v| v.parse().ok()).unwrap_or(0);
    }
    let (_, device, queue) = match vxl_phys_gpu::probe::device_for(adapter) {
        Ok(t) => t,
        Err(e) => {
            println!("⚠️ 没起 GPU 设备（{e}）——本探针需要适配器");
            return;
        }
    };
    let g = Vec3::new(0.0, -9.81, 0.0);
    let dt = 1.0 / 60.0;
    let (maxl, maxa) = (50.0f32, 40.0f32);
    let step = StepCfg {
        gravity: g,
        dt,
        max_lin: maxl,
        max_ang: maxa,
    };
    // CPU 链。
    let mut cpu = make_bodies();
    // 卡上链（同一初值）。
    let mut st = states_of(&cpu);
    let mut stage = BodyStage::new(&device);
    let mut trace: Vec<(usize, Vec3, Vec3)> = Vec::new();
    for k in 0..ticks {
        let (f, tq) = script((k as f32) * dt);
        for (i, bs) in st.iter_mut().enumerate() {
            cpu.force[i] = f;
            cpu.torque[i] = tq;
            bs.force = f;
            bs.torque = tq;
        }
        Integrator::integrate_velocities(&mut cpu, g, dt, maxl, maxa);
        Integrator::integrate_positions(&mut cpu, dt);
        stage.upload(&device, &queue, step, &st);
        let mut enc =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        stage.encode(&mut enc);
        queue.submit(Some(enc.finish()));
        st = stage.download(&device, &queue);
        let tk = k + 1;
        if matches!(tk, 1 | 5 | 20) || tk % 60 == 0 || tk == ticks {
            trace.push((tk, cpu.position[0], st[0].pos));
        }
    }
    let qa = cpu.rot(0);
    let qb = st[0].rot;
    let dq = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
    let ang = 2.0 * dq.abs().min(1.0).acos();
    let (mut dp, mut dv, mut dw) = (0.0f32, 0.0f32, 0.0f32);
    for (i, bs) in st.iter().enumerate() {
        dp = dp.max((cpu.position[i] - bs.pos).length());
        dv = dv.max((cpu.linvel[i] - bs.linvel).length());
        dw = dw.max((cpu.angvel(i) - bs.angvel).length());
    }
    println!("== 刚体前缀积分：卡上 `body_integrate.wgsl` vs CPU `Integrator` ==");
    println!(
        "  {} 体（盒 2 kg / 球 1 kg）| {ticks} tick × dt=1/60 | 重力 + 脚本 (F, τ) | 限速 {maxl}/{maxa}",
        cpu.len()
    );
    println!("  ① 体 0 位置（CPU vs 卡上）采样：");
    for (t, a, b) in &trace {
        println!(
            "     t={t:>4}：({:.5}, {:.5}, {:.5}) vs ({:.5}, {:.5}, {:.5})｜差 {:.2e} m",
            a.x,
            a.y,
            a.z,
            b.x,
            b.y,
            b.z,
            (*a - *b).length()
        );
    }
    println!(
        "  ② 末态（N={ticks}）：max|Δpos| {dp:.3e} m | max|Δv| {dv:.3e} m/s | max|Δω| {dw:.3e} rad/s | 体 0 姿态夹角 {ang:.3e} rad"
    );
    println!(
        "     标定：|pos| ≈ {:.3} m、|v| ≈ {:.3} m/s ⇒ 相对量 {:.2e} / {:.2e}",
        cpu.position[0].length(),
        cpu.linvel[0].length(),
        dp / cpu.position[0].length().max(1e-6),
        dv / cpu.linvel[0].length().max(1e-6)
    );
}
