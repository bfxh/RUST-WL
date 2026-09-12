//! # vxl-phys-replay
//!
//! 确定性验证（§5）：
//! - 状态哈希：对 `BodySet` 的位模式（`f32::to_bits`，little-endian）做 FNV-1a 64
//!   有序归约 —— 同输入 bit 级一致，跨平台可复现。
//!   （规格书目标哈希为 xxh3；M0 以零依赖 FNV-1a 占位，接口
//!   `StateHash` 允许 M1 无感替换为 xxh3，阻断标准不变：哈希不一致 = 阻断提交。）
//! - 回放 = 输入序列重放 + 每 60 tick 状态哈希比对。

#![forbid(unsafe_code)]

use vxl_phys_core::BodySet;

/// 状态哈希函数族（可替换：FNV-1a → xxh3）。
pub trait StateHash {
    fn hash_bodies(&self, bodies: &BodySet) -> u64;
}

/// M0 默认：FNV-1a 64。
#[derive(Clone, Copy, Debug, Default)]
pub struct Fnv1aHash;

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

struct Hasher(u64);

impl Hasher {
    fn new() -> Self {
        Hasher(FNV_OFFSET)
    }
    fn push(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(FNV_PRIME);
        }
    }
    fn push_f32(&mut self, v: f32) {
        self.push(&v.to_bits().to_le_bytes());
    }
    fn push_vec3(&mut self, v: vxl_phys_core::Vec3) {
        self.push_f32(v.x);
        self.push_f32(v.y);
        self.push_f32(v.z);
    }
    fn finish(self) -> u64 {
        self.0
    }
}

impl StateHash for Fnv1aHash {
    fn hash_bodies(&self, bodies: &BodySet) -> u64 {
        let mut h = Hasher::new();
        h.push(&(bodies.len() as u64).to_le_bytes());
        for i in 0..bodies.len() {
            // 类型 + 位姿 + 速度（awake/计时器等非物理态不入哈希，回放语义=物理态）。
            h.push(&[match bodies.body_type[i] {
                vxl_phys_core::BodyType::Static => 0u8,
                vxl_phys_core::BodyType::Dynamic => 1u8,
            }]);
            h.push_vec3(bodies.position[i]);
            let q = bodies.rotation[i];
            h.push_f32(q.x);
            h.push_f32(q.y);
            h.push_f32(q.z);
            h.push_f32(q.w);
            h.push_vec3(bodies.linvel[i]);
            h.push_vec3(bodies.angvel[i]);
        }
        h.finish()
    }
}

/// 周期哈希记录器：每 `period` tick 记录一次状态哈希（§5：每 60 tick）。
#[derive(Clone, Debug)]
pub struct Recorder {
    pub period: u64,
    pub hashes: Vec<(u64, u64)>,
}

impl Recorder {
    pub fn new(period: u64) -> Self {
        Self {
            period,
            hashes: Vec::new(),
        }
    }

    /// 在每个 tick 末调用（tick 从 1 计）。
    pub fn observe(&mut self, tick: u64, hash: u64) {
        if tick.is_multiple_of(self.period.max(1)) {
            self.hashes.push((tick, hash));
        }
    }

    /// 与另一条记录逐项比对。
    pub fn matches(&self, other: &Recorder) -> bool {
        self.hashes == other.hashes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vxl_phys_core::{Quat, Shape, Vec3};

    #[test]
    fn identical_states_hash_equal() {
        let build = || {
            let mut b = BodySet::new();
            b.push_dynamic(
                Shape::Sphere { radius: 1.0 },
                Vec3::new(1.0, 2.0, 3.0),
                Quat::IDENTITY,
                1.0,
            );
            b.push_static(
                Shape::Box {
                    half: Vec3::splat(1.0),
                },
                Vec3::ZERO,
                Quat::IDENTITY,
            );
            b
        };
        let (a, c) = (build(), build());
        assert_eq!(Fnv1aHash.hash_bodies(&a), Fnv1aHash.hash_bodies(&c));
    }

    #[test]
    fn perturbed_state_hash_differs() {
        let mut a = BodySet::new();
        a.push_dynamic(
            Shape::Sphere { radius: 1.0 },
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
        );
        let mut c = a.clone();
        c.position[0].y += 1e-9;
        assert_ne!(Fnv1aHash.hash_bodies(&a), Fnv1aHash.hash_bodies(&c));
    }

    #[test]
    fn recorder_period() {
        let mut r = Recorder::new(60);
        for t in 1..=180 {
            r.observe(t, t);
        }
        assert_eq!(r.hashes.len(), 3);
        assert_eq!(r.hashes[0], (60, 60));
    }
}
