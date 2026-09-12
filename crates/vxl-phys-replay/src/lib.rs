//! # vxl-phys-replay
//!
//! 确定性验证（§5：CPU xxh3-128 规范化哈希 = 回放对账唯一真值）：
//!
//! - **规范化打包**：体数（u64 LE）+ 逐体升序记录 `[kind u8][pos 3×f32][rot 4×f32]
//!   [lin 3×f32][ang 3×f32]`（每个 f32 取 `to_bits()` 的 little-endian 字节）——
//!   跨平台字节序一致，与内存布局（热/冷分离）无关；
//! - **xxh3-128**（`xxhash-rust`，E0）：流式更新，规范化过程用调用方提供的
//!   字节暂存（相位 arena / 栈缓冲），**全路径零堆分配**；
//! - 哈希与暂存尺寸、分块边界无关（单测：不同暂存尺寸 → 同一哈希）；
//! - 回放 = 输入序列重放 + 每 60 tick 状态哈希比对；不一致 = 阻断提交。

#![forbid(unsafe_code)]

use vxl_phys_core::{BodySet, Vec3};
use xxhash_rust::xxh3::Xxh3;

/// 单体内规范记录的字节数：1 + 3×4 + 4×4 + 3×4 + 3×4 = 53。
pub const BODY_RECORD_BYTES: usize = 53;

/// 状态哈希函数族（可替换实现；M0 规范实现 = `Xxh3Hash`）。
pub trait StateHash {
    fn hash_bodies(&self, bodies: &BodySet) -> u128;
}

/// M0 规范哈希：xxh3-128（§5 唯一真值）。
#[derive(Clone, Copy, Debug, Default)]
pub struct Xxh3Hash;

/// 流式规范哈希器：`scratch` 为调用方暂存（建议 ≥ 256B；越大越少分块更新）。
///
/// 确定性契约：输出只取决于体序列与 `scratch` 的**容量下限**（分块边界不影响
/// 结果——单测守门），不取决于 `scratch` 的起始地址或内容残留。
pub struct StateHasher<'a> {
    h: Xxh3,
    scratch: &'a mut [u8],
    cursor: usize,
}

impl<'a> StateHasher<'a> {
    pub fn new(scratch: &'a mut [u8]) -> Self {
        assert!(
            scratch.len() >= 128,
            "规范化暂存需 ≥ 128 字节（保证单条 53B 记录不分块）"
        );
        Self {
            h: Xxh3::new(),
            scratch,
            cursor: 0,
        }
    }

    fn flush(&mut self) {
        if self.cursor > 0 {
            self.h.update(&self.scratch[..self.cursor]);
            self.cursor = 0;
        }
    }

    fn push_bytes(&mut self, bytes: &[u8]) {
        debug_assert!(bytes.len() <= self.scratch.len());
        if self.cursor + bytes.len() > self.scratch.len() {
            self.flush();
        }
        self.scratch[self.cursor..self.cursor + bytes.len()].copy_from_slice(bytes);
        self.cursor += bytes.len();
    }

    fn push_f32(&mut self, v: f32) {
        self.push_bytes(&v.to_bits().to_le_bytes());
    }

    fn push_vec3(&mut self, v: Vec3) {
        self.push_f32(v.x);
        self.push_f32(v.y);
        self.push_f32(v.z);
    }

    /// 追加一体（kind：0 静态 / 1 动态；hot 记录位姿 + 速度）。
    pub fn push_body(
        &mut self,
        kind: u8,
        pos: Vec3,
        rot: vxl_phys_core::Quat,
        lin: Vec3,
        ang: Vec3,
    ) {
        self.push_bytes(&[kind]);
        self.push_vec3(pos);
        self.push_f32(rot.x);
        self.push_f32(rot.y);
        self.push_f32(rot.z);
        self.push_f32(rot.w);
        self.push_vec3(lin);
        self.push_vec3(ang);
    }

    pub fn finish(mut self) -> u128 {
        self.flush();
        self.h.digest128()
    }
}

/// 用调用方暂存对全体做规范哈希（按体索引升序；见模块注释的字节序契约）。
pub fn hash_bodies_streaming(bodies: &BodySet, scratch: &mut [u8]) -> u128 {
    let mut sh = StateHasher::new(scratch);
    sh.push_bytes(&(bodies.len() as u64).to_le_bytes());
    for i in 0..bodies.len() {
        let kind = match bodies.body_type[i] {
            vxl_phys_core::BodyType::Static => 0u8,
            vxl_phys_core::BodyType::Dynamic => 1u8,
        };
        // 按 id 序读取热组（32B 记录）与类型标签；awake/计时器等非物理态不入哈希。
        sh.push_body(
            kind,
            bodies.position[i],
            bodies.rot(i),
            bodies.linvel[i],
            bodies.angvel(i),
        );
    }
    sh.finish()
}

impl StateHash for Xxh3Hash {
    fn hash_bodies(&self, bodies: &BodySet) -> u128 {
        // 无外部暂存时的自备路径：栈缓冲（零堆分配，与流式路径同字节序）。
        let mut scratch = [0u8; 1024];
        hash_bodies_streaming(bodies, &mut scratch)
    }
}

/// 周期哈希记录器：每 `period` tick 记录一次状态哈希（§5：每 60 tick）。
#[derive(Clone, Debug)]
pub struct Recorder {
    pub period: u64,
    pub hashes: Vec<(u64, u128)>,
}

impl Recorder {
    pub fn new(period: u64) -> Self {
        Self {
            period,
            hashes: Vec::new(),
        }
    }

    /// 在每个 tick 末调用（tick 从 1 计）。
    pub fn observe(&mut self, tick: u64, hash: u128) {
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
    use vxl_phys_core::{Quat, Shape};

    fn sample() -> BodySet {
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
    }

    #[test]
    fn identical_states_hash_equal() {
        assert_eq!(
            Xxh3Hash.hash_bodies(&sample()),
            Xxh3Hash.hash_bodies(&sample())
        );
    }

    /// 1 ULP 的位模式差异必须改变哈希（回放对账的灵敏度下界）。
    #[test]
    fn perturbed_state_hash_differs() {
        let a = sample();
        let mut c = sample();
        // 相邻可表示值（+1 ULP）：比任何小数扰动都严格。
        let bits = c.position[0].y.to_bits();
        c.position[0].y = f32::from_bits(bits + 1);
        assert_ne!(Xxh3Hash.hash_bodies(&a), Xxh3Hash.hash_bodies(&c));
    }

    /// 暂存尺寸独立：128B / 256B / 4096B 暂存 → 同一哈希（分块边界无关）。
    #[test]
    fn scratch_size_independent() {
        let b = {
            let mut x = BodySet::new();
            for k in 0..97u32 {
                x.push_dynamic(
                    Shape::Box {
                        half: Vec3::splat(0.4),
                    },
                    Vec3::new(k as f32 * 0.5, 3.0, 0.25),
                    Quat::new(0.0, 0.0, 0.382_683_43, 0.923_879_5),
                    1.0,
                );
            }
            x
        };
        let mut s128 = [0u8; 128];
        let mut s256 = [0u8; 256];
        let mut s4096 = [0u8; 4096];
        let h128 = hash_bodies_streaming(&b, &mut s128);
        let h256 = hash_bodies_streaming(&b, &mut s256);
        let h4096 = hash_bodies_streaming(&b, &mut s4096);
        assert_eq!(h128, h256);
        assert_eq!(h256, h4096);
        assert_eq!(h4096, Xxh3Hash.hash_bodies(&b));
    }

    /// 与 crate 一次性 API 的规范字节序对拍（打包/流式实现的正确性锚）。
    #[test]
    fn matches_one_shot_over_reference_bytes() {
        let b = sample();
        let mut reference: Vec<u8> = Vec::new();
        reference.extend_from_slice(&(b.len() as u64).to_le_bytes());
        for i in 0..b.len() {
            reference.push(match b.body_type[i] {
                vxl_phys_core::BodyType::Static => 0,
                vxl_phys_core::BodyType::Dynamic => 1,
            });
            for v in [b.position[i].x, b.position[i].y, b.position[i].z] {
                reference.extend_from_slice(&v.to_bits().to_le_bytes());
            }
            let q = b.rot(i);
            for v in [q.x, q.y, q.z, q.w] {
                reference.extend_from_slice(&v.to_bits().to_le_bytes());
            }
            for v in [b.linvel[i].x, b.linvel[i].y, b.linvel[i].z] {
                reference.extend_from_slice(&v.to_bits().to_le_bytes());
            }
            let w = b.angvel(i);
            for v in [w.x, w.y, w.z] {
                reference.extend_from_slice(&v.to_bits().to_le_bytes());
            }
        }
        assert_eq!(
            reference.len(),
            8 + BODY_RECORD_BYTES * b.len(),
            "规范记录长度契约"
        );
        let mut scratch = [0u8; 128];
        assert_eq!(
            hash_bodies_streaming(&b, &mut scratch),
            xxhash_rust::xxh3::xxh3_128(&reference)
        );
    }

    #[test]
    fn recorder_period() {
        let mut r = Recorder::new(60);
        for t in 1..=180 {
            r.observe(t, t as u128);
        }
        assert_eq!(r.hashes.len(), 3);
        assert_eq!(r.hashes[0], (60, 60));
    }
}
