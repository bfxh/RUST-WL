//! 相位内存（§0.1 #10 / §8.1 缓存与分配杠杆）：
//!
//! - **相位 bump 分配器** `PhaseArena`：帧内 scratch 从固定容量缓冲 bump 出，
//!   相末 `reset` 零 free；布局由调用序唯一决定（同一 `(len, align)` 序列 →
//!   同一偏移序列，跨平台一致——确定性前提，单测守门）。
//! - **热/冷 32B 记录数组**：`Pose32`（pos + pad + rot = 32B）与 `Vel32`
//!   （lin + pad + ang + pad = 32B），`align(32)` → 底层缓冲 32B 对齐、
//!   逐体连续。热组 = `PoseArray` + `VelArray`（施工令的 hot 组落点）；
//!   冷组（形状/质量/休眠/材质）留在 `BodySet` 平铺字段，索引同序。
//!
//! 确定性：本模块不引入任何浮点归约与并行；偏移量全为整数运算。

#![allow(clippy::len_without_is_empty)]

use crate::math::{Quat, Vec3};

/// 相位 bump 分配器（固定容量，帧 arena；相末 `reset`，全帧零 free）。
///
/// 对齐契约：底层缓冲一次性超配 `MAX_ALIGN` 字节并做基址修正——`base` 是缓冲内
/// 首个 `MAX_ALIGN` 对齐的位置；支持 `align ≤ MAX_ALIGN`（32B 记录 / 64B 缓存线）
/// 的**绝对地址对齐**（不只是相对偏移）。
///
/// 不实现 `Clone`：基址修正与具体缓冲地址绑定，复制需重建（World 持有唯一实例）。
#[derive(Debug)]
pub struct PhaseArena {
    buf: Vec<u8>,
    /// 缓冲内基址修正量（`base % MAX_ALIGN == 0`）。
    base: usize,
    used: usize,
    high_water: usize,
    allocs: u64,
    overflows: u64,
}

/// 最大支持对齐（缓存线；覆盖 32B 记录与常见 SIMD 需求）。
pub const MAX_ALIGN: usize = 64;

impl PhaseArena {
    pub fn with_capacity(capacity: usize) -> Self {
        let buf = vec![0u8; capacity + MAX_ALIGN];
        let p = buf.as_ptr() as usize;
        let base = (MAX_ALIGN - (p % MAX_ALIGN)) % MAX_ALIGN;
        Self {
            buf,
            base,
            used: 0,
            high_water: 0,
            allocs: 0,
            overflows: 0,
        }
    }

    /// 可用容量（不含基址修正占用的字节）。
    pub fn capacity(&self) -> usize {
        self.buf.len() - self.base
    }

    /// 当前 bump 位置（相末 `reset` 后回 0）。
    pub fn used(&self) -> usize {
        self.used
    }

    /// 历史最高水位（= 峰值需求；600 tick 曲线平稳的直接证据）。
    pub fn high_water(&self) -> usize {
        self.high_water
    }

    /// 累计分配次数（溢出不计）。
    pub fn allocs(&self) -> u64 {
        self.allocs
    }

    /// 容量不足次数（> 0 表示预算需上调或调用方走了降级路径）。
    pub fn overflows(&self) -> u64 {
        self.overflows
    }

    /// 分配 `len` 字节、`align` 对齐（`align ≤ MAX_ALIGN` 的 2 的幂）。
    ///
    /// 布局契约：偏移只由「历史调用序 + 本次 (len, align)」决定；失败不推进
    /// bump 位置（后续调用不受失败影响）。
    pub fn alloc(&mut self, len: usize, align: usize) -> Option<&mut [u8]> {
        assert!(align.is_power_of_two() && align <= MAX_ALIGN, "align 非法");
        let start = self.base + ((self.used + (align - 1)) & !(align - 1));
        let end = start.checked_add(len)?;
        if end > self.buf.len() {
            self.overflows += 1;
            return None;
        }
        self.allocs += 1;
        self.used = end - self.base;
        self.high_water = self.high_water.max(self.used);
        Some(&mut self.buf[start..end])
    }

    /// 一次调用租借多段互不重叠的可变切片（按声明序 bump）。
    ///
    /// 安全无重叠的构造：先把全部段在整数域算好偏移，再对同一底层缓冲做
    /// 逐段 `split_at_mut`（跳过对齐 padding）——全程安全 Rust。
    pub fn alloc_many(&mut self, spans: &[(usize, usize)]) -> Option<Vec<&mut [u8]>> {
        let mut offsets: Vec<(usize, usize)> = Vec::with_capacity(spans.len());
        let mut cursor = self.used;
        for &(len, align) in spans {
            assert!(align.is_power_of_two() && align <= MAX_ALIGN, "align 非法");
            let rel = (cursor + (align - 1)) & !(align - 1);
            let start = self.base + rel;
            let end = start.checked_add(len)?;
            offsets.push((start, end));
            cursor = end - self.base;
        }
        if self.base + cursor > self.buf.len() {
            self.overflows += 1;
            return None;
        }
        self.allocs += spans.len() as u64;
        self.used = cursor;
        self.high_water = self.high_water.max(cursor);

        let mut out: Vec<&mut [u8]> = Vec::with_capacity(spans.len());
        let mut rest: &mut [u8] = &mut self.buf[self.base..self.base + cursor];
        let mut at = self.base;
        for &(start, end) in &offsets {
            let (_, after_pad) = rest.split_at_mut(start - at);
            let (seg, after_seg) = after_pad.split_at_mut(end - start);
            out.push(seg);
            rest = after_seg;
            at = end;
        }
        Some(out)
    }

    /// 相末归零（下一相同容量复用；缓冲永不增长、永不释放）。
    pub fn reset(&mut self) {
        self.used = 0;
    }
}

/// 位姿热记录：pos(12B) + pad(4B) + quat(16B) = **32B**。
/// `align(32)` 使数组缓冲 32B 对齐（§8.1：热块进同一缓存线/半线）。
#[repr(C, align(32))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose32 {
    pub pos: Vec3,
    pub _pad: f32,
    pub rot: Quat,
}

impl Default for Pose32 {
    fn default() -> Self {
        Self {
            pos: Vec3::ZERO,
            _pad: 0.0,
            rot: Quat::IDENTITY,
        }
    }
}

/// 速度热记录：lin(12B) + pad(4B) + ang(12B) + pad(4B) = **32B**。
#[repr(C, align(32))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vel32 {
    pub lin: Vec3,
    pub _pad0: f32,
    pub ang: Vec3,
    pub _pad1: f32,
}

impl Default for Vel32 {
    fn default() -> Self {
        Self {
            lin: Vec3::ZERO,
            _pad0: 0.0,
            ang: Vec3::ZERO,
            _pad1: 0.0,
        }
    }
}

/// 位姿热数组（hot 组）：`arr[i]` 读写 **位置**（pos 分量）；
/// 旋转经 `rot(i)` / `set_rot(i, q)`（同一 32B 记录的第二分量）。
#[derive(Clone, Debug, Default)]
pub struct PoseArray {
    recs: Vec<Pose32>,
}

impl PoseArray {
    pub fn new() -> Self {
        Self { recs: Vec::new() }
    }

    pub fn with_capacity(n: usize) -> Self {
        Self {
            recs: Vec::with_capacity(n),
        }
    }

    pub fn push(&mut self, pos: Vec3, rot: Quat) {
        self.recs.push(Pose32 {
            pos,
            _pad: 0.0,
            rot,
        });
    }

    pub fn len(&self) -> usize {
        self.recs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.recs.is_empty()
    }

    #[inline]
    pub fn rot(&self, i: usize) -> Quat {
        self.recs[i].rot
    }

    #[inline]
    pub fn set_rot(&mut self, i: usize, q: Quat) {
        self.recs[i].rot = q;
    }

    /// 原始 32B 记录视图（布局测试 / M1 SIMD 入口）。
    pub fn as_records(&self) -> &[Pose32] {
        &self.recs
    }
}

impl std::ops::Index<usize> for PoseArray {
    type Output = Vec3;
    #[inline]
    fn index(&self, i: usize) -> &Vec3 {
        &self.recs[i].pos
    }
}

impl std::ops::IndexMut<usize> for PoseArray {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut Vec3 {
        &mut self.recs[i].pos
    }
}

/// 速度热数组（hot 组）：`arr[i]` 读写 **线速度**（lin 分量）；
/// 角速度经 `ang(i)` / `set_ang(i, w)`。
#[derive(Clone, Debug, Default)]
pub struct VelArray {
    recs: Vec<Vel32>,
}

impl VelArray {
    pub fn new() -> Self {
        Self { recs: Vec::new() }
    }

    pub fn push(&mut self, lin: Vec3, ang: Vec3) {
        self.recs.push(Vel32 {
            lin,
            _pad0: 0.0,
            ang,
            _pad1: 0.0,
        });
    }

    pub fn len(&self) -> usize {
        self.recs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.recs.is_empty()
    }

    #[inline]
    pub fn ang(&self, i: usize) -> Vec3 {
        self.recs[i].ang
    }

    #[inline]
    pub fn set_ang(&mut self, i: usize, w: Vec3) {
        self.recs[i].ang = w;
    }

    /// 原始 32B 记录视图（布局测试 / M1 SIMD 入口）。
    pub fn as_records(&self) -> &[Vel32] {
        &self.recs
    }
}

impl std::ops::Index<usize> for VelArray {
    type Output = Vec3;
    #[inline]
    fn index(&self, i: usize) -> &Vec3 {
        &self.recs[i].lin
    }
}

impl std::ops::IndexMut<usize> for VelArray {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut Vec3 {
        &mut self.recs[i].lin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 记录一次分配相对缓冲基址的偏移（借用即还，只留整数）。
    fn offsets(arena: &mut PhaseArena, seq: &[(usize, usize)]) -> Vec<usize> {
        let base = {
            let probe = arena.alloc(0, 1).expect("探针分配");
            probe.as_ptr() as usize
        };
        let mut out = Vec::with_capacity(seq.len());
        for &(len, align) in seq {
            let seg = arena.alloc(len, align).expect("分配");
            out.push(seg.as_ptr() as usize - base);
        }
        out
    }

    const SEQ: &[(usize, usize)] = &[
        (13, 1),
        (7, 4),
        (64, 32),
        (1, 8),
        (96, 64),
        (5, 2),
        (128, 32),
    ];

    /// 验收①：同调用序 → 逐字节相同地址布局（偏移序列全等）。
    #[test]
    fn same_call_order_same_layout() {
        let mut a = PhaseArena::with_capacity(4096);
        let mut b = PhaseArena::with_capacity(4096);
        assert_eq!(offsets(&mut a, SEQ), offsets(&mut b, SEQ));
        // reset 后复用同一布局（帧间零增长复用的确定性前提）。
        a.reset();
        b.reset();
        assert_eq!(a.used(), 0);
        assert_eq!(offsets(&mut a, SEQ), offsets(&mut b, SEQ));
        assert_eq!(a.high_water(), b.high_water());
    }

    #[test]
    fn alignment_respected() {
        let mut a = PhaseArena::with_capacity(1024);
        for &(len, align) in SEQ {
            let seg = a.alloc(len, align).unwrap();
            assert_eq!(seg.len(), len);
            // 绝对地址对齐（不只相对偏移）：缓冲基址已修正。
            assert_eq!(seg.as_ptr() as usize % align, 0, "align {align}");
        }
        assert_eq!(a.high_water(), 384, "布局金样（偏移契约锁定）");
    }

    /// 验收②的机制面：容量恒定、reset 复用、高水位不增长（600 tick 曲线平稳前提）。
    #[test]
    fn capacity_fixed_and_reset_reuses() {
        let mut a = PhaseArena::with_capacity(4096);
        let cap = a.capacity();
        let seq: Vec<(usize, usize)> = (0..50).map(|k| (37 + k % 7, 32)).collect();
        let mut hw0 = 0usize;
        for round in 0..600 {
            let _ = offsets(&mut a, &seq);
            a.reset();
            assert_eq!(a.used(), 0);
            if round == 0 {
                hw0 = a.high_water();
            }
        }
        assert_eq!(a.capacity(), cap, "容量永不增长");
        assert_eq!(a.high_water(), hw0, "峰值水位跨轮恒定（曲线平稳）");
        assert!(a.high_water() <= cap);
        assert_eq!(a.overflows(), 0);
        assert_eq!(a.allocs(), 600 * 51, "51 = 探针 + 50 段");
    }

    /// 失败不推进 bump 位置（失败后布局仍与无失败序列一致）。
    #[test]
    fn overflow_is_rejected_without_bumping() {
        let mut a = PhaseArena::with_capacity(64);
        assert!(a.alloc(32, 1).is_some());
        let used_before = a.used();
        assert!(a.alloc(1024, 1).is_none());
        assert_eq!(a.overflows(), 1);
        assert_eq!(a.used(), used_before);
        assert!(a.alloc(32, 1).is_some(), "失败不阻塞后续分配");
        assert_eq!(a.allocs(), 2, "溢出不计入成功分配数");
    }

    #[test]
    fn alloc_many_disjoint_and_aligned() {
        let mut a = PhaseArena::with_capacity(1024);
        let spans = [(3usize, 1usize), (8, 32), (5, 16), (32, 64)];
        let segs = a.alloc_many(&spans).expect("多段租借");
        assert_eq!(segs.len(), spans.len());
        for (seg, &(len, align)) in segs.iter().zip(spans.iter()) {
            assert_eq!(seg.len(), len);
            assert_eq!(seg.as_ptr() as usize % align, 0);
        }
        // 互不重叠（地址区间两两不相交）。
        let ranges: Vec<(usize, usize)> = segs
            .iter()
            .map(|s| (s.as_ptr() as usize, s.as_ptr() as usize + s.len()))
            .collect();
        for i in 0..ranges.len() {
            for j in (i + 1)..ranges.len() {
                assert!(
                    ranges[i].1 <= ranges[j].0 || ranges[j].1 <= ranges[i].0,
                    "段 {i} 与段 {j} 重叠"
                );
            }
        }
        drop(segs);
        // 写后读隔离：单段租借写入不影响其他段（借用检查器已证明，此处验语义）。
        let mut b = PhaseArena::with_capacity(1024);
        {
            let mut segs = b.alloc_many(&spans).unwrap();
            segs[0].fill(0xAA);
            segs[1].fill(0xBB);
            assert!(segs[0].iter().all(|&x| x == 0xAA));
            assert!(segs[1].iter().all(|&x| x == 0xBB));
        }
        b.reset();
        assert_eq!(b.used(), 0);
    }

    /// 热记录 32B 布局契约（size/align/字段偏移）。
    #[test]
    fn hot_records_are_32b() {
        use std::mem::{align_of, offset_of, size_of};
        assert_eq!(size_of::<Pose32>(), 32);
        assert_eq!(align_of::<Pose32>(), 32);
        assert_eq!(offset_of!(Pose32, pos), 0);
        assert_eq!(offset_of!(Pose32, rot), 16);
        assert_eq!(size_of::<Vel32>(), 32);
        assert_eq!(align_of::<Vel32>(), 32);
        assert_eq!(offset_of!(Vel32, lin), 0);
        assert_eq!(offset_of!(Vel32, ang), 16);
    }

    #[test]
    fn hot_arrays_index_write_and_align() {
        let mut pa = PoseArray::new();
        let mut va = VelArray::new();
        for i in 0..8usize {
            pa.push(Vec3::new(i as f32, 0.0, 0.0), Quat::IDENTITY);
            va.push(Vec3::ZERO, Vec3::ZERO);
        }
        // 数组缓冲 32B 对齐；逐体记录连续 32B。
        assert_eq!(pa.as_records().as_ptr() as usize % 32, 0);
        assert_eq!(va.as_records().as_ptr() as usize % 32, 0);
        let recs = pa.as_records();
        let step = std::ptr::addr_of!(recs[1]) as usize - std::ptr::addr_of!(recs[0]) as usize;
        assert_eq!(step, 32, "记录间步长必须是 32B");
        // position[i] 读写 = pos 分量；rot/ang 经访问器。
        assert_eq!(pa[3], Vec3::new(3.0, 0.0, 0.0));
        pa[3] = Vec3::new(9.0, 1.0, 2.0);
        assert_eq!(pa[3], Vec3::new(9.0, 1.0, 2.0));
        let q = Quat::new(0.0, 0.5, 0.0, 0.5);
        pa.set_rot(3, q);
        assert_eq!(pa.rot(3), q);
        assert_eq!(va.ang(3), Vec3::ZERO);
        va.set_ang(3, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(va.ang(3), Vec3::new(0.0, 1.0, 0.0));
        // 位置写入不得触碰同记录的旋转分量。
        pa[5] = Vec3::new(1.0, 1.0, 1.0);
        assert_eq!(pa.rot(5), Quat::IDENTITY);
    }
}
