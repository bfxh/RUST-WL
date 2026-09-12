//! # vxl-phys-memfind
//!
//! 内存子串工具（§10）—— **用途限定**：资产管线索引、调试符号定位、序列化定位、
//! 碰撞模式查找；**不进求解核心**。
//!
//! M0：标量正确性路径（无 SIMD 依赖）；SIMD（memchr::memmem / aho-corasick /
//! StringZilla 对标 kernel）随 M1 引入，§7 三层律约束汇编。
//! 性能指标〔目标，Criterion 固定数据集〕：
//! 1B needle/64KB haystack ≥50 GB/s；短 needle ≥10 GB/s；长 needle ≥4 GB/s；
//! 多模式 1 万模式/1 MB ≥1 GB/s；GPU 批量 ≥100 GB/s。无 SIMD/GPU → 标量回退必测。

#![forbid(unsafe_code)]

/// 标量子串查找（std 内存安全实现，作为 SIMD 路径的正确性基线）。
pub fn find_scalar(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() {
        return Some(0);
    }
    if needle.len() > haystack.len() {
        return None;
    }
    let first = needle[0];
    let mut i = 0;
    while i + needle.len() <= haystack.len() {
        if haystack[i] == first && &haystack[i..i + needle.len()] == needle {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// 多模式查找（M0：最左匹配语义，位置相同取先序模式；aho-corasick 在 M1 接入）。
pub fn find_any_scalar(haystack: &[u8], needles: &[&[u8]]) -> Option<(usize, usize)> {
    let mut best: Option<(usize, usize)> = None;
    for (k, n) in needles.iter().enumerate() {
        if let Some(pos) = find_scalar(haystack, n) {
            let better = match best {
                None => true,
                Some((_, bp)) => pos < bp,
            };
            if better {
                best = Some((k, pos));
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_find_basics() {
        assert_eq!(find_scalar(b"hello world", b"wor"), Some(6));
        assert_eq!(find_scalar(b"hello", b"hello"), Some(0));
        assert_eq!(find_scalar(b"hello", b"hellx"), None);
        assert_eq!(find_scalar(b"ab", b"abc"), None);
        assert_eq!(find_scalar(b"abc", b""), Some(0));
    }

    #[test]
    fn multi_mode_first_wins() {
        let r = find_any_scalar(b"the quick brown fox", &[b"fox", b"quick"]);
        assert_eq!(r, Some((1, 4)));
        assert_eq!(find_any_scalar(b"zzz", &[b"a", b"b"]), None);
    }
}
