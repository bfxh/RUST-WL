//! 严格 f32 数学库（§5 确定性）。
//!
//! - 所有运算按固定表达式顺序展开，禁止编译器 FMA 重排（Rust 默认不契约）；
//! - 不使用 `std::simd` / `core::arch`（§7 分层后续在独立路径引入，sim 主路径保持标量语义）。

use core::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const ONE: Self = Self {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    pub const X: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    pub const Y: Self = Self {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    pub const Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub const fn splat(v: f32) -> Self {
        Self { x: v, y: v, z: v }
    }

    #[inline]
    pub fn dot(self, o: Self) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    #[inline]
    pub fn cross(self, o: Self) -> Self {
        Self {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// 归一化；零向量安全回退为 ZERO（不产生 NaN）。
    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 1e-12 {
            self * (1.0 / len)
        } else {
            Vec3::ZERO
        }
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    #[inline]
    pub fn min(self, o: Self) -> Self {
        Self::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    #[inline]
    pub fn max(self, o: Self) -> Self {
        Self::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    #[inline]
    pub fn scale(self, s: f32) -> Self {
        self * s
    }

    #[inline]
    pub fn mul_per_elem(self, o: Self) -> Self {
        Self::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    #[inline]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}
impl MulAssign<f32> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f32) {
        *self = *self * s;
    }
}

/// 单位四元数 (x, y, z, w)，Hamilton 约定。
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// 轴角构造（轴内部归一化；零轴回退 IDENTITY）。
    #[inline]
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let a = axis.normalize();
        if a.length_squared() < 0.5 {
            return Quat::IDENTITY;
        }
        let s = (0.5 * angle).sin();
        let c = (0.5 * angle).cos();
        Self::new(a.x * s, a.y * s, a.z * s, c)
    }

    /// Hamilton 乘积，使用 `*` 运算符（`self * o`）。
    #[inline]
    pub fn conjugate(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    /// 旋转向量 v（右手法则，世界系）。
    #[inline]
    pub fn rotate_vec3(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v) * 2.0;
        v + qv.cross(t) + t * self.w
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len2 = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        if len2 > 1e-16 {
            let inv = 1.0 / len2.sqrt();
            Self::new(self.x * inv, self.y * inv, self.z * inv, self.w * inv)
        } else {
            Quat::IDENTITY
        }
    }

    /// 角速度积分（世界系 ω）：q' = normalize(q + 0.5·dt·(0,ω)⊗q)。
    #[inline]
    pub fn integrate_angular(self, w: Vec3, dt: f32) -> Self {
        let wq = Self::new(w.x, w.y, w.z, 0.0);
        let dq = wq * self;
        Self::new(
            self.x + dq.x * (0.5 * dt),
            self.y + dq.y * (0.5 * dt),
            self.z + dq.z * (0.5 * dt),
            self.w + dq.w * (0.5 * dt),
        )
        .normalize()
    }
}

impl Mul for Quat {
    type Output = Quat;
    /// Hamilton 乘积 self * o。
    #[inline]
    fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }
}

/// 3×3 行主序矩阵。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    #[inline]
    pub fn diag(x: f32, y: f32, z: f32) -> Self {
        Self {
            m: [[x, 0.0, 0.0], [0.0, y, 0.0], [0.0, 0.0, z]],
        }
    }

    /// 旋转矩阵（行主序）。
    #[inline]
    pub fn from_quat(q: Quat) -> Self {
        let xx = q.x * q.x;
        let yy = q.y * q.y;
        let zz = q.z * q.z;
        let xy = q.x * q.y;
        let xz = q.x * q.z;
        let yz = q.y * q.z;
        let wx = q.w * q.x;
        let wy = q.w * q.y;
        let wz = q.w * q.z;
        Self {
            m: [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
        }
    }

    #[inline]
    pub fn mul_vec3(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    /// Rᵀ·v（列点积）。
    #[inline]
    pub fn transpose_mul_vec3(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[1][0] * v.y + self.m[2][0] * v.z,
            self.m[0][1] * v.x + self.m[1][1] * v.y + self.m[2][1] * v.z,
            self.m[0][2] * v.x + self.m[1][2] * v.y + self.m[2][2] * v.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn rotate_x_about_y_90() {
        let q = Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2);
        let v = q.rotate_vec3(Vec3::X);
        assert!(approx(v.x, 0.0) && approx(v.y, 0.0) && approx(v.z, -1.0));
    }

    #[test]
    fn quat_mul_identity() {
        let q = Quat::from_axis_angle(Vec3::Z, 0.7);
        assert!(approx((q * Quat::IDENTITY).x, q.x));
        assert!(approx((q * Quat::IDENTITY).w, q.w));
    }

    #[test]
    fn integrate_zero_omega_keeps_quat() {
        let q = Quat::from_axis_angle(Vec3::X, 1.3);
        let q2 = q.integrate_angular(Vec3::ZERO, 1.0 / 60.0);
        assert!(approx(q2.x, q.x) && approx(q2.w, q.w));
    }

    #[test]
    fn mat3_roundtrip() {
        let q = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.9);
        let m = Mat3::from_quat(q);
        for v in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(0.3, -1.2, 2.2)] {
            let r = m.mul_vec3(v);
            let r2 = q.rotate_vec3(v);
            assert!(approx(r.x, r2.x) && approx(r.y, r2.y) && approx(r.z, r2.z));
        }
    }

    #[test]
    fn normalize_zero_safe() {
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);
    }
}
