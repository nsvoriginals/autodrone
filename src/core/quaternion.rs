use super::vector::Vector3;
use super::matrix::Matrix3;
use std::ops::{Mul, MulAssign, Add, Sub, Div, Neg};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn zero() -> Self {
        Self { w: 0.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn from_axis_angle(axis: Vector3, angle: f64) -> Self {
        let half = angle * 0.5;
        let (s, c) = half.sin_cos();
        let axis = axis.normalize();
        Self {
            w: c,
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll * 0.5).sin_cos();
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn from_matrix(m: &Matrix3) -> Self {
        let trace = m.m[0][0] + m.m[1][1] + m.m[2][2];

        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Self {
                w: 0.25 * s,
                x: (m.m[2][1] - m.m[1][2]) / s,
                y: (m.m[0][2] - m.m[2][0]) / s,
                z: (m.m[1][0] - m.m[0][1]) / s,
            }
        } else if m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2] {
            let s = (1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]).sqrt() * 2.0;
            Self {
                w: (m.m[2][1] - m.m[1][2]) / s,
                x: 0.25 * s,
                y: (m.m[0][1] + m.m[1][0]) / s,
                z: (m.m[0][2] + m.m[2][0]) / s,
            }
        } else if m.m[1][1] > m.m[2][2] {
            let s = (1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]).sqrt() * 2.0;
            Self {
                w: (m.m[0][2] - m.m[2][0]) / s,
                x: (m.m[0][1] + m.m[1][0]) / s,
                y: 0.25 * s,
                z: (m.m[1][2] + m.m[2][1]) / s,
            }
        } else {
            let s = (1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]).sqrt() * 2.0;
            Self {
                w: (m.m[1][0] - m.m[0][1]) / s,
                x: (m.m[0][2] + m.m[2][0]) / s,
                y: (m.m[1][2] + m.m[2][1]) / s,
                z: 0.25 * s,
            }
        }
    }

    pub fn magnitude(&self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn magnitude_squared(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag < 1e-12 {
            Self::identity()
        } else {
            *self / mag
        }
    }

    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn inverse(&self) -> Self {
        let mag_sq = self.magnitude_squared();
        if mag_sq < 1e-12 {
            Self::identity()
        } else {
            self.conjugate() / mag_sq
        }
    }

    pub fn dot(&self, other: &Self) -> f64 {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn rotate_vector(&self, v: &Vector3) -> Vector3 {
        let qv = Quaternion::new(0.0, v.x, v.y, v.z);
        let rotated = *self * qv * self.inverse();
        Vector3::new(rotated.x, rotated.y, rotated.z)
    }

    pub fn to_matrix(&self) -> Matrix3 {
        let q = self.normalize();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        Matrix3::new(
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy - wz),
            2.0 * (xz + wy),
            2.0 * (xy + wz),
            1.0 - 2.0 * (xx + zz),
            2.0 * (yz - wx),
            2.0 * (xz - wy),
            2.0 * (yz + wx),
            1.0 - 2.0 * (xx + yy),
        )
    }

    pub fn to_euler(&self) -> (f64, f64, f64) {
        let q = self.normalize();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);

        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            sinp.signum() * std::f64::consts::FRAC_PI_2
        } else {
            sinp.asin()
        };

        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        Self {
            w: self.w + (other.w - self.w) * t,
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            z: self.z + (other.z - self.z) * t,
        }
    }

    pub fn slerp(&self, other: &Self, t: f64) -> Self {
        let mut dot = self.dot(other);
        let mut q2 = *other;

        if dot < 0.0 {
            dot = -dot;
            q2 = -q2;
        }

        if dot > 0.9995 {
            return self.lerp(&q2, t).normalize();
        }

        let theta = dot.acos();
        let sin_theta = theta.sin();
        let a = ((1.0 - t) * theta).sin() / sin_theta;
        let b = (t * theta).sin() / sin_theta;

        Self {
            w: self.w * a + q2.w * b,
            x: self.x * a + q2.x * b,
            y: self.y * a + q2.y * b,
            z: self.z * a + q2.z * b,
        }
    }

    pub fn nlerp(&self, other: &Self, t: f64) -> Self {
        self.lerp(other, t).normalize()
    }

    pub fn angle_between(&self, other: &Self) -> f64 {
        let dot = self.dot(other).clamp(-1.0, 1.0);
        2.0 * dot.acos()
    }
}

impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }
}

impl MulAssign for Quaternion {
    fn mul_assign(&mut self, other: Self) {
        *self = *self * other;
    }
}

impl Add for Quaternion {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            w: self.w + other.w,
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Quaternion {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            w: self.w - other.w,
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Mul<Quaternion> for f64 {
    type Output = Quaternion;
    fn mul(self, q: Quaternion) -> Quaternion {
        q * self
    }
}

impl Div<f64> for Quaternion {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        Self {
            w: self.w / scalar,
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl Neg for Quaternion {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            w: -self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl std::fmt::Display for Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "({:.3} + {:.3}i + {:.3}j + {:.3}k)",
            self.w, self.x, self.y, self.z
        )
    }
}
