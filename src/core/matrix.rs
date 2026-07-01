use super::vector::Vector3;
use std::ops::Mul;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3 {
    pub m: [[f64; 3]; 3],
}

impl Matrix3 {
    pub fn new(
        m00: f64, m01: f64, m02: f64,
        m10: f64, m11: f64, m12: f64,
        m20: f64, m21: f64, m22: f64,
    ) -> Self {
        Self {
            m: [
                [m00, m01, m02],
                [m10, m11, m12],
                [m20, m21, m22],
            ],
        }
    }

    pub fn identity() -> Self {
        Self {
            m: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    pub fn zero() -> Self {
        Self {
            m: [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
        }
    }

    pub fn rotation_x(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self::new(
            1.0, 0.0, 0.0,
            0.0, c, -s,
            0.0, s, c,
        )
    }

    pub fn rotation_y(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self::new(
            c, 0.0, s,
            0.0, 1.0, 0.0,
            -s, 0.0, c,
        )
    }

    pub fn rotation_z(angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        Self::new(
            c, -s, 0.0,
            s, c, 0.0,
            0.0, 0.0, 1.0,
        )
    }

    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let rz = Self::rotation_z(yaw);
        let ry = Self::rotation_y(pitch);
        let rx = Self::rotation_x(roll);
        rz * ry * rx
    }

    pub fn from_axis_angle(axis: Vector3, angle: f64) -> Self {
        let axis = axis.normalize();
        let (s, c) = angle.sin_cos();
        let (x, y, z) = (axis.x, axis.y, axis.z);
        let one_minus_c = 1.0 - c;

        Self::new(
            c + x * x * one_minus_c,
            x * y * one_minus_c - z * s,
            x * z * one_minus_c + y * s,
            y * x * one_minus_c + z * s,
            c + y * y * one_minus_c,
            y * z * one_minus_c - x * s,
            z * x * one_minus_c - y * s,
            z * y * one_minus_c + x * s,
            c + z * z * one_minus_c,
        )
    }

    pub fn transpose(&self) -> Self {
        Self::new(
            self.m[0][0], self.m[1][0], self.m[2][0],
            self.m[0][1], self.m[1][1], self.m[2][1],
            self.m[0][2], self.m[1][2], self.m[2][2],
        )
    }

    pub fn multiply(&self, other: &Self) -> Self {
        let mut result = Self::zero();
        for i in 0..3 {
            for j in 0..3 {
                result.m[i][j] =
                    self.m[i][0] * other.m[0][j] +
                    self.m[i][1] * other.m[1][j] +
                    self.m[i][2] * other.m[2][j];
            }
        }
        result
    }

    pub fn transform(&self, v: &Vector3) -> Vector3 {
        Vector3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    pub fn to_euler(&self) -> (f64, f64, f64) {
        let m = &self.m;
        let pitch = (-m[2][0]).asin();

        let roll = if pitch.abs() > 0.99999 {
            0.0
        } else {
            (m[2][1] / pitch.cos()).atan2(m[2][2] / pitch.cos())
        };

        let yaw = if pitch.abs() > 0.99999 {
            0.0
        } else {
            (m[1][0] / pitch.cos()).atan2(m[0][0] / pitch.cos())
        };

        (roll, pitch, yaw)
    }
}

impl Mul for Matrix3 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        self.multiply(&other)
    }
}

impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;
    fn mul(self, v: Vector3) -> Vector3 {
        self.transform(&v)
    }
}

impl std::fmt::Display for Matrix3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "[{:.3} {:.3} {:.3}]\n[{:.3} {:.3} {:.3}]\n[{:.3} {:.3} {:.3}]",
            self.m[0][0], self.m[0][1], self.m[0][2],
            self.m[1][0], self.m[1][1], self.m[1][2],
            self.m[2][0], self.m[2][1], self.m[2][2],
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix4 {
    pub m: [[f64; 4]; 4],
}

impl Matrix4 {
    pub fn new(
        m00: f64, m01: f64, m02: f64, m03: f64,
        m10: f64, m11: f64, m12: f64, m13: f64,
        m20: f64, m21: f64, m22: f64, m23: f64,
        m30: f64, m31: f64, m32: f64, m33: f64,
    ) -> Self {
        Self {
            m: [
                [m00, m01, m02, m03],
                [m10, m11, m12, m13],
                [m20, m21, m22, m23],
                [m30, m31, m32, m33],
            ],
        }
    }

    pub fn identity() -> Self {
        Self::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    pub fn translation(v: Vector3) -> Self {
        Self::new(
            1.0, 0.0, 0.0, v.x,
            0.0, 1.0, 0.0, v.y,
            0.0, 0.0, 1.0, v.z,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    pub fn scale(s: f64) -> Self {
        Self::new(
            s, 0.0, 0.0, 0.0,
            0.0, s, 0.0, 0.0,
            0.0, 0.0, s, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    pub fn perspective(fov: f64, aspect: f64, near: f64, far: f64) -> Self {
        let f = 1.0 / (fov / 2.0).tan();
        let depth = 1.0 / (near - far);

        Self::new(
            f / aspect, 0.0, 0.0, 0.0,
            0.0, f, 0.0, 0.0,
            0.0, 0.0, (near + far) * depth, 2.0 * near * far * depth,
            0.0, 0.0, -1.0, 0.0,
        )
    }

    pub fn multiply(&self, other: &Self) -> Self {
        let mut result = Self::identity();
        for i in 0..4 {
            for j in 0..4 {
                result.m[i][j] =
                    self.m[i][0] * other.m[0][j] +
                    self.m[i][1] * other.m[1][j] +
                    self.m[i][2] * other.m[2][j] +
                    self.m[i][3] * other.m[3][j];
            }
        }
        result
    }

    pub fn transform_point(&self, v: &Vector3) -> Vector3 {
        let x = self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z + self.m[0][3];
        let y = self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z + self.m[1][3];
        let z = self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z + self.m[2][3];
        let w = self.m[3][0] * v.x + self.m[3][1] * v.y + self.m[3][2] * v.z + self.m[3][3];

        if w != 0.0 {
            Vector3::new(x / w, y / w, z / w)
        } else {
            Vector3::new(x, y, z)
        }
    }

    pub fn to_matrix3(&self) -> Matrix3 {
        Matrix3::new(
            self.m[0][0], self.m[0][1], self.m[0][2],
            self.m[1][0], self.m[1][1], self.m[1][2],
            self.m[2][0], self.m[2][1], self.m[2][2],
        )
    }
}

impl Mul for Matrix4 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        self.multiply(&other)
    }
}
