use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, Div, Neg};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vector3 { x, y, z }
    }

    pub const fn zeros() -> Self {
        Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn magnitude(&self) -> f64 {
        self.magnitude_sq().sqrt()
    }

    pub fn magnitude_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn dot(&self, o: Vector3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(&self, o: Vector3) -> Vector3 {
        Vector3 {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }

    pub fn normalize(&self) -> Vector3 {
        let mag = self.magnitude();

        if mag == 0.0 {
            Vector3::zeros()
        } else {
            Vector3 {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        }
    }

    pub fn distance_to(&self, o: Vector3) -> f64 {
        (*self - o).magnitude()
    }

    pub fn distance_sq(&self, o: Vector3) -> f64 {
        (*self - o).magnitude_sq()
    }

    pub fn lerp(&self, o: Vector3, t: f64) -> Vector3 {
        *self + (o - *self) * t
    }

    pub fn clamp_magnitude(&self, max: f64) -> Vector3 {
        let mag = self.magnitude();
        if mag > max && mag > 0.0 {
            *self * (max / mag)
        } else {
            *self
        }
    }

    pub fn scale(&self, s: f64) -> Vector3 {
        *self * s
    }

    pub fn project_onto(&self, onto: Vector3) -> Vector3 {
        let denom = onto.magnitude_sq();
        if denom < 1e-12 {
            Vector3::zeros()
        } else {
            onto * (self.dot(onto) / denom)
        }
    }

    pub fn reject_from(&self, onto: Vector3) -> Vector3 {
        *self - self.project_onto(onto)
    }

    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vector3 {
    type Output = Vector3;
    fn add(self, o: Vector3) -> Vector3 {
        Vector3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl AddAssign for Vector3 {
    fn add_assign(&mut self, o: Vector3) {
        self.x += o.x;
        self.y += o.y;
        self.z += o.z;
    }
}

impl Sub for Vector3 {
    type Output = Vector3;
    fn sub(self, o: Vector3) -> Vector3 {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl SubAssign for Vector3 {
    fn sub_assign(&mut self, o: Vector3) {
        self.x -= o.x;
        self.y -= o.y;
        self.z -= o.z;
    }
}

impl Mul<f64> for Vector3 {
    type Output = Vector3;
    fn mul(self, s: f64) -> Vector3 {
        Vector3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vector3> for f64 {
    type Output = Vector3;
    fn mul(self, v: Vector3) -> Vector3 {
        v * self
    }
}

impl Div<f64> for Vector3 {
    type Output = Vector3;
    fn div(self, s: f64) -> Vector3 {
        Vector3::new(self.x / s, self.y / s, self.z / s)
    }
}

impl Neg for Vector3 {
    type Output = Vector3;
    fn neg(self) -> Vector3 {
        Vector3::new(-self.x, -self.y, -self.z)
    }
}

impl std::fmt::Display for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.2}, {:.2}, {:.2})", self.x, self.y, self.z)
    }
}
