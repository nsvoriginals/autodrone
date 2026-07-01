use nalgebra::{SMatrix, SVector};

use crate::core::vector::Vector3;
use crate::utils::config;

type State = SVector<f64, 9>;
type Covariance = SMatrix<f64, 9, 9>;
type Measurement = SVector<f64, 3>;
type MeasNoise = SMatrix<f64, 3, 3>;
type ObsModel = SMatrix<f64, 3, 9>;

pub struct KalmanFilter {
    x: State,
    p: Covariance,
    h: ObsModel,
    r: MeasNoise,
    pub last_innovation: f64,
    initialised: bool,
}

impl KalmanFilter {
    pub fn new() -> Self {
        let mut h = ObsModel::zeros();
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h[(2, 2)] = 1.0;

        let r = MeasNoise::identity()
            * (config::MEASUREMENT_NOISE_STD * config::MEASUREMENT_NOISE_STD);

        Self {
            x: State::zeros(),
            p: Covariance::identity() * 500.0,
            h,
            r,
            last_innovation: 0.0,
            initialised: false,
        }
    }

    fn transition(dt: f64) -> Covariance {
        let mut f = Covariance::identity();
        let half_dt2 = 0.5 * dt * dt;
        for i in 0..3 {
            f[(i, i + 3)] = dt;
            f[(i, i + 6)] = half_dt2;
            f[(i + 3, i + 6)] = dt;
        }
        f
    }

    fn process_noise(dt: f64) -> Covariance {
        let q = config::PROCESS_NOISE;
        let (dt2, dt3, dt4, dt5) = (dt * dt, dt.powi(3), dt.powi(4), dt.powi(5));

        let (qpp, qpv, qpa) = (dt5 / 20.0, dt4 / 8.0, dt3 / 6.0);
        let (qvv, qva) = (dt3 / 3.0, dt2 / 2.0);
        let qaa = dt;

        let mut m = Covariance::zeros();
        for i in 0..3 {
            let (p, v, a) = (i, i + 3, i + 6);
            m[(p, p)] = qpp;
            m[(p, v)] = qpv;
            m[(v, p)] = qpv;
            m[(p, a)] = qpa;
            m[(a, p)] = qpa;
            m[(v, v)] = qvv;
            m[(v, a)] = qva;
            m[(a, v)] = qva;
            m[(a, a)] = qaa;
        }
        m * q
    }

    pub fn predict(&mut self, dt: f64) {
        let f = Self::transition(dt);
        self.x = f * self.x;
        self.p = f * self.p * f.transpose() + Self::process_noise(dt);
    }

    pub fn update(&mut self, z: Vector3) {
        let z = Measurement::new(z.x, z.y, z.z);

        if !self.initialised {
            self.x[0] = z[0];
            self.x[1] = z[1];
            self.x[2] = z[2];
            self.initialised = true;
            return;
        }

        let y = z - self.h * self.x;
        let s = self.h * self.p * self.h.transpose() + self.r;
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return,
        };
        let k = self.p * self.h.transpose() * s_inv;

        self.x += k * y;
        let i = Covariance::identity();
        self.p = (i - k * self.h) * self.p;
        self.last_innovation = y.norm();
    }

    pub fn position(&self) -> Vector3 {
        Vector3::new(self.x[0], self.x[1], self.x[2])
    }

    pub fn velocity(&self) -> Vector3 {
        Vector3::new(self.x[3], self.x[4], self.x[5])
    }

    pub fn acceleration(&self) -> Vector3 {
        Vector3::new(self.x[6], self.x[7], self.x[8])
    }

    pub fn predict_position(&self, t: f64) -> Vector3 {
        self.position() + self.velocity() * t + self.acceleration() * (0.5 * t * t)
    }

    pub fn position_uncertainty(&self) -> f64 {
        (self.p[(0, 0)] + self.p[(1, 1)] + self.p[(2, 2)]).sqrt()
    }

    pub fn is_initialised(&self) -> bool {
        self.initialised
    }
}

impl Default for KalmanFilter {
    fn default() -> Self {
        Self::new()
    }
}
