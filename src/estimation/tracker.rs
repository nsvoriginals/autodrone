use std::collections::VecDeque;

use crate::core::math;
use crate::core::vector::Vector3;
use crate::estimation::kalman::KalmanFilter;
use crate::utils::config;

pub struct TargetTracker {
    filter: KalmanFilter,
    pub history: VecDeque<Vector3>,
    updates: usize,
    evasiveness: f64,
}

impl TargetTracker {
    pub fn new() -> Self {
        Self {
            filter: KalmanFilter::new(),
            history: VecDeque::with_capacity(config::TRACK_HISTORY),
            updates: 0,
            evasiveness: 0.0,
        }
    }

    pub fn update(&mut self, measurement: Vector3, dt: f64) {
        self.filter.predict(dt);
        self.filter.update(measurement);
        self.updates += 1;

        let pos = self.filter.position();
        if self.history.len() == config::TRACK_HISTORY {
            self.history.pop_front();
        }
        self.history.push_back(pos);

        let accel = self.filter.acceleration();
        let vel = self.filter.velocity();
        let lateral = if vel.magnitude() > 1e-3 {
            accel.reject_from(vel).magnitude()
        } else {
            accel.magnitude()
        };
        let raw = math::sigmoid((lateral - config::EVASIVE_ACCEL_SCALE) / (config::EVASIVE_ACCEL_SCALE * 0.5));
        self.evasiveness = math::lerp(self.evasiveness, raw, 0.1);
    }

    pub fn position(&self) -> Vector3 {
        self.filter.position()
    }

    pub fn velocity(&self) -> Vector3 {
        self.filter.velocity()
    }

    pub fn acceleration(&self) -> Vector3 {
        self.filter.acceleration()
    }

    pub fn predict_future(&self, t: f64) -> Vector3 {
        self.filter.predict_position(t)
    }

    pub fn uncertainty(&self) -> f64 {
        self.filter.position_uncertainty()
    }

    pub fn innovation(&self) -> f64 {
        self.filter.last_innovation
    }

    pub fn evasiveness(&self) -> f64 {
        self.evasiveness.clamp(0.0, 1.0)
    }

    pub fn has_track(&self) -> bool {
        self.filter.is_initialised() && self.updates > 6
    }
}

impl Default for TargetTracker {
    fn default() -> Self {
        Self::new()
    }
}
