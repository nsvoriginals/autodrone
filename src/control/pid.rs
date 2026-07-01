use crate::core::math;
use crate::core::vector::Vector3;
use crate::utils::config;

pub struct AdaptivePid {
    kp_base: f64,
    ki_base: f64,
    kd_base: f64,

    pub kp: f64,
    pub ki: f64,
    pub kd: f64,

    integral: Vector3,
    prev_error: Vector3,
    initialised: bool,
}

impl AdaptivePid {
    pub fn new() -> Self {
        Self {
            kp_base: config::PID_KP,
            ki_base: config::PID_KI,
            kd_base: config::PID_KD,
            kp: config::PID_KP,
            ki: config::PID_KI,
            kd: config::PID_KD,
            integral: Vector3::zeros(),
            prev_error: Vector3::zeros(),
            initialised: false,
        }
    }

    pub fn schedule(&mut self, speed_ratio: f64, distance: f64, evasiveness: f64) {
        let mut kp_scale = 1.0 + 0.5 * speed_ratio;
        let kd_scale = 1.0 - 0.3 * speed_ratio;

        let ki_scale = if distance < config::INTERCEPT_RADIUS { 2.0 } else { 1.0 };

        if evasiveness > 0.7 {
            kp_scale *= 1.5;
        }

        self.kp = self.kp_base * kp_scale;
        self.ki = self.ki_base * ki_scale;
        self.kd = self.kd_base * kd_scale;
    }

    pub fn update(&mut self, error: Vector3, dt: f64) -> Vector3 {
        let derivative = if self.initialised && dt > math::EPSILON {
            (error - self.prev_error) / dt
        } else {
            Vector3::zeros()
        };

        self.integral += error * dt;
        self.integral = self.integral.clamp_magnitude(config::PID_INTEGRAL_LIMIT);

        self.prev_error = error;
        self.initialised = true;

        error * self.kp + self.integral * self.ki + derivative * self.kd
    }

    pub fn reset(&mut self) {
        self.integral = Vector3::zeros();
        self.prev_error = Vector3::zeros();
        self.initialised = false;
    }
}

impl Default for AdaptivePid {
    fn default() -> Self {
        Self::new()
    }
}
