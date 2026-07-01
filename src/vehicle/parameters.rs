use crate::utils::config;

#[derive(Debug, Clone, Copy)]
pub struct Parameters {
    pub mass: f64,
    pub min_speed: f64,
    pub max_speed: f64,
    pub max_accel: f64,
    pub drag: f64,
}

impl Parameters {
    pub fn pursuer() -> Self {
        Self {
            mass: config::DRONE_MASS,
            min_speed: config::DRONE_MIN_SPEED,
            max_speed: config::DRONE_MAX_SPEED,
            max_accel: config::DRONE_MAX_ACCEL,
            drag: config::DRONE_DRAG,
        }
    }

    pub fn evader() -> Self {
        Self {
            mass: config::DRONE_MASS,
            min_speed: config::TARGET_MIN_SPEED,
            max_speed: config::TARGET_MAX_SPEED,
            max_accel: config::TARGET_MAX_ACCEL,
            drag: config::TARGET_DRAG,
        }
    }
}
