use crate::utils::config;

pub struct SpeedController {
    min_speed: f64,
    max_speed: f64,

    pub proximity_factor: f64,
    pub evasive_factor: f64,
    pub obstacle_factor: f64,
    pub desired_speed: f64,
}

impl SpeedController {
    pub fn new() -> Self {
        Self {
            min_speed: config::DRONE_MIN_SPEED,
            max_speed: config::DRONE_MAX_SPEED,
            proximity_factor: 1.0,
            evasive_factor: 1.0,
            obstacle_factor: 1.0,
            desired_speed: config::DRONE_MIN_SPEED,
        }
    }

    pub fn command(
        &mut self,
        target_speed: f64,
        distance: f64,
        evasiveness: f64,
        nearest_obstacle: f64,
    ) -> f64 {
        let base = (target_speed * config::INTERCEPT_SPEED_MARGIN).max(self.min_speed);

        self.proximity_factor = if distance < config::INTERCEPT_RADIUS {
            1.0 + config::PROXIMITY_BOOST * (1.0 - distance / config::INTERCEPT_RADIUS)
        } else {
            1.0
        };

        self.evasive_factor = 1.0 + config::EVASIVE_BOOST * evasiveness;

        self.obstacle_factor = if nearest_obstacle < config::OBSTACLE_SAFETY_RADIUS {
            let t = (config::OBSTACLE_SAFETY_RADIUS - nearest_obstacle.max(0.0))
                / config::OBSTACLE_SAFETY_RADIUS;
            (1.0 - 0.5 * t).max(0.35)
        } else {
            1.0
        };

        let desired =
            base * self.proximity_factor * self.evasive_factor * self.obstacle_factor;
        self.desired_speed = desired.clamp(self.min_speed, self.max_speed);
        self.desired_speed
    }
}

impl Default for SpeedController {
    fn default() -> Self {
        Self::new()
    }
}
