use crate::core::math::uniform_random;
use crate::core::vector::Vector3;
use crate::utils::config;

pub struct Scenario {
    pub drone_spawn: Vector3,
    pub target_spawn: Vector3,
    pub obstacle_count: usize,
}

impl Scenario {
    pub fn random_interception() -> Self {
        let span = config::WORLD_BOUND * 0.8;
        let alt = || uniform_random(config::WORLD_FLOOR + 30.0, config::WORLD_CEILING - 40.0);

        let drone_spawn = Vector3::new(-span, uniform_random(-span, span), alt());
        let target_spawn = Vector3::new(span, uniform_random(-span, span), alt());

        Self {
            drone_spawn,
            target_spawn,
            obstacle_count: 7,
        }
    }
}
