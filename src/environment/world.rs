use crate::core::math::uniform_random;
use crate::core::vector::Vector3;
use crate::environment::obstacles::Obstacle;
use crate::utils::config;

pub struct World {
    pub obstacles: Vec<Obstacle>,
    pub time: f64,
}

impl World {
    pub fn scattered(count: usize, keep_clear: &[Vector3]) -> Self {
        let mut obstacles = Vec::with_capacity(count);
        let mut attempts = 0;

        while obstacles.len() < count && attempts < count * 40 {
            attempts += 1;
            let radius = uniform_random(14.0, 30.0);
            let center = Vector3::new(
                uniform_random(-config::WORLD_BOUND * 0.7, config::WORLD_BOUND * 0.7),
                uniform_random(-config::WORLD_BOUND * 0.7, config::WORLD_BOUND * 0.7),
                uniform_random(config::WORLD_FLOOR + radius, config::WORLD_CEILING - radius),
            );
            let candidate = Obstacle::new(center, radius);

            let clear_of_spawns = keep_clear
                .iter()
                .all(|&p| candidate.surface_distance(p) > 45.0);
            let clear_of_others = obstacles
                .iter()
                .all(|o: &Obstacle| center.distance_to(o.center) > radius + o.radius + 12.0);

            if clear_of_spawns && clear_of_others {
                obstacles.push(candidate);
            }
        }

        Self { obstacles, time: 0.0 }
    }

    pub fn tick(&mut self, dt: f64) {
        self.time += dt;
    }

    pub fn nearest_obstacle_distance(&self, point: Vector3) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.surface_distance(point))
            .fold(f64::INFINITY, f64::min)
    }
}
