use crate::core::math::uniform_random;
use crate::core::vector::Vector3;
use crate::utils::config;

pub fn random_unit_vector() -> Vector3 {
    loop {
        let x1 = uniform_random(-1.0, 1.0);
        let x2 = uniform_random(-1.0, 1.0);
        let s = x1 * x1 + x2 * x2;
        if s < 1.0 {
            let f = 2.0 * (1.0 - s).sqrt();
            return Vector3::new(x1 * f, x2 * f, 1.0 - 2.0 * s);
        }
    }
}

pub fn random_jink() -> Vector3 {
    let mut v = random_unit_vector();
    v.z *= 0.5;
    v.normalize()
}

pub fn next_jink_interval() -> f64 {
    uniform_random(config::TARGET_JINK_MIN_INTERVAL, config::TARGET_JINK_MAX_INTERVAL)
}

pub fn patrol_route(count: usize) -> Vec<Vector3> {
    let span = config::WORLD_BOUND * 0.75;
    (0..count)
        .map(|_| {
            Vector3::new(
                uniform_random(-span, span),
                uniform_random(-span, span),
                uniform_random(config::WORLD_FLOOR + 20.0, config::WORLD_CEILING - 30.0),
            )
        })
        .collect()
}
