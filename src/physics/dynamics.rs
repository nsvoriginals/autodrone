use crate::core::vector::Vector3;
use crate::utils::config;

pub fn linear_drag(velocity: Vector3, k: f64) -> Vector3 {
    velocity * -k
}

pub fn boundary_repulsion(position: Vector3, strength: f64) -> Vector3 {
    let mut a = Vector3::zeros();
    let margin = config::BOUNDARY_MARGIN;

    let ramp = |dist: f64| {
        if dist < margin {
            let t = (margin - dist) / margin;
            t * t
        } else {
            0.0
        }
    };

    a.x += ramp(config::WORLD_BOUND - position.x) * -strength;
    a.x += ramp(position.x + config::WORLD_BOUND) * strength;
    a.y += ramp(config::WORLD_BOUND - position.y) * -strength;
    a.y += ramp(position.y + config::WORLD_BOUND) * strength;
    a.z += ramp(config::WORLD_CEILING - position.z) * -strength;
    a.z += ramp(position.z - config::WORLD_FLOOR) * strength;

    a
}
