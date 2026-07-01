use crate::core::vector::Vector3;
use crate::environment::obstacles::Obstacle;

pub fn avoidance_accel(
    position: Vector3,
    obstacles: &[Obstacle],
    strength: f64,
    influence: f64,
) -> Vector3 {
    let mut accel = Vector3::zeros();

    for obs in obstacles {
        let d = obs.surface_distance(position).max(0.5);
        if d >= influence {
            continue;
        }
        let dir = obs.outward_dir(position);
        let gradient = (1.0 / d - 1.0 / influence) * (1.0 / (d * d));
        accel += dir * (strength * gradient);
    }

    accel
}
