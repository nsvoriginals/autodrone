use crate::core::vector::Vector3;

pub fn semi_implicit_euler(
    position: Vector3,
    velocity: Vector3,
    acceleration: Vector3,
    dt: f64,
) -> (Vector3, Vector3) {
    let new_velocity = velocity + acceleration * dt;
    let new_position = position + new_velocity * dt;
    (new_position, new_velocity)
}
