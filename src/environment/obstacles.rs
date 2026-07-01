use crate::core::vector::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct Obstacle {
    pub center: Vector3,
    pub radius: f64,
}

impl Obstacle {
    pub fn new(center: Vector3, radius: f64) -> Self {
        Self { center, radius }
    }

    pub fn surface_distance(&self, point: Vector3) -> f64 {
        self.center.distance_to(point) - self.radius
    }

    pub fn outward_dir(&self, point: Vector3) -> Vector3 {
        let d = point - self.center;
        if d.magnitude() < 1e-9 {
            Vector3::new(0.0, 0.0, 1.0)
        } else {
            d.normalize()
        }
    }
}
