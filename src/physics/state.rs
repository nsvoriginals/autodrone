use crate::core::math;
use crate::core::quaternion::Quaternion;
use crate::core::vector::Vector3;
use crate::physics::integrator::semi_implicit_euler;

#[derive(Debug, Clone, Copy)]
pub struct Body {
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
    pub orientation: Quaternion,
}

impl Body {
    pub fn new(position: Vector3, velocity: Vector3) -> Self {
        Self {
            position,
            velocity,
            acceleration: Vector3::zeros(),
            orientation: Quaternion::identity(),
        }
    }

    pub fn speed(&self) -> f64 {
        self.velocity.magnitude()
    }

    pub fn heading(&self) -> f64 {
        self.velocity.y.atan2(self.velocity.x)
    }

    pub fn flight_path_angle(&self) -> f64 {
        let horizontal = (self.velocity.x * self.velocity.x
            + self.velocity.y * self.velocity.y)
            .sqrt();
        self.velocity.z.atan2(horizontal)
    }

    pub fn integrate(&mut self, acceleration: Vector3, dt: f64, max_speed: f64) {
        let (new_pos, mut new_vel) =
            semi_implicit_euler(self.position, self.velocity, acceleration, dt);
        new_vel = new_vel.clamp_magnitude(max_speed);

        self.position = new_pos;
        self.velocity = new_vel;
        self.acceleration = acceleration;
        self.update_orientation();
    }

    fn update_orientation(&mut self) {
        if self.speed() < math::EPSILON {
            return;
        }
        let yaw = self.heading();
        let pitch = self.flight_path_angle();

        let forward = self.velocity.normalize();
        let lateral = self.acceleration.reject_from(forward);
        let up = Vector3::new(0.0, 0.0, 1.0);
        let side = forward.cross(up);
        let bank = math::clamp(lateral.dot(side) * 0.03, -0.7, 0.7);

        self.orientation = Quaternion::from_euler(bank, -pitch, yaw);
    }
}
