use crate::core::vector::Vector3;
use crate::physics::dynamics::{boundary_repulsion, linear_drag};
use crate::physics::state::Body;
use crate::utils::config;
use crate::vehicle::parameters::Parameters;
use crate::vehicle::state_machine::FlightPhase;

#[derive(Debug, Clone, Copy)]
pub struct Drone {
    pub body: Body,
    pub params: Parameters,
    pub battery: f64,
    pub phase: FlightPhase,
}

impl Drone {
    pub fn new(position: Vector3) -> Self {
        Self {
            body: Body::new(position, Vector3::zeros()),
            params: Parameters::pursuer(),
            battery: 1.0,
            phase: FlightPhase::Search,
        }
    }

    pub fn position(&self) -> Vector3 {
        self.body.position
    }

    pub fn velocity(&self) -> Vector3 {
        self.body.velocity
    }

    pub fn speed(&self) -> f64 {
        self.body.speed()
    }

    pub fn step(&mut self, steering: Vector3, dt: f64) {
        let steering = steering.clamp_magnitude(self.params.max_accel);

        let drag = linear_drag(self.body.velocity, self.params.drag);
        let walls = boundary_repulsion(self.body.position, self.params.max_accel);
        let total = steering + drag + walls;

        self.body.integrate(total, dt, self.params.max_speed);

        let effort = steering.magnitude();
        self.battery =
            (self.battery - (0.15 + effort) * config::DRONE_BATTERY_DRAIN * dt).max(0.0);
    }
}
