use crate::core::math;
use crate::core::vector::Vector3;
use crate::environment::obstacles::Obstacle;
use crate::physics::dynamics::{boundary_repulsion, linear_drag};
use crate::physics::state::Body;
use crate::planning::potential;
use crate::target::behavior;
use crate::utils::config;
use crate::vehicle::parameters::Parameters;

pub struct Target {
    pub body: Body,
    pub params: Parameters,
    waypoints: Vec<Vector3>,
    current_wp: usize,
    pub panic_level: f64,
    jink_timer: f64,
    jink_dir: Vector3,
}

impl Target {
    pub fn new(position: Vector3) -> Self {
        let waypoints = behavior::patrol_route(6);
        Self {
            body: Body::new(position, Vector3::new(config::TARGET_MIN_SPEED, 0.0, 0.0)),
            params: Parameters::evader(),
            waypoints,
            current_wp: 0,
            panic_level: 0.0,
            jink_timer: 0.0,
            jink_dir: behavior::random_jink(),
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

    fn seek_waypoint(&mut self) -> Vector3 {
        let target = self.waypoints[self.current_wp];
        if self.body.position.distance_to(target) < 18.0 {
            self.current_wp = (self.current_wp + 1) % self.waypoints.len();
        }
        let desired = (target - self.body.position).normalize() * self.params.max_speed;
        (desired - self.body.velocity).clamp_magnitude(self.params.max_accel)
    }

    fn evade(&mut self, drone_pos: Vector3, dt: f64) -> Vector3 {
        let to_me = self.body.position - drone_pos;
        let distance = to_me.magnitude();

        if distance < config::TARGET_DETECTION_RADIUS {
            let closeness = 1.0 - distance / config::TARGET_DETECTION_RADIUS;
            self.panic_level = self.panic_level.max(closeness);
        } else {
            self.panic_level *= config::TARGET_PANIC_DECAY;
        }

        if self.panic_level < 0.02 {
            return Vector3::zeros();
        }

        self.jink_timer -= dt;
        if self.jink_timer <= 0.0 {
            self.jink_dir = behavior::random_jink();
            self.jink_timer = behavior::next_jink_interval();
        }

        let flee_dir = if distance > 1e-6 { to_me / distance } else { behavior::random_jink() };
        let perpendicular = flee_dir.cross(Vector3::new(0.0, 0.0, 1.0)).normalize();

        let run = flee_dir * 0.55;
        let brk = perpendicular * 0.45;
        let jink = self.jink_dir * config::TARGET_JINK_STRENGTH;

        (run + brk + jink) * (self.params.max_accel * self.panic_level)
    }

    pub fn step(&mut self, drone_pos: Vector3, obstacles: &[Obstacle], dt: f64) {
        let patrol = self.seek_waypoint();
        let evasion = self.evade(drone_pos, dt);

        let blend = self.panic_level.clamp(0.0, 1.0);
        let steering = patrol * (1.0 - blend) + evasion;

        let avoid = potential::avoidance_accel(
            self.body.position,
            obstacles,
            config::OBSTACLE_REPULSION,
            config::OBSTACLE_INFLUENCE,
        );
        let drag = linear_drag(self.body.velocity, self.params.drag);
        let walls = boundary_repulsion(self.body.position, self.params.max_accel);

        let total = (steering + avoid).clamp_magnitude(self.params.max_accel) + drag + walls;
        self.body.integrate(total, dt, self.params.max_speed);

        self.panic_level = math::clamp(self.panic_level, 0.0, 1.0);
    }
}
