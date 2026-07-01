use std::collections::VecDeque;

use crate::control::guidance;
use crate::control::pid::AdaptivePid;
use crate::control::speed::SpeedController;
use crate::core::math::gaussian_random;
use crate::core::vector::Vector3;
use crate::engine::metrics::Metrics;
use crate::engine::scenario::Scenario;
use crate::environment::world::World;
use crate::estimation::tracker::TargetTracker;
use crate::planning::potential;
use crate::target::target::Target;
use crate::utils::config;
use crate::vehicle::drone::Drone;
use crate::vehicle::state_machine::FlightPhase;

const TRAIL_LEN: usize = 220;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Running,
    Captured,
}

#[derive(Debug, Clone, Copy)]
pub struct Telemetry {
    pub true_distance: f64,
    pub est_distance: f64,
    pub closing_speed: f64,
    pub time_to_intercept: f64,
    pub los_rate: f64,
    pub desired_speed: f64,
    pub blend: f64,
    pub pn_accel: f64,
    pub steering: f64,
    pub nearest_obstacle: f64,
    pub aim_point: Vector3,
    pub measurement: Vector3,
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            true_distance: 0.0,
            est_distance: 0.0,
            closing_speed: 0.0,
            time_to_intercept: 0.0,
            los_rate: 0.0,
            desired_speed: 0.0,
            blend: 0.0,
            pn_accel: 0.0,
            steering: 0.0,
            nearest_obstacle: f64::INFINITY,
            aim_point: Vector3::zeros(),
            measurement: Vector3::zeros(),
        }
    }
}

pub struct Simulation {
    pub world: World,
    pub drone: Drone,
    pub target: Target,
    pub tracker: TargetTracker,
    pub speed_ctrl: SpeedController,
    pub pid: AdaptivePid,
    pub metrics: Metrics,
    pub telemetry: Telemetry,
    pub status: Status,
    pub drone_trail: VecDeque<Vector3>,
    pub target_trail: VecDeque<Vector3>,
}

impl Simulation {
    pub fn new() -> Self {
        let scenario = Scenario::random_interception();
        let world = World::scattered(
            scenario.obstacle_count,
            &[scenario.drone_spawn, scenario.target_spawn],
        );

        Self {
            world,
            drone: Drone::new(scenario.drone_spawn),
            target: Target::new(scenario.target_spawn),
            tracker: TargetTracker::new(),
            speed_ctrl: SpeedController::new(),
            pid: AdaptivePid::new(),
            metrics: Metrics::new(),
            telemetry: Telemetry::default(),
            status: Status::Running,
            drone_trail: VecDeque::with_capacity(TRAIL_LEN),
            target_trail: VecDeque::with_capacity(TRAIL_LEN),
        }
    }

    pub fn step(&mut self, dt: f64) {
        if self.status != Status::Running {
            return;
        }
        self.world.tick(dt);

        self.target
            .step(self.drone.position(), &self.world.obstacles, dt);

        let measurement = self.noisy_measurement();
        self.tracker.update(measurement, dt);

        let drone_pos = self.drone.position();
        let est_pos = self.tracker.position();
        let est_distance = drone_pos.distance_to(est_pos);
        let true_distance = drone_pos.distance_to(self.target.position());
        let nearest_obstacle = self.world.nearest_obstacle_distance(drone_pos);
        let evasiveness = self.tracker.evasiveness();

        let desired_speed = self.speed_ctrl.command(
            self.tracker.velocity().magnitude(),
            est_distance,
            evasiveness,
            nearest_obstacle,
        );

        let sol = guidance::solve(
            drone_pos,
            self.drone.velocity(),
            est_pos,
            self.tracker.velocity(),
            self.tracker.acceleration(),
            desired_speed,
        );

        let speed_ratio = self.drone.speed() / self.drone.params.max_speed;
        self.pid.schedule(speed_ratio, est_distance, evasiveness);
        let desired_velocity = sol.desired_direction * desired_speed;
        let vel_error = desired_velocity - self.drone.velocity();
        let pid_accel = self.pid.update(vel_error, dt);

        let pn_term = sol.pn_accel * sol.blend;
        let avoid = potential::avoidance_accel(
            drone_pos,
            &self.world.obstacles,
            config::OBSTACLE_REPULSION,
            config::OBSTACLE_INFLUENCE,
        );
        let steering = pid_accel + pn_term + avoid;

        self.drone.step(steering, dt);
        self.drone.phase = self
            .drone
            .phase
            .next(true_distance, self.tracker.has_track());

        push_trail(&mut self.drone_trail, self.drone.position());
        push_trail(&mut self.target_trail, self.target.position());
        self.metrics.update(true_distance, sol.closing_speed, dt);

        self.telemetry = Telemetry {
            true_distance,
            est_distance,
            closing_speed: sol.closing_speed,
            time_to_intercept: sol.time_to_intercept,
            los_rate: sol.los_rate,
            desired_speed,
            blend: sol.blend,
            pn_accel: pn_term.magnitude(),
            steering: steering.magnitude(),
            nearest_obstacle,
            aim_point: sol.aim_point,
            measurement,
        };

        if true_distance <= config::CAPTURE_RADIUS {
            self.status = Status::Captured;
            self.drone.phase = FlightPhase::Captured;
            self.metrics.capture_time = Some(self.metrics.elapsed);
        }
    }

    fn noisy_measurement(&self) -> Vector3 {
        let p = self.target.position();
        let n = config::MEASUREMENT_NOISE_STD;
        Vector3::new(
            p.x + gaussian_random(0.0, n),
            p.y + gaussian_random(0.0, n),
            p.z + gaussian_random(0.0, n),
        )
    }
}

impl Default for Simulation {
    fn default() -> Self {
        Self::new()
    }
}

fn push_trail(trail: &mut VecDeque<Vector3>, point: Vector3) {
    if trail.len() == TRAIL_LEN {
        trail.pop_front();
    }
    trail.push_back(point);
}
