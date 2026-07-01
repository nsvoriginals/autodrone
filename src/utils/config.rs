pub const DT: f64 = 1.0 / 60.0;

pub const WORLD_BOUND: f64 = 250.0;
pub const WORLD_CEILING: f64 = 200.0;
pub const WORLD_FLOOR: f64 = 5.0;
pub const BOUNDARY_MARGIN: f64 = 30.0;

pub const DRONE_MASS: f64 = 2.0;
pub const DRONE_MIN_SPEED: f64 = 14.0;
pub const DRONE_MAX_SPEED: f64 = 56.0;
pub const DRONE_MAX_ACCEL: f64 = 40.0;
pub const DRONE_DRAG: f64 = 0.15;
pub const DRONE_BATTERY_DRAIN: f64 = 0.0016;

pub const TARGET_MIN_SPEED: f64 = 10.0;
pub const TARGET_MAX_SPEED: f64 = 44.0;
pub const TARGET_MAX_ACCEL: f64 = 34.0;
pub const TARGET_DRAG: f64 = 0.12;

pub const TARGET_DETECTION_RADIUS: f64 = 90.0;
pub const TARGET_JINK_STRENGTH: f64 = 0.85;
pub const TARGET_JINK_MIN_INTERVAL: f64 = 0.6;
pub const TARGET_JINK_MAX_INTERVAL: f64 = 1.8;
pub const TARGET_PANIC_DECAY: f64 = 0.985;

pub const INTERCEPT_RADIUS: f64 = 60.0;
pub const CAPTURE_RADIUS: f64 = 4.0;
pub const INTERCEPT_SPEED_MARGIN: f64 = 1.22;
pub const PN_GAIN: f64 = 4.0;
pub const PROXIMITY_BOOST: f64 = 0.6;
pub const EVASIVE_BOOST: f64 = 0.45;

pub const OBSTACLE_INFLUENCE: f64 = 35.0;
pub const OBSTACLE_REPULSION: f64 = 900.0;
pub const OBSTACLE_SAFETY_RADIUS: f64 = 30.0;

pub const PID_KP: f64 = 3.4;
pub const PID_KI: f64 = 0.7;
pub const PID_KD: f64 = 0.35;
pub const PID_INTEGRAL_LIMIT: f64 = 30.0;

pub const MEASUREMENT_NOISE_STD: f64 = 1.6;
pub const PROCESS_NOISE: f64 = 12.0;
pub const TRACK_HISTORY: usize = 48;
pub const EVASIVE_ACCEL_SCALE: f64 = 16.0;
