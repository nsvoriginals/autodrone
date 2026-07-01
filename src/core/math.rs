pub const PI: f64 = std::f64::consts::PI;
pub const TAU: f64 = 2.0 * PI;
pub const DEG_TO_RAD: f64 = PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / PI;
pub const EPSILON: f64 = 1e-10;
pub const GRAVITY: f64 = 9.80665;
pub const AIR_DENSITY: f64 = 1.225;
pub const SPEED_OF_SOUND: f64 = 343.0;

pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.clamp(min, max)
}

pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

pub fn inverse_lerp(a: f64, b: f64, value: f64) -> f64 {
    (value - a) / (b - a)
}

pub fn remap(value: f64, from_min: f64, from_max: f64, to_min: f64, to_max: f64) -> f64 {
    let t = inverse_lerp(from_min, from_max, value);
    lerp(to_min, to_max, t)
}

pub fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn sigmoid(x: f64) -> f64 {
    1.0 / (1.0 + (-x).exp())
}

pub fn sign(x: f64) -> f64 {
    if x > 0.0 { 1.0 } else if x < 0.0 { -1.0 } else { 0.0 }
}

pub fn approx_eq(a: f64, b: f64, epsilon: f64) -> bool {
    (a - b).abs() < epsilon
}

pub fn safe_div(numerator: f64, denominator: f64) -> f64 {
    if denominator.abs() < EPSILON {
        0.0
    } else {
        numerator / denominator
    }
}

pub fn derivative(current: f64, previous: f64, dt: f64) -> f64 {
    if dt < EPSILON {
        0.0
    } else {
        (current - previous) / dt
    }
}

pub fn integral(current: f64, integral: f64, dt: f64) -> f64 {
    integral + current * dt
}

pub fn gaussian_pdf(x: f64, mean: f64, std_dev: f64) -> f64 {
    let z = (x - mean) / std_dev;
    (-0.5 * z * z).exp() / (std_dev * (2.0 * PI).sqrt())
}

pub fn degrees(radians: f64) -> f64 {
    radians * RAD_TO_DEG
}

pub fn radians(degrees: f64) -> f64 {
    degrees * DEG_TO_RAD
}

pub fn angle_normalize(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI { a -= TAU; }
    while a < -PI { a += TAU; }
    a
}

pub fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = b - a;
    angle_normalize(diff)
}

pub fn gaussian_random(mean: f64, std_dev: f64) -> f64 {
    use rand::RngExt;
    let mut rng = rand::rng();
    let u1: f64 = rng.random_range(f64::MIN_POSITIVE..1.0);
    let u2: f64 = rng.random();
    let z = (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos();
    mean + std_dev * z
}

pub fn uniform_random(min: f64, max: f64) -> f64 {
    use rand::RngExt;
    let mut rng = rand::rng();
    rng.random_range(min..max)
}

pub fn lerp_angle(a: f64, b: f64, t: f64) -> f64 {
    let diff = angle_diff(a, b);
    a + diff * t
}

pub fn smooth_angle(current: f64, target: f64, speed: f64, dt: f64) -> f64 {
    let diff = angle_diff(current, target);
    let step = diff.signum() * (diff.abs() * speed * dt).min(diff.abs());
    current + step
}
