use crate::core::vector::Vector3;
use crate::utils::config;

pub struct Solution {
    pub aim_point: Vector3,
    pub desired_direction: Vector3,
    pub time_to_intercept: f64,
    pub los_rate: f64,
    pub closing_speed: f64,
    pub pn_accel: Vector3,
    pub blend: f64,
}

pub fn solve(
    drone_pos: Vector3,
    drone_vel: Vector3,
    tgt_pos: Vector3,
    tgt_vel: Vector3,
    tgt_accel: Vector3,
    intercept_speed: f64,
) -> Solution {
    let r = tgt_pos - drone_pos;
    let range = r.magnitude();
    let los = if range > 1e-6 { r / range } else { Vector3::new(1.0, 0.0, 0.0) };

    let tti = time_to_intercept(r, tgt_vel, intercept_speed, range);

    let aim_point = tgt_pos + tgt_vel * tti + tgt_accel * (0.5 * tti * tti);
    let desired_direction = {
        let d = aim_point - drone_pos;
        if d.magnitude() > 1e-6 { d.normalize() } else { los }
    };

    let v_rel = tgt_vel - drone_vel;
    let omega = if range > 1e-6 {
        r.cross(v_rel) / (range * range)
    } else {
        Vector3::zeros()
    };
    let los_rate = omega.magnitude();
    let closing_speed = -v_rel.dot(los);
    let pn_accel = omega.cross(los) * (config::PN_GAIN * closing_speed.max(0.0));

    let ratio = range / config::INTERCEPT_RADIUS;
    let blend = if ratio > 3.0 {
        0.85
    } else if ratio > 1.5 {
        0.5
    } else {
        0.2
    };

    Solution {
        aim_point,
        desired_direction,
        time_to_intercept: tti,
        los_rate,
        closing_speed,
        pn_accel,
        blend,
    }
}

fn time_to_intercept(r: Vector3, tgt_vel: Vector3, s: f64, range: f64) -> f64 {
    let a = tgt_vel.dot(tgt_vel) - s * s;
    let b = 2.0 * r.dot(tgt_vel);
    let c = r.dot(r);

    let fallback = range / s.max(1e-3);

    if a.abs() < 1e-6 {
        if b.abs() < 1e-9 {
            return fallback;
        }
        let t = -c / b;
        return if t > 0.0 { t } else { fallback };
    }

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return fallback;
    }
    let sqrt_disc = disc.sqrt();
    let t1 = (-b + sqrt_disc) / (2.0 * a);
    let t2 = (-b - sqrt_disc) / (2.0 * a);

    let best = [t1, t2]
        .into_iter()
        .filter(|&t| t > 1e-3)
        .fold(f64::INFINITY, f64::min);

    if best.is_finite() { best.min(60.0) } else { fallback }
}
