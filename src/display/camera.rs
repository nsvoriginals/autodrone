use crate::core::vector::Vector3;
use crate::utils::config;

#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub center: Vector3,
    pub half_span: f64,
    pub yaw: f64,
    pub follow: bool,
}

impl Camera {
    pub fn new() -> Self {
        Self {
            center: Vector3::zeros(),
            half_span: 150.0,
            yaw: 0.0,
            follow: true,
        }
    }

    pub fn update(&mut self, drone: Vector3, target: Vector3) {
        if self.follow {
            self.center = (drone + target) * 0.5;
            let gap = drone.distance_to(target);
            self.half_span = (gap * 0.75 + 35.0).clamp(55.0, config::WORLD_BOUND * 1.2);
        }
    }

    pub fn toggle_follow(&mut self) {
        self.follow = !self.follow;
    }

    pub fn zoom(&mut self, factor: f64) {
        self.follow = false;
        self.half_span = (self.half_span * factor).clamp(25.0, config::WORLD_BOUND * 1.6);
    }

    pub fn rotate(&mut self, delta: f64) {
        self.yaw += delta;
    }

    pub fn pan(&mut self, local_dx: f64, local_dy: f64) {
        self.follow = false;
        let step = self.half_span * 0.15;
        let (s, c) = self.yaw.sin_cos();
        let dx = local_dx * step;
        let dy = local_dy * step;
        self.center.x += dx * c - dy * s;
        self.center.y += dx * s + dy * c;
    }

    pub fn mode_label(&self) -> &'static str {
        if self.follow { "FOLLOW" } else { "FREE" }
    }

    pub fn top(&self, p: Vector3) -> (f64, f64) {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let (s, c) = self.yaw.sin_cos();
        (dx * c + dy * s, -dx * s + dy * c)
    }

    pub fn side(&self, p: Vector3) -> (f64, f64) {
        let (hx, _) = self.top(p);
        (hx, p.z)
    }

    pub fn heading_local(&self, world_heading: f64) -> f64 {
        world_heading - self.yaw
    }

    pub fn glyph_size(&self) -> f64 {
        self.half_span * 0.055
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self::new()
    }
}
