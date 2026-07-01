pub struct Metrics {
    pub elapsed: f64,
    pub min_distance: f64,
    pub max_closing_speed: f64,
    pub capture_time: Option<f64>,
}

impl Metrics {
    pub fn new() -> Self {
        Self {
            elapsed: 0.0,
            min_distance: f64::INFINITY,
            max_closing_speed: 0.0,
            capture_time: None,
        }
    }

    pub fn update(&mut self, distance: f64, closing_speed: f64, dt: f64) {
        self.elapsed += dt;
        if distance < self.min_distance {
            self.min_distance = distance;
        }
        if closing_speed > self.max_closing_speed {
            self.max_closing_speed = closing_speed;
        }
    }
}

impl Default for Metrics {
    fn default() -> Self {
        Self::new()
    }
}
