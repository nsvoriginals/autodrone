use crate::utils::config;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightPhase {
    Search,
    Pursue,
    Terminal,
    Captured,
}

impl FlightPhase {
    pub fn label(&self) -> &'static str {
        match self {
            FlightPhase::Search => "SEARCH",
            FlightPhase::Pursue => "PURSUE",
            FlightPhase::Terminal => "TERMINAL",
            FlightPhase::Captured => "CAPTURED",
        }
    }

    pub fn next(self, distance: f64, has_track: bool) -> FlightPhase {
        if self == FlightPhase::Captured {
            return FlightPhase::Captured;
        }
        if distance <= config::CAPTURE_RADIUS {
            return FlightPhase::Captured;
        }
        if !has_track {
            return FlightPhase::Search;
        }
        if distance <= config::INTERCEPT_RADIUS {
            FlightPhase::Terminal
        } else {
            FlightPhase::Pursue
        }
    }
}
