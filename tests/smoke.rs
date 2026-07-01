use auto_drone::engine::orchestrator::{Simulation, Status};
use auto_drone::utils::config;

fn run_one(max_seconds: f64) -> (Status, f64) {
    let mut sim = Simulation::new();
    let steps = (max_seconds / config::DT) as usize;

    for _ in 0..steps {
        sim.step(config::DT);

        let d = sim.drone.position();
        let t = sim.target.position();
        assert!(d.is_finite(), "drone position went non-finite: {d:?}");
        assert!(t.is_finite(), "target position went non-finite: {t:?}");
        assert!(
            sim.tracker.position().is_finite(),
            "kalman estimate went non-finite"
        );

        if sim.status != Status::Running {
            break;
        }
    }

    (sim.status, sim.metrics.elapsed)
}

#[test]
fn stays_numerically_stable() {
    let (_status, elapsed) = run_one(120.0);
    assert!(elapsed > 0.0);
}

#[test]
fn intercepts_most_of_the_time() {
    let trials = 20;
    let mut captures = 0;
    let mut total_capture_time = 0.0;

    for _ in 0..trials {
        let (status, elapsed) = run_one(90.0);
        if status == Status::Captured {
            captures += 1;
            total_capture_time += elapsed;
        }
    }

    assert!(
        captures >= (trials * 3) / 4,
        "only {captures}/{trials} interceptions — guidance is underperforming"
    );

    let avg = total_capture_time / captures as f64;
    println!("captured {captures}/{trials}, avg capture time {avg:.1}s");
}
