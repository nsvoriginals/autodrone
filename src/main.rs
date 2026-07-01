use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind};

use auto_drone::display::camera::Camera;
use auto_drone::display::renderer;
use auto_drone::engine::orchestrator::{Simulation, Status};
use auto_drone::utils::config;

const MAX_STEPS_PER_FRAME: u32 = 8;
const FRAME: Duration = Duration::from_millis(16);
const ROTATE_STEP: f64 = 0.12;
const RESTART_AFTER: f64 = 2.5;

fn main() -> std::io::Result<()> {
    let mut terminal = ratatui::init();

    let mut sim = Simulation::new();
    let mut cam = Camera::new();
    let mut paused = false;
    let mut fps = 60.0_f64;
    let mut last = Instant::now();
    let mut accumulator = 0.0_f64;
    let mut capture_hold = 0.0_f64;

    loop {
        if event::poll(Duration::from_millis(1))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => break,
                        KeyCode::Char('r') => {
                            sim = Simulation::new();
                            paused = false;
                            accumulator = 0.0;
                            capture_hold = 0.0;
                        }
                        KeyCode::Char(' ') => paused = !paused,
                        KeyCode::Char('f') => cam.toggle_follow(),
                        KeyCode::Up => cam.pan(0.0, 1.0),
                        KeyCode::Down => cam.pan(0.0, -1.0),
                        KeyCode::Left => cam.pan(-1.0, 0.0),
                        KeyCode::Right => cam.pan(1.0, 0.0),
                        KeyCode::Char('+') | KeyCode::Char('=') => cam.zoom(0.85),
                        KeyCode::Char('-') | KeyCode::Char('_') => cam.zoom(1.18),
                        KeyCode::Char('<') | KeyCode::Char(',') => cam.rotate(-ROTATE_STEP),
                        KeyCode::Char('>') | KeyCode::Char('.') => cam.rotate(ROTATE_STEP),
                        _ => {}
                    }
                }
            }
        }

        let now = Instant::now();
        let frame_time = now.duration_since(last).as_secs_f64();
        last = now;
        if frame_time > 0.0 {
            fps = fps * 0.9 + (1.0 / frame_time) * 0.1;
        }

        if !paused {
            match sim.status {
                Status::Running => {
                    accumulator += frame_time;
                    let mut steps = 0;
                    while accumulator >= config::DT && steps < MAX_STEPS_PER_FRAME {
                        sim.step(config::DT);
                        accumulator -= config::DT;
                        steps += 1;
                    }
                    if steps == MAX_STEPS_PER_FRAME {
                        accumulator = 0.0;
                    }
                }
                Status::Captured => {
                    // Hold the capture on screen briefly, then start a fresh chase.
                    capture_hold += frame_time;
                    if capture_hold >= RESTART_AFTER {
                        sim = Simulation::new();
                        capture_hold = 0.0;
                        accumulator = 0.0;
                    }
                }
            }
        }

        cam.update(sim.drone.position(), sim.target.position());
        terminal.draw(|f| renderer::draw(f, &sim, &cam, fps))?;

        let elapsed = last.elapsed();
        if elapsed < FRAME {
            std::thread::sleep(FRAME - elapsed);
        }
    }

    ratatui::restore();
    Ok(())
}
