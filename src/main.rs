use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind};

use auto_drone::display::renderer;
use auto_drone::engine::orchestrator::{Simulation, Status};
use auto_drone::utils::config;

const MAX_STEPS_PER_FRAME: u32 = 8;
const FRAME: Duration = Duration::from_millis(16);

fn main() -> std::io::Result<()> {
    let mut terminal = ratatui::init();

    let mut sim = Simulation::new();
    let mut paused = false;
    let mut fps = 60.0_f64;
    let mut last = Instant::now();
    let mut accumulator = 0.0_f64;

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
                        }
                        KeyCode::Char(' ') => paused = !paused,
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

        if !paused && sim.status == Status::Running {
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

        terminal.draw(|f| renderer::draw(f, &sim, fps))?;

        let elapsed = last.elapsed();
        if elapsed < FRAME {
            std::thread::sleep(FRAME - elapsed);
        }
    }

    ratatui::restore();
    Ok(())
}
