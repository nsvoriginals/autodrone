use ratatui::layout::Rect;
use ratatui::style::Color;
use ratatui::symbols::Marker;
use ratatui::text::Span;
use ratatui::widgets::canvas::{Canvas, Circle, Line as CanvasLine, Points};
use ratatui::widgets::{Block, BorderType, Borders};
use ratatui::Frame;

use crate::engine::orchestrator::{Simulation, Status};
use crate::utils::config;

const DRONE_COLOR: Color = Color::Cyan;
const OBSTACLE_COLOR: Color = Color::Rgb(120, 90, 60);
const LOS_COLOR: Color = Color::Rgb(90, 90, 110);
const AIM_COLOR: Color = Color::Yellow;

fn target_color(sim: &Simulation) -> Color {
    let p = sim.target.panic_level.clamp(0.0, 1.0);
    let lerp = |a: u8, b: u8| (a as f64 + (b as f64 - a as f64) * p) as u8;
    Color::Rgb(lerp(80, 255), lerp(230, 90), lerp(120, 60))
}

fn block(title: &str) -> Block<'_> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(ratatui::style::Style::default().fg(Color::Rgb(70, 80, 100)))
        .title(Span::styled(
            format!(" {title} "),
            ratatui::style::Style::default().fg(Color::Rgb(150, 200, 255)),
        ))
}

pub fn draw_top_view(f: &mut Frame, area: Rect, sim: &Simulation) {
    let drone = sim.drone.position();
    let target = sim.target.position();
    let tgt_color = target_color(sim);

    let drone_trail: Vec<(f64, f64)> = sim.drone_trail.iter().map(|p| (p.x, p.y)).collect();
    let target_trail: Vec<(f64, f64)> = sim.target_trail.iter().map(|p| (p.x, p.y)).collect();

    let canvas = Canvas::default()
        .block(block("TACTICAL — TOP DOWN  (X → east,  Y → north)"))
        .marker(Marker::Braille)
        .x_bounds([-config::WORLD_BOUND, config::WORLD_BOUND])
        .y_bounds([-config::WORLD_BOUND, config::WORLD_BOUND])
        .paint(move |ctx| {
            for o in &sim.world.obstacles {
                ctx.draw(&Circle {
                    x: o.center.x,
                    y: o.center.y,
                    radius: o.radius,
                    color: OBSTACLE_COLOR,
                });
            }
            ctx.layer();

            ctx.draw(&Points { coords: &target_trail, color: Color::Rgb(40, 110, 60) });
            ctx.draw(&Points { coords: &drone_trail, color: Color::Rgb(40, 90, 120) });
            ctx.layer();

            ctx.draw(&CanvasLine {
                x1: drone.x,
                y1: drone.y,
                x2: target.x,
                y2: target.y,
                color: LOS_COLOR,
            });
            if sim.status == Status::Running {
                let aim = sim.telemetry.aim_point;
                ctx.print(aim.x, aim.y, Span::styled("+", ratatui::style::Style::default().fg(AIM_COLOR)));
            }
            ctx.layer();

            ctx.print(target.x, target.y, Span::styled("◆", ratatui::style::Style::default().fg(tgt_color)));
            ctx.print(drone.x, drone.y, Span::styled("▲", ratatui::style::Style::default().fg(DRONE_COLOR)));
        });

    f.render_widget(canvas, area);
}

pub fn draw_side_view(f: &mut Frame, area: Rect, sim: &Simulation) {
    let drone = sim.drone.position();
    let target = sim.target.position();
    let tgt_color = target_color(sim);

    let drone_trail: Vec<(f64, f64)> = sim.drone_trail.iter().map(|p| (p.x, p.z)).collect();
    let target_trail: Vec<(f64, f64)> = sim.target_trail.iter().map(|p| (p.x, p.z)).collect();

    let canvas = Canvas::default()
        .block(block("PROFILE — SIDE  (X → east,  Z → altitude)"))
        .marker(Marker::Braille)
        .x_bounds([-config::WORLD_BOUND, config::WORLD_BOUND])
        .y_bounds([0.0, config::WORLD_CEILING])
        .paint(move |ctx| {
            ctx.draw(&CanvasLine {
                x1: -config::WORLD_BOUND,
                y1: config::WORLD_FLOOR,
                x2: config::WORLD_BOUND,
                y2: config::WORLD_FLOOR,
                color: Color::Rgb(60, 50, 40),
            });
            for o in &sim.world.obstacles {
                ctx.draw(&Circle {
                    x: o.center.x,
                    y: o.center.z,
                    radius: o.radius,
                    color: OBSTACLE_COLOR,
                });
            }
            ctx.layer();

            ctx.draw(&Points { coords: &target_trail, color: Color::Rgb(40, 110, 60) });
            ctx.draw(&Points { coords: &drone_trail, color: Color::Rgb(40, 90, 120) });
            ctx.layer();

            ctx.draw(&CanvasLine {
                x1: drone.x,
                y1: drone.z,
                x2: target.x,
                y2: target.z,
                color: LOS_COLOR,
            });
            ctx.print(target.x, target.z, Span::styled("◆", ratatui::style::Style::default().fg(tgt_color)));
            ctx.print(drone.x, drone.z, Span::styled("▲", ratatui::style::Style::default().fg(DRONE_COLOR)));
        });

    f.render_widget(canvas, area);
}
