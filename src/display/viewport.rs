use ratatui::layout::Rect;
use ratatui::style::{Color, Style};
use ratatui::symbols::Marker;
use ratatui::text::Span;
use ratatui::widgets::canvas::{Canvas, Circle, Context, Line as CanvasLine, Points};
use ratatui::widgets::{Block, BorderType, Borders};
use ratatui::Frame;

use crate::display::camera::Camera;
use crate::engine::orchestrator::{Simulation, Status};
use crate::utils::config;

const DRONE_BODY: Color = Color::Rgb(90, 200, 255);
const DRONE_ROTOR: Color = Color::Rgb(180, 240, 255);
const OBSTACLE_COLOR: Color = Color::Rgb(150, 110, 70);
const LOS_COLOR: Color = Color::Rgb(70, 74, 92);
const RING_COLOR: Color = Color::Rgb(45, 60, 72);
const AIM_COLOR: Color = Color::Yellow;
const FLOOR_COLOR: Color = Color::Rgb(70, 58, 44);

fn target_colors(sim: &Simulation) -> (Color, Color) {
    let p = sim.target.panic_level.clamp(0.0, 1.0);
    let l = |a: u8, b: u8| (a as f64 + (b as f64 - a as f64) * p) as u8;
    (
        Color::Rgb(l(70, 255), l(220, 95), l(110, 70)),
        Color::Rgb(l(160, 255), l(245, 170), l(170, 150)),
    )
}

fn block(title: &str) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::Rgb(70, 80, 100)))
        .title(Span::styled(
            format!(" {title} "),
            Style::default().fg(Color::Rgb(150, 200, 255)),
        ))
}

fn quad_top(ctx: &mut Context, cx: f64, cy: f64, heading: f64, size: f64, body: Color, rotor: Color) {
    let arms = [
        heading + std::f64::consts::FRAC_PI_4,
        heading + 3.0 * std::f64::consts::FRAC_PI_4,
        heading - 3.0 * std::f64::consts::FRAC_PI_4,
        heading - std::f64::consts::FRAC_PI_4,
    ];
    for a in arms {
        let (s, c) = a.sin_cos();
        let tx = cx + size * c;
        let ty = cy + size * s;
        ctx.draw(&CanvasLine { x1: cx, y1: cy, x2: tx, y2: ty, color: body });
        ctx.draw(&Circle { x: tx, y: ty, radius: size * 0.42, color: rotor });
    }
    let (s, c) = heading.sin_cos();
    ctx.draw(&CanvasLine {
        x1: cx,
        y1: cy,
        x2: cx + size * 1.6 * c,
        y2: cy + size * 1.6 * s,
        color: rotor,
    });
}

fn quad_side(ctx: &mut Context, hx: f64, hz: f64, size: f64, body: Color, rotor: Color) {
    ctx.draw(&CanvasLine { x1: hx - size, y1: hz, x2: hx + size, y2: hz, color: body });
    for sx in [-1.0_f64, 1.0] {
        let ex = hx + sx * size;
        ctx.draw(&CanvasLine { x1: ex, y1: hz, x2: ex, y2: hz + size * 0.55, color: body });
        ctx.draw(&CanvasLine {
            x1: ex - size * 0.45,
            y1: hz + size * 0.55,
            x2: ex + size * 0.45,
            y2: hz + size * 0.55,
            color: rotor,
        });
    }
}

pub fn draw_top_view(f: &mut Frame, area: Rect, sim: &Simulation, cam: &Camera) {
    let drone = sim.drone.position();
    let target = sim.target.position();
    let (tgt_body, tgt_rotor) = target_colors(sim);
    let half = cam.half_span;
    let size = cam.glyph_size();

    let dc = cam.top(drone);
    let tc = cam.top(target);
    let aim = cam.top(sim.telemetry.aim_point);
    let drone_heading = cam.heading_local(sim.drone.body.heading());
    let target_heading = cam.heading_local(sim.target.body.heading());

    let drone_trail: Vec<(f64, f64)> = sim.drone_trail.iter().map(|p| cam.top(*p)).collect();
    let target_trail: Vec<(f64, f64)> = sim.target_trail.iter().map(|p| cam.top(*p)).collect();
    let obstacles: Vec<(f64, f64, f64)> = sim
        .world
        .obstacles
        .iter()
        .map(|o| {
            let (x, y) = cam.top(o.center);
            (x, y, o.radius)
        })
        .collect();

    let running = sim.status == Status::Running;

    let canvas = Canvas::default()
        .block(block(&format!("TACTICAL · TOP  [{}]  {:.0} m span", cam.mode_label(), half * 2.0)))
        .marker(Marker::Braille)
        .x_bounds([-half, half])
        .y_bounds([-half, half])
        .paint(move |ctx| {
            for r in [config::INTERCEPT_RADIUS, config::INTERCEPT_RADIUS * 2.0] {
                ctx.draw(&Circle { x: dc.0, y: dc.1, radius: r, color: RING_COLOR });
            }
            for (x, y, radius) in &obstacles {
                ctx.draw(&Circle { x: *x, y: *y, radius: *radius, color: OBSTACLE_COLOR });
            }
            ctx.layer();

            ctx.draw(&Points { coords: &target_trail, color: Color::Rgb(40, 120, 65) });
            ctx.draw(&Points { coords: &drone_trail, color: Color::Rgb(45, 100, 135) });
            ctx.layer();

            ctx.draw(&CanvasLine { x1: dc.0, y1: dc.1, x2: tc.0, y2: tc.1, color: LOS_COLOR });
            if running {
                ctx.print(aim.0, aim.1, Span::styled("⊕", Style::default().fg(AIM_COLOR)));
            }
            ctx.layer();

            quad_top(ctx, tc.0, tc.1, target_heading, size, tgt_body, tgt_rotor);
            quad_top(ctx, dc.0, dc.1, drone_heading, size, DRONE_BODY, DRONE_ROTOR);
        });

    f.render_widget(canvas, area);
}

pub fn draw_side_view(f: &mut Frame, area: Rect, sim: &Simulation, cam: &Camera) {
    let drone = sim.drone.position();
    let target = sim.target.position();
    let (tgt_body, tgt_rotor) = target_colors(sim);
    let half = cam.half_span;
    let size = cam.glyph_size();

    let dc = cam.side(drone);
    let tc = cam.side(target);

    let drone_trail: Vec<(f64, f64)> = sim.drone_trail.iter().map(|p| cam.side(*p)).collect();
    let target_trail: Vec<(f64, f64)> = sim.target_trail.iter().map(|p| cam.side(*p)).collect();
    let obstacles: Vec<(f64, f64, f64)> = sim
        .world
        .obstacles
        .iter()
        .map(|o| {
            let (x, _) = cam.side(o.center);
            (x, o.center.z, o.radius)
        })
        .collect();

    let canvas = Canvas::default()
        .block(block("PROFILE · SIDE  (Z → altitude)"))
        .marker(Marker::Braille)
        .x_bounds([-half, half])
        .y_bounds([0.0, config::WORLD_CEILING])
        .paint(move |ctx| {
            ctx.draw(&CanvasLine {
                x1: -half,
                y1: config::WORLD_FLOOR,
                x2: half,
                y2: config::WORLD_FLOOR,
                color: FLOOR_COLOR,
            });
            for (x, z, radius) in &obstacles {
                ctx.draw(&Circle { x: *x, y: *z, radius: *radius, color: OBSTACLE_COLOR });
            }
            ctx.layer();

            ctx.draw(&Points { coords: &target_trail, color: Color::Rgb(40, 120, 65) });
            ctx.draw(&Points { coords: &drone_trail, color: Color::Rgb(45, 100, 135) });
            ctx.layer();

            ctx.draw(&CanvasLine { x1: dc.0, y1: dc.1, x2: tc.0, y2: tc.1, color: LOS_COLOR });
            quad_side(ctx, tc.0, tc.1, size, tgt_body, tgt_rotor);
            quad_side(ctx, dc.0, dc.1, size, DRONE_BODY, DRONE_ROTOR);
        });

    f.render_widget(canvas, area);
}
