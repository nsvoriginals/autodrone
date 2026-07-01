use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Borders, Gauge, Paragraph};
use ratatui::Frame;

use crate::engine::orchestrator::Simulation;

const LABEL: Color = Color::Rgb(130, 140, 160);
const VALUE: Color = Color::Rgb(225, 230, 240);
const ACCENT: Color = Color::Rgb(150, 200, 255);

fn kmh(mps: f64) -> f64 {
    mps * 3.6
}

fn panel(title: &str) -> Block<'_> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::Rgb(70, 80, 100)))
        .title(Span::styled(
            format!(" {title} "),
            Style::default().fg(ACCENT).add_modifier(Modifier::BOLD),
        ))
}

fn row<'a>(label: &'a str, value: String, color: Color) -> Line<'a> {
    Line::from(vec![
        Span::styled(format!("{label:<13}"), Style::default().fg(LABEL)),
        Span::styled(value, Style::default().fg(color).add_modifier(Modifier::BOLD)),
    ])
}

pub fn draw(f: &mut Frame, area: Rect, sim: &Simulation) {
    let rows = Layout::vertical([
        Constraint::Length(6),
        Constraint::Length(6),
        Constraint::Length(8),
        Constraint::Length(7),
        Constraint::Min(6),
    ])
    .split(area);

    draw_drone(f, rows[0], sim);
    draw_target(f, rows[1], sim);
    draw_guidance(f, rows[2], sim);
    draw_estimation(f, rows[3], sim);
    draw_gauges(f, rows[4], sim);
}

fn draw_drone(f: &mut Frame, area: Rect, sim: &Simulation) {
    let d = &sim.drone;
    let p = d.position();
    let lines = vec![
        row("SPEED", format!("{:>7.1} km/h", kmh(d.speed())), Color::Cyan),
        row("POSITION", format!("{:>6.0} {:>6.0} {:>6.0}", p.x, p.y, p.z), VALUE),
        row("ACCEL", format!("{:>7.1} m/s²", d.body.acceleration.magnitude()), VALUE),
        row("PHASE", d.phase.label().to_string(), Color::Yellow),
    ];
    f.render_widget(Paragraph::new(lines).block(panel("DRONE  ▲")), area);
}

fn draw_target(f: &mut Frame, area: Rect, sim: &Simulation) {
    let t = &sim.target;
    let p = t.position();
    let panic = t.panic_level.clamp(0.0, 1.0);
    let panic_color = if panic > 0.5 { Color::Rgb(255, 90, 60) } else { Color::Green };
    let lines = vec![
        row("SPEED", format!("{:>7.1} km/h", kmh(t.speed())), Color::Green),
        row("POSITION", format!("{:>6.0} {:>6.0} {:>6.0}", p.x, p.y, p.z), VALUE),
        row("EVASIVENESS", format!("{:>6.0} %", sim.tracker.evasiveness() * 100.0), panic_color),
        row("PANIC", format!("{:>6.0} %", panic * 100.0), panic_color),
    ];
    f.render_widget(Paragraph::new(lines).block(panel("TARGET  ◆")), area);
}

fn draw_guidance(f: &mut Frame, area: Rect, sim: &Simulation) {
    let tm = &sim.telemetry;
    let law = if tm.blend > 0.7 {
        "PROP-NAV"
    } else if tm.blend > 0.35 {
        "PN + LEAD"
    } else {
        "PURE PURSUIT"
    };
    let lines = vec![
        row("DISTANCE", format!("{:>7.1} m", tm.true_distance), ACCENT),
        row("CLOSING VEL", format!("{:>7.1} m/s", tm.closing_speed), VALUE),
        row("TIME-TO-INT", format!("{:>7.1} s", tm.time_to_intercept), VALUE),
        row("LOS RATE", format!("{:>7.2} °/s", tm.los_rate.to_degrees()), VALUE),
        row("GUIDANCE", law.to_string(), Color::Yellow),
        row("PN ACCEL", format!("{:>7.1} m/s²", tm.pn_accel), VALUE),
    ];
    f.render_widget(Paragraph::new(lines).block(panel("INTERCEPTION")), area);
}

fn draw_estimation(f: &mut Frame, area: Rect, sim: &Simulation) {
    let pid = &sim.pid;
    let tm = &sim.telemetry;
    let lines = vec![
        row("PID Kp/Ki/Kd", format!("{:.2} {:.2} {:.2}", pid.kp, pid.ki, pid.kd), VALUE),
        row("KALMAN σ", format!("{:>7.2} m", sim.tracker.uncertainty()), VALUE),
        row("INNOVATION", format!("{:>7.2} m", sim.tracker.innovation()), VALUE),
        row("EST. ERROR", format!("{:>7.2} m", (tm.est_distance - tm.true_distance).abs()), VALUE),
        row("DES. SPEED", format!("{:>7.1} km/h", kmh(tm.desired_speed)), Color::Cyan),
    ];
    f.render_widget(Paragraph::new(lines).block(panel("ESTIMATION & CONTROL")), area);
}

fn draw_gauges(f: &mut Frame, area: Rect, sim: &Simulation) {
    let block = panel("STATE");
    let inner = block.inner(area);
    f.render_widget(block, area);

    let rows = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Length(1),
    ])
    .split(inner);

    let battery = sim.drone.battery.clamp(0.0, 1.0);
    let battery_color = if battery < 0.2 { Color::Red } else if battery < 0.5 { Color::Yellow } else { Color::Green };
    labelled_gauge(f, rows[0], "BAT", battery, battery_color, format!("{:>3.0}%", battery * 100.0));

    let speed_ratio = (sim.drone.speed() / sim.drone.params.max_speed).clamp(0.0, 1.0);
    labelled_gauge(f, rows[1], "SPD", speed_ratio, Color::Cyan, format!("{:>3.0}%", speed_ratio * 100.0));

    let evade = sim.tracker.evasiveness().clamp(0.0, 1.0);
    labelled_gauge(f, rows[2], "EVA", evade, Color::Rgb(255, 120, 70), format!("{:>3.0}%", evade * 100.0));

    let conv = (1.0 - (sim.tracker.uncertainty() / 30.0).clamp(0.0, 1.0)).clamp(0.0, 1.0);
    labelled_gauge(f, rows[3], "KAL", conv, Color::Rgb(120, 180, 255), format!("{:>3.0}%", conv * 100.0));
}

fn labelled_gauge(f: &mut Frame, area: Rect, label: &str, ratio: f64, color: Color, text: String) {
    let cols = Layout::horizontal([Constraint::Length(4), Constraint::Min(0)]).split(area);
    f.render_widget(
        Paragraph::new(Span::styled(label, Style::default().fg(LABEL))),
        cols[0],
    );
    f.render_widget(
        Gauge::default()
            .gauge_style(Style::default().fg(color).bg(Color::Rgb(30, 34, 42)))
            .ratio(ratio.clamp(0.0, 1.0))
            .label(Span::styled(text, Style::default().fg(VALUE))),
        cols[1],
    );
}
