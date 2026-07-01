use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Borders, Paragraph};
use ratatui::Frame;

use crate::display::camera::Camera;
use crate::display::{panels, viewport};
use crate::engine::orchestrator::{Simulation, Status};

pub fn draw(f: &mut Frame, sim: &Simulation, cam: &Camera, fps: f64) {
    let root = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(0),
        Constraint::Length(3),
    ])
    .split(f.area());

    draw_header(f, root[0], sim, cam, fps);

    let body = Layout::horizontal([Constraint::Percentage(62), Constraint::Percentage(38)])
        .split(root[1]);

    let scene = Layout::vertical([Constraint::Percentage(64), Constraint::Percentage(36)])
        .split(body[0]);
    viewport::draw_top_view(f, scene[0], sim, cam);
    viewport::draw_side_view(f, scene[1], sim, cam);

    panels::draw(f, body[1], sim);

    draw_footer(f, root[2], sim, cam);
}

fn draw_header(f: &mut Frame, area: Rect, sim: &Simulation, cam: &Camera, fps: f64) {
    let (status_text, status_color) = match sim.status {
        Status::Running => ("● PURSUING", Color::Cyan),
        Status::Captured => ("✔ INTERCEPTED", Color::Green),
    };

    let line = Line::from(vec![
        Span::styled("  AUTO-DRONE ", Style::default().fg(Color::Rgb(150, 200, 255)).add_modifier(Modifier::BOLD)),
        Span::styled("· 3D Pursuit-Evasion Twin", Style::default().fg(Color::Rgb(110, 120, 140))),
        Span::raw("      "),
        Span::styled(status_text, Style::default().fg(status_color).add_modifier(Modifier::BOLD)),
        Span::raw("      "),
        Span::styled(format!("T+{:>6.1}s", sim.world.time), Style::default().fg(Color::Rgb(200, 210, 225))),
        Span::raw("   "),
        Span::styled(format!("CAM {}", cam.mode_label()), Style::default().fg(Color::Rgb(150, 170, 200))),
        Span::raw("   "),
        Span::styled(format!("{fps:>4.0} fps"), Style::default().fg(Color::Rgb(110, 120, 140))),
    ]);

    let block = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::Rgb(70, 80, 100)));
    f.render_widget(Paragraph::new(line).block(block), area);
}

fn draw_footer(f: &mut Frame, area: Rect, sim: &Simulation, _cam: &Camera) {
    let m = &sim.metrics;
    let controls = "[↑↓←→] pan  [+/-] zoom  [< >] rotate  [f] follow  [space] pause  [r] new  [q] quit";
    let content = match sim.status {
        Status::Captured => {
            let t = m.capture_time.unwrap_or(m.elapsed);
            Line::from(vec![
                Span::styled("  ✔ TARGET INTERCEPTED  ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                Span::styled(
                    format!("in {t:.1}s · peak closing {:.1} m/s · new chase starting…",
                        m.max_closing_speed),
                    Style::default().fg(Color::Rgb(180, 200, 190)),
                ),
            ])
        }
        Status::Running => {
            let mut spans = Vec::new();
            spans.extend(metric("RANGE", format!("{:.0} m", sim.telemetry.true_distance)));
            spans.push(sep());
            spans.extend(metric("MIN", format!("{:.1} m", m.min_distance)));
            spans.push(sep());
            spans.push(Span::styled(
                controls,
                Style::default().fg(Color::Rgb(110, 120, 140)),
            ));
            Line::from(spans)
        }
    };

    let block = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::Rgb(70, 80, 100)));
    f.render_widget(Paragraph::new(content).block(block).alignment(Alignment::Left), area);
}

fn metric<'a>(label: &'a str, value: String) -> Vec<Span<'a>> {
    vec![
        Span::styled(format!("  {label} "), Style::default().fg(Color::Rgb(130, 140, 160))),
        Span::styled(value, Style::default().fg(Color::Rgb(210, 220, 235)).add_modifier(Modifier::BOLD)),
    ]
}

fn sep() -> Span<'static> {
    Span::styled(" │ ", Style::default().fg(Color::Rgb(60, 68, 84)))
}
