use std::f32::consts::PI;

use eframe::egui;
use eframe::egui::epaint::StrokeKind;

use super::theme::DashboardTheme;

pub(super) struct GaugeSpec<'a> {
    pub(super) label: &'a str,
    pub(super) value: f64,
    pub(super) min: f64,
    pub(super) max: f64,
    pub(super) unit: &'a str,
    pub(super) accent: egui::Color32,
    pub(super) footer: &'a str,
}

pub(super) struct LinearMeterSpec<'a> {
    pub(super) label: &'a str,
    pub(super) value: f64,
    pub(super) min: f64,
    pub(super) max: f64,
    pub(super) accent: egui::Color32,
    pub(super) value_text: String,
}

pub(super) fn section_label(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    label: &str,
    accent: egui::Color32,
) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new("///")
                .color(accent)
                .monospace()
                .strong(),
        );
        ui.label(
            egui::RichText::new(label)
                .color(theme.text_soft)
                .monospace()
                .strong(),
        );
    });
}

pub(super) fn monitor_heading(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    title: &str,
    subtitle: &str,
    accent: egui::Color32,
) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(title)
                .color(theme.text_main)
                .strong()
                .size(17.0),
        );
        ui.separator();
        ui.label(
            egui::RichText::new(subtitle)
                .color(accent)
                .monospace()
                .size(11.0),
        );
    });
    ui.add_space(4.0);
}

pub(super) fn annunciator(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    label: &str,
    active: bool,
    accent: egui::Color32,
) {
    let desired = egui::vec2(112.0, 34.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 9.0, theme.panel_alt_bg);
    painter.rect_stroke(
        rect,
        9.0,
        egui::Stroke::new(1.0, theme.bezel),
        StrokeKind::Inside,
    );

    let lamp_center = egui::pos2(rect.left() + 16.0, rect.center().y);
    let lamp_color = if active { accent } else { theme.lamp_off };
    painter.circle_filled(lamp_center, 6.0, lamp_color);
    painter.circle_stroke(
        lamp_center,
        6.0,
        egui::Stroke::new(1.0, accent.gamma_multiply(0.55)),
    );
    painter.text(
        egui::pos2(rect.left() + 30.0, rect.center().y),
        egui::Align2::LEFT_CENTER,
        label,
        egui::FontId::monospace(11.0),
        theme.text_main,
    );
}

pub(super) fn digital_readout(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    accent: egui::Color32,
    label: &str,
    value: &str,
    unit: &str,
    footer: &str,
) {
    theme.instrument_frame(accent).show(ui, |ui| {
        ui.set_min_width(132.0);
        ui.set_max_width(240.0);
        ui.label(
            egui::RichText::new(label)
                .color(theme.text_soft)
                .monospace()
                .size(10.5),
        );
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(value)
                    .color(accent)
                    .monospace()
                    .strong()
                    .size(24.0),
            );
            if !unit.is_empty() {
                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new(unit)
                        .color(theme.text_main)
                        .monospace()
                        .size(11.0),
                );
            }
        });
        if !footer.is_empty() {
            ui.label(
                egui::RichText::new(footer)
                    .color(theme.text_soft)
                    .monospace()
                    .size(9.5),
            );
        }
    });
}

pub(super) fn metric_row(ui: &mut egui::Ui, theme: DashboardTheme, label: &str, value: String) {
    ui.label(
        egui::RichText::new(label)
            .color(theme.text_soft)
            .monospace()
            .size(11.5),
    );
    ui.label(
        egui::RichText::new(value)
            .color(theme.text_main)
            .monospace()
            .size(12.5),
    );
    ui.end_row();
}

pub(super) fn linear_meter(ui: &mut egui::Ui, theme: DashboardTheme, spec: LinearMeterSpec<'_>) {
    let card_width = 232.0;
    theme.instrument_frame(spec.accent).show(ui, |ui| {
        ui.set_min_width(card_width);
        ui.set_max_width(card_width);
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(spec.label)
                    .color(theme.text_soft)
                    .monospace()
                    .size(10.0),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    egui::RichText::new(spec.value_text)
                        .color(theme.text_main)
                        .monospace()
                        .size(10.0),
                );
            });
        });

        let desired = egui::vec2((card_width - 20.0).max(132.0), 16.0);
        let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let ratio = ((spec.value - spec.min) / (spec.max - spec.min).max(f64::EPSILON))
            .clamp(0.0, 1.0) as f32;
        let fill_rect = egui::Rect::from_min_max(
            rect.min,
            egui::pos2(rect.left() + rect.width() * ratio, rect.bottom()),
        );

        painter.rect_filled(rect, 8.0, theme.lamp_off);
        painter.rect_stroke(
            rect,
            8.0,
            egui::Stroke::new(1.0, theme.bezel),
            StrokeKind::Inside,
        );
        painter.rect_filled(fill_rect, 8.0, spec.accent.gamma_multiply(0.78));

        for tick in 1..4 {
            let x = egui::lerp(rect.left()..=rect.right(), tick as f32 / 4.0);
            painter.line_segment(
                [
                    egui::pos2(x, rect.top() + 3.0),
                    egui::pos2(x, rect.bottom() - 3.0),
                ],
                egui::Stroke::new(1.0, theme.chrome.gamma_multiply(0.5)),
            );
        }
    });
}

// The dyno-cell feel comes mostly from dense digital cards and compact semicircular gauges.
pub(super) fn gauge(ui: &mut egui::Ui, theme: DashboardTheme, spec: GaugeSpec<'_>) {
    let desired = egui::vec2(156.0, 116.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 12.0, theme.panel_alt_bg);
    painter.rect_stroke(
        rect,
        12.0,
        egui::Stroke::new(1.0, spec.accent.gamma_multiply(0.45)),
        StrokeKind::Inside,
    );

    let center = egui::pos2(rect.center().x, rect.bottom() - 20.0);
    let radius = rect.width().min(rect.height() * 1.35) * 0.38;
    let start_angle = PI * 5.0 / 6.0;
    let end_angle = PI / 6.0;
    let value_ratio =
        ((spec.value - spec.min) / (spec.max - spec.min).max(f64::EPSILON)).clamp(0.0, 1.0) as f32;
    let sweep_angle = start_angle + (end_angle - start_angle) * value_ratio;

    painter.text(
        egui::pos2(rect.left() + 12.0, rect.top() + 10.0),
        egui::Align2::LEFT_TOP,
        spec.label,
        egui::FontId::monospace(10.0),
        theme.text_soft,
    );

    let arc_points = |to_angle: f32| {
        let mut points = Vec::new();
        for step in 0..=48 {
            let t = step as f32 / 48.0;
            let angle = start_angle + (to_angle - start_angle) * t;
            points.push(egui::pos2(
                center.x + radius * angle.cos(),
                center.y - radius * angle.sin(),
            ));
        }
        points
    };

    painter.add(egui::Shape::line(
        arc_points(end_angle),
        egui::Stroke::new(7.0, theme.lamp_off),
    ));
    painter.add(egui::Shape::line(
        arc_points(sweep_angle),
        egui::Stroke::new(7.0, spec.accent),
    ));

    for tick in 0..=8 {
        let t = tick as f32 / 8.0;
        let angle = start_angle + (end_angle - start_angle) * t;
        let inner = egui::pos2(
            center.x + radius * 0.78 * angle.cos(),
            center.y - radius * 0.78 * angle.sin(),
        );
        let outer = egui::pos2(
            center.x + radius * 0.95 * angle.cos(),
            center.y - radius * 0.95 * angle.sin(),
        );
        painter.line_segment([inner, outer], egui::Stroke::new(1.0, theme.chrome));
    }

    let needle_angle = sweep_angle;
    let needle_tip = egui::pos2(
        center.x + radius * 0.7 * needle_angle.cos(),
        center.y - radius * 0.7 * needle_angle.sin(),
    );
    painter.line_segment(
        [center, needle_tip],
        egui::Stroke::new(2.2, spec.accent.gamma_multiply(0.95)),
    );
    painter.circle_filled(center, 4.0, spec.accent);

    painter.text(
        egui::pos2(rect.center().x, rect.center().y + 8.0),
        egui::Align2::CENTER_CENTER,
        format!("{:.1}", spec.value),
        egui::FontId::monospace(22.0),
        spec.accent,
    );
    painter.text(
        egui::pos2(rect.center().x, rect.center().y + 28.0),
        egui::Align2::CENTER_TOP,
        spec.unit,
        egui::FontId::monospace(10.0),
        theme.text_main,
    );
    painter.text(
        egui::pos2(rect.center().x, rect.bottom() - 8.0),
        egui::Align2::CENTER_BOTTOM,
        spec.footer,
        egui::FontId::monospace(9.0),
        theme.text_soft,
    );
}
