use eframe::egui;

use super::super::theme::DashboardTheme;
use super::super::view_model::{GraphLayoutHeights, SchematicViewModel};
use super::super::widgets::{metric_row, monitor_heading};

pub(super) fn render_engine_motion_schematic(
    ui: &mut egui::Ui,
    theme: DashboardTheme,
    graph_heights: GraphLayoutHeights,
    vm: &SchematicViewModel,
) {
    theme.monitor_frame().show(ui, |ui| {
        let flame_radius_norm = finite_f32(vm.flame_radius_norm, 0.0).clamp(0.0, 1.0);
        let flame_speed_mps = finite_f32(vm.flame_speed_mps, 0.0).max(0.0);
        monitor_heading(
            ui,
            theme,
            "Engine Motion Schematic",
            "single-cylinder cutaway showing piston, crank, poppet valves, tent-roof chamber, and flame travel from current bore, stroke, ignition, and VVT",
            theme.green,
        );

        let desired_height = (graph_heights.standard_plot_px * 3.4).clamp(360.0, 460.0);
        let desired_width = ui.available_width().min(460.0).max(340.0);
        let desired_size = egui::vec2(desired_width, desired_height);

        let chamber_base = if vm.fuel_cmd {
            theme.cyan.gamma_multiply(0.14)
        } else {
            theme.panel_bg
        };
        let chamber_hot = theme.red.gamma_multiply(0.34);
        let chamber_fill = if vm.burn_active || vm.ignition_flash {
            lerp_color(chamber_base, chamber_hot, flame_radius_norm.max(0.22))
        } else {
            chamber_base
        };

        ui.vertical_centered(|ui| {
            let (rect, _) = ui.allocate_exact_size(desired_size, egui::Sense::hover());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 12.0, theme.panel_alt_bg);
            painter.rect_stroke(
                rect,
                12.0,
                egui::Stroke::new(1.0, theme.bezel),
                egui::epaint::StrokeKind::Inside,
            );

            let center_x = rect.center().x;
            let top_y = rect.top() + 26.0;
            let deck_y = rect.top() + 112.0;
            let max_geom = vm.stroke_m.max(vm.bore_m * 0.92);
            let bore_ratio = (vm.bore_m / max_geom.max(f64::EPSILON)) as f32;
            let stroke_ratio = (vm.stroke_m / max_geom.max(f64::EPSILON)) as f32;
            let bore_px = (136.0 + 38.0 * bore_ratio).clamp(126.0, 176.0);
            let stroke_px = (168.0 + 64.0 * stroke_ratio).clamp(170.0, 240.0);
            let bore_left = center_x - bore_px * 0.5;
            let bore_right = center_x + bore_px * 0.5;
            let cylinder_bottom = deck_y + stroke_px;
            let cylinder_rect = egui::Rect::from_min_max(
                egui::pos2(bore_left, deck_y - 12.0),
                egui::pos2(bore_right, cylinder_bottom),
            );
            let chamber_apex = egui::pos2(center_x, deck_y - 38.0);
            let intake_port_rect = egui::Rect::from_min_max(
                egui::pos2(bore_left - 98.0, top_y + 8.0),
                egui::pos2(center_x - 18.0, top_y + 44.0),
            );
            let exhaust_port_rect = egui::Rect::from_min_max(
                egui::pos2(center_x + 18.0, top_y + 8.0),
                egui::pos2(bore_right + 98.0, top_y + 44.0),
            );
            let intake_stem_x = bore_left + bore_px * 0.33;
            let exhaust_stem_x = bore_right - bore_px * 0.33;
            let seat_y = deck_y - 5.0;
            let valve_max_lift_mm = vm.intake_max_lift_mm.max(vm.exhaust_max_lift_mm).max(1.0);
            let lift_scale = 32.0 / valve_max_lift_mm as f32;
            let intake_lift_px = vm.intake_lift_mm as f32 * lift_scale;
            let exhaust_lift_px = vm.exhaust_lift_mm as f32 * lift_scale;
            let valve_closed_head_y = seat_y + 5.0;
            let intake_head_y = valve_closed_head_y + intake_lift_px;
            let exhaust_head_y = valve_closed_head_y + exhaust_lift_px;
            let piston_phase = vm
                .local_cycle_deg
                .to_radians()
                .rem_euclid(std::f64::consts::TAU);
            let piston_frac = 0.5 * (1.0 - piston_phase.cos());
            let piston_h = 26.0;
            let piston_y = deck_y + (stroke_px - piston_h - 8.0) * piston_frac as f32;
            let piston_rect = egui::Rect::from_min_max(
                egui::pos2(bore_left + 6.0, piston_y),
                egui::pos2(bore_right - 6.0, piston_y + piston_h),
            );
            let crank_center = egui::pos2(center_x, cylinder_bottom + 46.0);
            let crank_radius = 28.0;
            let crank_pin = egui::pos2(
                center_x + crank_radius * (piston_phase.sin() as f32),
                crank_center.y - crank_radius * (piston_phase.cos() as f32),
            );
            let rod_top = piston_rect.center_bottom();
            let chamber_poly = vec![
                egui::pos2(bore_left, deck_y),
                egui::pos2(bore_left + 20.0, deck_y - 10.0),
                chamber_apex,
                egui::pos2(bore_right - 20.0, deck_y - 10.0),
                egui::pos2(bore_right, deck_y),
            ];

            painter.rect_filled(cylinder_rect, 10.0, chamber_fill);
            painter.add(egui::Shape::convex_polygon(
                chamber_poly.clone(),
                chamber_fill,
                egui::Stroke::NONE,
            ));
            painter.rect_stroke(
                cylinder_rect,
                10.0,
                egui::Stroke::new(1.4, theme.chrome),
                egui::epaint::StrokeKind::Inside,
            );
            painter.add(egui::Shape::closed_line(
                chamber_poly,
                egui::Stroke::new(2.0, theme.bezel),
            ));

            painter.rect_filled(intake_port_rect, 8.0, theme.cyan.gamma_multiply(0.14));
            painter.rect_stroke(
                intake_port_rect,
                8.0,
                egui::Stroke::new(1.2, theme.cyan),
                egui::epaint::StrokeKind::Inside,
            );
            painter.rect_filled(exhaust_port_rect, 8.0, theme.red.gamma_multiply(0.14));
            painter.rect_stroke(
                exhaust_port_rect,
                8.0,
                egui::Stroke::new(1.2, theme.red),
                egui::epaint::StrokeKind::Inside,
            );
            painter.text(
                intake_port_rect.center_top() + egui::vec2(0.0, 6.0),
                egui::Align2::CENTER_TOP,
                "INTAKE",
                egui::FontId::monospace(10.0),
                theme.cyan,
            );
            painter.text(
                exhaust_port_rect.center_top() + egui::vec2(0.0, 6.0),
                egui::Align2::CENTER_TOP,
                "EXHAUST",
                egui::FontId::monospace(10.0),
                theme.red,
            );
            painter.line_segment(
                [
                    intake_port_rect.center_bottom(),
                    egui::pos2(intake_stem_x - 12.0, seat_y - 20.0),
                ],
                egui::Stroke::new(1.6, theme.cyan),
            );
            painter.line_segment(
                [
                    exhaust_port_rect.center_bottom(),
                    egui::pos2(exhaust_stem_x + 12.0, seat_y - 20.0),
                ],
                egui::Stroke::new(1.6, theme.red),
            );

            let guide_top_y = top_y + 18.0;
            let guide_bottom_y = seat_y - 18.0;
            painter.line_segment(
                [
                    egui::pos2(intake_stem_x, guide_top_y),
                    egui::pos2(intake_stem_x, intake_head_y),
                ],
                egui::Stroke::new(2.2, theme.cyan),
            );
            painter.line_segment(
                [
                    egui::pos2(exhaust_stem_x, guide_top_y),
                    egui::pos2(exhaust_stem_x, exhaust_head_y),
                ],
                egui::Stroke::new(2.2, theme.red),
            );
            painter.rect_filled(
                egui::Rect::from_center_size(
                    egui::pos2(intake_stem_x, guide_top_y - 4.0),
                    egui::vec2(18.0, 8.0),
                ),
                2.0,
                theme.cyan.gamma_multiply(0.25),
            );
            painter.rect_filled(
                egui::Rect::from_center_size(
                    egui::pos2(exhaust_stem_x, guide_top_y - 4.0),
                    egui::vec2(18.0, 8.0),
                ),
                2.0,
                theme.red.gamma_multiply(0.25),
            );
            painter.line_segment(
                [
                    egui::pos2(intake_stem_x, guide_top_y),
                    egui::pos2(intake_stem_x, guide_bottom_y),
                ],
                egui::Stroke::new(1.0, theme.chrome.gamma_multiply(0.55)),
            );
            painter.line_segment(
                [
                    egui::pos2(exhaust_stem_x, guide_top_y),
                    egui::pos2(exhaust_stem_x, guide_bottom_y),
                ],
                egui::Stroke::new(1.0, theme.chrome.gamma_multiply(0.55)),
            );

            painter.line_segment(
                [
                    egui::pos2(intake_stem_x - 15.0, seat_y),
                    egui::pos2(intake_stem_x - 4.0, seat_y + 8.0),
                ],
                egui::Stroke::new(1.5, theme.chrome),
            );
            painter.line_segment(
                [
                    egui::pos2(intake_stem_x + 15.0, seat_y),
                    egui::pos2(intake_stem_x + 4.0, seat_y + 8.0),
                ],
                egui::Stroke::new(1.5, theme.chrome),
            );
            painter.line_segment(
                [
                    egui::pos2(exhaust_stem_x - 15.0, seat_y),
                    egui::pos2(exhaust_stem_x - 4.0, seat_y + 8.0),
                ],
                egui::Stroke::new(1.5, theme.chrome),
            );
            painter.line_segment(
                [
                    egui::pos2(exhaust_stem_x + 15.0, seat_y),
                    egui::pos2(exhaust_stem_x + 4.0, seat_y + 8.0),
                ],
                egui::Stroke::new(1.5, theme.chrome),
            );
            painter.line_segment(
                [
                    egui::pos2(intake_stem_x - 12.0, intake_head_y),
                    egui::pos2(intake_stem_x + 12.0, intake_head_y),
                ],
                egui::Stroke::new(4.0, theme.cyan),
            );
            painter.line_segment(
                [
                    egui::pos2(exhaust_stem_x - 12.0, exhaust_head_y),
                    egui::pos2(exhaust_stem_x + 12.0, exhaust_head_y),
                ],
                egui::Stroke::new(4.0, theme.red),
            );

            painter.rect_filled(piston_rect, 5.0, theme.amber.gamma_multiply(0.90));
            painter.rect_stroke(
                piston_rect,
                5.0,
                egui::Stroke::new(1.0, theme.amber),
                egui::epaint::StrokeKind::Inside,
            );
            painter.line_segment([rod_top, crank_pin], egui::Stroke::new(2.6, theme.text_main));
            painter.circle_stroke(
                crank_center,
                crank_radius,
                egui::Stroke::new(1.6, theme.chrome),
            );
            painter.line_segment(
                [crank_center, crank_pin],
                egui::Stroke::new(2.2, theme.text_main),
            );
            painter.circle_filled(crank_pin, 4.4, theme.text_main);
            painter.circle_filled(crank_center, 3.8, theme.chrome);
            painter.line_segment(
                [
                    egui::pos2(center_x - 42.0, crank_center.y + 30.0),
                    egui::pos2(center_x + 42.0, crank_center.y + 30.0),
                ],
                egui::Stroke::new(1.2, theme.bezel),
            );

            let spark_plug_pos = egui::pos2(center_x, chamber_apex.y + 10.0);
            painter.line_segment(
                [
                    egui::pos2(center_x, chamber_apex.y - 18.0),
                    egui::pos2(center_x, chamber_apex.y + 2.0),
                ],
                egui::Stroke::new(2.0, theme.text_soft),
            );
            painter.line_segment(
                [
                    egui::pos2(center_x - 5.0, chamber_apex.y - 12.0),
                    egui::pos2(center_x + 5.0, chamber_apex.y - 12.0),
                ],
                egui::Stroke::new(1.2, theme.text_soft),
            );
            painter.circle_filled(spark_plug_pos, 3.0, theme.text_main);

            let chamber_clip = egui::Rect::from_min_max(
                egui::pos2(bore_left + 2.0, chamber_apex.y - 4.0),
                egui::pos2(bore_right - 2.0, piston_rect.top() - 2.0),
            );
            let chamber_painter = painter.with_clip_rect(chamber_clip);
            if vm.ignition_flash {
                for (angle_deg, length) in [
                    (-55.0_f32, 16.0_f32),
                    (-18.0_f32, 18.0_f32),
                    (20.0_f32, 18.0_f32),
                    (54.0_f32, 16.0_f32),
                ] {
                    let angle = angle_deg.to_radians();
                    let tip = egui::pos2(
                        spark_plug_pos.x + length * angle.cos() as f32,
                        spark_plug_pos.y + length * angle.sin() as f32,
                    );
                    chamber_painter.line_segment(
                        [spark_plug_pos, tip],
                        egui::Stroke::new(1.8, theme.red),
                    );
                }
            }
            if vm.burn_active {
                let max_radius_x = (bore_px * 0.44).max(18.0);
                let max_radius_y = ((piston_rect.top() - spark_plug_pos.y - 10.0).max(18.0))
                    .min(stroke_px * 0.62);
                let radius_x = egui::lerp(10.0..=max_radius_x, flame_radius_norm.powf(0.88));
                let radius_y = egui::lerp(12.0..=max_radius_y, flame_radius_norm.powf(0.80));
                let outer_front = flame_front_points(
                    spark_plug_pos,
                    radius_x,
                    radius_y,
                    bore_left,
                    bore_right,
                    chamber_apex,
                    deck_y,
                    piston_rect.top(),
                );
                let mid_front = flame_front_points(
                    spark_plug_pos,
                    radius_x * 0.68,
                    radius_y * 0.66,
                    bore_left,
                    bore_right,
                    chamber_apex,
                    deck_y,
                    piston_rect.top(),
                );
                let inner_front = flame_front_points(
                    spark_plug_pos,
                    radius_x * 0.36,
                    radius_y * 0.34,
                    bore_left,
                    bore_right,
                    chamber_apex,
                    deck_y,
                    piston_rect.top(),
                );

                chamber_painter.add(egui::Shape::convex_polygon(
                    outer_front.clone(),
                    theme.red.gamma_multiply(0.08 + 0.10 * (1.0 - flame_radius_norm)),
                    egui::Stroke::NONE,
                ));
                chamber_painter.add(egui::Shape::convex_polygon(
                    mid_front.clone(),
                    theme.amber.gamma_multiply(0.11 + 0.10 * (1.0 - flame_radius_norm)),
                    egui::Stroke::NONE,
                ));
                chamber_painter.add(egui::Shape::convex_polygon(
                    inner_front.clone(),
                    theme.red.gamma_multiply(0.18 + 0.10 * (1.0 - flame_radius_norm)),
                    egui::Stroke::NONE,
                ));
                chamber_painter.add(egui::Shape::closed_line(
                    outer_front,
                    egui::Stroke::new(2.0, theme.amber.gamma_multiply(0.95)),
                ));
                chamber_painter.add(egui::Shape::closed_line(
                    mid_front,
                    egui::Stroke::new(1.2, theme.red.gamma_multiply(0.78)),
                ));
                chamber_painter.circle_filled(
                    spark_plug_pos,
                    (8.0 + 8.0 * (1.0 - flame_radius_norm)).max(5.0),
                    theme.amber.gamma_multiply(0.82),
                );
            }

            painter.text(
                rect.left_top() + egui::vec2(18.0, 14.0),
                egui::Align2::LEFT_TOP,
                "CYL 1 CUTAWAY",
                egui::FontId::monospace(10.5),
                theme.amber,
            );
            painter.text(
                egui::pos2(bore_left - 18.0, seat_y + 18.0),
                egui::Align2::RIGHT_TOP,
                "intake valve",
                egui::FontId::monospace(10.0),
                theme.cyan,
            );
            painter.text(
                egui::pos2(bore_right + 18.0, seat_y + 18.0),
                egui::Align2::LEFT_TOP,
                "exhaust valve",
                egui::FontId::monospace(10.0),
                theme.red,
            );
            painter.text(
                egui::pos2(center_x, piston_rect.bottom() + 12.0),
                egui::Align2::CENTER_TOP,
                "piston",
                egui::FontId::monospace(10.0),
                theme.amber,
            );
            painter.text(
                egui::pos2(center_x, crank_center.y + 36.0),
                egui::Align2::CENTER_TOP,
                "crank",
                egui::FontId::monospace(10.0),
                theme.text_soft,
            );
        });

        ui.add_space(6.0);
        egui::Grid::new("engine_motion_metrics")
            .num_columns(2)
            .spacing(egui::vec2(16.0, 4.0))
            .show(ui, |ui| {
                metric_row(
                    ui,
                    theme,
                    "Display phase",
                    format!("{:.0} degCA", vm.local_cycle_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "Current stroke",
                    vm.current_stroke_label.to_owned(),
                );
                metric_row(
                    ui,
                    theme,
                    "Bore / Stroke",
                    format!("{:.1} mm / {:.1} mm", vm.bore_m * 1.0e3, vm.stroke_m * 1.0e3),
                );
                metric_row(
                    ui,
                    theme,
                    "Intake / Exhaust lift",
                    format!("{:.1} mm / {:.1} mm", vm.intake_lift_mm, vm.exhaust_lift_mm),
                );
                metric_row(
                    ui,
                    theme,
                    "Intake / Exhaust VVT",
                    format!("{:.1} deg / {:.1} deg", vm.intake_vvt_deg, vm.exhaust_vvt_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "Spark / Burn start",
                    format!("{:.0} / {:.0} degCA", vm.spark_event_deg, vm.burn_start_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "IVO / IVC",
                    format!("{:.0} / {:.0} degCA", vm.intake_open_deg, vm.intake_close_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "EVO / EVC",
                    format!("{:.0} / {:.0} degCA", vm.exhaust_open_deg, vm.exhaust_close_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "Actual / display phase",
                    format!("{:.0} / {:.0} degCA", vm.actual_cycle_deg, vm.local_cycle_deg),
                );
                metric_row(
                    ui,
                    theme,
                    "Combustion",
                    if vm.burn_active {
                        format!(
                            "flame {:.0} % / {:.1} m/s",
                            flame_radius_norm * 100.0,
                            flame_speed_mps
                        )
                    } else if vm.ignition_flash {
                        "spark discharge".to_owned()
                    } else if vm.fuel_cmd {
                        "fresh charge".to_owned()
                    } else {
                        "motoring".to_owned()
                    },
                );
            });
    });
}

fn lerp_color(a: egui::Color32, b: egui::Color32, t: f32) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);
    let lerp_channel = |lhs: u8, rhs: u8| -> u8 {
        (lhs as f32 + (rhs as f32 - lhs as f32) * t)
            .round()
            .clamp(0.0, 255.0) as u8
    };
    egui::Color32::from_rgba_unmultiplied(
        lerp_channel(a.r(), b.r()),
        lerp_channel(a.g(), b.g()),
        lerp_channel(a.b(), b.b()),
        lerp_channel(a.a(), b.a()),
    )
}

fn tent_roof_y(
    x: f32,
    bore_left: f32,
    bore_right: f32,
    chamber_apex: egui::Pos2,
    deck_y: f32,
) -> f32 {
    let x = finite_f32(x, 0.0);
    let deck_y = finite_f32(deck_y, 0.0);
    let bore_left = finite_f32(bore_left, x - 1.0);
    let bore_right = finite_f32(bore_right, x + 1.0);
    let apex_x = finite_f32(chamber_apex.x, 0.5 * (bore_left + bore_right));
    let apex_y = finite_f32(chamber_apex.y, deck_y - 10.0);
    if x <= apex_x {
        let t = finite_f32(
            (x - bore_left) / (apex_x - bore_left).abs().max(f32::EPSILON),
            0.0,
        )
        .clamp(0.0, 1.0);
        egui::lerp(deck_y..=apex_y, t)
    } else {
        let t = finite_f32(
            (x - apex_x) / (bore_right - apex_x).abs().max(f32::EPSILON),
            0.0,
        )
        .clamp(0.0, 1.0);
        egui::lerp(apex_y..=deck_y, t)
    }
}

fn flame_front_points(
    spark_pos: egui::Pos2,
    radius_x: f32,
    radius_y: f32,
    bore_left: f32,
    bore_right: f32,
    chamber_apex: egui::Pos2,
    deck_y: f32,
    piston_top_y: f32,
) -> Vec<egui::Pos2> {
    let mut points = Vec::with_capacity(49);
    let spark_x = finite_f32(spark_pos.x, 0.0);
    let spark_y = finite_f32(spark_pos.y, 0.0);
    let safe_left = finite_f32(bore_left.min(bore_right), spark_x - 12.0);
    let safe_right = finite_f32(bore_left.max(bore_right), spark_x + 12.0);
    let mut x_min = safe_left + 6.0;
    let mut x_max = safe_right - 6.0;
    if !x_min.is_finite() || !x_max.is_finite() || x_min > x_max {
        x_min = spark_x - 6.0;
        x_max = spark_x + 6.0;
    }
    let radius_x = finite_f32(radius_x.abs(), 0.0);
    let radius_y = finite_f32(radius_y.abs(), 0.0);
    let y_bottom = finite_f32((piston_top_y - 6.0).max(spark_y + 8.0), spark_y + 8.0);
    for step in 0..=48 {
        let angle = std::f32::consts::TAU * step as f32 / 48.0;
        let mut x = finite_f32(spark_x + radius_x * angle.cos(), spark_x);
        x = x.clamp(x_min, x_max);
        let roof_y = finite_f32(
            tent_roof_y(x, bore_left, bore_right, chamber_apex, deck_y) + 5.0,
            spark_y - 4.0,
        );
        let local_y = radius_y * angle.sin();
        let stretch = if local_y < 0.0 { 0.64 } else { 1.08 };
        let mut y = finite_f32(spark_y + local_y * stretch, spark_y);
        let y_min = roof_y.min(y_bottom);
        let y_max = roof_y.max(y_bottom);
        y = y.clamp(y_min, y_max);
        points.push(egui::pos2(x, y));
    }
    points
}

fn finite_f32(value: f32, fallback: f32) -> f32 {
    if value.is_finite() { value } else { fallback }
}

#[cfg(test)]
mod tests {
    use eframe::egui;

    use super::flame_front_points;

    #[test]
    fn flame_front_points_stay_finite_when_radius_is_nan() {
        let points = flame_front_points(
            egui::pos2(100.0, 120.0),
            f32::NAN,
            24.0,
            60.0,
            140.0,
            egui::pos2(100.0, 80.0),
            100.0,
            180.0,
        );

        assert_eq!(points.len(), 49);
        assert!(
            points
                .iter()
                .all(|point| point.x.is_finite() && point.y.is_finite())
        );
    }
}
