use std::f64::consts::PI;

use crate::constants::FIXED_CYLINDER_COUNT;

use super::{AppConfig, ExternalLoadConfig};

#[derive(Debug, Default, Clone)]
pub(crate) struct ConfigAuditReport {
    pub(crate) warnings: Vec<String>,
    pub(crate) errors: Vec<String>,
}

impl ConfigAuditReport {
    fn error(&mut self, message: impl Into<String>) {
        self.errors.push(message.into());
    }

    fn warning(&mut self, message: impl Into<String>) {
        self.warnings.push(message.into());
    }
}

pub(crate) fn validate_app_config(cfg: &AppConfig) -> Result<ConfigAuditReport, String> {
    let report = audit_app_config(cfg);
    if report.errors.is_empty() {
        Ok(report)
    } else {
        Err(format_audit_report(&report))
    }
}

pub(crate) fn audit_app_config(cfg: &AppConfig) -> ConfigAuditReport {
    let mut report = ConfigAuditReport::default();

    audit_environment(cfg, &mut report);
    audit_engine(cfg, &mut report);
    audit_cam(cfg, &mut report);
    audit_controls(cfg, &mut report);
    audit_model(cfg, &mut report);
    audit_numerics(cfg, &mut report);
    audit_ui(cfg, &mut report);
    audit_plot(cfg, &mut report);

    report
}

fn format_audit_report(report: &ConfigAuditReport) -> String {
    let mut lines = Vec::new();
    for error in &report.errors {
        lines.push(format!("- {error}"));
    }
    if !report.warnings.is_empty() {
        lines.push("Warnings:".to_string());
        for warning in &report.warnings {
            lines.push(format!("- {warning}"));
        }
    }
    lines.join("\n")
}

fn audit_environment(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let env = &cfg.environment;
    require_range(
        report,
        env.ambient_pressure_pa,
        "environment.ambient_pressure_pa",
        60_000.0,
        110_000.0,
    );
    require_range(
        report,
        env.intake_temp_k,
        "environment.intake_temp_k",
        220.0,
        360.0,
    );
    require_range(
        report,
        env.exhaust_temp_k,
        "environment.exhaust_temp_k",
        400.0,
        1_200.0,
    );
    require_range(report, env.dt, "environment.dt", 1.0e-5, 0.05);
    require_lt(
        report,
        env.intake_temp_k,
        env.exhaust_temp_k,
        "environment.intake_temp_k",
        "environment.exhaust_temp_k",
    );
}

fn audit_engine(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let engine = &cfg.engine;
    require_range(report, engine.bore_m, "engine.bore_m", 0.055, 0.110);
    require_range(report, engine.stroke_m, "engine.stroke_m", 0.055, 0.120);
    require_range(
        report,
        engine.bore_m / engine.stroke_m.max(f64::EPSILON),
        "engine.bore_m / engine.stroke_m",
        0.65,
        1.25,
    );
    require_range(
        report,
        engine.compression_ratio,
        "engine.compression_ratio",
        7.0,
        16.5,
    );
    require_range(
        report,
        engine.inertia_kgm2,
        "engine.inertia_kgm2",
        0.02,
        1.2,
    );
    require_range(
        report,
        engine.friction_c0_nm,
        "engine.friction_c0_nm",
        0.0,
        25.0,
    );
    require_range(
        report,
        engine.friction_c1_nms,
        "engine.friction_c1_nms",
        0.0,
        0.05,
    );
    require_range(
        report,
        engine.friction_c2_nms2,
        "engine.friction_c2_nms2",
        0.0,
        0.001,
    );
    require_range(
        report,
        engine.intake_volume_m3,
        "engine.intake_volume_m3",
        0.0005,
        0.020,
    );
    require_range(
        report,
        engine.intake_runner_volume_m3,
        "engine.intake_runner_volume_m3",
        0.00005,
        0.003,
    );
    require_range(
        report,
        engine.exhaust_volume_m3,
        "engine.exhaust_volume_m3",
        0.0005,
        0.025,
    );
    require_range(
        report,
        engine.exhaust_runner_volume_m3,
        "engine.exhaust_runner_volume_m3",
        0.00005,
        0.003,
    );
    require_range(
        report,
        equivalent_diameter_m(engine.throttle_area_max_m2),
        "equivalent diameter of engine.throttle_area_max_m2",
        0.025,
        0.090,
    );
    require_range(
        report,
        equivalent_diameter_m(engine.tailpipe_area_m2),
        "equivalent diameter of engine.tailpipe_area_m2",
        0.030,
        0.100,
    );
    require_range(
        report,
        engine.default_target_rpm,
        "engine.default_target_rpm",
        500.0,
        1_500.0,
    );
    require_range(report, engine.max_rpm, "engine.max_rpm", 2_000.0, 12_000.0);
    require_lt(
        report,
        engine.default_target_rpm,
        engine.max_rpm,
        "engine.default_target_rpm",
        "engine.max_rpm",
    );

    let displacement = engine.derived_displacement_m3();
    require_range(
        report,
        displacement,
        "engine.displacement_m3",
        0.0004,
        0.0080,
    );
    require_close_relative(
        report,
        engine.displacement_m3,
        displacement,
        "engine.displacement_m3",
        1.0e-9,
    );
    require_range(
        report,
        engine.intake_volume_m3 / displacement.max(f64::EPSILON),
        "engine.intake_volume_m3 / displacement",
        0.2,
        6.0,
    );
    require_range(
        report,
        engine.exhaust_volume_m3 / displacement.max(f64::EPSILON),
        "engine.exhaust_volume_m3 / displacement",
        0.2,
        8.0,
    );
    let mean_piston_speed = 2.0 * engine.stroke_m * engine.max_rpm / 60.0;
    require_range(
        report,
        mean_piston_speed,
        "mean piston speed at engine.max_rpm",
        3.0,
        35.0,
    );
    for rpm in [
        engine.default_target_rpm.max(700.0),
        3_000.0,
        engine.max_rpm,
    ] {
        let omega = rpm * PI / 30.0;
        let friction = engine.friction_c0_nm
            + engine.friction_c1_nms * omega
            + engine.friction_c2_nms2 * omega.powi(2);
        require_range(
            report,
            friction,
            &format!("engine friction torque at {rpm:.0} rpm"),
            0.5,
            100.0,
        );
    }
}

fn audit_cam(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let cam = &cfg.cam;
    for (value, path) in [
        (cam.intake_centerline_deg, "cam.intake_centerline_deg"),
        (cam.exhaust_centerline_deg, "cam.exhaust_centerline_deg"),
    ] {
        require_range(report, value, path, 0.0, 720.0);
    }
    require_range(
        report,
        cam.intake_duration_deg,
        "cam.intake_duration_deg",
        160.0,
        320.0,
    );
    require_range(
        report,
        cam.exhaust_duration_deg,
        "cam.exhaust_duration_deg",
        160.0,
        320.0,
    );
    require_range(
        report,
        cam.intake_max_lift_mm,
        "cam.intake_max_lift_mm",
        4.0,
        20.0,
    );
    require_range(
        report,
        cam.exhaust_max_lift_mm,
        "cam.exhaust_max_lift_mm",
        4.0,
        20.0,
    );
    require_range(
        report,
        cam.display_y_max_mm,
        "cam.display_y_max_mm",
        6.0,
        30.0,
    );
    require_lt(
        report,
        cam.intake_max_lift_mm,
        cam.display_y_max_mm,
        "cam.intake_max_lift_mm",
        "cam.display_y_max_mm",
    );
    require_lt(
        report,
        cam.exhaust_max_lift_mm,
        cam.display_y_max_mm,
        "cam.exhaust_max_lift_mm",
        "cam.display_y_max_mm",
    );
}

fn audit_controls(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let defaults = &cfg.control_defaults;
    require_unit_interval(
        report,
        defaults.throttle_cmd,
        "control_defaults.throttle_cmd",
    );
    require_range(
        report,
        defaults.load_cmd,
        "control_defaults.load_cmd",
        -1.0,
        1.0,
    );
    require_range(
        report,
        defaults.ignition_timing_deg,
        "control_defaults.ignition_timing_deg",
        -10.0,
        50.0,
    );
    require_range(
        report,
        defaults.vvt_intake_deg,
        "control_defaults.vvt_intake_deg",
        -60.0,
        60.0,
    );
    require_range(
        report,
        defaults.vvt_exhaust_deg,
        "control_defaults.vvt_exhaust_deg",
        -60.0,
        60.0,
    );
}

fn audit_model(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let ve = &cfg.model.volumetric_efficiency;
    for (value, path, min, max) in [
        (
            ve.rpm_base,
            "model.volumetric_efficiency.rpm_base",
            -1.0,
            2.0,
        ),
        (
            ve.rpm_gain,
            "model.volumetric_efficiency.rpm_gain",
            -2.0,
            2.0,
        ),
        (
            ve.rpm_center,
            "model.volumetric_efficiency.rpm_center",
            500.0,
            10_000.0,
        ),
        (
            ve.rpm_width,
            "model.volumetric_efficiency.rpm_width",
            100.0,
            100_000_000.0,
        ),
        (ve.rpm_min, "model.volumetric_efficiency.rpm_min", 0.1, 2.0),
        (ve.rpm_max, "model.volumetric_efficiency.rpm_max", 0.2, 2.0),
        (
            ve.vvt_rpm_low,
            "model.volumetric_efficiency.vvt_rpm_low",
            500.0,
            6_000.0,
        ),
        (
            ve.vvt_rpm_high,
            "model.volumetric_efficiency.vvt_rpm_high",
            1_500.0,
            12_000.0,
        ),
        (
            ve.vvt_intake_coeff,
            "model.volumetric_efficiency.vvt_intake_coeff",
            -0.05,
            0.05,
        ),
        (
            ve.vvt_exhaust_coeff,
            "model.volumetric_efficiency.vvt_exhaust_coeff",
            -0.05,
            0.05,
        ),
        (
            ve.vvt_intake_opt_low_deg,
            "model.volumetric_efficiency.vvt_intake_opt_low_deg",
            -50.0,
            50.0,
        ),
        (
            ve.vvt_intake_opt_high_deg,
            "model.volumetric_efficiency.vvt_intake_opt_high_deg",
            -50.0,
            50.0,
        ),
        (
            ve.vvt_intake_opt_window_deg,
            "model.volumetric_efficiency.vvt_intake_opt_window_deg",
            2.0,
            60.0,
        ),
        (
            ve.vvt_intake_opt_gain,
            "model.volumetric_efficiency.vvt_intake_opt_gain",
            0.0,
            0.25,
        ),
        (
            ve.vvt_exhaust_opt_low_deg,
            "model.volumetric_efficiency.vvt_exhaust_opt_low_deg",
            -50.0,
            50.0,
        ),
        (
            ve.vvt_exhaust_opt_high_deg,
            "model.volumetric_efficiency.vvt_exhaust_opt_high_deg",
            -50.0,
            50.0,
        ),
        (
            ve.vvt_exhaust_opt_window_deg,
            "model.volumetric_efficiency.vvt_exhaust_opt_window_deg",
            2.0,
            60.0,
        ),
        (
            ve.vvt_exhaust_opt_gain,
            "model.volumetric_efficiency.vvt_exhaust_opt_gain",
            0.0,
            0.25,
        ),
        (
            ve.throttle_base,
            "model.volumetric_efficiency.throttle_base",
            0.0,
            2.0,
        ),
        (
            ve.throttle_gain,
            "model.volumetric_efficiency.throttle_gain",
            0.0,
            2.0,
        ),
        (
            ve.throttle_min,
            "model.volumetric_efficiency.throttle_min",
            0.0,
            1.5,
        ),
        (
            ve.throttle_max,
            "model.volumetric_efficiency.throttle_max",
            0.1,
            2.0,
        ),
        (
            ve.overall_min,
            "model.volumetric_efficiency.overall_min",
            0.1,
            1.5,
        ),
        (
            ve.overall_max,
            "model.volumetric_efficiency.overall_max",
            0.2,
            2.5,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        ve.rpm_min,
        ve.rpm_max,
        "model.volumetric_efficiency.rpm_min",
        "model.volumetric_efficiency.rpm_max",
    );
    require_lt(
        report,
        ve.vvt_rpm_low,
        ve.vvt_rpm_high,
        "model.volumetric_efficiency.vvt_rpm_low",
        "model.volumetric_efficiency.vvt_rpm_high",
    );
    require_lt(
        report,
        ve.throttle_min,
        ve.throttle_max,
        "model.volumetric_efficiency.throttle_min",
        "model.volumetric_efficiency.throttle_max",
    );
    require_lt(
        report,
        ve.overall_min,
        ve.overall_max,
        "model.volumetric_efficiency.overall_min",
        "model.volumetric_efficiency.overall_max",
    );

    let pv = &cfg.model.pv_model;
    for (value, path, min, max) in [
        (
            pv.gamma_compression,
            "model.pv_model.gamma_compression",
            1.05,
            1.50,
        ),
        (
            pv.gamma_expansion,
            "model.pv_model.gamma_expansion",
            1.05,
            1.50,
        ),
        (pv.evo_deg, "model.pv_model.evo_deg", 380.0, 650.0),
        (
            pv.blowdown_end_deg,
            "model.pv_model.blowdown_end_deg",
            420.0,
            720.0,
        ),
        (
            pv.intake_pulsation_amplitude,
            "model.pv_model.intake_pulsation_amplitude",
            0.0,
            0.5,
        ),
        (
            pv.expansion_floor_exhaust_ratio,
            "model.pv_model.expansion_floor_exhaust_ratio",
            0.8,
            1.5,
        ),
        (
            pv.pressure_floor_pa,
            "model.pv_model.pressure_floor_pa",
            100.0,
            50_000.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        pv.evo_deg,
        pv.blowdown_end_deg,
        "model.pv_model.evo_deg",
        "model.pv_model.blowdown_end_deg",
    );

    let gas_path = &cfg.model.gas_path;
    for (value, path, min, max) in [
        (
            gas_path.intake_runner_inertance_pa_s2_per_kg,
            "model.gas_path.intake_runner_inertance_pa_s2_per_kg",
            10.0,
            2.0e6,
        ),
        (
            gas_path.exhaust_runner_inertance_pa_s2_per_kg,
            "model.gas_path.exhaust_runner_inertance_pa_s2_per_kg",
            10.0,
            2.0e6,
        ),
        (
            gas_path.intake_runner_damping_per_s,
            "model.gas_path.intake_runner_damping_per_s",
            0.0,
            500.0,
        ),
        (
            gas_path.exhaust_runner_damping_per_s,
            "model.gas_path.exhaust_runner_damping_per_s",
            0.0,
            500.0,
        ),
        (
            gas_path.intake_runner_length_m,
            "model.gas_path.intake_runner_length_m",
            0.10,
            1.00,
        ),
        (
            gas_path.exhaust_runner_length_m,
            "model.gas_path.exhaust_runner_length_m",
            0.20,
            1.50,
        ),
        (
            gas_path.intake_runner_diameter_m,
            "model.gas_path.intake_runner_diameter_m",
            0.015,
            0.080,
        ),
        (
            gas_path.exhaust_runner_diameter_m,
            "model.gas_path.exhaust_runner_diameter_m",
            0.015,
            0.090,
        ),
        (
            gas_path.intake_runner_friction_factor,
            "model.gas_path.intake_runner_friction_factor",
            0.0,
            0.20,
        ),
        (
            gas_path.exhaust_runner_friction_factor,
            "model.gas_path.exhaust_runner_friction_factor",
            0.0,
            0.20,
        ),
        (
            gas_path.intake_runner_local_loss_coeff,
            "model.gas_path.intake_runner_local_loss_coeff",
            0.0,
            20.0,
        ),
        (
            gas_path.exhaust_runner_local_loss_coeff,
            "model.gas_path.exhaust_runner_local_loss_coeff",
            0.0,
            20.0,
        ),
        (
            gas_path.intake_pulse_blend,
            "model.gas_path.intake_pulse_blend",
            0.0,
            1.5,
        ),
        (
            gas_path.exhaust_pulse_blend,
            "model.gas_path.exhaust_pulse_blend",
            0.0,
            1.5,
        ),
        (
            gas_path.intake_boundary_runner_weight,
            "model.gas_path.intake_boundary_runner_weight",
            0.0,
            1.0,
        ),
        (
            gas_path.exhaust_boundary_runner_weight,
            "model.gas_path.exhaust_boundary_runner_weight",
            0.0,
            1.0,
        ),
        (
            gas_path.runner_flow_limit_kg_s,
            "model.gas_path.runner_flow_limit_kg_s",
            0.01,
            5.0,
        ),
        (
            gas_path.runner_pressure_min_pa,
            "model.gas_path.runner_pressure_min_pa",
            5_000.0,
            200_000.0,
        ),
        (
            gas_path.runner_pressure_max_pa,
            "model.gas_path.runner_pressure_max_pa",
            30_000.0,
            500_000.0,
        ),
        (
            gas_path.overlap_pressure_coeff,
            "model.gas_path.overlap_pressure_coeff",
            -4.0,
            4.0,
        ),
        (
            gas_path.overlap_flow_coeff,
            "model.gas_path.overlap_flow_coeff",
            -4.0,
            4.0,
        ),
        (
            gas_path.overlap_flow_reference_kg_s,
            "model.gas_path.overlap_flow_reference_kg_s",
            0.001,
            1.0,
        ),
        (
            gas_path.overlap_effect_min,
            "model.gas_path.overlap_effect_min",
            0.1,
            2.0,
        ),
        (
            gas_path.overlap_effect_max,
            "model.gas_path.overlap_effect_max",
            0.2,
            3.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        gas_path.runner_pressure_min_pa,
        gas_path.runner_pressure_max_pa,
        "model.gas_path.runner_pressure_min_pa",
        "model.gas_path.runner_pressure_max_pa",
    );
    require_lt(
        report,
        gas_path.overlap_effect_min,
        gas_path.overlap_effect_max,
        "model.gas_path.overlap_effect_min",
        "model.gas_path.overlap_effect_max",
    );

    let wave = &cfg.model.wave_action;
    require_usize_range(
        report,
        wave.intake_group_count,
        "model.wave_action.intake_group_count",
        1,
        FIXED_CYLINDER_COUNT,
    );
    require_usize_range(
        report,
        wave.exhaust_group_count,
        "model.wave_action.exhaust_group_count",
        1,
        FIXED_CYLINDER_COUNT,
    );
    require_usize_range(
        report,
        wave.event_memory,
        "model.wave_action.event_memory",
        1,
        12,
    );
    for (value, path, min, max) in [
        (
            wave.intake_runner_length_m,
            "model.wave_action.intake_runner_length_m",
            0.10,
            1.00,
        ),
        (
            wave.exhaust_primary_length_m,
            "model.wave_action.exhaust_primary_length_m",
            0.20,
            1.50,
        ),
        (
            wave.intake_delay_scale,
            "model.wave_action.intake_delay_scale",
            0.1,
            5.0,
        ),
        (
            wave.exhaust_delay_scale,
            "model.wave_action.exhaust_delay_scale",
            0.1,
            5.0,
        ),
        (
            wave.intake_decay_time_s,
            "model.wave_action.intake_decay_time_s",
            1.0e-4,
            0.2,
        ),
        (
            wave.exhaust_decay_time_s,
            "model.wave_action.exhaust_decay_time_s",
            1.0e-4,
            0.2,
        ),
        (
            wave.intake_pressure_gain,
            "model.wave_action.intake_pressure_gain",
            0.0,
            5.0,
        ),
        (
            wave.exhaust_pressure_gain,
            "model.wave_action.exhaust_pressure_gain",
            0.0,
            5.0,
        ),
        (
            wave.intake_pressure_limit_pa,
            "model.wave_action.intake_pressure_limit_pa",
            0.0,
            50_000.0,
        ),
        (
            wave.exhaust_pressure_limit_pa,
            "model.wave_action.exhaust_pressure_limit_pa",
            0.0,
            80_000.0,
        ),
        (
            wave.intake_flow_reference_kg_s,
            "model.wave_action.intake_flow_reference_kg_s",
            0.001,
            1.0,
        ),
        (
            wave.exhaust_flow_reference_kg_s,
            "model.wave_action.exhaust_flow_reference_kg_s",
            0.001,
            1.5,
        ),
        (
            wave.intake_resonance_damping_ratio,
            "model.wave_action.intake_resonance_damping_ratio",
            0.01,
            2.0,
        ),
        (
            wave.exhaust_resonance_damping_ratio,
            "model.wave_action.exhaust_resonance_damping_ratio",
            0.01,
            2.0,
        ),
        (
            wave.intake_resonance_gain_blend,
            "model.wave_action.intake_resonance_gain_blend",
            0.0,
            1.5,
        ),
        (
            wave.exhaust_resonance_gain_blend,
            "model.wave_action.exhaust_resonance_gain_blend",
            0.0,
            1.5,
        ),
        (
            wave.resonance_gain_min,
            "model.wave_action.resonance_gain_min",
            0.1,
            5.0,
        ),
        (
            wave.resonance_gain_max,
            "model.wave_action.resonance_gain_max",
            0.1,
            10.0,
        ),
        (
            wave.intake_flow_wave_gain,
            "model.wave_action.intake_flow_wave_gain",
            0.0,
            5.0,
        ),
        (
            wave.exhaust_flow_wave_gain,
            "model.wave_action.exhaust_flow_wave_gain",
            0.0,
            5.0,
        ),
        (
            wave.intake_ram_gain,
            "model.wave_action.intake_ram_gain",
            0.0,
            5.0,
        ),
        (
            wave.exhaust_scavenge_gain,
            "model.wave_action.exhaust_scavenge_gain",
            0.0,
            5.0,
        ),
        (
            wave.ve_pulse_min,
            "model.wave_action.ve_pulse_min",
            0.2,
            2.0,
        ),
        (
            wave.ve_pulse_max,
            "model.wave_action.ve_pulse_max",
            0.5,
            3.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        wave.resonance_gain_min,
        wave.resonance_gain_max,
        "model.wave_action.resonance_gain_min",
        "model.wave_action.resonance_gain_max",
    );
    require_lt(
        report,
        wave.ve_pulse_min,
        wave.ve_pulse_max,
        "model.wave_action.ve_pulse_min",
        "model.wave_action.ve_pulse_max",
    );

    let ht = &cfg.model.heat_transfer;
    for (value, path, min, max) in [
        (
            ht.wall_temp_k,
            "model.heat_transfer.wall_temp_k",
            320.0,
            700.0,
        ),
        (
            ht.base_h_w_m2k,
            "model.heat_transfer.base_h_w_m2k",
            0.0,
            2_000.0,
        ),
        (
            ht.pressure_exponent,
            "model.heat_transfer.pressure_exponent",
            -1.0,
            2.0,
        ),
        (
            ht.temperature_exponent,
            "model.heat_transfer.temperature_exponent",
            -2.0,
            2.0,
        ),
        (
            ht.piston_speed_exponent,
            "model.heat_transfer.piston_speed_exponent",
            -1.0,
            2.0,
        ),
        (
            ht.reference_pressure_pa,
            "model.heat_transfer.reference_pressure_pa",
            50_000.0,
            10.0e6,
        ),
        (
            ht.reference_temp_k,
            "model.heat_transfer.reference_temp_k",
            200.0,
            3_000.0,
        ),
        (
            ht.reference_piston_speed_mps,
            "model.heat_transfer.reference_piston_speed_mps",
            0.1,
            50.0,
        ),
        (
            ht.gas_temp_rise_gain,
            "model.heat_transfer.gas_temp_rise_gain",
            0.0,
            2.0,
        ),
        (
            ht.gas_temp_max_k,
            "model.heat_transfer.gas_temp_max_k",
            600.0,
            4_000.0,
        ),
        (
            ht.wall_area_scale,
            "model.heat_transfer.wall_area_scale",
            0.1,
            5.0,
        ),
        (
            ht.duration_scale,
            "model.heat_transfer.duration_scale",
            0.0,
            5.0,
        ),
        (
            ht.heat_loss_fraction_max,
            "model.heat_transfer.heat_loss_fraction_max",
            0.0,
            0.60,
        ),
        (
            ht.exhaust_temp_cooling_gain,
            "model.heat_transfer.exhaust_temp_cooling_gain",
            0.0,
            2.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        ht.wall_temp_k,
        ht.gas_temp_max_k,
        "model.heat_transfer.wall_temp_k",
        "model.heat_transfer.gas_temp_max_k",
    );

    let evap = &cfg.model.fuel_evaporation;
    require_range(
        report,
        evap.latent_heat_j_per_kg,
        "model.fuel_evaporation.latent_heat_j_per_kg",
        100_000.0,
        800_000.0,
    );
    require_range(
        report,
        evap.charge_cooling_effectiveness,
        "model.fuel_evaporation.charge_cooling_effectiveness",
        0.0,
        1.0,
    );
    require_range(
        report,
        evap.intake_charge_temp_min_k,
        "model.fuel_evaporation.intake_charge_temp_min_k",
        180.0,
        cfg.environment.intake_temp_k,
    );

    let gas = &cfg.model.gas_thermo;
    for (value, path, min, max) in [
        (
            gas.fresh_cp_j_per_kgk,
            "model.gas_thermo.fresh_cp_j_per_kgk",
            850.0,
            1_400.0,
        ),
        (
            gas.burned_cp_ref_j_per_kgk,
            "model.gas_thermo.burned_cp_ref_j_per_kgk",
            900.0,
            1_600.0,
        ),
        (
            gas.burned_cp_reference_temp_k,
            "model.gas_thermo.burned_cp_reference_temp_k",
            400.0,
            2_000.0,
        ),
        (
            gas.burned_cp_temp_coeff_j_per_kgk2,
            "model.gas_thermo.burned_cp_temp_coeff_j_per_kgk2",
            -1.0,
            2.0,
        ),
        (
            gas.burned_cp_egr_coeff_j_per_kgk,
            "model.gas_thermo.burned_cp_egr_coeff_j_per_kgk",
            -500.0,
            1_000.0,
        ),
        (
            gas.cp_min_j_per_kgk,
            "model.gas_thermo.cp_min_j_per_kgk",
            700.0,
            1_400.0,
        ),
        (
            gas.cp_max_j_per_kgk,
            "model.gas_thermo.cp_max_j_per_kgk",
            900.0,
            2_000.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        gas.cp_min_j_per_kgk,
        gas.cp_max_j_per_kgk,
        "model.gas_thermo.cp_min_j_per_kgk",
        "model.gas_thermo.cp_max_j_per_kgk",
    );

    let egr = &cfg.model.internal_egr;
    for (value, path, min, max) in [
        (
            egr.overlap_base_fraction,
            "model.internal_egr.overlap_base_fraction",
            0.0,
            0.20,
        ),
        (
            egr.pressure_backflow_gain,
            "model.internal_egr.pressure_backflow_gain",
            0.0,
            5.0,
        ),
        (
            egr.wave_backflow_gain,
            "model.internal_egr.wave_backflow_gain",
            0.0,
            5.0,
        ),
        (
            egr.reverse_flow_gain,
            "model.internal_egr.reverse_flow_gain",
            0.0,
            5.0,
        ),
        (
            egr.reverse_flow_reference_kg_s,
            "model.internal_egr.reverse_flow_reference_kg_s",
            0.001,
            1.0,
        ),
        (
            egr.fraction_min,
            "model.internal_egr.fraction_min",
            0.0,
            0.5,
        ),
        (
            egr.fraction_max,
            "model.internal_egr.fraction_max",
            0.0,
            0.8,
        ),
        (
            egr.burn_duration_gain_deg_per_fraction,
            "model.internal_egr.burn_duration_gain_deg_per_fraction",
            0.0,
            120.0,
        ),
        (
            egr.phase_dilution_gain,
            "model.internal_egr.phase_dilution_gain",
            0.0,
            2.0,
        ),
        (
            egr.phase_dilution_min,
            "model.internal_egr.phase_dilution_min",
            0.0,
            1.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_lt(
        report,
        egr.fraction_min,
        egr.fraction_max,
        "model.internal_egr.fraction_min",
        "model.internal_egr.fraction_max",
    );

    audit_external_load(&cfg.model.external_load, "model.external_load", report);

    let model = &cfg.model;
    require_unit_interval(
        report,
        model.initial_throttle_eff,
        "model.initial_throttle_eff",
    );
    require_range(
        report,
        model.initial_intake_pressure_pa,
        "model.initial_intake_pressure_pa",
        10_000.0,
        120_000.0,
    );
    require_range(
        report,
        model.max_rpm_floor,
        "model.max_rpm_floor",
        300.0,
        4_000.0,
    );
    require_usize_range(
        report,
        model.history_capacity_floor,
        "model.history_capacity_floor",
        16,
        100_000,
    );
    require_usize_range(
        report,
        model.cycle_metric_history_capacity,
        "model.cycle_metric_history_capacity",
        1,
        1_000,
    );
    require_usize_range(
        report,
        model.pv_capacity_scale,
        "model.pv_capacity_scale",
        16,
        100_000,
    );
    require_usize_range(
        report,
        model.pv_capacity_min,
        "model.pv_capacity_min",
        16,
        100_000,
    );
    require_usize_range(
        report,
        model.pv_display_raw_min_points,
        "model.pv_display_raw_min_points",
        8,
        50_000,
    );
    require_usize_range(
        report,
        model.pv_display_bins,
        "model.pv_display_bins",
        90,
        10_000,
    );
    require_usize_range(
        report,
        model.pv_display_min_populated_bins,
        "model.pv_display_min_populated_bins",
        8,
        model.pv_display_bins,
    );
    for (value, path, min, max) in [
        (
            model.ignition_timing_min_deg,
            "model.ignition_timing_min_deg",
            -20.0,
            40.0,
        ),
        (
            model.ignition_timing_max_deg,
            "model.ignition_timing_max_deg",
            0.0,
            80.0,
        ),
        (model.mbt_base_deg, "model.mbt_base_deg", -5.0, 40.0),
        (
            model.mbt_rpm_slope_deg_per_rpm,
            "model.mbt_rpm_slope_deg_per_rpm",
            -0.01,
            0.01,
        ),
        (
            model.mbt_rpm_reference,
            "model.mbt_rpm_reference",
            0.0,
            12_000.0,
        ),
        (model.mbt_load_coeff, "model.mbt_load_coeff", -10.0, 10.0),
        (model.mbt_min_deg, "model.mbt_min_deg", -10.0, 40.0),
        (model.mbt_max_deg, "model.mbt_max_deg", 0.0, 60.0),
        (
            model.phase_sigma_advanced_deg,
            "model.phase_sigma_advanced_deg",
            0.1,
            40.0,
        ),
        (
            model.phase_sigma_retarded_deg,
            "model.phase_sigma_retarded_deg",
            0.1,
            60.0,
        ),
        (
            model.phase_stable_advanced_limit_deg,
            "model.phase_stable_advanced_limit_deg",
            0.0,
            60.0,
        ),
        (
            model.phase_stable_retarded_limit_deg,
            "model.phase_stable_retarded_limit_deg",
            0.0,
            80.0,
        ),
        (
            model.exhaust_temp_retard_gain,
            "model.exhaust_temp_retard_gain",
            0.0,
            0.20,
        ),
        (
            model.exhaust_temp_advance_gain,
            "model.exhaust_temp_advance_gain",
            0.0,
            0.20,
        ),
        (
            model.exhaust_temp_scale_min,
            "model.exhaust_temp_scale_min",
            0.1,
            2.0,
        ),
        (
            model.exhaust_temp_scale_max,
            "model.exhaust_temp_scale_max",
            0.2,
            4.0,
        ),
        (
            model.exhaust_temp_min_k,
            "model.exhaust_temp_min_k",
            300.0,
            1_500.0,
        ),
        (
            model.exhaust_temp_max_k,
            "model.exhaust_temp_max_k",
            600.0,
            3_000.0,
        ),
        (
            model.throttle_area_offset,
            "model.throttle_area_offset",
            0.0,
            1.0,
        ),
        (
            model.throttle_area_gain,
            "model.throttle_area_gain",
            0.0,
            2.0,
        ),
        (
            model.throttle_area_exponent,
            "model.throttle_area_exponent",
            0.1,
            6.0,
        ),
        (
            model.throttle_area_min_scale,
            "model.throttle_area_min_scale",
            0.0,
            1.0,
        ),
        (
            model.throttle_discharge_coeff,
            "model.throttle_discharge_coeff",
            0.2,
            1.0,
        ),
        (
            model.tailpipe_discharge_coeff,
            "model.tailpipe_discharge_coeff",
            0.2,
            1.0,
        ),
        (model.load_min, "model.load_min", 0.0, 2.0),
        (model.load_max, "model.load_max", 0.1, 3.0),
        (
            model.burn_start_base_deg,
            "model.burn_start_base_deg",
            300.0,
            420.0,
        ),
        (
            model.burn_start_vvt_intake_coeff,
            "model.burn_start_vvt_intake_coeff",
            -2.0,
            2.0,
        ),
        (
            model.burn_duration_base_deg,
            "model.burn_duration_base_deg",
            10.0,
            120.0,
        ),
        (
            model.burn_duration_throttle_coeff,
            "model.burn_duration_throttle_coeff",
            -50.0,
            50.0,
        ),
        (
            model.burn_duration_min_deg,
            "model.burn_duration_min_deg",
            5.0,
            120.0,
        ),
        (
            model.burn_duration_max_deg,
            "model.burn_duration_max_deg",
            5.0,
            180.0,
        ),
        (model.wiebe_a, "model.wiebe_a", 0.1, 20.0),
        (model.wiebe_m, "model.wiebe_m", 0.0, 10.0),
        (model.lambda_base, "model.lambda_base", 0.6, 1.8),
        (
            model.lambda_throttle_coeff,
            "model.lambda_throttle_coeff",
            -2.0,
            2.0,
        ),
        (model.lambda_min, "model.lambda_min", 0.5, 1.6),
        (model.lambda_max, "model.lambda_max", 0.6, 2.0),
        (model.stoich_afr, "model.stoich_afr", 13.0, 16.0),
        (model.eta_base_offset, "model.eta_base_offset", -0.2, 1.0),
        (model.eta_load_coeff, "model.eta_load_coeff", -1.0, 1.0),
        (
            model.eta_rpm_abs_coeff,
            "model.eta_rpm_abs_coeff",
            0.0,
            0.001,
        ),
        (
            model.eta_rpm_reference,
            "model.eta_rpm_reference",
            0.0,
            12_000.0,
        ),
        (model.eta_base_min, "model.eta_base_min", 0.0, 1.0),
        (model.eta_base_max, "model.eta_base_max", 0.0, 1.0),
        (model.eta_min, "model.eta_min", 0.0, 1.0),
        (model.eta_max, "model.eta_max", 0.0, 1.0),
        (
            model.combustion_enable_rpm_min,
            "model.combustion_enable_rpm_min",
            0.0,
            2_000.0,
        ),
        (
            model.combustion_enable_intake_pressure_min_pa,
            "model.combustion_enable_intake_pressure_min_pa",
            5_000.0,
            120_000.0,
        ),
        (
            model.combustion_enable_running_rpm_min,
            "model.combustion_enable_running_rpm_min",
            0.0,
            4_000.0,
        ),
        (
            model.combustion_rate_max,
            "model.combustion_rate_max",
            0.1,
            20.0,
        ),
        (
            model.pumping_torque_min_nm,
            "model.pumping_torque_min_nm",
            -300.0,
            0.0,
        ),
        (
            model.pumping_torque_max_nm,
            "model.pumping_torque_max_nm",
            0.0,
            300.0,
        ),
        (
            model.throttle_time_constant_s,
            "model.throttle_time_constant_s",
            0.001,
            2.0,
        ),
        (
            model.intake_pressure_min_pa,
            "model.intake_pressure_min_pa",
            1_000.0,
            100_000.0,
        ),
        (
            model.intake_pressure_max_pa,
            "model.intake_pressure_max_pa",
            20_000.0,
            400_000.0,
        ),
        (
            model.exhaust_pressure_min_ambient_ratio,
            "model.exhaust_pressure_min_ambient_ratio",
            0.3,
            2.0,
        ),
        (
            model.exhaust_pressure_max_over_ambient_pa,
            "model.exhaust_pressure_max_over_ambient_pa",
            0.0,
            300_000.0,
        ),
        (
            model.running_set_rpm,
            "model.running_set_rpm",
            100.0,
            2_500.0,
        ),
        (
            model.running_clear_rpm,
            "model.running_clear_rpm",
            0.0,
            2_000.0,
        ),
        (
            model.net_torque_filter_time_constant_s,
            "model.net_torque_filter_time_constant_s",
            1.0e-4,
            5.0,
        ),
        (
            model.fuel_mass_presence_threshold_kg,
            "model.fuel_mass_presence_threshold_kg",
            1.0e-12,
            1.0e-3,
        ),
        (
            model.compression_peak_gamma,
            "model.compression_peak_gamma",
            1.0,
            1.6,
        ),
        (
            model.compression_peak_max_pa,
            "model.compression_peak_max_pa",
            100_000.0,
            50.0e6,
        ),
        (
            model.combustion_pressure_gain,
            "model.combustion_pressure_gain",
            0.0,
            10.0,
        ),
        (
            model.combustion_pressure_load_exponent,
            "model.combustion_pressure_load_exponent",
            -5.0,
            5.0,
        ),
        (
            model.peak_pressure_max_pa,
            "model.peak_pressure_max_pa",
            100_000.0,
            50.0e6,
        ),
        (
            model.swept_volume_floor_sampling_m3,
            "model.swept_volume_floor_sampling_m3",
            1.0e-8,
            1.0e-3,
        ),
        (
            model.clearance_volume_floor_m3,
            "model.clearance_volume_floor_m3",
            1.0e-8,
            1.0e-3,
        ),
        (
            model.compression_ratio_guard,
            "model.compression_ratio_guard",
            1.0,
            30.0,
        ),
        (
            model.eta_indicated_min,
            "model.eta_indicated_min",
            -1.0,
            1.0,
        ),
        (model.eta_indicated_max, "model.eta_indicated_max", 0.0, 1.0),
        (
            model.imep_swept_volume_floor_m3,
            "model.imep_swept_volume_floor_m3",
            1.0e-10,
            1.0e-3,
        ),
        (
            model.theoretical_efficiency_compression_floor,
            "model.theoretical_efficiency_compression_floor",
            1.0,
            30.0,
        ),
        (
            model.cam_half_duration_min_deg,
            "model.cam_half_duration_min_deg",
            1.0,
            120.0,
        ),
        (
            model.cam_shape_exponent,
            "model.cam_shape_exponent",
            0.2,
            10.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_usize_range(
        report,
        model.net_torque_smoothing_cycles,
        "model.net_torque_smoothing_cycles",
        1,
        256,
    );
    require_usize_range(
        report,
        model.eta_indicated_average_cycles,
        "model.eta_indicated_average_cycles",
        1,
        128,
    );
    require_usize_range(
        report,
        model.cam_profile_samples,
        "model.cam_profile_samples",
        90,
        20_000,
    );
    require_lt(
        report,
        model.ignition_timing_min_deg,
        model.ignition_timing_max_deg,
        "model.ignition_timing_min_deg",
        "model.ignition_timing_max_deg",
    );
    require_lt(
        report,
        model.mbt_min_deg,
        model.mbt_max_deg,
        "model.mbt_min_deg",
        "model.mbt_max_deg",
    );
    require_lt(
        report,
        model.exhaust_temp_scale_min,
        model.exhaust_temp_scale_max,
        "model.exhaust_temp_scale_min",
        "model.exhaust_temp_scale_max",
    );
    require_lt(
        report,
        model.exhaust_temp_min_k,
        model.exhaust_temp_max_k,
        "model.exhaust_temp_min_k",
        "model.exhaust_temp_max_k",
    );
    require_lt(
        report,
        model.load_min,
        model.load_max,
        "model.load_min",
        "model.load_max",
    );
    require_lt(
        report,
        model.burn_duration_min_deg,
        model.burn_duration_max_deg,
        "model.burn_duration_min_deg",
        "model.burn_duration_max_deg",
    );
    require_lt(
        report,
        model.lambda_min,
        model.lambda_max,
        "model.lambda_min",
        "model.lambda_max",
    );
    require_lt(
        report,
        model.eta_base_min,
        model.eta_base_max,
        "model.eta_base_min",
        "model.eta_base_max",
    );
    require_lt(
        report,
        model.eta_min,
        model.eta_max,
        "model.eta_min",
        "model.eta_max",
    );
    require_lt(
        report,
        model.running_clear_rpm,
        model.running_set_rpm,
        "model.running_clear_rpm",
        "model.running_set_rpm",
    );
    require_lt(
        report,
        model.eta_indicated_min,
        model.eta_indicated_max,
        "model.eta_indicated_min",
        "model.eta_indicated_max",
    );
    require_lt(
        report,
        model.intake_pressure_min_pa,
        model.intake_pressure_max_pa,
        "model.intake_pressure_min_pa",
        "model.intake_pressure_max_pa",
    );

    if model.eta_indicated_max > 0.52 {
        report.warning(format!(
            "'model.eta_indicated_max' = {} is above a typical SI indicated-efficiency ceiling",
            model.eta_indicated_max
        ));
    }
}

fn audit_numerics(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let n = &cfg.numerics;
    for (value, path, min, max) in [
        (
            n.rpm_link_base_dt_min_s,
            "numerics.rpm_link_base_dt_min_s",
            1.0e-6,
            0.05,
        ),
        (
            n.rpm_link_reference_rpm_min,
            "numerics.rpm_link_reference_rpm_min",
            10.0,
            2_000.0,
        ),
        (
            n.rpm_link_deg_per_step_min,
            "numerics.rpm_link_deg_per_step_min",
            0.1,
            90.0,
        ),
        (
            n.rpm_link_deg_per_step_max,
            "numerics.rpm_link_deg_per_step_max",
            0.1,
            180.0,
        ),
        (
            n.rpm_link_rpm_floor,
            "numerics.rpm_link_rpm_floor",
            1.0,
            5_000.0,
        ),
        (
            n.rpm_link_dt_min_factor,
            "numerics.rpm_link_dt_min_factor",
            0.001,
            10.0,
        ),
        (
            n.rpm_link_dt_min_floor_s,
            "numerics.rpm_link_dt_min_floor_s",
            1.0e-7,
            0.05,
        ),
        (
            n.rpm_link_dt_max_factor,
            "numerics.rpm_link_dt_max_factor",
            0.01,
            100.0,
        ),
        (
            n.rpm_link_dt_max_cap_s,
            "numerics.rpm_link_dt_max_cap_s",
            1.0e-6,
            0.5,
        ),
        (
            n.state_error_omega_bias,
            "numerics.state_error_omega_bias",
            1.0e-6,
            10_000.0,
        ),
        (
            n.state_error_theta_scale_rad,
            "numerics.state_error_theta_scale_rad",
            1.0e-8,
            10.0,
        ),
        (
            n.state_error_p_intake_bias_pa,
            "numerics.state_error_p_intake_bias_pa",
            1.0e-6,
            1.0e6,
        ),
        (
            n.state_error_p_intake_rel,
            "numerics.state_error_p_intake_rel",
            0.0,
            10.0,
        ),
        (
            n.state_error_p_intake_runner_bias_pa,
            "numerics.state_error_p_intake_runner_bias_pa",
            1.0e-6,
            1.0e6,
        ),
        (
            n.state_error_p_intake_runner_rel,
            "numerics.state_error_p_intake_runner_rel",
            0.0,
            10.0,
        ),
        (
            n.state_error_p_exhaust_bias_pa,
            "numerics.state_error_p_exhaust_bias_pa",
            1.0e-6,
            1.0e6,
        ),
        (
            n.state_error_p_exhaust_rel,
            "numerics.state_error_p_exhaust_rel",
            0.0,
            10.0,
        ),
        (
            n.state_error_p_exhaust_runner_bias_pa,
            "numerics.state_error_p_exhaust_runner_bias_pa",
            1.0e-6,
            1.0e6,
        ),
        (
            n.state_error_p_exhaust_runner_rel,
            "numerics.state_error_p_exhaust_runner_rel",
            0.0,
            10.0,
        ),
        (
            n.state_error_m_dot_intake_runner_scale_kg_s,
            "numerics.state_error_m_dot_intake_runner_scale_kg_s",
            1.0e-6,
            10.0,
        ),
        (
            n.state_error_m_dot_exhaust_runner_scale_kg_s,
            "numerics.state_error_m_dot_exhaust_runner_scale_kg_s",
            1.0e-6,
            10.0,
        ),
        (
            n.state_error_throttle_scale,
            "numerics.state_error_throttle_scale",
            1.0e-6,
            10.0,
        ),
        (
            n.realtime_probe_dt_min_s,
            "numerics.realtime_probe_dt_min_s",
            1.0e-7,
            0.05,
        ),
        (
            n.realtime_margin_factor,
            "numerics.realtime_margin_factor",
            0.1,
            10.0,
        ),
        (
            n.realtime_floor_min_s,
            "numerics.realtime_floor_min_s",
            1.0e-7,
            0.05,
        ),
        (
            n.realtime_floor_probe_factor_max,
            "numerics.realtime_floor_probe_factor_max",
            0.1,
            100.0,
        ),
        (
            n.realtime_fixed_dt_headroom_ratio,
            "numerics.realtime_fixed_dt_headroom_ratio",
            0.1,
            100.0,
        ),
        (
            n.realtime_fixed_dt_max_deg_per_step,
            "numerics.realtime_fixed_dt_max_deg_per_step",
            0.1,
            180.0,
        ),
        (
            n.accuracy_target_deg_per_step,
            "numerics.accuracy_target_deg_per_step",
            0.1,
            30.0,
        ),
        (
            n.accuracy_dt_max_s,
            "numerics.accuracy_dt_max_s",
            1.0e-6,
            0.05,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_usize_range(
        report,
        n.realtime_warmup_steps,
        "numerics.realtime_warmup_steps",
        1,
        1_000_000,
    );
    require_usize_range(
        report,
        n.realtime_sample_steps,
        "numerics.realtime_sample_steps",
        1,
        1_000_000,
    );
    require_lt(
        report,
        n.rpm_link_deg_per_step_min,
        n.rpm_link_deg_per_step_max,
        "numerics.rpm_link_deg_per_step_min",
        "numerics.rpm_link_deg_per_step_max",
    );
    require_lt(
        report,
        n.rpm_link_dt_min_floor_s,
        n.rpm_link_dt_max_cap_s,
        "numerics.rpm_link_dt_min_floor_s",
        "numerics.rpm_link_dt_max_cap_s",
    );
}

fn audit_ui(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let ui = &cfg.ui;
    if ui.simulated_time_per_frame_s <= 0.0 {
        report.error("ui.simulated_time_per_frame_s must be > 0");
    }
    for (value, path, min, max) in [
        (
            ui.simulated_time_per_frame_s,
            "ui.simulated_time_per_frame_s",
            1.0e-4,
            0.5,
        ),
        (ui.min_base_dt_s, "ui.min_base_dt_s", 1.0e-7, 0.1),
        (
            ui.realtime_dt_min_factor,
            "ui.realtime_dt_min_factor",
            0.001,
            10.0,
        ),
        (
            ui.realtime_dt_max_factor,
            "ui.realtime_dt_max_factor",
            0.01,
            100.0,
        ),
        (
            ui.realtime_dt_max_over_min_factor,
            "ui.realtime_dt_max_over_min_factor",
            1.0,
            1_000.0,
        ),
        (ui.throttle_key_step, "ui.throttle_key_step", 0.0, 1.0),
        (ui.dt_smoothing_factor, "ui.dt_smoothing_factor", 0.0, 1.0),
        (ui.dt_epsilon_s, "ui.dt_epsilon_s", 0.0, 0.1),
        (ui.vvt_slider_min_deg, "ui.vvt_slider_min_deg", -90.0, 90.0),
        (ui.vvt_slider_max_deg, "ui.vvt_slider_max_deg", -90.0, 90.0),
        (
            ui.ignition_slider_min_deg,
            "ui.ignition_slider_min_deg",
            -30.0,
            80.0,
        ),
        (
            ui.ignition_slider_max_deg,
            "ui.ignition_slider_max_deg",
            -30.0,
            100.0,
        ),
        (ui.plot_height_px as f64, "ui.plot_height_px", 50.0, 2_000.0),
        (
            ui.pv_plot_height_px as f64,
            "ui.pv_plot_height_px",
            50.0,
            2_000.0,
        ),
        (ui.line_width_px as f64, "ui.line_width_px", 0.1, 20.0),
        (
            ui.crank_line_width_px as f64,
            "ui.crank_line_width_px",
            0.1,
            20.0,
        ),
        (ui.torque_min_span_nm, "ui.torque_min_span_nm", 1.0, 500.0),
        (ui.torque_margin_ratio, "ui.torque_margin_ratio", 0.0, 2.0),
        (ui.torque_floor_abs_nm, "ui.torque_floor_abs_nm", 0.0, 500.0),
        (
            ui.trapped_air_min_y_max_mg,
            "ui.trapped_air_min_y_max_mg",
            1.0,
            10_000.0,
        ),
        (
            ui.trapped_air_headroom_ratio,
            "ui.trapped_air_headroom_ratio",
            0.0,
            2.0,
        ),
        (ui.pv_headroom_ratio, "ui.pv_headroom_ratio", 0.0, 2.0),
        (ui.pv_min_headroom_kpa, "ui.pv_min_headroom_kpa", 0.0, 500.0),
        (
            ui.window_width_px as f64,
            "ui.window_width_px",
            200.0,
            8_000.0,
        ),
        (
            ui.window_height_px as f64,
            "ui.window_height_px",
            200.0,
            8_000.0,
        ),
    ] {
        require_range(report, value, path, min, max);
    }
    require_usize_range(
        report,
        ui.max_steps_per_frame,
        "ui.max_steps_per_frame",
        1,
        1_000_000,
    );
    require_u32_range(report, ui.repaint_hz, "ui.repaint_hz", 1, 240);
    require_lt(
        report,
        ui.vvt_slider_min_deg,
        ui.vvt_slider_max_deg,
        "ui.vvt_slider_min_deg",
        "ui.vvt_slider_max_deg",
    );
    require_lt(
        report,
        ui.ignition_slider_min_deg,
        ui.ignition_slider_max_deg,
        "ui.ignition_slider_min_deg",
        "ui.ignition_slider_max_deg",
    );
}

fn audit_plot(cfg: &AppConfig, report: &mut ConfigAuditReport) {
    let plot = &cfg.plot;
    require_usize_range(
        report,
        plot.rpm_history_capacity,
        "plot.rpm_history_capacity",
        16,
        1_000_000,
    );
    require_usize_range(
        report,
        plot.history_recent_cycles,
        "plot.history_recent_cycles",
        1,
        64,
    );
    require_usize_range(
        report,
        plot.pv_recent_cycles,
        "plot.pv_recent_cycles",
        1,
        64,
    );
    require_usize_range(
        report,
        plot.pv_subsamples_per_step,
        "plot.pv_subsamples_per_step",
        1,
        1_000,
    );
    require_lt(
        report,
        plot.pv_x_min,
        plot.pv_x_max,
        "plot.pv_x_min",
        "plot.pv_x_max",
    );
    require_lt(
        report,
        plot.pv_y_min_kpa,
        plot.pv_y_max_kpa,
        "plot.pv_y_min_kpa",
        "plot.pv_y_max_kpa",
    );
}

fn require_range(report: &mut ConfigAuditReport, value: f64, path: &str, min: f64, max: f64) {
    if !value.is_finite() {
        report.error(format!("'{path}' must be finite"));
    } else if !(min..=max).contains(&value) {
        report.error(format!(
            "'{path}' must stay within [{min}, {max}] but was {value}"
        ));
    }
}

fn require_u32_range(report: &mut ConfigAuditReport, value: u32, path: &str, min: u32, max: u32) {
    if !(min..=max).contains(&value) {
        report.error(format!(
            "'{path}' must stay within [{min}, {max}] but was {value}"
        ));
    }
}

fn require_usize_range(
    report: &mut ConfigAuditReport,
    value: usize,
    path: &str,
    min: usize,
    max: usize,
) {
    if !(min..=max).contains(&value) {
        report.error(format!(
            "'{path}' must stay within [{min}, {max}] but was {value}"
        ));
    }
}

fn require_unit_interval(report: &mut ConfigAuditReport, value: f64, path: &str) {
    require_range(report, value, path, 0.0, 1.0);
}

fn require_lt(report: &mut ConfigAuditReport, a: f64, b: f64, a_name: &str, b_name: &str) {
    if !(a < b) {
        report.error(format!("'{a_name}' must stay below '{b_name}'"));
    }
}

fn require_close_relative(
    report: &mut ConfigAuditReport,
    value: f64,
    reference: f64,
    path: &str,
    rel_tol: f64,
) {
    let scale = reference.abs().max(1.0);
    if (value - reference).abs() > rel_tol * scale {
        report.error(format!(
            "'{path}' must match its derived geometry value {reference} but was {value}"
        ));
    }
}

fn equivalent_diameter_m(area_m2: f64) -> f64 {
    if area_m2 <= 0.0 {
        return f64::NAN;
    }
    (4.0 * area_m2 / PI).sqrt()
}

fn audit_external_load(load: &ExternalLoadConfig, prefix: &str, report: &mut ConfigAuditReport) {
    for (value, path, min, max) in [
        (
            load.command_exponent,
            format!("{prefix}.command_exponent"),
            0.1,
            5.0,
        ),
        (
            load.base_torque_nm,
            format!("{prefix}.base_torque_nm"),
            0.0,
            2_000.0,
        ),
        (
            load.speed_linear_nms,
            format!("{prefix}.speed_linear_nms"),
            0.0,
            1.0,
        ),
        (
            load.speed_quadratic_nms2,
            format!("{prefix}.speed_quadratic_nms2"),
            0.0,
            0.1,
        ),
        (
            load.torque_min_nm,
            format!("{prefix}.torque_min_nm"),
            -5_000.0,
            2_000.0,
        ),
        (
            load.torque_max_nm,
            format!("{prefix}.torque_max_nm"),
            0.0,
            5_000.0,
        ),
        (
            load.absorber_rotor_inertia_kgm2,
            format!("{prefix}.absorber_rotor_inertia_kgm2"),
            0.0,
            20.0,
        ),
        (
            load.absorber_power_limit_kw,
            format!("{prefix}.absorber_power_limit_kw"),
            1.0,
            5_000.0,
        ),
        (
            load.absorber_speed_limit_rpm,
            format!("{prefix}.absorber_speed_limit_rpm"),
            200.0,
            50_000.0,
        ),
        (
            load.vehicle.vehicle_mass_kg,
            format!("{prefix}.vehicle.vehicle_mass_kg"),
            300.0,
            40_000.0,
        ),
        (
            load.vehicle.equivalent_mass_factor,
            format!("{prefix}.vehicle.equivalent_mass_factor"),
            0.5,
            3.0,
        ),
        (
            load.vehicle.wheel_radius_m,
            format!("{prefix}.vehicle.wheel_radius_m"),
            0.15,
            0.60,
        ),
        (
            load.vehicle.drivetrain_ratio,
            format!("{prefix}.vehicle.drivetrain_ratio"),
            1.0,
            40.0,
        ),
        (
            load.vehicle.driveline_efficiency,
            format!("{prefix}.vehicle.driveline_efficiency"),
            0.1,
            1.0,
        ),
        (
            load.vehicle.rolling_resistance_coeff,
            format!("{prefix}.vehicle.rolling_resistance_coeff"),
            0.0,
            0.10,
        ),
        (
            load.vehicle.drag_coeff,
            format!("{prefix}.vehicle.drag_coeff"),
            0.1,
            1.5,
        ),
        (
            load.vehicle.frontal_area_m2,
            format!("{prefix}.vehicle.frontal_area_m2"),
            0.5,
            15.0,
        ),
        (
            load.vehicle.air_density_kg_m3,
            format!("{prefix}.vehicle.air_density_kg_m3"),
            0.5,
            2.0,
        ),
        (
            load.vehicle.road_grade_percent,
            format!("{prefix}.vehicle.road_grade_percent"),
            -40.0,
            40.0,
        ),
        (
            load.vehicle.accessory_torque_nm,
            format!("{prefix}.vehicle.accessory_torque_nm"),
            0.0,
            500.0,
        ),
    ] {
        require_range(report, value, &path, min, max);
    }
    require_lt(
        report,
        load.torque_min_nm,
        load.torque_max_nm,
        &format!("{prefix}.torque_min_nm"),
        &format!("{prefix}.torque_max_nm"),
    );
}
