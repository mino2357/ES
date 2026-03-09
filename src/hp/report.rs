use std::f64::consts::PI;

use crate::hp::config::{CYLINDER_COUNT, HeadlessConfig};
use crate::hp::model::{OperatingPointResult, SweepResult, TorqueCurveSample};

#[derive(Debug, Clone, Copy)]
pub struct CurveSummary {
    pub peak_torque_index: usize,
    pub peak_power_index: usize,
}

pub fn summarize_curve(samples: &[TorqueCurveSample]) -> Option<CurveSummary> {
    if samples.is_empty() {
        return None;
    }
    let peak_torque_index = samples
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.brake_torque_nm.total_cmp(&b.1.brake_torque_nm))?
        .0;
    let peak_power_index = samples
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.brake_power_kw.total_cmp(&b.1.brake_power_kw))?
        .0;
    Some(CurveSummary {
        peak_torque_index,
        peak_power_index,
    })
}

pub fn format_sweep_report(config: &HeadlessConfig, result: &SweepResult) -> String {
    let mut lines = vec![format_config_metrics(config)];
    if let Some(summary) = summarize_curve(&result.samples) {
        let peak_torque = &result.samples[summary.peak_torque_index];
        let peak_power = &result.samples[summary.peak_power_index];
        lines.push(format!(
            "Peak brake torque: {:.1} Nm @ {:.0} rpm",
            peak_torque.brake_torque_nm, peak_torque.rpm
        ));
        lines.push(format!(
            "Peak brake power: {:.1} kW @ {:.0} rpm",
            peak_power.brake_power_kw, peak_power.rpm
        ));
        lines.push(format!(
            "PV captures written for {} operating point(s)",
            result.pv_records_by_rpm.len()
        ));
    }
    lines.join("\n")
}

pub fn format_point_report(config: &HeadlessConfig, point: &OperatingPointResult) -> String {
    format!(
        "{}\nOperating point: {:.0} rpm\nBrake torque: {:.2} Nm\nBrake power: {:.2} kW\nIndicated torque: {:.2} Nm\nTrapped air: {:.2} mg/cycle/cyl\nVolumetric efficiency: {:.3}\nBMEP: {:.3} bar\nPV samples: {}",
        format_config_metrics(config),
        point.sample.rpm,
        point.sample.brake_torque_nm,
        point.sample.brake_power_kw,
        point.sample.indicated_torque_nm,
        point.sample.trapped_air_mg,
        point.sample.volumetric_efficiency,
        point.sample.bmep_bar,
        point.pv_records.len()
    )
}

pub fn format_config_metrics(config: &HeadlessConfig) -> String {
    let engine = &config.engine;
    let displacement_total_m3 =
        0.25 * PI * engine.bore_m.powi(2) * engine.stroke_m * CYLINDER_COUNT as f64;
    let displacement_l = displacement_total_m3 * 1.0e3;
    let rod_ratio = engine.conrod_m / engine.stroke_m.max(f64::EPSILON);
    let throttle_diameter_mm = (4.0 * engine.throttle_area_max_m2 / PI).sqrt() * 1.0e3;
    let tailpipe_diameter_mm = (4.0 * engine.tailpipe_area_m2 / PI).sqrt() * 1.0e3;
    let max_rpm = config
        .sweep
        .rpm_points
        .iter()
        .copied()
        .max_by(f64::total_cmp)
        .unwrap_or(0.0);
    let mean_piston_speed = 2.0 * engine.stroke_m * max_rpm / 60.0;
    format!(
        "Displacement: {:.3} L\nCompression ratio: {:.2}\nRod ratio: {:.3}\nThrottle eq. diameter: {:.1} mm\nTailpipe eq. diameter: {:.1} mm\nMean piston speed @ {:.0} rpm: {:.2} m/s",
        displacement_l,
        engine.compression_ratio,
        rod_ratio,
        throttle_diameter_mm,
        tailpipe_diameter_mm,
        max_rpm,
        mean_piston_speed
    )
}
