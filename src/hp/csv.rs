use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::hp::model::{OperatingPointResult, PvRecord, SweepResult, TorqueCurveSample};
use crate::hp::report::{format_point_report, format_sweep_report, summarize_curve};

pub fn write_outputs(output_dir: &Path, result: &SweepResult) -> Result<(), String> {
    fs::create_dir_all(output_dir)
        .map_err(|err| format!("failed to create '{}': {err}", output_dir.display()))?;
    write_curve_csv(&output_dir.join("torque_curve.csv"), &result.samples)?;
    write_curve_summary_csv(&output_dir.join("torque_summary.csv"), result)?;
    for (rpm, pv_records) in &result.pv_records_by_rpm {
        let filename = format!("pv_{:.0}rpm.csv", rpm);
        write_pv_csv(&output_dir.join(filename), pv_records)?;
    }
    Ok(())
}

pub fn write_sweep_report(
    output_dir: &Path,
    config: &crate::hp::config::HeadlessConfig,
    result: &SweepResult,
) -> Result<(), String> {
    fs::create_dir_all(output_dir)
        .map_err(|err| format!("failed to create '{}': {err}", output_dir.display()))?;
    fs::write(
        output_dir.join("sweep_report.txt"),
        format_sweep_report(config, result),
    )
    .map_err(|err| format!("failed to write sweep report: {err}"))
}

pub fn write_point_outputs(
    output_dir: &Path,
    config: &crate::hp::config::HeadlessConfig,
    point: &OperatingPointResult,
) -> Result<(), String> {
    fs::create_dir_all(output_dir)
        .map_err(|err| format!("failed to create '{}': {err}", output_dir.display()))?;
    let summary_path = output_dir.join(format!("point_{:.0}rpm_summary.txt", point.sample.rpm));
    fs::write(&summary_path, format_point_report(config, point))
        .map_err(|err| format!("failed to write '{}': {err}", summary_path.display()))?;
    if !point.pv_records.is_empty() {
        let pv_path = output_dir.join(format!("pv_{:.0}rpm.csv", point.sample.rpm));
        write_pv_csv(&pv_path, &point.pv_records)?;
    }
    Ok(())
}

fn write_curve_csv(path: &Path, rows: &[TorqueCurveSample]) -> Result<(), String> {
    let file = File::create(path)
        .map_err(|err| format!("failed to create '{}': {err}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(
        writer,
        "rpm,indicated_torque_nm,brake_torque_nm,indicated_power_kw,brake_power_kw,trapped_air_mg,volumetric_efficiency,bmep_bar"
    )
    .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    for row in rows {
        writeln!(
            writer,
            "{:.1},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            row.rpm,
            row.indicated_torque_nm,
            row.brake_torque_nm,
            row.indicated_power_kw,
            row.brake_power_kw,
            row.trapped_air_mg,
            row.volumetric_efficiency,
            row.bmep_bar
        )
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    }
    Ok(())
}

fn write_curve_summary_csv(path: &Path, result: &SweepResult) -> Result<(), String> {
    let file = File::create(path)
        .map_err(|err| format!("failed to create '{}': {err}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "metric,rpm,value,unit")
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    if let Some(summary) = summarize_curve(&result.samples) {
        let peak_torque = &result.samples[summary.peak_torque_index];
        let peak_power = &result.samples[summary.peak_power_index];
        writeln!(
            writer,
            "peak_brake_torque,{:.1},{:.6},Nm",
            peak_torque.rpm, peak_torque.brake_torque_nm
        )
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
        writeln!(
            writer,
            "peak_brake_power,{:.1},{:.6},kW",
            peak_power.rpm, peak_power.brake_power_kw
        )
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    }
    Ok(())
}

fn write_pv_csv(path: &Path, rows: &[PvRecord]) -> Result<(), String> {
    let file = File::create(path)
        .map_err(|err| format!("failed to create '{}': {err}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(
        writer,
        "cycle_deg,cylinder_index,volume_m3,pressure_pa,temperature_k,mass_kg,intake_lift_m,exhaust_lift_m,gas_torque_nm"
    )
    .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    for row in rows {
        writeln!(
            writer,
            "{:.6},{},{:.9},{:.6},{:.6},{:.9},{:.9},{:.9},{:.6}",
            row.cycle_deg,
            row.cylinder_index,
            row.volume_m3,
            row.pressure_pa,
            row.temperature_k,
            row.mass_kg,
            row.intake_lift_m,
            row.exhaust_lift_m,
            row.gas_torque_nm
        )
        .map_err(|err| format!("failed to write '{}': {err}", path.display()))?;
    }
    Ok(())
}
