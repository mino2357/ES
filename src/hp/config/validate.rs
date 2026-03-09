use super::{CYLINDER_COUNT, EngineConfig, HeadlessConfig};

pub(super) fn validate_headless_config(cfg: &HeadlessConfig) -> Result<(), String> {
    if cfg.output_dir.trim().is_empty() {
        return Err("'output_dir' must not be empty".to_string());
    }

    let engine = &cfg.engine;
    let combustion = &cfg.combustion;
    let sweep = &cfg.sweep;
    let numerics = &cfg.numerics;

    require_range(engine.bore_m, "engine.bore_m", 0.055, 0.110)?;
    require_range(engine.stroke_m, "engine.stroke_m", 0.055, 0.120)?;
    require_range(engine.conrod_m, "engine.conrod_m", 0.090, 0.230)?;
    require_range(
        engine.conrod_m / engine.stroke_m.max(f64::EPSILON),
        "engine.conrod_m / engine.stroke_m",
        1.35,
        2.20,
    )?;
    require_range(
        engine.compression_ratio,
        "engine.compression_ratio",
        7.0,
        16.5,
    )?;
    require_range(
        engine.ambient_pressure_pa,
        "engine.ambient_pressure_pa",
        80_000.0,
        110_000.0,
    )?;
    require_range(engine.intake_temp_k, "engine.intake_temp_k", 250.0, 370.0)?;
    require_range(engine.wall_temp_k, "engine.wall_temp_k", 320.0, 650.0)?;
    require_range(
        engine.exhaust_temp_k,
        "engine.exhaust_temp_k",
        500.0,
        1_300.0,
    )?;
    require_ordered_triplet(
        engine.intake_temp_k,
        engine.wall_temp_k,
        engine.exhaust_temp_k,
        "engine.intake_temp_k < engine.wall_temp_k < engine.exhaust_temp_k",
    )?;

    let displacement_total = total_displacement_m3(engine);
    require_range(
        engine.plenum_volume_m3,
        "engine.plenum_volume_m3",
        0.0005,
        0.020,
    )?;
    require_range(
        engine.plenum_volume_m3 / displacement_total.max(f64::EPSILON),
        "engine.plenum_volume_m3 / displacement_total",
        0.25,
        6.0,
    )?;
    require_range(
        engine.collector_volume_m3,
        "engine.collector_volume_m3",
        0.0004,
        0.020,
    )?;
    require_range(
        engine.collector_volume_m3 / displacement_total.max(f64::EPSILON),
        "engine.collector_volume_m3 / displacement_total",
        0.20,
        6.0,
    )?;

    require_equivalent_diameter(
        engine.throttle_area_max_m2,
        "engine.throttle_area_max_m2",
        0.025,
        0.090,
    )?;
    require_range(
        engine.throttle_discharge_coeff,
        "engine.throttle_discharge_coeff",
        0.55,
        1.00,
    )?;
    require_equivalent_diameter(
        engine.tailpipe_area_m2,
        "engine.tailpipe_area_m2",
        0.030,
        0.090,
    )?;
    require_range(
        engine.tailpipe_discharge_coeff,
        "engine.tailpipe_discharge_coeff",
        0.55,
        1.00,
    )?;

    require_range(
        engine.intake_runner_length_m,
        "engine.intake_runner_length_m",
        0.15,
        0.80,
    )?;
    require_range(
        engine.intake_runner_diameter_m,
        "engine.intake_runner_diameter_m",
        0.020,
        0.060,
    )?;
    require_range(
        engine.intake_runner_loss_coeff,
        "engine.intake_runner_loss_coeff",
        0.10,
        10.0,
    )?;
    require_range(
        engine.exhaust_runner_length_m,
        "engine.exhaust_runner_length_m",
        0.30,
        1.20,
    )?;
    require_range(
        engine.exhaust_runner_diameter_m,
        "engine.exhaust_runner_diameter_m",
        0.020,
        0.070,
    )?;
    require_range(
        engine.exhaust_runner_loss_coeff,
        "engine.exhaust_runner_loss_coeff",
        0.10,
        12.0,
    )?;

    require_range(engine.friction_c0_nm, "engine.friction_c0_nm", 0.0, 25.0)?;
    require_range(engine.friction_c1_nms, "engine.friction_c1_nms", 0.0, 0.05)?;
    require_range(
        engine.friction_c2_nms2,
        "engine.friction_c2_nms2",
        0.0,
        0.001,
    )?;
    // Derived checks catch coefficient combinations that look reasonable individually but imply absurd losses.
    for rpm in [1_000.0, 3_000.0, 7_000.0] {
        let omega_rad_s = rpm * std::f64::consts::PI / 30.0;
        let friction_torque = engine.friction_c0_nm
            + engine.friction_c1_nms * omega_rad_s
            + engine.friction_c2_nms2 * omega_rad_s.powi(2);
        require_range(
            friction_torque,
            &format!("friction torque at {rpm:.0} rpm"),
            2.0,
            80.0,
        )?;
    }

    require_cam_window(engine.intake_open_deg, engine.intake_close_deg, "intake")?;
    require_cam_window(engine.exhaust_open_deg, engine.exhaust_close_deg, "exhaust")?;
    require_range(
        engine.intake_valve_diameter_m,
        "engine.intake_valve_diameter_m",
        0.020,
        0.050,
    )?;
    require_range(
        engine.exhaust_valve_diameter_m,
        "engine.exhaust_valve_diameter_m",
        0.018,
        0.045,
    )?;
    require_range(engine.intake_valve_cd, "engine.intake_valve_cd", 0.55, 0.95)?;
    require_range(
        engine.exhaust_valve_cd,
        "engine.exhaust_valve_cd",
        0.55,
        0.95,
    )?;
    require_range(
        engine.intake_max_lift_m,
        "engine.intake_max_lift_m",
        0.004,
        0.018,
    )?;
    require_range(
        engine.exhaust_max_lift_m,
        "engine.exhaust_max_lift_m",
        0.004,
        0.018,
    )?;
    require_range(
        engine.intake_max_lift_m / engine.intake_valve_diameter_m.max(f64::EPSILON),
        "engine.intake_max_lift_m / engine.intake_valve_diameter_m",
        0.10,
        0.45,
    )?;
    require_range(
        engine.exhaust_max_lift_m / engine.exhaust_valve_diameter_m.max(f64::EPSILON),
        "engine.exhaust_max_lift_m / engine.exhaust_valve_diameter_m",
        0.10,
        0.45,
    )?;

    require_range(combustion.stoich_afr, "combustion.stoich_afr", 13.5, 15.5)?;
    require_range(
        combustion.lambda_target,
        "combustion.lambda_target",
        0.75,
        1.60,
    )?;
    require_range(
        combustion.fuel_lhv_j_per_kg,
        "combustion.fuel_lhv_j_per_kg",
        40.0e6,
        45.0e6,
    )?;
    require_range(
        combustion.ignition_advance_deg,
        "combustion.ignition_advance_deg",
        0.0,
        45.0,
    )?;
    require_range(
        combustion.burn_duration_deg,
        "combustion.burn_duration_deg",
        20.0,
        90.0,
    )?;
    require_range(
        combustion.wall_htc_w_m2k,
        "combustion.wall_htc_w_m2k",
        50.0,
        1_000.0,
    )?;

    require_range(sweep.throttle, "sweep.throttle", 0.0, 1.0)?;
    require_rpm_list(&sweep.rpm_points, "sweep.rpm_points")?;
    require_optional_rpm_list(&sweep.pv_rpm_points, "sweep.pv_rpm_points")?;
    for pv_rpm in &sweep.pv_rpm_points {
        if !sweep
            .rpm_points
            .iter()
            .any(|rpm| (rpm - pv_rpm).abs() < 0.5)
        {
            return Err(format!(
                "'sweep.pv_rpm_points' value {pv_rpm} must also appear in 'sweep.rpm_points'"
            ));
        }
    }

    require_range(numerics.dt_deg, "numerics.dt_deg", 0.05, 1.00)?;
    require_usize_range(
        numerics.intake_runner_cells,
        "numerics.intake_runner_cells",
        2,
        32,
    )?;
    require_usize_range(
        numerics.exhaust_runner_cells,
        "numerics.exhaust_runner_cells",
        2,
        32,
    )?;
    require_usize_range(numerics.warmup_cycles, "numerics.warmup_cycles", 1, 20)?;
    require_usize_range(numerics.sample_cycles, "numerics.sample_cycles", 1, 20)?;
    Ok(())
}

fn require_range(value: f64, path: &str, min: f64, max: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("'{path}' must be finite"));
    }
    if !(min..=max).contains(&value) {
        return Err(format!(
            "'{path}' must stay within [{min}, {max}] but was {value}"
        ));
    }
    Ok(())
}

fn require_usize_range(value: usize, path: &str, min: usize, max: usize) -> Result<(), String> {
    if !(min..=max).contains(&value) {
        return Err(format!(
            "'{path}' must stay within [{min}, {max}] but was {value}"
        ));
    }
    Ok(())
}

fn require_ordered_triplet(a: f64, b: f64, c: f64, label: &str) -> Result<(), String> {
    if a < b && b < c {
        Ok(())
    } else {
        Err(format!("'{label}' must hold"))
    }
}

fn require_equivalent_diameter(
    area_m2: f64,
    path: &str,
    min_diameter_m: f64,
    max_diameter_m: f64,
) -> Result<(), String> {
    require_range(area_m2, path, 1.0e-6, 0.01)?;
    let diameter_m = (4.0 * area_m2 / std::f64::consts::PI).sqrt();
    require_range(
        diameter_m,
        &format!("equivalent diameter of {path}"),
        min_diameter_m,
        max_diameter_m,
    )
}

fn require_cam_window(open_deg: f64, close_deg: f64, name: &str) -> Result<(), String> {
    require_range(open_deg, &format!("engine.{name}_open_deg"), 0.0, 720.0)?;
    require_range(close_deg, &format!("engine.{name}_close_deg"), 0.0, 720.0)?;
    let duration_deg = wrapped_duration_deg(open_deg, close_deg);
    require_range(
        duration_deg,
        &format!("{name} valve duration"),
        180.0,
        320.0,
    )?;
    Ok(())
}

fn require_rpm_list(values: &[f64], path: &str) -> Result<(), String> {
    if values.is_empty() {
        return Err(format!("'{path}' must not be empty"));
    }
    require_nonempty_sorted_rpm_list(values, path)
}

fn require_optional_rpm_list(values: &[f64], path: &str) -> Result<(), String> {
    if values.is_empty() {
        return Ok(());
    }
    require_nonempty_sorted_rpm_list(values, path)
}

fn require_nonempty_sorted_rpm_list(values: &[f64], path: &str) -> Result<(), String> {
    let mut previous = None;
    for (index, value) in values.iter().copied().enumerate() {
        require_range(value, &format!("{path}[{index}]"), 600.0, 10_000.0)?;
        if let Some(prev) = previous {
            if value <= prev {
                return Err(format!("'{path}' must be strictly increasing"));
            }
        }
        previous = Some(value);
    }
    Ok(())
}

fn wrapped_duration_deg(open_deg: f64, close_deg: f64) -> f64 {
    if close_deg >= open_deg {
        close_deg - open_deg
    } else {
        close_deg + 720.0 - open_deg
    }
}

fn total_displacement_m3(engine: &EngineConfig) -> f64 {
    0.25 * std::f64::consts::PI * engine.bore_m.powi(2) * engine.stroke_m * CYLINDER_COUNT as f64
}
