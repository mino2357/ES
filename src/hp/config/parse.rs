use std::collections::BTreeMap;

use crate::hp::yaml::YamlValue;

use super::{CombustionConfig, EngineConfig, HeadlessConfig, NumericsConfig, SweepConfig};

const ROOT_KEYS: &[&str] = &["output_dir", "engine", "combustion", "sweep", "numerics"];
const ENGINE_KEYS: &[&str] = &[
    "bore_m",
    "stroke_m",
    "conrod_m",
    "compression_ratio",
    "ambient_pressure_pa",
    "intake_temp_k",
    "exhaust_temp_k",
    "wall_temp_k",
    "plenum_volume_m3",
    "collector_volume_m3",
    "throttle_area_max_m2",
    "throttle_discharge_coeff",
    "tailpipe_area_m2",
    "tailpipe_discharge_coeff",
    "intake_runner_length_m",
    "intake_runner_diameter_m",
    "intake_runner_loss_coeff",
    "exhaust_runner_length_m",
    "exhaust_runner_diameter_m",
    "exhaust_runner_loss_coeff",
    "friction_c0_nm",
    "friction_c1_nms",
    "friction_c2_nms2",
    "intake_open_deg",
    "intake_close_deg",
    "exhaust_open_deg",
    "exhaust_close_deg",
    "intake_valve_diameter_m",
    "exhaust_valve_diameter_m",
    "intake_valve_cd",
    "exhaust_valve_cd",
    "intake_max_lift_m",
    "exhaust_max_lift_m",
];
const COMBUSTION_KEYS: &[&str] = &[
    "stoich_afr",
    "lambda_target",
    "fuel_lhv_j_per_kg",
    "ignition_advance_deg",
    "burn_duration_deg",
    "wall_htc_w_m2k",
];
const SWEEP_KEYS: &[&str] = &["throttle", "rpm_points", "pv_rpm_points"];
const NUMERICS_KEYS: &[&str] = &[
    "dt_deg",
    "intake_runner_cells",
    "exhaust_runner_cells",
    "warmup_cycles",
    "sample_cycles",
];

pub(super) fn parse_headless_config(root: &YamlValue) -> Result<HeadlessConfig, String> {
    let map = expect_map(root, "root")?;
    reject_unknown_keys(map, "root", ROOT_KEYS)?;
    let output_dir = get_string(get_required(map, "output_dir")?, "output_dir")?;

    let engine_map = expect_map(get_required(map, "engine")?, "engine")?;
    reject_unknown_keys(engine_map, "engine", ENGINE_KEYS)?;
    let combustion_map = expect_map(get_required(map, "combustion")?, "combustion")?;
    reject_unknown_keys(combustion_map, "combustion", COMBUSTION_KEYS)?;
    let sweep_map = expect_map(get_required(map, "sweep")?, "sweep")?;
    reject_unknown_keys(sweep_map, "sweep", SWEEP_KEYS)?;
    let numerics_map = expect_map(get_required(map, "numerics")?, "numerics")?;
    reject_unknown_keys(numerics_map, "numerics", NUMERICS_KEYS)?;

    Ok(HeadlessConfig {
        output_dir,
        engine: EngineConfig {
            bore_m: get_f64(get_required(engine_map, "bore_m")?, "engine.bore_m")?,
            stroke_m: get_f64(get_required(engine_map, "stroke_m")?, "engine.stroke_m")?,
            conrod_m: get_f64(get_required(engine_map, "conrod_m")?, "engine.conrod_m")?,
            compression_ratio: get_f64(
                get_required(engine_map, "compression_ratio")?,
                "engine.compression_ratio",
            )?,
            ambient_pressure_pa: get_f64(
                get_required(engine_map, "ambient_pressure_pa")?,
                "engine.ambient_pressure_pa",
            )?,
            intake_temp_k: get_f64(
                get_required(engine_map, "intake_temp_k")?,
                "engine.intake_temp_k",
            )?,
            exhaust_temp_k: get_f64(
                get_required(engine_map, "exhaust_temp_k")?,
                "engine.exhaust_temp_k",
            )?,
            wall_temp_k: get_f64(
                get_required(engine_map, "wall_temp_k")?,
                "engine.wall_temp_k",
            )?,
            plenum_volume_m3: get_f64(
                get_required(engine_map, "plenum_volume_m3")?,
                "engine.plenum_volume_m3",
            )?,
            collector_volume_m3: get_f64(
                get_required(engine_map, "collector_volume_m3")?,
                "engine.collector_volume_m3",
            )?,
            throttle_area_max_m2: get_f64(
                get_required(engine_map, "throttle_area_max_m2")?,
                "engine.throttle_area_max_m2",
            )?,
            throttle_discharge_coeff: get_f64(
                get_required(engine_map, "throttle_discharge_coeff")?,
                "engine.throttle_discharge_coeff",
            )?,
            tailpipe_area_m2: get_f64(
                get_required(engine_map, "tailpipe_area_m2")?,
                "engine.tailpipe_area_m2",
            )?,
            tailpipe_discharge_coeff: get_f64(
                get_required(engine_map, "tailpipe_discharge_coeff")?,
                "engine.tailpipe_discharge_coeff",
            )?,
            intake_runner_length_m: get_f64(
                get_required(engine_map, "intake_runner_length_m")?,
                "engine.intake_runner_length_m",
            )?,
            intake_runner_diameter_m: get_f64(
                get_required(engine_map, "intake_runner_diameter_m")?,
                "engine.intake_runner_diameter_m",
            )?,
            intake_runner_loss_coeff: get_f64(
                get_required(engine_map, "intake_runner_loss_coeff")?,
                "engine.intake_runner_loss_coeff",
            )?,
            exhaust_runner_length_m: get_f64(
                get_required(engine_map, "exhaust_runner_length_m")?,
                "engine.exhaust_runner_length_m",
            )?,
            exhaust_runner_diameter_m: get_f64(
                get_required(engine_map, "exhaust_runner_diameter_m")?,
                "engine.exhaust_runner_diameter_m",
            )?,
            exhaust_runner_loss_coeff: get_f64(
                get_required(engine_map, "exhaust_runner_loss_coeff")?,
                "engine.exhaust_runner_loss_coeff",
            )?,
            friction_c0_nm: get_f64(
                get_required(engine_map, "friction_c0_nm")?,
                "engine.friction_c0_nm",
            )?,
            friction_c1_nms: get_f64(
                get_required(engine_map, "friction_c1_nms")?,
                "engine.friction_c1_nms",
            )?,
            friction_c2_nms2: get_f64(
                get_required(engine_map, "friction_c2_nms2")?,
                "engine.friction_c2_nms2",
            )?,
            intake_open_deg: get_f64(
                get_required(engine_map, "intake_open_deg")?,
                "engine.intake_open_deg",
            )?,
            intake_close_deg: get_f64(
                get_required(engine_map, "intake_close_deg")?,
                "engine.intake_close_deg",
            )?,
            exhaust_open_deg: get_f64(
                get_required(engine_map, "exhaust_open_deg")?,
                "engine.exhaust_open_deg",
            )?,
            exhaust_close_deg: get_f64(
                get_required(engine_map, "exhaust_close_deg")?,
                "engine.exhaust_close_deg",
            )?,
            intake_valve_diameter_m: get_f64(
                get_required(engine_map, "intake_valve_diameter_m")?,
                "engine.intake_valve_diameter_m",
            )?,
            exhaust_valve_diameter_m: get_f64(
                get_required(engine_map, "exhaust_valve_diameter_m")?,
                "engine.exhaust_valve_diameter_m",
            )?,
            intake_valve_cd: get_f64(
                get_required(engine_map, "intake_valve_cd")?,
                "engine.intake_valve_cd",
            )?,
            exhaust_valve_cd: get_f64(
                get_required(engine_map, "exhaust_valve_cd")?,
                "engine.exhaust_valve_cd",
            )?,
            intake_max_lift_m: get_f64(
                get_required(engine_map, "intake_max_lift_m")?,
                "engine.intake_max_lift_m",
            )?,
            exhaust_max_lift_m: get_f64(
                get_required(engine_map, "exhaust_max_lift_m")?,
                "engine.exhaust_max_lift_m",
            )?,
        },
        combustion: CombustionConfig {
            stoich_afr: get_f64(
                get_required(combustion_map, "stoich_afr")?,
                "combustion.stoich_afr",
            )?,
            lambda_target: get_f64(
                get_required(combustion_map, "lambda_target")?,
                "combustion.lambda_target",
            )?,
            fuel_lhv_j_per_kg: get_f64(
                get_required(combustion_map, "fuel_lhv_j_per_kg")?,
                "combustion.fuel_lhv_j_per_kg",
            )?,
            ignition_advance_deg: get_f64(
                get_required(combustion_map, "ignition_advance_deg")?,
                "combustion.ignition_advance_deg",
            )?,
            burn_duration_deg: get_f64(
                get_required(combustion_map, "burn_duration_deg")?,
                "combustion.burn_duration_deg",
            )?,
            wall_htc_w_m2k: get_f64(
                get_required(combustion_map, "wall_htc_w_m2k")?,
                "combustion.wall_htc_w_m2k",
            )?,
        },
        sweep: SweepConfig {
            throttle: get_f64(get_required(sweep_map, "throttle")?, "sweep.throttle")?,
            rpm_points: get_f64_list(get_required(sweep_map, "rpm_points")?, "sweep.rpm_points")?,
            pv_rpm_points: get_f64_list(
                get_required(sweep_map, "pv_rpm_points")?,
                "sweep.pv_rpm_points",
            )?,
        },
        numerics: NumericsConfig {
            dt_deg: get_f64(get_required(numerics_map, "dt_deg")?, "numerics.dt_deg")?,
            intake_runner_cells: get_usize(
                get_required(numerics_map, "intake_runner_cells")?,
                "numerics.intake_runner_cells",
            )?,
            exhaust_runner_cells: get_usize(
                get_required(numerics_map, "exhaust_runner_cells")?,
                "numerics.exhaust_runner_cells",
            )?,
            warmup_cycles: get_usize(
                get_required(numerics_map, "warmup_cycles")?,
                "numerics.warmup_cycles",
            )?,
            sample_cycles: get_usize(
                get_required(numerics_map, "sample_cycles")?,
                "numerics.sample_cycles",
            )?,
        },
    })
}

// Fail fast on stale keys so the checked-in YAML cannot silently accumulate dead knobs.
fn reject_unknown_keys(
    map: &BTreeMap<String, YamlValue>,
    path: &str,
    allowed: &[&str],
) -> Result<(), String> {
    for key in map.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(format!("unknown key '{path}.{key}'"));
        }
    }
    Ok(())
}

fn get_required<'a>(
    map: &'a BTreeMap<String, YamlValue>,
    key: &str,
) -> Result<&'a YamlValue, String> {
    map.get(key)
        .ok_or_else(|| format!("missing required key '{key}'"))
}

fn expect_map<'a>(
    value: &'a YamlValue,
    path: &str,
) -> Result<&'a BTreeMap<String, YamlValue>, String> {
    match value {
        YamlValue::Map(map) => Ok(map),
        _ => Err(format!("'{path}' must be a YAML map")),
    }
}

fn get_f64(value: &YamlValue, path: &str) -> Result<f64, String> {
    match value {
        YamlValue::Number(number) => Ok(*number),
        _ => Err(format!("'{path}' must be numeric")),
    }
}

fn get_usize(value: &YamlValue, path: &str) -> Result<usize, String> {
    let number = get_f64(value, path)?;
    if number < 0.0 || (number.fract()).abs() > 1.0e-12 {
        return Err(format!("'{path}' must be a non-negative integer"));
    }
    Ok(number as usize)
}

fn get_string(value: &YamlValue, path: &str) -> Result<String, String> {
    match value {
        YamlValue::String(string) => Ok(string.clone()),
        _ => Err(format!("'{path}' must be a string")),
    }
}

fn get_f64_list(value: &YamlValue, path: &str) -> Result<Vec<f64>, String> {
    let YamlValue::List(list) = value else {
        return Err(format!("'{path}' must be a YAML list"));
    };
    list.iter()
        .enumerate()
        .map(|(index, item)| get_f64(item, &format!("{path}[{index}]")))
        .collect()
}
