mod parse;
mod validate;

use std::fs;
use std::path::Path;

use crate::hp::yaml::{YamlValue, parse_document};

use self::parse::parse_headless_config;
use self::validate::validate_headless_config;

pub const CYLINDER_COUNT: usize = 4;
pub const DEFAULT_WIEBE_A: f64 = 5.0;
pub const DEFAULT_WIEBE_M: f64 = 2.0;

#[derive(Debug, Clone)]
pub struct HeadlessConfig {
    pub output_dir: String,
    pub engine: EngineConfig,
    pub combustion: CombustionConfig,
    pub sweep: SweepConfig,
    pub numerics: NumericsConfig,
}

#[derive(Debug, Clone)]
pub struct EngineConfig {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub conrod_m: f64,
    pub compression_ratio: f64,
    pub ambient_pressure_pa: f64,
    pub intake_temp_k: f64,
    pub exhaust_temp_k: f64,
    pub wall_temp_k: f64,
    pub plenum_volume_m3: f64,
    pub collector_volume_m3: f64,
    pub throttle_area_max_m2: f64,
    pub throttle_discharge_coeff: f64,
    pub tailpipe_area_m2: f64,
    pub tailpipe_discharge_coeff: f64,
    pub intake_runner_length_m: f64,
    pub intake_runner_diameter_m: f64,
    pub intake_runner_loss_coeff: f64,
    pub exhaust_runner_length_m: f64,
    pub exhaust_runner_diameter_m: f64,
    pub exhaust_runner_loss_coeff: f64,
    pub friction_c0_nm: f64,
    pub friction_c1_nms: f64,
    pub friction_c2_nms2: f64,
    pub intake_open_deg: f64,
    pub intake_close_deg: f64,
    pub exhaust_open_deg: f64,
    pub exhaust_close_deg: f64,
    pub intake_valve_diameter_m: f64,
    pub exhaust_valve_diameter_m: f64,
    pub intake_valve_cd: f64,
    pub exhaust_valve_cd: f64,
    pub intake_max_lift_m: f64,
    pub exhaust_max_lift_m: f64,
}

#[derive(Debug, Clone)]
pub struct CombustionConfig {
    pub stoich_afr: f64,
    pub lambda_target: f64,
    pub fuel_lhv_j_per_kg: f64,
    pub ignition_advance_deg: f64,
    pub burn_duration_deg: f64,
    pub wall_htc_w_m2k: f64,
}

#[derive(Debug, Clone)]
pub struct SweepConfig {
    pub throttle: f64,
    pub rpm_points: Vec<f64>,
    pub pv_rpm_points: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct NumericsConfig {
    pub dt_deg: f64,
    pub intake_runner_cells: usize,
    pub exhaust_runner_cells: usize,
    pub warmup_cycles: usize,
    pub sample_cycles: usize,
}

impl HeadlessConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .map_err(|err| format!("failed to read '{}': {err}", path.display()))?;
        let parsed = parse_document(&text)?;
        Self::from_yaml(&parsed)
    }

    pub fn from_yaml(root: &YamlValue) -> Result<Self, String> {
        let cfg = parse_headless_config(root)?;
        validate_headless_config(&cfg)?;
        Ok(cfg)
    }
}

#[cfg(test)]
mod tests {
    use super::{CYLINDER_COUNT, DEFAULT_WIEBE_A, DEFAULT_WIEBE_M, HeadlessConfig};
    use crate::hp::yaml::parse_document;

    fn reference_document() -> String {
        r#"
output_dir: "dist/hp"
engine:
  bore_m: 0.0805
  stroke_m: 0.0976
  conrod_m: 0.154
  compression_ratio: 13.0
  ambient_pressure_pa: 101325.0
  intake_temp_k: 305.0
  exhaust_temp_k: 930.0
  wall_temp_k: 420.0
  plenum_volume_m3: 0.0032
  collector_volume_m3: 0.0050
  throttle_area_max_m2: 0.0027
  throttle_discharge_coeff: 0.82
  tailpipe_area_m2: 0.0022
  tailpipe_discharge_coeff: 0.87
  intake_runner_length_m: 0.36
  intake_runner_diameter_m: 0.034
  intake_runner_loss_coeff: 1.6
  exhaust_runner_length_m: 0.74
  exhaust_runner_diameter_m: 0.036
  exhaust_runner_loss_coeff: 1.9
  friction_c0_nm: 5.1
  friction_c1_nms: 0.0040
  friction_c2_nms2: 0.000020
  intake_open_deg: 340.0
  intake_close_deg: 580.0
  exhaust_open_deg: 140.0
  exhaust_close_deg: 380.0
  intake_valve_diameter_m: 0.032
  exhaust_valve_diameter_m: 0.028
  intake_valve_cd: 0.72
  exhaust_valve_cd: 0.74
  intake_max_lift_m: 0.0105
  exhaust_max_lift_m: 0.0098
combustion:
  stoich_afr: 14.7
  lambda_target: 0.88
  fuel_lhv_j_per_kg: 43000000.0
  ignition_advance_deg: 20.0
  burn_duration_deg: 42.0
  wall_htc_w_m2k: 220.0
sweep:
  throttle: 1.0
  rpm_points: [1500, 2500, 3500]
  pv_rpm_points: [3500]
numerics:
  dt_deg: 0.25
  intake_runner_cells: 6
  exhaust_runner_cells: 6
  warmup_cycles: 6
  sample_cycles: 4
"#
        .to_string()
    }

    #[test]
    fn headless_config_parses_reference_document() {
        let parsed = parse_document(&reference_document()).expect("yaml should parse");
        let cfg = HeadlessConfig::from_yaml(&parsed).expect("config should parse");
        assert_eq!(CYLINDER_COUNT, 4);
        assert_eq!(cfg.sweep.rpm_points.len(), 3);
        assert!((cfg.combustion.lambda_target - 0.88).abs() < 1.0e-12);
        assert_eq!(cfg.numerics.intake_runner_cells, 6);
        assert_eq!(DEFAULT_WIEBE_A, 5.0);
        assert_eq!(DEFAULT_WIEBE_M, 2.0);
    }

    #[test]
    fn headless_config_rejects_removed_nonphysical_knobs() {
        let doc = reference_document()
            .replace("engine:\n", "engine:\n  cylinders: 4\n")
            .replace(
                "  wall_htc_w_m2k: 220.0\n",
                "  wiebe_a: 5.0\n  wall_htc_w_m2k: 220.0\n",
            );
        let parsed = parse_document(&doc).expect("yaml should parse");
        let err = HeadlessConfig::from_yaml(&parsed).expect_err("config should reject old keys");
        assert!(err.contains("unknown key"));
    }

    #[test]
    fn headless_config_rejects_out_of_range_geometry() {
        let doc = reference_document().replace("  bore_m: 0.0805\n", "  bore_m: 0.160\n");
        let parsed = parse_document(&doc).expect("yaml should parse");
        let err = HeadlessConfig::from_yaml(&parsed).expect_err("config should reject geometry");
        assert!(err.contains("engine.bore_m"));
    }
}
