use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::constants::FIXED_CYLINDER_COUNT;

#[path = "config/audit.rs"]
mod audit;
pub(crate) use audit::validate_app_config;

// These structs intentionally mirror config/sim.yaml section-for-section so serde can
// deserialize the checked-in tuning file directly into runtime configuration.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct EnvironmentConfig {
    pub(crate) ambient_pressure_pa: f64,
    pub(crate) intake_temp_k: f64,
    pub(crate) exhaust_temp_k: f64,
    pub(crate) dt: f64,
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            ambient_pressure_pa: 101_325.0,
            intake_temp_k: 305.0,
            exhaust_temp_k: 880.0,
            dt: 0.0015,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct EngineConfig {
    pub(crate) displacement_m3: f64,
    pub(crate) compression_ratio: f64,
    pub(crate) bore_m: f64,
    pub(crate) stroke_m: f64,
    pub(crate) inertia_kgm2: f64,
    pub(crate) friction_c0_nm: f64,
    pub(crate) friction_c1_nms: f64,
    pub(crate) friction_c2_nms2: f64,
    pub(crate) intake_volume_m3: f64,
    pub(crate) intake_runner_volume_m3: f64,
    pub(crate) exhaust_volume_m3: f64,
    pub(crate) exhaust_runner_volume_m3: f64,
    pub(crate) throttle_area_max_m2: f64,
    pub(crate) tailpipe_area_m2: f64,
    #[serde(alias = "idle_target_rpm")]
    pub(crate) default_target_rpm: f64,
    pub(crate) max_rpm: f64,
}

impl Default for EngineConfig {
    fn default() -> Self {
        let mut cfg = Self {
            displacement_m3: 0.0,
            compression_ratio: 13.0,
            bore_m: 0.0805,
            stroke_m: 0.0976,
            inertia_kgm2: 0.16,
            friction_c0_nm: 5.1,
            friction_c1_nms: 0.0040,
            friction_c2_nms2: 0.000020,
            intake_volume_m3: 0.0032,
            intake_runner_volume_m3: 0.00040,
            exhaust_volume_m3: 0.0050,
            exhaust_runner_volume_m3: 0.00055,
            throttle_area_max_m2: 3.0e-3,
            tailpipe_area_m2: 2.5e-3,
            default_target_rpm: 850.0,
            max_rpm: 7000.0,
        };
        cfg.sync_derived_geometry();
        cfg
    }
}

impl EngineConfig {
    pub(crate) fn derived_displacement_m3(&self) -> f64 {
        0.25 * PI * self.bore_m.powi(2) * self.stroke_m * FIXED_CYLINDER_COUNT as f64
    }

    pub(crate) fn sync_derived_geometry(&mut self) {
        self.displacement_m3 = self.derived_displacement_m3();
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct CamConfig {
    pub(crate) intake_centerline_deg: f64,
    pub(crate) exhaust_centerline_deg: f64,
    pub(crate) intake_duration_deg: f64,
    pub(crate) exhaust_duration_deg: f64,
    pub(crate) intake_max_lift_mm: f64,
    pub(crate) exhaust_max_lift_mm: f64,
    pub(crate) display_y_max_mm: f64,
}

impl Default for CamConfig {
    fn default() -> Self {
        Self {
            intake_centerline_deg: 470.0,
            exhaust_centerline_deg: 250.0,
            intake_duration_deg: 248.0,
            exhaust_duration_deg: 244.0,
            intake_max_lift_mm: 10.5,
            exhaust_max_lift_mm: 9.8,
            display_y_max_mm: 14.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct ControlDefaults {
    pub(crate) throttle_cmd: f64,
    pub(crate) load_cmd: f64,
    pub(crate) spark_cmd: bool,
    pub(crate) fuel_cmd: bool,
    #[serde(alias = "spark_timing_offset_deg")]
    pub(crate) ignition_timing_deg: f64,
    pub(crate) vvt_intake_deg: f64,
    pub(crate) vvt_exhaust_deg: f64,
}

impl Default for ControlDefaults {
    fn default() -> Self {
        Self {
            throttle_cmd: 0.05,
            load_cmd: 0.0,
            spark_cmd: false,
            fuel_cmd: false,
            ignition_timing_deg: 12.0,
            vvt_intake_deg: 0.0,
            vvt_exhaust_deg: 0.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct VolumetricEfficiencyConfig {
    pub(crate) rpm_base: f64,
    pub(crate) rpm_gain: f64,
    pub(crate) rpm_center: f64,
    pub(crate) rpm_width: f64,
    pub(crate) rpm_high_width_scale: f64,
    pub(crate) rpm_min: f64,
    pub(crate) rpm_max: f64,
    pub(crate) vvt_rpm_low: f64,
    pub(crate) vvt_rpm_high: f64,
    pub(crate) vvt_intake_coeff: f64,
    pub(crate) vvt_exhaust_coeff: f64,
    pub(crate) vvt_intake_opt_low_deg: f64,
    pub(crate) vvt_intake_opt_high_deg: f64,
    pub(crate) vvt_intake_opt_window_deg: f64,
    pub(crate) vvt_intake_opt_gain: f64,
    pub(crate) vvt_exhaust_opt_low_deg: f64,
    pub(crate) vvt_exhaust_opt_high_deg: f64,
    pub(crate) vvt_exhaust_opt_window_deg: f64,
    pub(crate) vvt_exhaust_opt_gain: f64,
    pub(crate) throttle_base: f64,
    pub(crate) throttle_gain: f64,
    pub(crate) throttle_min: f64,
    pub(crate) throttle_max: f64,
    pub(crate) overall_min: f64,
    pub(crate) overall_max: f64,
}

impl Default for VolumetricEfficiencyConfig {
    fn default() -> Self {
        Self {
            rpm_base: 0.82,
            rpm_gain: 0.24,
            rpm_center: 5100.0,
            rpm_width: 1.05e7,
            rpm_high_width_scale: 1.0,
            rpm_min: 0.62,
            rpm_max: 1.04,
            vvt_rpm_low: 1_500.0,
            vvt_rpm_high: 6_500.0,
            vvt_intake_coeff: 0.0022,
            vvt_exhaust_coeff: 0.0008,
            vvt_intake_opt_low_deg: 14.0,
            vvt_intake_opt_high_deg: -10.0,
            vvt_intake_opt_window_deg: 20.0,
            vvt_intake_opt_gain: 0.08,
            vvt_exhaust_opt_low_deg: -8.0,
            vvt_exhaust_opt_high_deg: 12.0,
            vvt_exhaust_opt_window_deg: 18.0,
            vvt_exhaust_opt_gain: 0.08,
            throttle_base: 0.38,
            throttle_gain: 0.62,
            throttle_min: 0.38,
            throttle_max: 1.03,
            overall_min: 0.30,
            overall_max: 1.10,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct PvModelConfig {
    pub(crate) gamma_compression: f64,
    pub(crate) gamma_expansion: f64,
    pub(crate) evo_deg: f64,
    pub(crate) blowdown_end_deg: f64,
    pub(crate) intake_pulsation_amplitude: f64,
    pub(crate) expansion_floor_exhaust_ratio: f64,
    pub(crate) pressure_floor_pa: f64,
}

impl Default for PvModelConfig {
    fn default() -> Self {
        Self {
            gamma_compression: 1.34,
            gamma_expansion: 1.24,
            evo_deg: 565.0,
            blowdown_end_deg: 620.0,
            intake_pulsation_amplitude: 0.02,
            expansion_floor_exhaust_ratio: 1.05,
            pressure_floor_pa: 20_000.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct GasPathConfig {
    pub(crate) intake_runner_inertance_pa_s2_per_kg: f64,
    pub(crate) exhaust_runner_inertance_pa_s2_per_kg: f64,
    pub(crate) intake_runner_damping_per_s: f64,
    pub(crate) exhaust_runner_damping_per_s: f64,
    pub(crate) intake_runner_length_m: f64,
    pub(crate) exhaust_runner_length_m: f64,
    pub(crate) intake_runner_diameter_m: f64,
    pub(crate) exhaust_runner_diameter_m: f64,
    pub(crate) intake_runner_friction_factor: f64,
    pub(crate) exhaust_runner_friction_factor: f64,
    pub(crate) intake_runner_local_loss_coeff: f64,
    pub(crate) exhaust_runner_local_loss_coeff: f64,
    pub(crate) intake_pulse_blend: f64,
    pub(crate) exhaust_pulse_blend: f64,
    pub(crate) intake_boundary_runner_weight: f64,
    pub(crate) exhaust_boundary_runner_weight: f64,
    pub(crate) intake_boundary_runner_weight_high_rpm: f64,
    pub(crate) exhaust_boundary_runner_weight_high_rpm: f64,
    pub(crate) boundary_weight_rpm_center: f64,
    pub(crate) boundary_weight_rpm_width: f64,
    pub(crate) runner_flow_limit_kg_s: f64,
    pub(crate) runner_pressure_min_pa: f64,
    pub(crate) runner_pressure_max_pa: f64,
    pub(crate) overlap_pressure_coeff: f64,
    pub(crate) overlap_flow_coeff: f64,
    pub(crate) overlap_flow_reference_kg_s: f64,
    pub(crate) overlap_rpm_decay_center: f64,
    pub(crate) overlap_rpm_decay_width: f64,
    pub(crate) overlap_effect_min: f64,
    pub(crate) overlap_effect_max: f64,
}

impl Default for GasPathConfig {
    fn default() -> Self {
        Self {
            intake_runner_inertance_pa_s2_per_kg: 1.8e4,
            exhaust_runner_inertance_pa_s2_per_kg: 1.4e4,
            intake_runner_damping_per_s: 6.0,
            exhaust_runner_damping_per_s: 6.0,
            intake_runner_length_m: 0.36,
            exhaust_runner_length_m: 0.74,
            intake_runner_diameter_m: 0.034,
            exhaust_runner_diameter_m: 0.036,
            intake_runner_friction_factor: 0.028,
            exhaust_runner_friction_factor: 0.030,
            intake_runner_local_loss_coeff: 1.6,
            exhaust_runner_local_loss_coeff: 1.9,
            intake_pulse_blend: 0.32,
            exhaust_pulse_blend: 0.40,
            intake_boundary_runner_weight: 0.35,
            exhaust_boundary_runner_weight: 0.45,
            intake_boundary_runner_weight_high_rpm: 0.60,
            exhaust_boundary_runner_weight_high_rpm: 0.62,
            boundary_weight_rpm_center: 5000.0,
            boundary_weight_rpm_width: 1400.0,
            runner_flow_limit_kg_s: 0.45,
            runner_pressure_min_pa: 18_000.0,
            runner_pressure_max_pa: 320_000.0,
            overlap_pressure_coeff: 0.22,
            overlap_flow_coeff: 0.08,
            overlap_flow_reference_kg_s: 0.08,
            overlap_rpm_decay_center: 3800.0,
            overlap_rpm_decay_width: 1200.0,
            overlap_effect_min: 0.88,
            overlap_effect_max: 1.12,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct WaveActionConfig {
    pub(crate) intake_group_count: usize,
    pub(crate) exhaust_group_count: usize,
    pub(crate) event_memory: usize,
    pub(crate) intake_runner_length_m: f64,
    pub(crate) exhaust_primary_length_m: f64,
    pub(crate) intake_delay_scale: f64,
    pub(crate) exhaust_delay_scale: f64,
    pub(crate) intake_decay_time_s: f64,
    pub(crate) exhaust_decay_time_s: f64,
    pub(crate) intake_pressure_gain: f64,
    pub(crate) exhaust_pressure_gain: f64,
    pub(crate) intake_pressure_limit_pa: f64,
    pub(crate) exhaust_pressure_limit_pa: f64,
    pub(crate) intake_flow_reference_kg_s: f64,
    pub(crate) exhaust_flow_reference_kg_s: f64,
    pub(crate) intake_resonance_damping_ratio: f64,
    pub(crate) exhaust_resonance_damping_ratio: f64,
    pub(crate) intake_resonance_gain_blend: f64,
    pub(crate) exhaust_resonance_gain_blend: f64,
    pub(crate) resonance_gain_min: f64,
    pub(crate) resonance_gain_max: f64,
    pub(crate) intake_flow_wave_gain: f64,
    pub(crate) exhaust_flow_wave_gain: f64,
    pub(crate) intake_ram_gain: f64,
    pub(crate) exhaust_scavenge_gain: f64,
    pub(crate) ve_pulse_min: f64,
    pub(crate) ve_pulse_max: f64,
}

impl Default for WaveActionConfig {
    fn default() -> Self {
        Self {
            intake_group_count: 4,
            exhaust_group_count: 2,
            event_memory: 3,
            intake_runner_length_m: 0.36,
            exhaust_primary_length_m: 0.74,
            intake_delay_scale: 2.0,
            exhaust_delay_scale: 2.0,
            intake_decay_time_s: 0.012,
            exhaust_decay_time_s: 0.010,
            intake_pressure_gain: 0.06,
            exhaust_pressure_gain: 0.08,
            intake_pressure_limit_pa: 6_000.0,
            exhaust_pressure_limit_pa: 9_000.0,
            intake_flow_reference_kg_s: 0.055,
            exhaust_flow_reference_kg_s: 0.075,
            intake_resonance_damping_ratio: 0.24,
            exhaust_resonance_damping_ratio: 0.28,
            intake_resonance_gain_blend: 0.35,
            exhaust_resonance_gain_blend: 0.45,
            resonance_gain_min: 0.85,
            resonance_gain_max: 1.65,
            intake_flow_wave_gain: 0.0,
            exhaust_flow_wave_gain: 0.0,
            intake_ram_gain: 0.22,
            exhaust_scavenge_gain: 0.18,
            ve_pulse_min: 0.94,
            ve_pulse_max: 1.08,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct HeatTransferConfig {
    pub(crate) wall_temp_k: f64,
    pub(crate) base_h_w_m2k: f64,
    pub(crate) pressure_exponent: f64,
    pub(crate) temperature_exponent: f64,
    pub(crate) piston_speed_exponent: f64,
    pub(crate) reference_pressure_pa: f64,
    pub(crate) reference_temp_k: f64,
    pub(crate) reference_piston_speed_mps: f64,
    pub(crate) gas_temp_rise_gain: f64,
    pub(crate) gas_temp_max_k: f64,
    pub(crate) wall_area_scale: f64,
    pub(crate) duration_scale: f64,
    pub(crate) heat_loss_fraction_max: f64,
    pub(crate) exhaust_temp_cooling_gain: f64,
}

impl Default for HeatTransferConfig {
    fn default() -> Self {
        Self {
            wall_temp_k: 470.0,
            base_h_w_m2k: 120.0,
            pressure_exponent: 0.8,
            temperature_exponent: -0.45,
            piston_speed_exponent: 0.8,
            reference_pressure_pa: 1.0e6,
            reference_temp_k: 1_000.0,
            reference_piston_speed_mps: 10.0,
            gas_temp_rise_gain: 0.30,
            gas_temp_max_k: 2_800.0,
            wall_area_scale: 1.0,
            duration_scale: 0.85,
            heat_loss_fraction_max: 0.08,
            exhaust_temp_cooling_gain: 0.35,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct FuelEvaporationConfig {
    pub(crate) latent_heat_j_per_kg: f64,
    pub(crate) charge_cooling_effectiveness: f64,
    pub(crate) intake_charge_temp_min_k: f64,
}

impl Default for FuelEvaporationConfig {
    fn default() -> Self {
        Self {
            latent_heat_j_per_kg: 350_000.0,
            charge_cooling_effectiveness: 0.72,
            intake_charge_temp_min_k: 255.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct GasThermoConfig {
    pub(crate) fresh_cp_j_per_kgk: f64,
    pub(crate) burned_cp_ref_j_per_kgk: f64,
    pub(crate) burned_cp_reference_temp_k: f64,
    pub(crate) burned_cp_temp_coeff_j_per_kgk2: f64,
    pub(crate) burned_cp_egr_coeff_j_per_kgk: f64,
    pub(crate) cp_min_j_per_kgk: f64,
    pub(crate) cp_max_j_per_kgk: f64,
}

impl Default for GasThermoConfig {
    fn default() -> Self {
        Self {
            fresh_cp_j_per_kgk: 1_005.0,
            burned_cp_ref_j_per_kgk: 1_150.0,
            burned_cp_reference_temp_k: 900.0,
            burned_cp_temp_coeff_j_per_kgk2: 0.12,
            burned_cp_egr_coeff_j_per_kgk: 120.0,
            cp_min_j_per_kgk: 950.0,
            cp_max_j_per_kgk: 1_450.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct InternalEgrConfig {
    pub(crate) overlap_base_fraction: f64,
    pub(crate) pressure_backflow_gain: f64,
    pub(crate) wave_backflow_gain: f64,
    pub(crate) reverse_flow_gain: f64,
    pub(crate) reverse_flow_reference_kg_s: f64,
    pub(crate) rpm_decay_center: f64,
    pub(crate) rpm_decay_width: f64,
    pub(crate) fraction_min: f64,
    pub(crate) fraction_max: f64,
    pub(crate) burn_duration_gain_deg_per_fraction: f64,
    pub(crate) phase_dilution_gain: f64,
    pub(crate) phase_dilution_min: f64,
}

impl Default for InternalEgrConfig {
    fn default() -> Self {
        Self {
            overlap_base_fraction: 0.010,
            pressure_backflow_gain: 0.12,
            wave_backflow_gain: 0.10,
            reverse_flow_gain: 0.16,
            reverse_flow_reference_kg_s: 0.06,
            rpm_decay_center: 3600.0,
            rpm_decay_width: 1100.0,
            fraction_min: 0.0,
            fraction_max: 0.18,
            burn_duration_gain_deg_per_fraction: 22.0,
            phase_dilution_gain: 0.30,
            phase_dilution_min: 0.86,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum ExternalLoadMode {
    BrakeMap,
    VehicleEquivalent,
}

impl ExternalLoadMode {
    #[allow(dead_code)]
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::BrakeMap => "Brake dyno",
            Self::VehicleEquivalent => "Vehicle equivalent",
        }
    }
}

impl Default for ExternalLoadMode {
    fn default() -> Self {
        Self::BrakeMap
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct VehicleLoadConfig {
    pub(crate) vehicle_mass_kg: f64,
    pub(crate) equivalent_mass_factor: f64,
    pub(crate) wheel_radius_m: f64,
    pub(crate) drivetrain_ratio: f64,
    pub(crate) driveline_efficiency: f64,
    pub(crate) rolling_resistance_coeff: f64,
    pub(crate) drag_coeff: f64,
    pub(crate) frontal_area_m2: f64,
    pub(crate) air_density_kg_m3: f64,
    pub(crate) road_grade_percent: f64,
    pub(crate) accessory_torque_nm: f64,
}

impl Default for VehicleLoadConfig {
    fn default() -> Self {
        Self {
            vehicle_mass_kg: 1_450.0,
            equivalent_mass_factor: 1.08,
            wheel_radius_m: 0.315,
            drivetrain_ratio: 10.8,
            driveline_efficiency: 0.92,
            rolling_resistance_coeff: 0.015,
            drag_coeff: 0.30,
            frontal_area_m2: 2.10,
            air_density_kg_m3: 1.18,
            road_grade_percent: 0.0,
            accessory_torque_nm: 8.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct ExternalLoadConfig {
    pub(crate) mode: ExternalLoadMode,
    pub(crate) command_exponent: f64,
    pub(crate) base_torque_nm: f64,
    pub(crate) speed_linear_nms: f64,
    pub(crate) speed_quadratic_nms2: f64,
    pub(crate) torque_min_nm: f64,
    pub(crate) torque_max_nm: f64,
    pub(crate) absorber_rotor_inertia_kgm2: f64,
    pub(crate) absorber_power_limit_kw: f64,
    pub(crate) absorber_speed_limit_rpm: f64,
    pub(crate) vehicle: VehicleLoadConfig,
}

impl Default for ExternalLoadConfig {
    fn default() -> Self {
        Self {
            mode: ExternalLoadMode::default(),
            command_exponent: 1.20,
            base_torque_nm: 14.0,
            speed_linear_nms: 0.012,
            speed_quadratic_nms2: 0.00018,
            torque_min_nm: -220.0,
            torque_max_nm: 220.0,
            absorber_rotor_inertia_kgm2: 0.24,
            absorber_power_limit_kw: 170.0,
            absorber_speed_limit_rpm: 8_000.0,
            vehicle: VehicleLoadConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct ModelConfig {
    pub(crate) initial_intake_pressure_pa: f64,
    pub(crate) initial_throttle_eff: f64,
    pub(crate) max_rpm_floor: f64,
    pub(crate) history_capacity_floor: usize,
    pub(crate) cycle_metric_history_capacity: usize,
    pub(crate) pv_capacity_scale: usize,
    pub(crate) pv_capacity_min: usize,
    pub(crate) pv_display_raw_min_points: usize,
    pub(crate) pv_display_bins: usize,
    pub(crate) pv_display_min_populated_bins: usize,
    pub(crate) ignition_timing_min_deg: f64,
    pub(crate) ignition_timing_max_deg: f64,
    pub(crate) mbt_base_deg: f64,
    pub(crate) mbt_rpm_slope_deg_per_rpm: f64,
    pub(crate) mbt_rpm_reference: f64,
    pub(crate) mbt_load_coeff: f64,
    pub(crate) mbt_min_deg: f64,
    pub(crate) mbt_max_deg: f64,
    pub(crate) phase_sigma_advanced_deg: f64,
    pub(crate) phase_sigma_retarded_deg: f64,
    pub(crate) phase_stable_advanced_limit_deg: f64,
    pub(crate) phase_stable_retarded_limit_deg: f64,
    pub(crate) exhaust_temp_retard_gain: f64,
    pub(crate) exhaust_temp_advance_gain: f64,
    pub(crate) exhaust_temp_scale_min: f64,
    pub(crate) exhaust_temp_scale_max: f64,
    pub(crate) exhaust_temp_min_k: f64,
    pub(crate) exhaust_temp_max_k: f64,
    pub(crate) throttle_area_offset: f64,
    pub(crate) throttle_area_gain: f64,
    pub(crate) throttle_area_exponent: f64,
    pub(crate) throttle_area_min_scale: f64,
    pub(crate) throttle_discharge_coeff: f64,
    pub(crate) tailpipe_discharge_coeff: f64,
    pub(crate) load_min: f64,
    pub(crate) load_max: f64,
    pub(crate) burn_start_base_deg: f64,
    pub(crate) burn_start_vvt_intake_coeff: f64,
    pub(crate) burn_duration_base_deg: f64,
    pub(crate) burn_duration_throttle_coeff: f64,
    pub(crate) burn_duration_min_deg: f64,
    pub(crate) burn_duration_max_deg: f64,
    pub(crate) wiebe_a: f64,
    pub(crate) wiebe_m: f64,
    pub(crate) lambda_base: f64,
    pub(crate) lambda_throttle_coeff: f64,
    pub(crate) lambda_min: f64,
    pub(crate) lambda_max: f64,
    pub(crate) stoich_afr: f64,
    pub(crate) eta_base_offset: f64,
    pub(crate) eta_load_coeff: f64,
    pub(crate) eta_rpm_abs_coeff: f64,
    pub(crate) eta_rpm_reference: f64,
    pub(crate) eta_base_min: f64,
    pub(crate) eta_base_max: f64,
    pub(crate) eta_min: f64,
    pub(crate) eta_max: f64,
    pub(crate) combustion_enable_rpm_min: f64,
    pub(crate) combustion_enable_intake_pressure_min_pa: f64,
    pub(crate) combustion_enable_running_rpm_min: f64,
    pub(crate) combustion_rate_max: f64,
    pub(crate) pumping_torque_min_nm: f64,
    pub(crate) pumping_torque_max_nm: f64,
    pub(crate) throttle_time_constant_s: f64,
    pub(crate) intake_pressure_min_pa: f64,
    pub(crate) intake_pressure_max_pa: f64,
    pub(crate) exhaust_pressure_min_ambient_ratio: f64,
    pub(crate) exhaust_pressure_max_over_ambient_pa: f64,
    pub(crate) running_set_rpm: f64,
    pub(crate) running_clear_rpm: f64,
    pub(crate) net_torque_smoothing_cycles: usize,
    pub(crate) net_torque_filter_time_constant_s: f64,
    pub(crate) fuel_mass_presence_threshold_kg: f64,
    pub(crate) compression_peak_gamma: f64,
    pub(crate) compression_peak_max_pa: f64,
    pub(crate) combustion_pressure_gain: f64,
    pub(crate) combustion_pressure_load_exponent: f64,
    pub(crate) peak_pressure_max_pa: f64,
    pub(crate) swept_volume_floor_sampling_m3: f64,
    pub(crate) clearance_volume_floor_m3: f64,
    pub(crate) compression_ratio_guard: f64,
    pub(crate) eta_indicated_average_cycles: usize,
    pub(crate) eta_indicated_min: f64,
    pub(crate) eta_indicated_max: f64,
    pub(crate) imep_swept_volume_floor_m3: f64,
    pub(crate) theoretical_efficiency_compression_floor: f64,
    pub(crate) cam_profile_samples: usize,
    pub(crate) cam_half_duration_min_deg: f64,
    pub(crate) cam_shape_exponent: f64,
    pub(crate) volumetric_efficiency: VolumetricEfficiencyConfig,
    pub(crate) pv_model: PvModelConfig,
    pub(crate) gas_path: GasPathConfig,
    pub(crate) wave_action: WaveActionConfig,
    pub(crate) heat_transfer: HeatTransferConfig,
    pub(crate) fuel_evaporation: FuelEvaporationConfig,
    pub(crate) gas_thermo: GasThermoConfig,
    pub(crate) internal_egr: InternalEgrConfig,
    pub(crate) external_load: ExternalLoadConfig,
}

impl Default for ModelConfig {
    fn default() -> Self {
        Self {
            initial_intake_pressure_pa: 38_000.0,
            initial_throttle_eff: 0.05,
            max_rpm_floor: 1000.0,
            history_capacity_floor: 256,
            cycle_metric_history_capacity: 16,
            pv_capacity_scale: 1600,
            pv_capacity_min: 2000,
            pv_display_raw_min_points: 128,
            pv_display_bins: 1440,
            pv_display_min_populated_bins: 180,
            ignition_timing_min_deg: -5.0,
            ignition_timing_max_deg: 45.0,
            mbt_base_deg: 12.0,
            mbt_rpm_slope_deg_per_rpm: 0.0018,
            mbt_rpm_reference: 900.0,
            mbt_load_coeff: 3.0,
            mbt_min_deg: 8.0,
            mbt_max_deg: 26.0,
            phase_sigma_advanced_deg: 6.5,
            phase_sigma_retarded_deg: 15.0,
            phase_stable_advanced_limit_deg: 14.0,
            phase_stable_retarded_limit_deg: 32.0,
            exhaust_temp_retard_gain: 0.024,
            exhaust_temp_advance_gain: 0.010,
            exhaust_temp_scale_min: 0.75,
            exhaust_temp_scale_max: 1.70,
            exhaust_temp_min_k: 500.0,
            exhaust_temp_max_k: 1900.0,
            throttle_area_offset: 0.005,
            throttle_area_gain: 0.98,
            throttle_area_exponent: 1.8,
            throttle_area_min_scale: 0.005,
            throttle_discharge_coeff: 0.82,
            tailpipe_discharge_coeff: 0.87,
            load_min: 0.20,
            load_max: 1.20,
            burn_start_base_deg: 360.0,
            burn_start_vvt_intake_coeff: 0.08,
            burn_duration_base_deg: 66.0,
            burn_duration_throttle_coeff: 8.0,
            burn_duration_min_deg: 38.0,
            burn_duration_max_deg: 75.0,
            wiebe_a: 5.2,
            wiebe_m: 2.0,
            lambda_base: 1.15,
            lambda_throttle_coeff: 0.35,
            lambda_min: 0.82,
            lambda_max: 1.12,
            stoich_afr: 14.7,
            eta_base_offset: 0.172,
            eta_load_coeff: 0.215,
            eta_rpm_abs_coeff: 0.000011,
            eta_rpm_reference: 2500.0,
            eta_base_min: 0.08,
            eta_base_max: 0.40,
            eta_min: 0.02,
            eta_max: 1.00,
            combustion_enable_rpm_min: 120.0,
            combustion_enable_intake_pressure_min_pa: 25_000.0,
            combustion_enable_running_rpm_min: 450.0,
            combustion_rate_max: 5.5,
            pumping_torque_min_nm: -12.0,
            pumping_torque_max_nm: 25.0,
            throttle_time_constant_s: 0.060,
            intake_pressure_min_pa: 18_000.0,
            intake_pressure_max_pa: 130_000.0,
            exhaust_pressure_min_ambient_ratio: 0.78,
            exhaust_pressure_max_over_ambient_pa: 120_000.0,
            running_set_rpm: 520.0,
            running_clear_rpm: 260.0,
            net_torque_smoothing_cycles: 4,
            net_torque_filter_time_constant_s: 0.180,
            fuel_mass_presence_threshold_kg: 1.0e-9,
            compression_peak_gamma: 1.32,
            compression_peak_max_pa: 6.0e6,
            combustion_pressure_gain: 0.62,
            combustion_pressure_load_exponent: 2.5,
            peak_pressure_max_pa: 12.0e6,
            swept_volume_floor_sampling_m3: 1.0e-6,
            clearance_volume_floor_m3: 1.0e-6,
            compression_ratio_guard: 1.0,
            eta_indicated_average_cycles: 3,
            eta_indicated_min: -0.20,
            eta_indicated_max: 1.00,
            imep_swept_volume_floor_m3: 1.0e-9,
            theoretical_efficiency_compression_floor: 1.001,
            cam_profile_samples: 721,
            cam_half_duration_min_deg: 1.0,
            cam_shape_exponent: 1.2,
            volumetric_efficiency: VolumetricEfficiencyConfig::default(),
            pv_model: PvModelConfig::default(),
            gas_path: GasPathConfig::default(),
            wave_action: WaveActionConfig::default(),
            heat_transfer: HeatTransferConfig::default(),
            fuel_evaporation: FuelEvaporationConfig::default(),
            gas_thermo: GasThermoConfig::default(),
            internal_egr: InternalEgrConfig::default(),
            external_load: ExternalLoadConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct NumericsConfig {
    pub(crate) rpm_link_base_dt_min_s: f64,
    #[serde(alias = "rpm_link_idle_rpm_min")]
    pub(crate) rpm_link_reference_rpm_min: f64,
    pub(crate) rpm_link_deg_per_step_min: f64,
    pub(crate) rpm_link_deg_per_step_max: f64,
    pub(crate) rpm_link_rpm_floor: f64,
    pub(crate) rpm_link_dt_min_factor: f64,
    pub(crate) rpm_link_dt_min_floor_s: f64,
    pub(crate) rpm_link_dt_max_factor: f64,
    pub(crate) rpm_link_dt_max_cap_s: f64,
    pub(crate) state_error_omega_bias: f64,
    pub(crate) state_error_theta_scale_rad: f64,
    pub(crate) state_error_p_intake_bias_pa: f64,
    pub(crate) state_error_p_intake_rel: f64,
    pub(crate) state_error_p_intake_runner_bias_pa: f64,
    pub(crate) state_error_p_intake_runner_rel: f64,
    pub(crate) state_error_p_exhaust_bias_pa: f64,
    pub(crate) state_error_p_exhaust_rel: f64,
    pub(crate) state_error_p_exhaust_runner_bias_pa: f64,
    pub(crate) state_error_p_exhaust_runner_rel: f64,
    pub(crate) state_error_m_dot_intake_runner_scale_kg_s: f64,
    pub(crate) state_error_m_dot_exhaust_runner_scale_kg_s: f64,
    pub(crate) state_error_throttle_scale: f64,
    pub(crate) realtime_probe_dt_min_s: f64,
    pub(crate) realtime_warmup_steps: usize,
    pub(crate) realtime_sample_steps: usize,
    pub(crate) realtime_margin_factor: f64,
    pub(crate) realtime_floor_min_s: f64,
    pub(crate) realtime_floor_probe_factor_max: f64,
    pub(crate) realtime_fixed_dt_headroom_ratio: f64,
    pub(crate) realtime_fixed_dt_max_deg_per_step: f64,
    pub(crate) accuracy_target_deg_per_step: f64,
    pub(crate) accuracy_dt_max_s: f64,
}

impl Default for NumericsConfig {
    fn default() -> Self {
        Self {
            rpm_link_base_dt_min_s: 0.00005,
            rpm_link_reference_rpm_min: 200.0,
            rpm_link_deg_per_step_min: 1.0,
            rpm_link_deg_per_step_max: 12.0,
            rpm_link_rpm_floor: 80.0,
            rpm_link_dt_min_factor: 0.08,
            rpm_link_dt_min_floor_s: 0.00005,
            rpm_link_dt_max_factor: 2.5,
            rpm_link_dt_max_cap_s: 0.012,
            state_error_omega_bias: 10.0,
            state_error_theta_scale_rad: 0.008726646259971648,
            state_error_p_intake_bias_pa: 150.0,
            state_error_p_intake_rel: 0.01,
            state_error_p_intake_runner_bias_pa: 150.0,
            state_error_p_intake_runner_rel: 0.01,
            state_error_p_exhaust_bias_pa: 180.0,
            state_error_p_exhaust_rel: 0.01,
            state_error_p_exhaust_runner_bias_pa: 180.0,
            state_error_p_exhaust_runner_rel: 0.01,
            state_error_m_dot_intake_runner_scale_kg_s: 0.03,
            state_error_m_dot_exhaust_runner_scale_kg_s: 0.03,
            state_error_throttle_scale: 0.01,
            realtime_probe_dt_min_s: 1.0e-5,
            realtime_warmup_steps: 48,
            realtime_sample_steps: 240,
            realtime_margin_factor: 1.25,
            realtime_floor_min_s: 1.0e-5,
            realtime_floor_probe_factor_max: 6.0,
            realtime_fixed_dt_headroom_ratio: 2.5,
            realtime_fixed_dt_max_deg_per_step: 12.0,
            accuracy_target_deg_per_step: 1.5,
            accuracy_dt_max_s: 0.0008,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct UiConfig {
    pub(crate) sync_to_wall_clock: bool,
    pub(crate) simulated_time_per_frame_s: f64,
    pub(crate) min_base_dt_s: f64,
    pub(crate) realtime_dt_min_factor: f64,
    pub(crate) realtime_dt_max_factor: f64,
    pub(crate) realtime_dt_max_over_min_factor: f64,
    pub(crate) throttle_key_step: f64,
    pub(crate) max_steps_per_frame: usize,
    pub(crate) dt_smoothing_factor: f64,
    pub(crate) dt_epsilon_s: f64,
    pub(crate) vvt_slider_min_deg: f64,
    pub(crate) vvt_slider_max_deg: f64,
    pub(crate) ignition_slider_min_deg: f64,
    pub(crate) ignition_slider_max_deg: f64,
    pub(crate) plot_height_px: f32,
    pub(crate) pv_plot_height_px: f32,
    pub(crate) line_width_px: f32,
    pub(crate) crank_line_width_px: f32,
    pub(crate) torque_min_span_nm: f64,
    pub(crate) torque_margin_ratio: f64,
    pub(crate) torque_floor_abs_nm: f64,
    pub(crate) trapped_air_min_y_max_mg: f64,
    pub(crate) trapped_air_headroom_ratio: f64,
    pub(crate) pv_headroom_ratio: f64,
    pub(crate) pv_min_headroom_kpa: f64,
    pub(crate) repaint_hz: u32,
    pub(crate) window_width_px: f32,
    pub(crate) window_height_px: f32,
}

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            sync_to_wall_clock: false,
            simulated_time_per_frame_s: 0.008,
            min_base_dt_s: 0.0002,
            realtime_dt_min_factor: 0.05,
            realtime_dt_max_factor: 3.0,
            realtime_dt_max_over_min_factor: 1.5,
            throttle_key_step: 0.01,
            max_steps_per_frame: 600,
            dt_smoothing_factor: 0.25,
            dt_epsilon_s: 1.0e-6,
            vvt_slider_min_deg: -40.0,
            vvt_slider_max_deg: 40.0,
            ignition_slider_min_deg: -5.0,
            ignition_slider_max_deg: 45.0,
            plot_height_px: 126.0,
            pv_plot_height_px: 244.0,
            line_width_px: 2.0,
            crank_line_width_px: 1.0,
            torque_min_span_nm: 10.0,
            torque_margin_ratio: 0.15,
            torque_floor_abs_nm: 5.0,
            trapped_air_min_y_max_mg: 100.0,
            trapped_air_headroom_ratio: 1.15,
            pv_headroom_ratio: 1.08,
            pv_min_headroom_kpa: 100.0,
            repaint_hz: 45,
            window_width_px: 1440.0,
            window_height_px: 860.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct PlotConfig {
    pub(crate) rpm_history_capacity: usize,
    pub(crate) history_recent_cycles: usize,
    pub(crate) pv_recent_cycles: usize,
    pub(crate) pv_subsamples_per_step: usize,
    pub(crate) pv_x_min: f64,
    pub(crate) pv_x_max: f64,
    pub(crate) pv_y_min_kpa: f64,
    pub(crate) pv_y_max_kpa: f64,
}

impl Default for PlotConfig {
    fn default() -> Self {
        Self {
            rpm_history_capacity: 3600,
            history_recent_cycles: 1,
            pv_recent_cycles: 4,
            pv_subsamples_per_step: 4,
            pv_x_min: 0.0,
            pv_x_max: 1.15,
            pv_y_min_kpa: 0.0,
            pv_y_max_kpa: 2500.0,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub(crate) struct AppConfig {
    pub(crate) environment: EnvironmentConfig,
    pub(crate) engine: EngineConfig,
    pub(crate) cam: CamConfig,
    pub(crate) control_defaults: ControlDefaults,
    pub(crate) model: ModelConfig,
    pub(crate) numerics: NumericsConfig,
    pub(crate) ui: UiConfig,
    pub(crate) plot: PlotConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            environment: EnvironmentConfig::default(),
            engine: EngineConfig::default(),
            cam: CamConfig::default(),
            control_defaults: ControlDefaults::default(),
            model: ModelConfig::default(),
            numerics: NumericsConfig::default(),
            ui: UiConfig::default(),
            plot: PlotConfig::default(),
        }
    }
}

impl AppConfig {
    fn sync_derived_fields(&mut self) {
        self.engine.sync_derived_geometry();
    }
}

fn push_config_candidate(candidates: &mut Vec<PathBuf>, candidate: PathBuf) {
    if !candidates.iter().any(|existing| existing == &candidate) {
        candidates.push(candidate);
    }
}

fn config_candidate_paths(path: &Path, executable_dir: Option<&Path>) -> Vec<PathBuf> {
    let mut candidates = Vec::new();
    push_config_candidate(&mut candidates, path.to_path_buf());

    if path.is_absolute() {
        return candidates;
    }

    if let Some(dir) = executable_dir {
        push_config_candidate(&mut candidates, dir.join(path));
    }

    if let Some(file_name) = path.file_name() {
        push_config_candidate(&mut candidates, PathBuf::from(file_name));
        if let Some(dir) = executable_dir {
            push_config_candidate(&mut candidates, dir.join(file_name));
        }
    }

    candidates
}

pub(crate) fn load_config(path: impl AsRef<Path>) -> AppConfig {
    // Fall back to defaults on missing or malformed YAML so the app remains runnable.
    let path = path.as_ref();
    let executable_dir = std::env::current_exe()
        .ok()
        .and_then(|current_exe| current_exe.parent().map(Path::to_path_buf));

    let Some((resolved_path, text)) = config_candidate_paths(path, executable_dir.as_deref())
        .into_iter()
        .find_map(|candidate| match fs::read_to_string(&candidate) {
            Ok(text) => Some((candidate, text)),
            Err(_) => None,
        })
    else {
        return AppConfig::default();
    };

    let resolved_path = resolved_path.canonicalize().unwrap_or(resolved_path);
    let source_name = resolved_path.display().to_string();
    match parse_config_text(&text, &source_name) {
        Some(config) => config,
        None => AppConfig::default(),
    }
}

fn parse_config_text(text: &str, source_name: impl AsRef<str>) -> Option<AppConfig> {
    match serde_yaml::from_str::<AppConfig>(text) {
        Ok(mut cfg) => {
            cfg.sync_derived_fields();
            match validate_app_config(&cfg) {
                Ok(report) => {
                    for warning in report.warnings {
                        eprintln!(
                            "Config plausibility warning in {}: {warning}",
                            source_name.as_ref()
                        );
                    }
                    Some(cfg)
                }
                Err(report) => {
                    eprintln!(
                        "Config plausibility audit failed for {}:\n{report}\nFalling back to defaults.",
                        source_name.as_ref()
                    );
                    None
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to parse {}: {e}", source_name.as_ref());
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use super::{ExternalLoadMode, config_candidate_paths, load_config, validate_app_config};

    #[test]
    fn checked_in_yaml_with_comments_parses() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("sim.yaml");
        let cfg = load_config(&path);
        assert!((cfg.model.combustion_pressure_gain - 0.62).abs() < 1.0e-12);
        assert!((cfg.engine.bore_m - 0.0805).abs() < 1.0e-12);
        assert!((cfg.engine.stroke_m - 0.0976).abs() < 1.0e-12);
        assert!((cfg.engine.compression_ratio - 13.0).abs() < 1.0e-12);
        assert!((cfg.control_defaults.load_cmd - 0.0).abs() < 1.0e-12);
        assert!(
            (cfg.engine.displacement_m3 - cfg.engine.derived_displacement_m3()).abs() < 1.0e-12
        );
        assert!((cfg.model.gas_path.overlap_pressure_coeff - 0.22).abs() < 1.0e-12);
        assert!((cfg.model.gas_path.intake_runner_length_m - 0.36).abs() < 1.0e-12);
        assert!((cfg.model.gas_path.exhaust_runner_diameter_m - 0.036).abs() < 1.0e-12);
        assert_eq!(cfg.model.wave_action.intake_group_count, 4);
        assert_eq!(cfg.model.wave_action.exhaust_group_count, 2);
        assert!((cfg.model.heat_transfer.base_h_w_m2k - 120.0).abs() < 1.0e-12);
        assert_eq!(
            cfg.model.external_load.mode,
            ExternalLoadMode::VehicleEquivalent
        );
        assert!((cfg.model.external_load.torque_max_nm - 220.0).abs() < 1.0e-12);
        assert!((cfg.model.external_load.vehicle.vehicle_mass_kg - 1_450.0).abs() < 1.0e-12);
        assert!((cfg.model.fuel_evaporation.latent_heat_j_per_kg - 350_000.0).abs() < 1.0e-12);
        assert_eq!(cfg.plot.history_recent_cycles, 1);
        assert_eq!(cfg.plot.pv_recent_cycles, 4);
        assert!(!cfg.ui.sync_to_wall_clock);
        assert!((cfg.ui.simulated_time_per_frame_s - 0.008).abs() < 1.0e-12);
        assert_eq!(cfg.ui.repaint_hz, 45);
        assert!((cfg.ui.pv_plot_height_px - 244.0).abs() < 1.0e-12);
        assert!((cfg.ui.window_width_px - 1440.0).abs() < 1.0e-12);
        assert!((cfg.ui.window_height_px - 860.0).abs() < 1.0e-12);
    }

    #[test]
    fn checked_in_yaml_passes_physical_plausibility_audit() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("sim.yaml");
        let cfg = load_config(&path);
        validate_app_config(&cfg).expect("checked-in sim.yaml should pass plausibility audit");
    }

    #[test]
    fn flat_release_layout_is_checked_after_nested_config_path() {
        let executable_dir = Path::new("dist").join("release-assets");
        let candidates = config_candidate_paths(
            Path::new("config").join("sim.yaml").as_path(),
            Some(&executable_dir),
        );

        assert_eq!(
            candidates,
            vec![
                PathBuf::from("config").join("sim.yaml"),
                executable_dir.join("config").join("sim.yaml"),
                PathBuf::from("sim.yaml"),
                executable_dir.join("sim.yaml"),
            ]
        );
    }
}
