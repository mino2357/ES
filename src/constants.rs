// Shared physical constants used by both the engine model and the exhaust synth.
pub(crate) const R_AIR: f64 = 287.0;
pub(crate) const GAMMA_AIR: f64 = 1.4;
pub(crate) const FUEL_LHV_J_PER_KG: f64 = 43.0e6;
pub(crate) const W_PER_KW: f64 = 1_000.0;
pub(crate) const W_PER_MECHANICAL_HP: f64 = 745.699_871_582_270_1;
pub(crate) const FIXED_CYLINDER_COUNT: usize = 4;

// Default config path for the desktop binary.
pub(crate) const DEFAULT_CONFIG_PATH: &str = "config/sim.yaml";
