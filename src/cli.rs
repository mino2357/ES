use std::fs;
use std::path::{Path, PathBuf};

use crate::config::{load_config, AppConfig, ExternalLoadMode};
use crate::constants::DEFAULT_CONFIG_PATH;
use crate::simulator::{
    accuracy_priority_dt, estimate_mbt_deg, external_load_command_for_torque_nm, rpm_to_rad_s,
    shaft_power_kw, Observation, Simulator,
};

const DEFAULT_OUTPUT_DIR: &str = "output/cli";
const DEFAULT_SETTLE_TIME_S: f64 = 12.0;
const DEFAULT_AVERAGE_TIME_S: f64 = 1.5;
const DEFAULT_DIAGNOSTIC_SAMPLES: usize = 720;

pub(crate) fn run(args: Vec<String>) -> Result<(), String> {
    let command = Command::parse(args)?;
    match command {
        Command::Sweep(options) => run_sweep(options),
        Command::Help => {
            print_usage();
            Ok(())
        }
    }
}

enum Command {
    Sweep(SweepOptions),
    Help,
}

#[derive(Debug, Clone)]
struct SweepOptions {
    config_path: PathBuf,
    output_dir: PathBuf,
    rpm_start: Option<f64>,
    rpm_end: Option<f64>,
    rpm_step: f64,
    settle_time_s: f64,
    average_time_s: f64,
    diagnostic_samples: usize,
}

impl Command {
    fn parse(args: Vec<String>) -> Result<Self, String> {
        let mut iter = args.into_iter();
        let _bin = iter.next();
        let sub = iter.next();
        match sub.as_deref() {
            None | Some("help") | Some("--help") | Some("-h") => Ok(Self::Help),
            Some("sweep") => Ok(Self::Sweep(SweepOptions::parse(iter.collect())?)),
            Some(other) => Err(format!("unknown subcommand: {other}\n\n{}", usage_text())),
        }
    }
}

impl SweepOptions {
    fn parse(args: Vec<String>) -> Result<Self, String> {
        let mut options = Self {
            config_path: PathBuf::from(DEFAULT_CONFIG_PATH),
            output_dir: PathBuf::from(DEFAULT_OUTPUT_DIR),
            rpm_start: None,
            rpm_end: None,
            rpm_step: 200.0,
            settle_time_s: DEFAULT_SETTLE_TIME_S,
            average_time_s: DEFAULT_AVERAGE_TIME_S,
            diagnostic_samples: DEFAULT_DIAGNOSTIC_SAMPLES,
        };

        let mut i = 0usize;
        while i < args.len() {
            let flag = &args[i];
            let value = |i: &mut usize| -> Result<String, String> {
                *i += 1;
                args.get(*i)
                    .cloned()
                    .ok_or_else(|| format!("missing value for {flag}"))
            };
            match flag.as_str() {
                "--config" => options.config_path = PathBuf::from(value(&mut i)?),
                "--output-dir" => options.output_dir = PathBuf::from(value(&mut i)?),
                "--rpm-start" => options.rpm_start = Some(parse_f64(&value(&mut i)?, flag)?),
                "--rpm-end" => options.rpm_end = Some(parse_f64(&value(&mut i)?, flag)?),
                "--rpm-step" => options.rpm_step = parse_f64(&value(&mut i)?, flag)?,
                "--settle-time" => options.settle_time_s = parse_f64(&value(&mut i)?, flag)?,
                "--average-time" => options.average_time_s = parse_f64(&value(&mut i)?, flag)?,
                "--diagnostic-samples" => {
                    options.diagnostic_samples = parse_usize(&value(&mut i)?, flag)?
                }
                "--help" | "-h" => return Ok(options),
                other => return Err(format!("unknown option: {other}\n\n{}", usage_text())),
            }
            i += 1;
        }

        if options.rpm_step <= 0.0 {
            return Err("--rpm-step must be positive".to_string());
        }
        if options.settle_time_s <= 0.0 || options.average_time_s <= 0.0 {
            return Err("--settle-time and --average-time must be positive".to_string());
        }
        if options.diagnostic_samples < 180 {
            return Err("--diagnostic-samples must be at least 180".to_string());
        }
        Ok(options)
    }
}

#[derive(Debug, Clone)]
struct SweepPointResult {
    target_rpm: f64,
    mean_rpm: f64,
    torque_nm: f64,
    power_kw: f64,
    map_kpa: f64,
    air_flow_gps: f64,
    eta_indicated: f64,
    load_cmd: f64,
    output_subdir: PathBuf,
}

fn run_sweep(options: SweepOptions) -> Result<(), String> {
    let mut cfg = load_config(&options.config_path);
    cfg.model.external_load.mode = ExternalLoadMode::BrakeMap;
    cfg.model.external_load.base_torque_nm = cfg.model.external_load.base_torque_nm.max(400.0);
    cfg.model.external_load.torque_max_nm = cfg.model.external_load.torque_max_nm.max(500.0);
    cfg.model.external_load.absorber_power_limit_kw = cfg.model.external_load.absorber_power_limit_kw.max(500.0);
    cfg.model.external_load.speed_linear_nms = cfg.model.external_load.speed_linear_nms.max(0.0);
    cfg.model.external_load.speed_quadratic_nms2 = cfg.model.external_load.speed_quadratic_nms2.max(0.0);
    let rpm_start = options
        .rpm_start
        .unwrap_or(cfg.engine.default_target_rpm.round());
    let rpm_end = options.rpm_end.unwrap_or(cfg.engine.max_rpm.floor());
    if rpm_end < rpm_start {
        return Err("rpm range is invalid: end must be >= start".to_string());
    }

    fs::create_dir_all(&options.output_dir)
        .map_err(|e| format!("failed to create {}: {e}", options.output_dir.display()))?;

    let mut results = Vec::new();
    for target_rpm in rpm_grid(rpm_start, rpm_end, options.rpm_step) {
        let result = solve_operating_point(&cfg, &options, target_rpm)?;
        results.push(result);
    }

    write_sweep_summary(&options.output_dir, &cfg, &results, options.rpm_step)?;
    write_gnuplot_script(&options.output_dir)?;

    println!(
        "wrote {} operating points to {}",
        results.len(),
        options.output_dir.display()
    );
    Ok(())
}

fn solve_operating_point(
    cfg: &AppConfig,
    options: &SweepOptions,
    target_rpm: f64,
) -> Result<SweepPointResult, String> {
    let mut sim = Simulator::new(cfg);
    sim.set_pressure_trace_output_enabled(true);
    sim.control.spark_cmd = true;
    sim.control.fuel_cmd = true;
    sim.control.throttle_cmd = 1.0;
    sim.control.vvt_intake_deg = cfg.control_defaults.vvt_intake_deg;
    sim.control.vvt_exhaust_deg = cfg.control_defaults.vvt_exhaust_deg;
    let seed_load = (cfg.environment.ambient_pressure_pa / cfg.environment.ambient_pressure_pa)
        .clamp(sim.model.load_min, sim.model.load_max);
    sim.control.ignition_timing_deg = estimate_mbt_deg(&sim.model, target_rpm, seed_load);
    sim.seed_operating_point(target_rpm, 1.0, sim.control.ignition_timing_deg);

    let dt = accuracy_priority_dt(target_rpm, &sim.numerics).min(cfg.environment.dt);
    let settle_steps = (options.settle_time_s / dt).ceil() as usize;
    let average_steps = (options.average_time_s / dt).ceil() as usize;
    let mut rpm_integral = 0.0;
    let kp = 0.060;
    let ki = 0.400;
    let mut last_obs = sim.step(dt);

    for _ in 0..settle_steps.max(1) {
        let load = (last_obs.map_kpa * 1.0e3 / sim.env.ambient_pressure_pa)
            .clamp(sim.model.load_min, sim.model.load_max);
        sim.control.ignition_timing_deg = estimate_mbt_deg(&sim.model, last_obs.rpm, load);
        let rpm_error = last_obs.rpm - target_rpm;
        rpm_integral = (rpm_integral + rpm_error * dt).clamp(-8_000.0, 8_000.0);
        let desired_load_torque = (last_obs.torque_load_nm + kp * rpm_error + ki * rpm_integral)
            .clamp(0.0, cfg.model.external_load.torque_max_nm);
        sim.control.load_cmd = external_load_command_for_torque_nm(
            desired_load_torque,
            rpm_to_rad_s(last_obs.rpm),
            &cfg.model.external_load,
        )
        .clamp(0.0, 1.0);
        last_obs = sim.step(dt);
    }

    let mut rpm_sum = 0.0;
    let mut torque_sum = 0.0;
    let mut power_sum = 0.0;
    let mut map_sum = 0.0;
    let mut air_sum = 0.0;
    let mut eta_sum = 0.0;
    for _ in 0..average_steps.max(1) {
        let load = (last_obs.map_kpa * 1.0e3 / sim.env.ambient_pressure_pa)
            .clamp(sim.model.load_min, sim.model.load_max);
        sim.control.ignition_timing_deg = estimate_mbt_deg(&sim.model, last_obs.rpm, load);
        let rpm_error = last_obs.rpm - target_rpm;
        rpm_integral = (rpm_integral + rpm_error * dt).clamp(-8_000.0, 8_000.0);
        let desired_load_torque = (last_obs.torque_load_nm + kp * rpm_error + ki * rpm_integral)
            .clamp(0.0, cfg.model.external_load.torque_max_nm);
        sim.control.load_cmd = external_load_command_for_torque_nm(
            desired_load_torque,
            rpm_to_rad_s(last_obs.rpm),
            &cfg.model.external_load,
        )
        .clamp(0.0, 1.0);
        last_obs = sim.step(dt);
        rpm_sum += last_obs.rpm;
        torque_sum += last_obs.torque_net_nm;
        power_sum += shaft_power_kw(last_obs.rpm, last_obs.torque_net_nm);
        map_sum += last_obs.map_kpa;
        air_sum += last_obs.air_flow_gps;
        eta_sum += last_obs.eta_thermal_indicated_pv;
    }

    let denom = average_steps.max(1) as f64;
    let mean_rpm = rpm_sum / denom;
    if (mean_rpm - target_rpm).abs() > 80.0 {
        return Err(format!(
            "failed to converge near target rpm {:.0}: settled at {:.1} rpm",
            target_rpm, mean_rpm
        ));
    }

    let point_dir = options
        .output_dir
        .join(format!("point_{:04.0}rpm", target_rpm.round()));
    fs::create_dir_all(&point_dir)
        .map_err(|e| format!("failed to create {}: {e}", point_dir.display()))?;
    write_operating_point_outputs(&point_dir, &sim, &last_obs, options.diagnostic_samples)?;

    Ok(SweepPointResult {
        target_rpm,
        mean_rpm,
        torque_nm: torque_sum / denom,
        power_kw: power_sum / denom,
        map_kpa: map_sum / denom,
        air_flow_gps: air_sum / denom,
        eta_indicated: eta_sum / denom,
        load_cmd: sim.control.load_cmd,
        output_subdir: point_dir,
    })
}

fn write_operating_point_outputs(
    point_dir: &Path,
    sim: &Simulator,
    obs: &Observation,
    diagnostic_samples: usize,
) -> Result<(), String> {
    let pv_path = point_dir.join("pv.tsv");
    let pv = obs
        .pv_points
        .iter()
        .map(|(v, p)| format!("{v:.8}\t{p:.3}\n"))
        .collect::<String>();
    fs::write(&pv_path, format!("volume_ratio\tpressure_pa\n{pv}"))
        .map_err(|e| format!("failed to write {}: {e}", pv_path.display()))?;

    let ptheta_path = point_dir.join("ptheta.tsv");
    let curves = sim.build_ptheta_display_curves(diagnostic_samples);
    let count = curves.iter().map(Vec::len).max().unwrap_or(0);
    let mut ptheta = String::from("theta_deg\tcyl1_pressure_pa\tcyl2_pressure_pa\tcyl3_pressure_pa\tcyl4_pressure_pa\n");
    for i in 0..count {
        let theta = curves
            .first()
            .and_then(|curve| curve.get(i))
            .map(|(theta, _)| *theta)
            .unwrap_or(i as f64);
        ptheta.push_str(&format!("{theta:.3}"));
        for curve in &curves {
            let pressure = curve.get(i).map(|(_, p)| *p).unwrap_or(f64::NAN);
            ptheta.push_str(&format!("\t{pressure:.3}"));
        }
        ptheta.push('\n');
    }
    fs::write(&ptheta_path, ptheta)
        .map_err(|e| format!("failed to write {}: {e}", ptheta_path.display()))?;

    let ts_path = point_dir.join("ts.tsv");
    let ts = sim.build_ts_diagram(diagnostic_samples);
    let mut ts_body = String::from("theta_deg\ttemperature_k\tentropy_rel_j_per_kgk\tpressure_pa\tvolume_ratio\n");
    for sample in ts {
        ts_body.push_str(&format!(
            "{:.3}\t{:.6}\t{:.6}\t{:.3}\t{:.8}\n",
            sample.theta_deg,
            sample.temperature_k,
            sample.entropy_rel_j_per_kgk,
            sample.pressure_pa,
            sample.volume_ratio,
        ));
    }
    fs::write(&ts_path, ts_body)
        .map_err(|e| format!("failed to write {}: {e}", ts_path.display()))?;

    let summary_path = point_dir.join("summary.tsv");
    let summary = format!(
        "rpm\ttorque_net_nm\tpower_kw\tmap_kpa\tair_flow_gps\teta_indicated\n{:.3}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\n",
        obs.rpm,
        obs.torque_net_nm,
        shaft_power_kw(obs.rpm, obs.torque_net_nm),
        obs.map_kpa,
        obs.air_flow_gps,
        obs.eta_thermal_indicated_pv,
    );
    fs::write(&summary_path, summary)
        .map_err(|e| format!("failed to write {}: {e}", summary_path.display()))?;

    Ok(())
}

fn write_sweep_summary(
    output_dir: &Path,
    cfg: &AppConfig,
    results: &[SweepPointResult],
    rpm_step: f64,
) -> Result<(), String> {
    let torque_curve_path = output_dir.join("torque_curve.tsv");
    let mut body = String::from(
        "target_rpm\tmean_rpm\ttorque_nm\tpower_kw\tmap_kpa\tair_flow_gps\teta_indicated\tload_cmd\toutput_dir\n",
    );
    for result in results {
        body.push_str(&format!(
            "{:.3}\t{:.3}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{}\n",
            result.target_rpm,
            result.mean_rpm,
            result.torque_nm,
            result.power_kw,
            result.map_kpa,
            result.air_flow_gps,
            result.eta_indicated,
            result.load_cmd,
            result.output_subdir.display(),
        ));
    }
    fs::write(&torque_curve_path, body)
        .map_err(|e| format!("failed to write {}: {e}", torque_curve_path.display()))?;

    let manifest_path = output_dir.join("run_manifest.yaml");
    let manifest = format!(
        "config_path: {}\nexternal_load_mode: brake_map\nrpm_step: {}\nrpm_start: {}\nrpm_end: {}\npoint_count: {}\n",
        DEFAULT_CONFIG_PATH,
        rpm_step,
        results.first().map(|x| x.target_rpm).unwrap_or(cfg.engine.default_target_rpm),
        results.last().map(|x| x.target_rpm).unwrap_or(cfg.engine.max_rpm),
        results.len(),
    );
    fs::write(&manifest_path, manifest)
        .map_err(|e| format!("failed to write {}: {e}", manifest_path.display()))?;
    Ok(())
}

fn write_gnuplot_script(output_dir: &Path) -> Result<(), String> {
    let script_path = output_dir.join("plot_torque_curve.gp");
    let script = r#"set datafile separator '\t'
set key autotitle columnhead
set xlabel 'Engine speed [rpm]'
set ylabel 'Torque [Nm]'
set grid
plot 'torque_curve.tsv' using 'mean_rpm':'torque_nm' with linespoints lw 2 pt 7
"#;
    fs::write(&script_path, script)
        .map_err(|e| format!("failed to write {}: {e}", script_path.display()))?;
    Ok(())
}

fn rpm_grid(start: f64, end: f64, step: f64) -> Vec<f64> {
    let mut grid = Vec::new();
    let mut rpm = start;
    while rpm <= end + 1.0e-9 {
        grid.push(rpm);
        rpm += step;
    }
    if grid.last().is_some_and(|last| (end - *last).abs() > 1.0e-9) {
        grid.push(end);
    }
    grid
}

fn parse_f64(value: &str, flag: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .map_err(|e| format!("invalid value for {flag}: {value} ({e})"))
}

fn parse_usize(value: &str, flag: &str) -> Result<usize, String> {
    value
        .parse::<usize>()
        .map_err(|e| format!("invalid value for {flag}: {value} ({e})"))
}

fn print_usage() {
    println!("{}", usage_text());
}

fn usage_text() -> &'static str {
    "es_sim sweep [--config path] [--output-dir dir] [--rpm-start rpm] [--rpm-end rpm] [--rpm-step rpm] [--settle-time s] [--average-time s] [--diagnostic-samples n]\n\nRuns the headless CLI torque sweep and writes TSV/YAML outputs for gnuplot, p-V, T-S, and p-theta diagrams."
}
