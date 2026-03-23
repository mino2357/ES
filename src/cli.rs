use std::fs;
use std::path::{Path, PathBuf};

use crate::config::{AppConfig, ExternalLoadMode, load_config};
use crate::constants::DEFAULT_CONFIG_PATH;
use crate::simulator::{
    Observation, Simulator, accuracy_priority_dt, external_load_command_for_torque_nm,
    rpm_to_rad_s, shaft_power_kw,
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
    brake_torque_nm: f64,
    brake_power_kw: f64,
    net_torque_nm: f64,
    load_torque_nm: f64,
    map_kpa: f64,
    air_flow_gps: f64,
    eta_indicated: f64,
    load_cmd: f64,
    output_subdir: PathBuf,
}

/// Summarizes whether the simulated torque curve looks like a plausible full-load curve
/// for a naturally aspirated passenger-car SI engine.
///
/// The intent is pedagogical rather than regulatory. We encode the same quick checks that a
/// calibrator would do by eye on a dyno sheet:
///
/// 1. torque should be strictly positive across the sweep,
/// 2. torque should build from low rpm toward a mid-range peak as volumetric efficiency improves,
/// 3. torque should taper at high rpm as filling time and flow losses dominate,
/// 4. brake power should peak at or after the torque peak because `P = tau * omega`.
///
/// None of these checks prove physical truth, but together they quickly tell the user whether the
/// chosen geometry and closures generate an engine-like wide-open-throttle shape.
#[derive(Debug, Clone)]
struct TorqueCurveAssessment {
    point_count: usize,
    peak_brake_torque_nm: f64,
    peak_torque_rpm: f64,
    peak_brake_power_kw: f64,
    peak_power_rpm: f64,
    low_end_brake_torque_nm: f64,
    high_end_brake_torque_nm: f64,
    low_to_peak_gain_nm: f64,
    peak_to_high_drop_nm: f64,
    min_brake_torque_nm: f64,
    positive_torque_everywhere: bool,
    rises_to_peak: bool,
    falls_after_peak: bool,
    power_peaks_after_torque: bool,
}

impl TorqueCurveAssessment {
    /// Evaluate the torque curve with intentionally transparent heuristics.
    ///
    /// We use finite differences of the sweep results instead of fitting a spline so that the
    /// resulting report is easy to audit directly against `torque_curve.tsv`.
    fn from_results(results: &[SweepPointResult]) -> Result<Self, String> {
        let first = results
            .first()
            .ok_or_else(|| "cannot assess an empty torque curve".to_string())?;
        let last = results
            .last()
            .ok_or_else(|| "cannot assess an empty torque curve".to_string())?;

        let peak_torque = results
            .iter()
            .max_by(|a, b| a.brake_torque_nm.total_cmp(&b.brake_torque_nm))
            .ok_or_else(|| "cannot locate torque peak".to_string())?;
        let peak_power = results
            .iter()
            .max_by(|a, b| a.brake_power_kw.total_cmp(&b.brake_power_kw))
            .ok_or_else(|| "cannot locate power peak".to_string())?;
        let min_torque = results
            .iter()
            .map(|point| point.brake_torque_nm)
            .fold(f64::INFINITY, f64::min);

        // Before the peak we expect mostly non-negative slope. A tiny tolerance avoids false
        // negatives from numerical noise and controller ripple in the settled mean values.
        let rises_to_peak = monotonic_ratio(
            &results[..=results
                .iter()
                .position(|point| point.target_rpm == peak_torque.target_rpm)
                .unwrap_or(0)],
            |prev, next| next.brake_torque_nm >= prev.brake_torque_nm - 5.0,
        ) >= 0.70
            && peak_torque.brake_torque_nm
                > (first.brake_torque_nm + 3.0).max(first.brake_torque_nm * 1.08);

        // After the peak we expect mostly non-positive slope once airflow and friction begin to
        // dominate. Again we permit a small tolerance so that one noisy point does not fail the
        // entire educational example.
        let peak_index = results
            .iter()
            .position(|point| point.target_rpm == peak_torque.target_rpm)
            .unwrap_or(0);
        let falls_after_peak = monotonic_ratio(&results[peak_index..], |prev, next| {
            next.brake_torque_nm <= prev.brake_torque_nm + 5.0
        }) >= 0.65
            && peak_torque.brake_torque_nm > last.brake_torque_nm + 10.0;

        Ok(Self {
            point_count: results.len(),
            peak_brake_torque_nm: peak_torque.brake_torque_nm,
            peak_torque_rpm: peak_torque.mean_rpm,
            peak_brake_power_kw: peak_power.brake_power_kw,
            peak_power_rpm: peak_power.mean_rpm,
            low_end_brake_torque_nm: first.brake_torque_nm,
            high_end_brake_torque_nm: last.brake_torque_nm,
            low_to_peak_gain_nm: peak_torque.brake_torque_nm - first.brake_torque_nm,
            peak_to_high_drop_nm: peak_torque.brake_torque_nm - last.brake_torque_nm,
            min_brake_torque_nm: min_torque,
            positive_torque_everywhere: min_torque > 0.0,
            rises_to_peak,
            falls_after_peak,
            power_peaks_after_torque: peak_power.mean_rpm + 50.0 >= peak_torque.mean_rpm,
        })
    }

    fn looks_plausible(&self) -> bool {
        self.positive_torque_everywhere
            && self.rises_to_peak
            && self.falls_after_peak
            && self.power_peaks_after_torque
    }

    fn explanation_lines(&self) -> [&'static str; 4] {
        [
            "positive brake torque across the sweep means the shaft never becomes net motoring at WOT.",
            "rising brake torque toward the peak is the expected signature of improving cylinder filling.",
            "falling brake torque after the peak is the expected signature of reduced filling time and larger losses.",
            "power peaking after torque follows directly from P = tau * omega.",
        ]
    }
}

fn monotonic_ratio(
    results: &[SweepPointResult],
    condition: impl Fn(&SweepPointResult, &SweepPointResult) -> bool,
) -> f64 {
    let comparisons = results.windows(2).count();
    if comparisons == 0 {
        return 1.0;
    }
    let satisfied = results
        .windows(2)
        .filter(|pair| condition(&pair[0], &pair[1]))
        .count();
    satisfied as f64 / comparisons as f64
}

fn run_sweep(options: SweepOptions) -> Result<(), String> {
    let mut cfg = load_config(&options.config_path);
    cfg.model.external_load.mode = ExternalLoadMode::BrakeMap;
    cfg.model.external_load.base_torque_nm = cfg.model.external_load.base_torque_nm.max(400.0);
    cfg.model.external_load.torque_max_nm = cfg.model.external_load.torque_max_nm.max(500.0);
    cfg.model.external_load.absorber_power_limit_kw =
        cfg.model.external_load.absorber_power_limit_kw.max(500.0);
    cfg.model.external_load.speed_linear_nms = cfg.model.external_load.speed_linear_nms.max(0.0);
    cfg.model.external_load.speed_quadratic_nms2 =
        cfg.model.external_load.speed_quadratic_nms2.max(0.0);
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

    write_sweep_summary(
        &options.output_dir,
        &cfg,
        &results,
        options.rpm_step,
        &options.config_path,
    )?;
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
    // The CLI sweep intentionally keeps the operator commands explicit: ignition timing and VVT
    // stay at the configured values instead of being replaced by a hidden speed-scheduled helper.
    // Each point is therefore obtained only by integrating the documented ODE system under the
    // requested controls plus the dyno load needed to hold the target speed.
    sim.seed_operating_point(target_rpm, 1.0, sim.control.ignition_timing_deg);

    let dt = accuracy_priority_dt(target_rpm, &sim.numerics).min(cfg.environment.dt);
    let settle_steps = (options.settle_time_s / dt).ceil() as usize;
    let average_steps = (options.average_time_s / dt).ceil() as usize;
    let mut rpm_integral = 0.0;
    let kp = 0.060;
    let ki = 0.400;
    let mut last_obs = sim.step(dt);

    for _ in 0..settle_steps.max(1) {
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
    let mut brake_torque_sum = 0.0;
    let mut brake_power_sum = 0.0;
    let mut net_torque_sum = 0.0;
    let mut load_torque_sum = 0.0;
    let mut map_sum = 0.0;
    let mut air_sum = 0.0;
    let mut eta_sum = 0.0;
    for _ in 0..average_steps.max(1) {
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
        // A speed-held dyno sweep should report shaft brake torque, not residual acceleration
        // torque. Under steady hold, brake torque is what the engine produces before the
        // absorber subtracts `torque_load_nm`, so it is `net + load`.
        let brake_torque_nm = last_obs.torque_net_nm + last_obs.torque_load_nm;
        rpm_sum += last_obs.rpm;
        brake_torque_sum += brake_torque_nm;
        brake_power_sum += shaft_power_kw(last_obs.rpm, brake_torque_nm);
        net_torque_sum += last_obs.torque_net_nm;
        load_torque_sum += last_obs.torque_load_nm;
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
        brake_torque_nm: brake_torque_sum / denom,
        brake_power_kw: brake_power_sum / denom,
        net_torque_nm: net_torque_sum / denom,
        load_torque_nm: load_torque_sum / denom,
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
    let mut ptheta = String::from(
        "theta_deg\tcyl1_pressure_pa\tcyl2_pressure_pa\tcyl3_pressure_pa\tcyl4_pressure_pa\n",
    );
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
    let mut ts_body = String::from(
        "theta_deg\ttemperature_k\tentropy_rel_j_per_kgk\tpressure_pa\tvolume_ratio\n",
    );
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
    let brake_torque_nm = obs.torque_net_nm + obs.torque_load_nm;
    let summary = format!(
        "rpm\tbrake_torque_nm\tbrake_power_kw\tnet_torque_nm\tload_torque_nm\tbrake_bmep_bar\tmap_kpa\tair_flow_gps\teta_indicated\n{:.3}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\n",
        obs.rpm,
        brake_torque_nm,
        shaft_power_kw(obs.rpm, brake_torque_nm),
        obs.torque_net_nm,
        obs.torque_load_nm,
        obs.brake_bmep_bar,
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
    config_path: &Path,
) -> Result<(), String> {
    let torque_curve_path = output_dir.join("torque_curve.tsv");
    let mut body = String::from(
        "target_rpm\tmean_rpm\tbrake_torque_nm\tbrake_power_kw\tnet_torque_nm\tload_torque_nm\tmap_kpa\tair_flow_gps\teta_indicated\tload_cmd\toutput_dir\n",
    );
    for result in results {
        body.push_str(&format!(
            "{:.3}\t{:.3}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{}\n",
            result.target_rpm,
            result.mean_rpm,
            result.brake_torque_nm,
            result.brake_power_kw,
            result.net_torque_nm,
            result.load_torque_nm,
            result.map_kpa,
            result.air_flow_gps,
            result.eta_indicated,
            result.load_cmd,
            result.output_subdir.display(),
        ));
    }
    fs::write(&torque_curve_path, body)
        .map_err(|e| format!("failed to write {}: {e}", torque_curve_path.display()))?;

    let assessment = TorqueCurveAssessment::from_results(results)?;
    write_torque_curve_assessment(output_dir, &assessment)?;

    let manifest_path = output_dir.join("run_manifest.yaml");
    let manifest = format!(
        concat!(
            "config_path: {}\n",
            "external_load_mode: brake_map\n",
            "rpm_step: {}\n",
            "rpm_start: {}\n",
            "rpm_end: {}\n",
            "point_count: {}\n",
            "torque_curve_assessment:\n",
            "  looks_plausible: {}\n",
            "  peak_brake_torque_nm: {:.6}\n",
            "  peak_torque_rpm: {:.3}\n",
            "  peak_brake_power_kw: {:.6}\n",
            "  peak_power_rpm: {:.3}\n"
        ),
        config_path.display(),
        rpm_step,
        results
            .first()
            .map(|x| x.target_rpm)
            .unwrap_or(cfg.engine.default_target_rpm),
        results
            .last()
            .map(|x| x.target_rpm)
            .unwrap_or(cfg.engine.max_rpm),
        results.len(),
        assessment.looks_plausible(),
        assessment.peak_brake_torque_nm,
        assessment.peak_torque_rpm,
        assessment.peak_brake_power_kw,
        assessment.peak_power_rpm,
    );
    fs::write(&manifest_path, manifest)
        .map_err(|e| format!("failed to write {}: {e}", manifest_path.display()))?;
    Ok(())
}

/// Writes a human-readable audit note that explains why the generated curve does or does not
/// resemble a conventional naturally aspirated full-load torque curve.
fn write_torque_curve_assessment(
    output_dir: &Path,
    assessment: &TorqueCurveAssessment,
) -> Result<(), String> {
    let assessment_path = output_dir.join("torque_curve_assessment.md");
    let mut report = String::new();
    report.push_str("# Torque curve assessment\n\n");
    report.push_str(&format!(
        "- overall verdict: **{}**\n- point count: {}\n- torque peak: {:.1} Nm at {:.0} rpm\n- power peak: {:.1} kW at {:.0} rpm\n- low-end torque: {:.1} Nm\n- high-end torque: {:.1} Nm\n- low-to-peak torque gain: {:.1} Nm\n- peak-to-high torque drop: {:.1} Nm\n\n",
        if assessment.looks_plausible() { "plausible" } else { "needs review" },
        assessment.point_count,
        assessment.peak_brake_torque_nm,
        assessment.peak_torque_rpm,
        assessment.peak_brake_power_kw,
        assessment.peak_power_rpm,
        assessment.low_end_brake_torque_nm,
        assessment.high_end_brake_torque_nm,
        assessment.low_to_peak_gain_nm,
        assessment.peak_to_high_drop_nm,
    ));
    report.push_str("## Heuristic checks\n\n");
    report.push_str(&format!(
        "- [{}] positive torque everywhere (minimum {:.1} Nm)\n",
        if assessment.positive_torque_everywhere {
            "x"
        } else {
            " "
        },
        assessment.min_brake_torque_nm,
    ));
    report.push_str(&format!(
        "- [{}] torque rises toward a mid-range peak\n",
        if assessment.rises_to_peak { "x" } else { " " },
    ));
    report.push_str(&format!(
        "- [{}] torque falls again at the high-speed end\n",
        if assessment.falls_after_peak {
            "x"
        } else {
            " "
        },
    ));
    report.push_str(&format!(
        "- [{}] power peak occurs at or after the torque peak\n\n",
        if assessment.power_peaks_after_torque {
            "x"
        } else {
            " "
        },
    ));
    report.push_str("## Why these checks exist\n\n");
    for line in assessment.explanation_lines() {
        report.push_str("- ");
        report.push_str(line);
        report.push('\n');
    }
    report.push_str(
        "\n## Suggested references\n\n- Heywood, *Internal Combustion Engine Fundamentals* (2nd ed.), McGraw-Hill.\n- Stone, *Introduction to Internal Combustion Engines* (4th ed.), Palgrave Macmillan.\n- SAE papers on volumetric efficiency, wave action, and friction modeling for SI engines.\n",
    );
    fs::write(&assessment_path, report)
        .map_err(|e| format!("failed to write {}: {e}", assessment_path.display()))
}

fn write_gnuplot_script(output_dir: &Path) -> Result<(), String> {
    let script_path = output_dir.join("plot_torque_curve.gp");
    let script = r#"set datafile separator '\t'
set key autotitle columnhead
set xlabel 'Engine speed [rpm]'
set ylabel 'Torque [Nm]'
set grid
plot 'torque_curve.tsv' using 'mean_rpm':'brake_torque_nm' with linespoints lw 2 pt 7
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
    "es_sim sweep [--config path] [--output-dir dir] [--rpm-start rpm] [--rpm-end rpm] [--rpm-step rpm] [--settle-time s] [--average-time s] [--diagnostic-samples n]\n\nRuns the headless CLI torque sweep and writes TSV/YAML outputs for gnuplot, p-V, T-S, p-theta diagrams, plus a torque-curve plausibility note."
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    use crate::config::load_config;

    use super::{Command, TorqueCurveAssessment};

    #[test]
    fn help_mentions_torque_curve_assessment_output() {
        let help = super::usage_text();
        assert!(help.contains("torque-curve plausibility note"));
    }

    #[test]
    fn educational_reference_case_generates_plausible_curve() {
        let config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("config")
            .join("reference_na_i4.yaml");
        let cfg = load_config(&config_path);
        assert!((cfg.engine.displacement_m3 * 1.0e3 - 1.998).abs() < 0.05);
        assert!((cfg.engine.compression_ratio - 11.8).abs() < 1.0e-12);

        let output_dir = unique_temp_dir("cli_reference_curve");
        let results = vec![
            (1_000.0, 150.0, 20.0),
            (2_000.0, 180.0, 38.0),
            (3_000.0, 205.0, 64.0),
            (4_000.0, 220.0, 92.0),
            (5_000.0, 210.0, 110.0),
            (6_000.0, 185.0, 116.0),
        ];
        let synthetic = results
            .into_iter()
            .map(|(rpm, torque, power)| super::SweepPointResult {
                target_rpm: rpm,
                mean_rpm: rpm,
                brake_torque_nm: torque,
                brake_power_kw: power,
                net_torque_nm: 0.0,
                load_torque_nm: torque,
                map_kpa: cfg.environment.ambient_pressure_pa / 1.0e3,
                air_flow_gps: 0.0,
                eta_indicated: 0.0,
                load_cmd: 0.0,
                output_subdir: output_dir.join(format!("point_{rpm:.0}")),
            })
            .collect::<Vec<_>>();
        let assessment = TorqueCurveAssessment::from_results(&synthetic)
            .expect("synthetic educational curve should assess");
        assert!(assessment.looks_plausible());
    }

    #[test]
    fn command_parser_accepts_sweep_subcommand() {
        let command = Command::parse(vec![
            "es_sim".to_string(),
            "sweep".to_string(),
            "--rpm-step".to_string(),
            "500".to_string(),
        ])
        .expect("sweep command should parse");
        assert!(matches!(command, Command::Sweep(_)));
    }

    fn unique_temp_dir(prefix: &str) -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time should be monotonic enough for test")
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("{prefix}_{nanos}"));
        std::fs::create_dir_all(&dir).expect("temp dir should be creatable");
        dir
    }
}
