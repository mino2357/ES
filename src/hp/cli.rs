use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use crate::hp::config::HeadlessConfig;
use crate::hp::csv::{write_outputs, write_point_outputs, write_sweep_report};
use crate::hp::model::{run_point, run_sweep};
use crate::hp::report::{format_config_metrics, format_point_report, format_sweep_report};

const DEFAULT_CONFIG_PATH: &str = "config/high_precision.yaml";

pub fn run_from_env(program_name: &str) -> Result<(), String> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    run(program_name, &args)
}

fn run(program_name: &str, args: &[String]) -> Result<(), String> {
    if args.is_empty() {
        return run_sweep_command(DEFAULT_CONFIG_PATH, None, true, true);
    }

    match args[0].as_str() {
        "-h" | "--help" | "help" => {
            print_help(program_name);
            Ok(())
        }
        "validate" | "audit" => run_validate_command(
            args.get(1)
                .map(String::as_str)
                .unwrap_or(DEFAULT_CONFIG_PATH),
        ),
        "sweep" => {
            let (config_path, output_dir, write_outputs_flag, write_report_flag) =
                parse_sweep_args(&args[1..])?;
            run_sweep_command(
                &config_path,
                output_dir.as_deref(),
                write_outputs_flag,
                write_report_flag,
            )
        }
        "report" => {
            let (config_path, output_dir, _, _) = parse_sweep_args(&args[1..])?;
            run_sweep_command(&config_path, output_dir.as_deref(), true, true)
        }
        "point" => {
            let parsed = parse_point_args(&args[1..])?;
            run_point_command(
                &parsed.config_path,
                parsed.output_dir.as_deref(),
                parsed.rpm,
                parsed.write_pv,
            )
        }
        "list-presets" => run_list_presets_command(args.get(1).map(String::as_str)),
        other => run_sweep_command(other, None, true, true),
    }
}

fn run_validate_command(config_path: &str) -> Result<(), String> {
    let config = HeadlessConfig::load(config_path)?;
    println!("Configuration: {config_path}");
    println!("{}", format_config_metrics(&config));
    println!("Validation: OK");
    Ok(())
}

fn run_sweep_command(
    config_path: &str,
    output_dir_override: Option<&str>,
    write_outputs_flag: bool,
    write_report_flag: bool,
) -> Result<(), String> {
    let mut config = HeadlessConfig::load(config_path)?;
    if let Some(output_dir) = output_dir_override {
        config.output_dir = output_dir.to_string();
    }
    let result = run_sweep(&config)?;
    let report = format_sweep_report(&config, &result);
    println!("{report}");
    if write_outputs_flag {
        let output_dir = PathBuf::from(&config.output_dir);
        write_outputs(&output_dir, &result)?;
        if write_report_flag {
            write_sweep_report(&output_dir, &config, &result)?;
        }
        println!("Output directory: {}", output_dir.display());
    }
    Ok(())
}

fn run_point_command(
    config_path: &str,
    output_dir_override: Option<&str>,
    rpm: f64,
    write_pv: bool,
) -> Result<(), String> {
    let mut config = HeadlessConfig::load(config_path)?;
    if let Some(output_dir) = output_dir_override {
        config.output_dir = output_dir.to_string();
    }
    let point = run_point(&config, rpm, write_pv)?;
    println!("{}", format_point_report(&config, &point));
    if write_pv {
        let output_dir = PathBuf::from(&config.output_dir);
        write_point_outputs(&output_dir, &config, &point)?;
        println!("Output directory: {}", output_dir.display());
    }
    Ok(())
}

fn run_list_presets_command(root_override: Option<&str>) -> Result<(), String> {
    let mut roots = Vec::new();
    if let Some(root) = root_override {
        roots.push(PathBuf::from(root));
    } else {
        roots.push(PathBuf::from("config"));
        roots.push(PathBuf::from("config/presets"));
    }

    let mut listed = Vec::new();
    for root in roots {
        if !root.exists() {
            continue;
        }
        for entry in fs::read_dir(&root)
            .map_err(|err| format!("failed to read '{}': {err}", root.display()))?
        {
            let entry = entry.map_err(|err| format!("failed to read entry: {err}"))?;
            let path = entry.path();
            if is_yaml_path(&path) {
                listed.push(path);
            }
        }
    }
    listed.sort();
    listed.dedup();
    for path in listed {
        println!("{}", path.display());
    }
    Ok(())
}

fn print_help(program_name: &str) {
    println!(
        "{program_name} commands:
  {program_name} [config.yaml]
  {program_name} sweep [config.yaml] [--output-dir DIR] [--no-write] [--no-report]
  {program_name} report [config.yaml] [--output-dir DIR]
  {program_name} point [config.yaml] --rpm RPM [--output-dir DIR] [--write-pv]
  {program_name} validate [config.yaml]
  {program_name} list-presets [DIR]"
    );
}

fn parse_sweep_args(args: &[String]) -> Result<(String, Option<String>, bool, bool), String> {
    let mut config_path = DEFAULT_CONFIG_PATH.to_string();
    let mut output_dir = None;
    let mut write_outputs_flag = true;
    let mut write_report_flag = true;
    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--output-dir" => {
                i += 1;
                let Some(path) = args.get(i) else {
                    return Err("--output-dir requires a path".to_string());
                };
                output_dir = Some(path.clone());
            }
            "--no-write" => write_outputs_flag = false,
            "--no-report" => write_report_flag = false,
            candidate if candidate.starts_with('-') => {
                return Err(format!("unknown option '{candidate}'"));
            }
            candidate => {
                config_path = candidate.to_string();
            }
        }
        i += 1;
    }
    Ok((
        config_path,
        output_dir,
        write_outputs_flag,
        write_report_flag,
    ))
}

#[derive(Debug)]
struct PointArgs {
    config_path: String,
    output_dir: Option<String>,
    rpm: f64,
    write_pv: bool,
}

fn parse_point_args(args: &[String]) -> Result<PointArgs, String> {
    let mut config_path = DEFAULT_CONFIG_PATH.to_string();
    let mut output_dir = None;
    let mut rpm = None;
    let mut write_pv = false;
    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--rpm" => {
                i += 1;
                let Some(value) = args.get(i) else {
                    return Err("--rpm requires a value".to_string());
                };
                rpm = Some(
                    value
                        .parse::<f64>()
                        .map_err(|_| format!("invalid rpm value '{value}'"))?,
                );
            }
            "--output-dir" => {
                i += 1;
                let Some(path) = args.get(i) else {
                    return Err("--output-dir requires a path".to_string());
                };
                output_dir = Some(path.clone());
            }
            "--write-pv" => write_pv = true,
            candidate if candidate.starts_with('-') => {
                return Err(format!("unknown option '{candidate}'"));
            }
            candidate => {
                config_path = candidate.to_string();
            }
        }
        i += 1;
    }
    let Some(rpm) = rpm else {
        return Err("point command requires --rpm".to_string());
    };
    Ok(PointArgs {
        config_path,
        output_dir,
        rpm,
        write_pv,
    })
}

fn is_yaml_path(path: &Path) -> bool {
    path.extension()
        .and_then(|ext| ext.to_str())
        .map(|ext| matches!(ext, "yaml" | "yml"))
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::{parse_point_args, parse_sweep_args};

    #[test]
    fn sweep_args_accept_config_and_output_dir() {
        let args = vec![
            "config/high_precision.yaml".to_string(),
            "--output-dir".to_string(),
            "dist/test".to_string(),
        ];
        let (config, output_dir, write_outputs_flag, write_report_flag) =
            parse_sweep_args(&args).expect("args should parse");
        assert_eq!(config, "config/high_precision.yaml");
        assert_eq!(output_dir.as_deref(), Some("dist/test"));
        assert!(write_outputs_flag);
        assert!(write_report_flag);
    }

    #[test]
    fn point_args_require_rpm() {
        let args = vec!["config/high_precision.yaml".to_string()];
        let err = parse_point_args(&args).expect_err("rpm should be required");
        assert!(err.contains("--rpm"));
    }

    #[test]
    fn point_args_accept_write_pv_and_override_dir() {
        let args = vec![
            "config/high_precision.yaml".to_string(),
            "--rpm".to_string(),
            "3500".to_string(),
            "--write-pv".to_string(),
            "--output-dir".to_string(),
            "dist/out".to_string(),
        ];
        let parsed = parse_point_args(&args).expect("args should parse");
        assert_eq!(parsed.config_path, "config/high_precision.yaml");
        assert_eq!(parsed.rpm, 3500.0);
        assert!(parsed.write_pv);
        assert_eq!(parsed.output_dir.as_deref(), Some("dist/out"));
    }
}
