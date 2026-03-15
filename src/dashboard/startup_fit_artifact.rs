use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::build_info;
use super::startup_fit::{
    StartupFitControls, StartupFitTorqueCurvePoint, StartupFitWotTorquePoint,
};
use crate::config::LoadedAppConfig;

const STARTUP_FIT_ARTIFACT_FORMAT_VERSION: u32 = 3;

#[derive(Debug, Clone)]
pub(super) struct StartupFitCacheContext {
    artifact_path: PathBuf,
    build_identity: String,
    config_yaml_hash: String,
}

impl StartupFitCacheContext {
    pub(super) fn from_loaded_config(loaded: &LoadedAppConfig) -> Option<Self> {
        let resolved_path = loaded.resolved_path.as_ref()?;
        let source_text = loaded.source_text.as_ref()?;
        let build_identity = build_info::cache_identity();
        let config_yaml_hash = stable_text_hash_hex(source_text);
        let build_hash = stable_text_hash_hex(&build_identity);
        let artifact_dir = artifact_dir_for_resolved_config(resolved_path);
        let artifact_path = artifact_dir.join(format!(
            "startup_fit_v{STARTUP_FIT_ARTIFACT_FORMAT_VERSION}__build_{build_hash}__config_{config_yaml_hash}.yaml"
        ));
        Some(Self {
            artifact_path,
            build_identity,
            config_yaml_hash,
        })
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(super) fn artifact_path(&self) -> &Path {
        &self.artifact_path
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub(super) struct StartupFitArtifactEvaluation {
    pub(super) avg_rpm: f64,
    pub(super) avg_net_torque_nm: f64,
    pub(super) required_brake_torque_nm: f64,
    pub(super) load_cmd: f64,
    pub(super) periodic_error_norm: f64,
    pub(super) converged: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct StartupFitArtifactSnapshot {
    pub(super) target_rpm: f64,
    pub(super) timed_out: bool,
    pub(super) release_controls: StartupFitControls,
    pub(super) release_evaluation: StartupFitArtifactEvaluation,
    pub(super) best_required_brake_torque_nm: f64,
    pub(super) torque_margin_to_best_nm: f64,
    pub(super) torque_curve: Vec<StartupFitTorqueCurvePoint>,
    pub(super) wot_torque_curve: Vec<StartupFitWotTorquePoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub(super) struct StartupFitArtifact {
    format_version: u32,
    build_identity: String,
    config_yaml_hash: String,
    pub(super) target_rpm: f64,
    pub(super) timed_out: bool,
    pub(super) release_controls: StartupFitControls,
    pub(super) release_evaluation: StartupFitArtifactEvaluation,
    pub(super) best_required_brake_torque_nm: f64,
    pub(super) torque_margin_to_best_nm: f64,
    pub(super) torque_curve: Vec<StartupFitTorqueCurvePoint>,
    #[serde(default)]
    pub(super) wot_torque_curve: Vec<StartupFitWotTorquePoint>,
}

impl StartupFitArtifact {
    pub(super) fn new(
        context: &StartupFitCacheContext,
        snapshot: StartupFitArtifactSnapshot,
    ) -> Self {
        Self {
            format_version: STARTUP_FIT_ARTIFACT_FORMAT_VERSION,
            build_identity: context.build_identity.clone(),
            config_yaml_hash: context.config_yaml_hash.clone(),
            target_rpm: snapshot.target_rpm,
            timed_out: snapshot.timed_out,
            release_controls: snapshot.release_controls,
            release_evaluation: snapshot.release_evaluation,
            best_required_brake_torque_nm: snapshot.best_required_brake_torque_nm,
            torque_margin_to_best_nm: snapshot.torque_margin_to_best_nm,
            torque_curve: snapshot.torque_curve,
            wot_torque_curve: snapshot.wot_torque_curve,
        }
    }
}

pub(super) fn load_matching_artifact(
    context: &StartupFitCacheContext,
) -> Result<Option<StartupFitArtifact>, String> {
    if !context.artifact_path.is_file() {
        return Ok(None);
    }

    let text = fs::read_to_string(&context.artifact_path).map_err(|err| {
        format!(
            "failed to read startup-fit artifact {}: {err}",
            context.artifact_path.display()
        )
    })?;
    let artifact = serde_yaml::from_str::<StartupFitArtifact>(&text).map_err(|err| {
        format!(
            "failed to parse startup-fit artifact {}: {err}",
            context.artifact_path.display()
        )
    })?;
    let compatible = artifact.format_version == STARTUP_FIT_ARTIFACT_FORMAT_VERSION
        && artifact.build_identity == context.build_identity
        && artifact.config_yaml_hash == context.config_yaml_hash;
    Ok(compatible.then_some(artifact))
}

pub(super) fn save_artifact(
    context: &StartupFitCacheContext,
    artifact: &StartupFitArtifact,
) -> Result<(), String> {
    let Some(parent) = context.artifact_path.parent() else {
        return Err(format!(
            "invalid startup-fit artifact path: {}",
            context.artifact_path.display()
        ));
    };
    fs::create_dir_all(parent).map_err(|err| {
        format!(
            "failed to create startup-fit cache directory {}: {err}",
            parent.display()
        )
    })?;
    let text = serde_yaml::to_string(artifact).map_err(|err| {
        format!(
            "failed to serialize startup-fit artifact {}: {err}",
            context.artifact_path.display()
        )
    })?;
    fs::write(&context.artifact_path, text).map_err(|err| {
        format!(
            "failed to write startup-fit artifact {}: {err}",
            context.artifact_path.display()
        )
    })
}

fn artifact_dir_for_resolved_config(resolved_config_path: &Path) -> PathBuf {
    let Some(parent) = resolved_config_path.parent() else {
        return PathBuf::from("cache").join("startup_fit");
    };
    if parent.file_name().and_then(|name| name.to_str()) == Some("config") {
        if let Some(root) = parent.parent() {
            return root.join("cache").join("startup_fit");
        }
    }
    parent.join("cache").join("startup_fit")
}

fn stable_text_hash_hex(text: &str) -> String {
    const OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;

    let mut hash = OFFSET_BASIS;
    for byte in text.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    format!("{hash:016x}")
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    use crate::config::{AppConfig, LoadedAppConfig};

    use super::{
        StartupFitArtifact, StartupFitArtifactEvaluation, StartupFitArtifactSnapshot,
        StartupFitCacheContext, load_matching_artifact, save_artifact, stable_text_hash_hex,
    };
    use crate::dashboard::startup_fit::{
        StartupFitControls, StartupFitTorqueCurvePoint, StartupFitWotTorquePoint,
    };

    fn unique_temp_root() -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time before unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!("es_sim_startup_fit_cache_test_{nanos}"))
    }

    fn loaded_config(root: &Path, source_text: &str) -> LoadedAppConfig {
        let resolved_path = root.join("config").join("sim.yaml");
        LoadedAppConfig {
            config: AppConfig::default(),
            resolved_path: Some(resolved_path),
            source_text: Some(source_text.to_owned()),
        }
    }

    #[test]
    fn yaml_hash_changes_when_text_changes() {
        assert_ne!(
            stable_text_hash_hex("engine:\n  max_rpm: 7000\n"),
            stable_text_hash_hex("engine:\n  max_rpm: 7200\n")
        );
    }

    #[test]
    fn cache_context_places_artifacts_under_repo_cache_directory() {
        let root = unique_temp_root();
        let context = StartupFitCacheContext::from_loaded_config(&loaded_config(
            &root,
            "engine:\n  max_rpm: 7000\n",
        ))
        .expect("cache context");

        assert_eq!(
            context.artifact_path().parent().expect("artifact parent"),
            root.join("cache").join("startup_fit").as_path()
        );
    }

    #[test]
    fn artifact_roundtrip_requires_matching_build_and_yaml_identity() {
        let root = unique_temp_root();
        let loaded = loaded_config(&root, "engine:\n  max_rpm: 7000\n");
        let context =
            StartupFitCacheContext::from_loaded_config(&loaded).expect("cache context exists");
        let artifact = StartupFitArtifact::new(
            &context,
            StartupFitArtifactSnapshot {
                target_rpm: 2_000.0,
                timed_out: false,
                release_controls: StartupFitControls {
                    throttle_cmd: 0.22,
                    ignition_timing_deg: 21.5,
                    vvt_intake_deg: 0.0,
                    vvt_exhaust_deg: 0.0,
                    load_cmd: 0.18,
                },
                release_evaluation: StartupFitArtifactEvaluation {
                    avg_rpm: 1_998.0,
                    avg_net_torque_nm: 0.4,
                    required_brake_torque_nm: 18.1,
                    load_cmd: 0.18,
                    periodic_error_norm: 0.012,
                    converged: true,
                },
                best_required_brake_torque_nm: 18.4,
                torque_margin_to_best_nm: 0.3,
                torque_curve: vec![StartupFitTorqueCurvePoint {
                    throttle_cmd: 0.22,
                    required_brake_torque_nm: 18.1,
                }],
                wot_torque_curve: vec![StartupFitWotTorquePoint {
                    engine_speed_rpm: 2_000.0,
                    available_brake_torque_nm: 18.1,
                    ignition_timing_deg: 21.5,
                }],
            },
        );

        save_artifact(&context, &artifact).expect("artifact saved");
        let loaded_artifact = load_matching_artifact(&context)
            .expect("artifact load succeeds")
            .expect("artifact hit");
        assert_eq!(loaded_artifact, artifact);

        let mismatched = StartupFitCacheContext::from_loaded_config(&loaded_config(
            &root,
            "engine:\n  max_rpm: 7200\n",
        ))
        .expect("mismatched cache context");
        assert!(
            load_matching_artifact(&mismatched)
                .expect("mismatched load succeeds")
                .is_none()
        );

        let _ = fs::remove_dir_all(root);
    }
}
