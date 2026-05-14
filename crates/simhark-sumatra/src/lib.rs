use std::env;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SumatraLaunchConfig {
    pub repo_root: PathBuf,
    pub moduli: String,
    pub headless: bool,
    pub ai_blue: bool,
    pub ai_yellow: bool,
    pub auto_ref: bool,
    pub max_speed: bool,
    pub host: Option<String>,
    pub remote_client: bool,
}

impl Default for SumatraLaunchConfig {
    fn default() -> Self {
        Self {
            repo_root: default_sumatra_repo(),
            moduli: "simulation_protocol".to_string(),
            headless: true,
            ai_blue: true,
            ai_yellow: true,
            auto_ref: false,
            max_speed: true,
            host: None,
            remote_client: false,
        }
    }
}

pub struct SumatraInstance {
    child: Child,
}

impl SumatraInstance {
    pub fn spawn(config: &SumatraLaunchConfig) -> Result<Self> {
        ensure_sumatra_repo(&config.repo_root)?;
        let launcher = sumatra_launcher(&config.repo_root);
        if !launcher.exists() {
            bail!("missing built Sumatra launcher at {}", launcher.display());
        }

        let mut sumatra_args = Vec::new();
        let moduli = if config.remote_client {
            "sim_client"
        } else {
            &config.moduli
        };
        sumatra_args.push(format!("--moduli={moduli}"));
        if config.headless {
            sumatra_args.push("--headless".to_string());
        }
        if config.ai_blue {
            sumatra_args.push("--aiBlue".to_string());
        }
        if config.ai_yellow {
            sumatra_args.push("--aiYellow".to_string());
        }
        if config.auto_ref {
            sumatra_args.push("--autoRef".to_string());
        }
        // sim_client has no local simulator, so Sumatra's max-speed setup crashes there.
        if config.max_speed && !config.remote_client {
            sumatra_args.push("--maxSpeed".to_string());
        }
        if let Some(host) = &config.host {
            sumatra_args.push(format!("--host={host}"));
        }

        let child = Command::new(&launcher)
            .args(sumatra_args)
            .current_dir(&config.repo_root)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()
            .with_context(|| {
                format!(
                    "failed to start Sumatra from {}",
                    config.repo_root.display()
                )
            })?;
        Ok(Self { child })
    }

    pub fn try_wait(&mut self) -> Result<Option<std::process::ExitStatus>> {
        Ok(self.child.try_wait()?)
    }

    pub fn kill(&mut self) -> Result<()> {
        if self.child.try_wait()?.is_none() {
            self.child
                .kill()
                .context("failed to kill Sumatra process")?;
        }
        Ok(())
    }
}

impl Drop for SumatraInstance {
    fn drop(&mut self) {
        let _ = self.kill();
    }
}

pub fn default_sumatra_repo() -> PathBuf {
    env::var_os("SIMHARK_SUMATRA_REPO_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../../Sumatra")
                .canonicalize()
                .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../Sumatra"))
        })
}

fn ensure_sumatra_repo(path: &Path) -> Result<()> {
    if !path.exists() {
        bail!(
            "Sumatra checkout not found at {}. Please clone it manually there.",
            path.display()
        );
    }
    if !path.join("gradlew").exists() {
        bail!("{} does not look like a Sumatra checkout", path.display());
    }
    Ok(())
}

fn sumatra_launcher(repo_root: &Path) -> PathBuf {
    let name = if cfg!(windows) {
        "sumatra.bat"
    } else {
        "sumatra"
    };
    repo_root.join("build/install/sumatra/bin").join(name)
}
