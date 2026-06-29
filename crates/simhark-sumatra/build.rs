use std::env;
use std::io::Result;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

fn main() -> Result<()> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=SIMHARK_SUMATRA_REPO_ROOT");

    build_protos()?;
    build_sumatra();
    Ok(())
}

fn build_protos() -> Result<()> {
    let proto_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("proto");
    let proto_files = [
        "SimBotAction.proto",
        "SimCommon.proto",
        "SimReferee.proto",
        "SimRegister.proto",
        "SimRequest.proto",
        "SimResponse.proto",
        "SimState.proto",
    ]
    .map(|file| proto_dir.join(file));

    let mut config = prost_build::Config::new();
    config.extern_path(".google.protobuf.Any", "::prost_types::Any");
    config.compile_protos(&proto_files, &[proto_dir])?;

    println!("cargo:rerun-if-changed=proto/SimBotAction.proto");
    println!("cargo:rerun-if-changed=proto/SimCommon.proto");
    println!("cargo:rerun-if-changed=proto/SimReferee.proto");
    println!("cargo:rerun-if-changed=proto/SimRegister.proto");
    println!("cargo:rerun-if-changed=proto/SimRequest.proto");
    println!("cargo:rerun-if-changed=proto/SimResponse.proto");
    println!("cargo:rerun-if-changed=proto/SimState.proto");
    Ok(())
}

fn build_sumatra() {
    let repo_root = default_sumatra_repo();
    println!(
        "cargo:rerun-if-changed={}",
        repo_root.join("build.gradle").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        repo_root.join("settings.gradle").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        repo_root.join("gradle.properties").display()
    );
    println!("cargo:rerun-if-changed={}", repo_root.join("src").display());
    println!(
        "cargo:rerun-if-changed={}",
        repo_root.join("modules").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        repo_root.join("config").display()
    );

    ensure_sumatra_repo(&repo_root);
    let gradlew = gradle_launcher(&repo_root);
    let status = Command::new(&gradlew)
        .arg("installDist")
        .current_dir(&repo_root)
        .stdin(Stdio::null())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .unwrap_or_else(|error| panic!("failed to start {}: {error}", gradlew.display()));

    if !status.success() {
        panic!(
            "Sumatra build failed in {} with status {status}",
            repo_root.display()
        );
    }
}

fn default_sumatra_repo() -> PathBuf {
    env::var_os("SIMHARK_SUMATRA_REPO_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../../Sumatra")
                .canonicalize()
                .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../Sumatra"))
        })
}

fn ensure_sumatra_repo(path: &Path) {
    if !path.exists() {
        panic!(
            "Sumatra checkout not found at {}. Please clone it manually there or set SIMHARK_SUMATRA_REPO_ROOT.",
            path.display()
        );
    }
    if !gradle_launcher(path).exists() {
        panic!("{} does not look like a Sumatra checkout", path.display());
    }
}

fn gradle_launcher(path: &Path) -> PathBuf {
    let name = if cfg!(windows) {
        "gradlew.bat"
    } else {
        "gradlew"
    };
    path.join(name)
}
