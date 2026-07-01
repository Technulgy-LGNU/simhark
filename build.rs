use std::io::Result;
use std::path::Path;
use std::process::Command;

fn main() -> Result<()> {
  build_protos()?;
  build_frontend();
  Ok(())
}

fn build_protos() -> Result<()> {
  let proto_files = [
    "proto/grSim_Commands.proto",
    "proto/grSim_Packet.proto",
    "proto/grSim_Replacement.proto",
    "proto/grSim_Robotstatus.proto",
    "proto/ssl_gc_common.proto",
    "proto/ssl_simulation_config.proto",
    "proto/ssl_simulation_control.proto",
    "proto/ssl_simulation_error.proto",
    "proto/ssl_simulation_robot_control.proto",
    "proto/ssl_simulation_robot_feedback.proto",
    "proto/ssl_vision_detection.proto",
    "proto/ssl_vision_geometry.proto",
    "proto/ssl_vision_wrapper.proto",
  ];

  let mut config = prost_build::Config::new();
  config.extern_path(".google.protobuf.Any", "::prost_types::Any");
  config.compile_protos(&proto_files, &["proto"])?;

  println!("cargo:rerun-if-changed=proto");
  Ok(())
}

fn build_frontend() {
  println!("cargo:rerun-if-env-changed=SIMHARK_SKIP_FRONTEND");
  if std::env::var_os("SIMHARK_SKIP_FRONTEND").is_some() {
    return;
  }

  let frontend_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("frontend");
  if !frontend_dir.exists() {
    return;
  }

  // Track source files so cargo only re-runs the build when something
  // relevant changes.
  for entry in &[
    "frontend/package.json",
    "frontend/package-lock.json",
    "frontend/vite.config.ts",
    "frontend/tsconfig.json",
    "frontend/tsconfig.app.json",
    "frontend/index.html",
    "frontend/src",
  ] {
    println!("cargo:rerun-if-changed={entry}");
  }

  let node_modules = frontend_dir.join("node_modules");
  if !node_modules.exists() {
    let lockfile = frontend_dir.join("package-lock.json");
    let status = if lockfile.exists() {
      Command::new("npm")
        .arg("ci")
        .current_dir(&frontend_dir)
        .status()
    } else {
      Command::new("npm")
        .arg("install")
        .current_dir(&frontend_dir)
        .status()
    };
    match status {
      Ok(status) if status.success() => {}
      Ok(status) => panic!("npm install/ci failed with status {status}"),
      Err(err) => panic!(
        "failed to run npm in {}: {err}. Install Node.js or set SIMHARK_SKIP_FRONTEND=1.",
        frontend_dir.display()
      ),
    }
  }

  let build_status = Command::new("npm")
    .args(["run", "build"])
    .current_dir(&frontend_dir)
    .status();
  match build_status {
    Ok(status) if status.success() => {}
    Ok(status) => panic!("`npm run build` failed with status {status}"),
    Err(err) => panic!("failed to run `npm run build`: {err}"),
  }

  let dist_index = frontend_dir.join("dist").join("index.html");
  if !dist_index.exists() {
    panic!(
      "frontend build did not produce {} — check vite-plugin-singlefile output",
      dist_index.display()
    );
  }
}
