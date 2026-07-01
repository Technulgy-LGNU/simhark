use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use simhark::{SimulationEngine, WorldConfig};
use simhark_sumatra::{
  SumatraInstance, SumatraLaunchConfig, SumatraSimNetConfig, SumatraSimNetServer,
};

const WORLD_COUNT: usize = 2;
const BASE_PORT: u16 = 14242;

fn main() -> Result<()> {
  let runtime = Duration::from_secs(20);
  let base_config = WorldConfig::division_a();
  let mut engine = SimulationEngine::new(WORLD_COUNT, base_config);

  let mut servers = [
    SumatraSimNetServer::bind_for_world(
      SumatraSimNetConfig {
        bind_addr: format!("127.0.0.1:{}", BASE_PORT)
          .parse()
          .expect("valid port"),
      },
      0,
    )?,
    SumatraSimNetServer::bind_for_world(
      SumatraSimNetConfig {
        bind_addr: format!("127.0.0.1:{}", BASE_PORT + 1)
          .parse()
          .expect("valid port"),
      },
      1,
    )?,
  ];

  let mut clients = [
    SumatraInstance::spawn(&SumatraLaunchConfig {
      remote_client: true,
      host: Some("127.0.0.1".to_string()),
      sim_net_port: Some(BASE_PORT),
      ..SumatraLaunchConfig::default()
    })?,
    SumatraInstance::spawn(&SumatraLaunchConfig {
      remote_client: true,
      host: Some("127.0.0.1".to_string()),
      sim_net_port: Some(BASE_PORT + 1),
      ..SumatraLaunchConfig::default()
    })?,
  ];

  println!(
    "running {} Sumatra self-play worlds on ports {} and {}",
    WORLD_COUNT,
    BASE_PORT,
    BASE_PORT + 1
  );

  let start = Instant::now();
  while start.elapsed() < runtime {
    servers[0].step(&mut engine)?;
    servers[1].step(&mut engine)?;

    for (world_index, client) in clients.iter_mut().enumerate() {
      if let Some(status) = client.try_wait()? {
        anyhow::bail!(
          "Sumatra client for world {} exited early with status {}",
          world_index,
          status
        );
      }
    }

    thread::sleep(Duration::from_millis(16));
  }

  for client in &mut clients {
    client.kill()?;
  }

  Ok(())
}
