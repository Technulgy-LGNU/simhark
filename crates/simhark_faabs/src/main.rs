use std::thread;
use simhark::{SimulationEngine, WorldCommand, WorldConfig};
use simhark_faabs::Faabs;

fn main() {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            tokio::task::spawn_blocking(run).await.unwrap();
        });
}

fn run() {
    let config = WorldConfig::division_b();

    let mut engine = SimulationEngine::new(1, config.clone());

    let mut faabs = Faabs::with_interface(6);

    let command = WorldCommand::default();

    let mut state = engine.step_with_commands(&[command]).remove(0);

    loop {
        let mut command = WorldCommand::default();

        faabs.step(&state, &mut command, None);

        // dbg!(&command);

        state = engine.step_with_commands(&[command]).remove(0);

        thread::sleep(std::time::Duration::from_millis(2));
    }
}
