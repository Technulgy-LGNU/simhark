//! CLI entry point for the RoboCup SSL Simulator.
//!
//! Supports headless mode for training and a debug viewer mode.

use robocup_sim::{
    domain_randomization::RandomizationConfig, GrSimCompatConfig, GrSimCompatServer, MoveCommand,
    RobotCommand, SimulationEngine, WorldCommand, WorldConfig,
};
use std::time::Instant;

fn main() {
    env_logger::init();

    let args: Vec<String> = std::env::args().collect();
    let num_worlds: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(64);
    let num_steps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1000);
    let randomize = args.iter().any(|a| a == "--randomize");
    let grsim_api = args.iter().any(|a| a == "--grsim-api");

    println!("RoboCup SSL Simulator (Rust)");
    println!("  Worlds: {num_worlds}");
    println!("  Steps:  {num_steps}");
    println!("  Domain randomization: {randomize}");
    println!();

    let config = WorldConfig::division_a();

    let start = Instant::now();
    let mut engine = if randomize {
        SimulationEngine::new_randomized(num_worlds, config, RandomizationConfig::moderate())
    } else {
        SimulationEngine::new(num_worlds, config)
    };
    let init_time = start.elapsed();
    println!("Initialization: {init_time:.2?}");

    if grsim_api {
        let mut server = GrSimCompatServer::bind(GrSimCompatConfig::default())
            .expect("failed to bind grSim compatibility sockets");
        println!("grSim compatibility API enabled");
        println!("  Legacy command port: 20011");
        println!("  Simulation control: 10300");
        println!("  Blue robot control:  10301");
        println!("  Yellow robot control: 10302");
        println!("  Vision multicast: 224.5.23.2:10020");
        println!();

        loop {
            server
                .step(&mut engine)
                .expect("grSim compatibility server step failed");
        }
    }

    // Create a simple command: all blue robots drive forward
    let command = WorldCommand {
        blue: (0..11)
            .map(|id| RobotCommand {
                id,
                move_command: Some(MoveCommand::LocalVelocity {
                    forward: 1.0,
                    left: 0.0,
                    angular: 0.0,
                }),
                kick_speed: 0.0,
                kick_angle: 0.0,
                dribbler_on: false,
            })
            .collect(),
        yellow: vec![],
        teleport_ball: None,
        teleport_robots: vec![],
    };

    let start = Instant::now();
    for step in 0..num_steps {
        let states = engine.step_all_same(&command);
        if step == 0 || step == num_steps - 1 {
            let s = &states[0];
            println!(
                "Step {step:>5}: ball=({:.3}, {:.3}, {:.3}) blue[0]=({:.3}, {:.3}) frame={}",
                s.ball.x, s.ball.y, s.ball.z, s.blue_robots[0].x, s.blue_robots[0].y, s.frame,
            );
        }
    }
    let sim_time = start.elapsed();

    let total_steps = num_worlds as u64 * num_steps as u64;
    let steps_per_sec = total_steps as f64 / sim_time.as_secs_f64();
    println!();
    println!("Simulation: {sim_time:.2?}");
    println!("Total world-steps: {total_steps}");
    println!("Throughput: {steps_per_sec:.0} world-steps/sec");
    println!(
        "Real-time factor per world: {:.1}x (at 60 FPS)",
        steps_per_sec / num_worlds as f64 / 60.0
    );
}
