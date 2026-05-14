use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use simhark::{
    SimulationEngine, SumatraSimNetConfig, SumatraSimNetServer, TeamColor, TeleportBall,
    TeleportRobot, WorldCommand, WorldConfig,
};
use simhark_sumatra::{SumatraInstance, SumatraLaunchConfig};

fn main() -> Result<()> {
    let config = WorldConfig::division_b();
    let robots_per_team = config.robots_per_team;
    let mut engine = SimulationEngine::new(1, config);
    let mut server = SumatraSimNetServer::bind(SumatraSimNetConfig::default())?;

    let mut blue = SumatraInstance::spawn(&SumatraLaunchConfig {
        remote_client: true,
        ai_blue: true,
        ai_yellow: false,
        host: Some("127.0.0.1".to_string()),
        ..SumatraLaunchConfig::default()
    })?;

    let start = Instant::now();
    let mut goals = 0usize;
    while start.elapsed() < Duration::from_secs(45) && goals < 5 {
        let command = WorldCommand {
            teleport_robots: (0..robots_per_team)
                .map(|id| TeleportRobot {
                    id,
                    team: TeamColor::Yellow,
                    x: Some(100.0 + id as f64),
                    y: Some(100.0 + id as f64),
                    orientation: Some(0.0),
                    vx: Some(0.0),
                    vy: Some(0.0),
                    v_angular: Some(0.0),
                    present: Some(false),
                })
                .collect(),
            ..Default::default()
        };

        let state = server.step_with_local_commands(&mut engine, &[command])?.remove(0);
        let blue_near_ball = state
            .blue_robots
            .iter()
            .map(|robot| {
                let dx = robot.x - state.ball.x;
                let dy = robot.y - state.ball.y;
                let d = (dx * dx + dy * dy).sqrt();
                (robot, d)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));

        if state.goal_blue {
            goals += 1;
            println!("goal {} at t={:.2}", goals, state.sim_time);
            let reset = WorldCommand {
                teleport_ball: Some(TeleportBall {
                    x: Some(0.0),
                    y: Some(0.0),
                    z: Some(0.0),
                    vx: Some(0.0),
                    vy: Some(0.0),
                    vz: Some(0.0),
                }),
                ..Default::default()
            };
            let reset_state = engine.step_with_commands(&[reset]).remove(0);
            println!(
                "after-reset t={:.2} ball=({:.2},{:.2}) v=({:.2},{:.2}) goal_blue={} goal_yellow={}",
                reset_state.sim_time,
                reset_state.ball.x,
                reset_state.ball.y,
                reset_state.ball.vx,
                reset_state.ball.vy,
                reset_state.goal_blue,
                reset_state.goal_yellow,
            );
        }

        if state.frame % 30 == 0 {
            if let Some((robot, distance)) = blue_near_ball {
                println!(
                    "t={:.2} ball=({:.2},{:.2}) v=({:.2},{:.2}) nearest=B{} d={:.2} pos=({:.2},{:.2}) v={:.2} av={:.2} ir={} dribbler={} kick={:?} on={} goals={}",
                    state.sim_time,
                    state.ball.x,
                    state.ball.y,
                    state.ball.vx,
                    state.ball.vy,
                    robot.id,
                    distance,
                    robot.x,
                    robot.y,
                    (robot.vx * robot.vx + robot.vy * robot.vy).sqrt(),
                    robot.v_angular,
                    robot.infrared,
                    robot.dribbler_on,
                    robot.kick_status,
                    robot.is_on,
                    goals,
                );
            }
        }

        if blue.try_wait()?.is_some() {
            break;
        }

        thread::sleep(Duration::from_millis(16));
    }

    blue.kill()?;
    println!("finished with {goals} goals");
    Ok(())
}
