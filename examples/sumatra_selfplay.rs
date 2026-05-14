use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use referris::domain::{TrackedBall, TrackedRobot};
use referris::{
    AutoRef, Command, FieldGeometry, InputEnvelope, RawDetectionFrame, RefereeSnapshot, Team,
    TeamInfo, TrackedFrame,
};
#[cfg(feature = "motion-audit")]
use simhark::motion_audit::{MotionAuditor, robot_motion_summary};
use simhark::viewer::{ViewerConfig, ViewerServer};
use simhark::{
    RobotState, SimulationEngine, SumatraSimNetConfig, SumatraSimNetServer, TeamColor, WorldConfig,
    WorldState,
};
use simhark_sumatra::{SumatraInstance, SumatraLaunchConfig};

const MOTION_LOG_EVERY_FRAMES: u64 = 15;

fn main() -> Result<()> {
    let mut engine = SimulationEngine::new(1, WorldConfig::division_b());
    let mut autoref = AutoRef::default();
    let viewer_config = ViewerConfig::default();
    let viewer = ViewerServer::bind(viewer_config, 1, &engine.world(0).config)?;
    println!("viewer: {}", viewer_config.http_url());

    let mut server = SumatraSimNetServer::bind(SumatraSimNetConfig::default())?;
    let mut yellow = SumatraInstance::spawn(&SumatraLaunchConfig {
        remote_client: true,
        ai_blue: false,
        ai_yellow: true,
        host: Some("127.0.0.1".to_string()),
        ..SumatraLaunchConfig::default()
    })?;
    let mut blue = SumatraInstance::spawn(&SumatraLaunchConfig {
        remote_client: true,
        ai_blue: true,
        ai_yellow: false,
        host: Some("127.0.0.1".to_string()),
        ..SumatraLaunchConfig::default()
    })?;

    let start = Instant::now();
    let mut previous_state = None;
    #[cfg(feature = "motion-audit")]
    let mut motion_auditor = MotionAuditor::default();
    while start.elapsed() < Duration::from_secs(20) {
        server.step(&mut engine)?;
        let state = engine.world(0).get_state();
        let world = engine.world(0);
        let input = referris_input(&state, &world.config);
        let step = autoref.step(&input);
        for event in step.events {
            println!("referris: {event:?}");
        }
        viewer.publish(&state);
        log_motion(
            &state,
            previous_state.as_ref(),
            &world.config,
            #[cfg(feature = "motion-audit")]
            &mut motion_auditor,
        );
        previous_state = Some(state.clone());
        if yellow.try_wait()?.is_some() || blue.try_wait()?.is_some() {
            break;
        }
        thread::sleep(Duration::from_millis(16));
    }

    Ok(())
}

fn referris_input(state: &simhark::WorldState, config: &WorldConfig) -> InputEnvelope {
    InputEnvelope {
        geometry: Some(FieldGeometry {
            field_length: config.field.field_length,
            field_width: config.field.field_width,
            goal_width: config.field.goal_width,
            goal_depth: config.field.goal_depth,
            boundary_width: config.field.margin_touch_line,
            boundary_width_goal_line: config.field.margin_goal_line,
            defense_area_depth: config.field.penalty_depth,
            defense_area_width: config.field.penalty_width,
            center_circle_radius: config.field.field_center_radius,
            line_thickness: config.field.field_line_width,
            goal_height: config.field.goal_height,
            ball_radius: config.ball.radius,
            max_robot_radius: config.blue_robots.radius,
        }),
        referee: Some(RefereeSnapshot {
            timestamp: state.sim_time,
            command: Command::ForceStart,
            command_counter: state.frame as u32,
            blue_on_positive_half: Some(false),
            next_command: None,
            current_action_time_remaining: None,
            designated_position: None,
            yellow: TeamInfo {
                name: "Yellow".into(),
                goalkeeper: Some(0),
                max_allowed_bots: Some(state.yellow_robots.len() as u32),
            },
            blue: TeamInfo {
                name: "Blue".into(),
                goalkeeper: Some(0),
                max_allowed_bots: Some(state.blue_robots.len() as u32),
            },
        }),
        detections: Vec::<RawDetectionFrame>::new(),
        tracked: Some(TrackedFrame {
            frame_number: state.frame as u32,
            timestamp: state.sim_time,
            ball: Some(TrackedBall {
                pos: referris::math::Vec3 {
                    x: state.ball.x,
                    y: state.ball.y,
                    z: state.ball.z,
                },
                vel: referris::math::Vec3 {
                    x: state.ball.vx,
                    y: state.ball.vy,
                    z: state.ball.vz,
                },
                visible: true,
            }),
            robots: state
                .blue_robots
                .iter()
                .map(|robot| tracked_robot(robot, Team::Blue))
                .chain(
                    state
                        .yellow_robots
                        .iter()
                        .map(|robot| tracked_robot(robot, Team::Yellow)),
                )
                .collect(),
            kicked_ball: None,
        }),
    }
}

fn tracked_robot(robot: &simhark::RobotState, team: Team) -> TrackedRobot {
    let _ = match robot.team {
        TeamColor::Blue => Team::Blue,
        TeamColor::Yellow => Team::Yellow,
    };
    TrackedRobot {
        id: robot.id as u32,
        team,
        pos: referris::math::Vec2 {
            x: robot.x,
            y: robot.y,
        },
        orientation: robot.orientation,
        vel: referris::math::Vec2 {
            x: robot.vx,
            y: robot.vy,
        },
        angular_velocity: robot.v_angular,
        visible: robot.is_on,
    }
}

fn log_motion(
    state: &WorldState,
    previous_state: Option<&WorldState>,
    config: &WorldConfig,
    #[cfg(feature = "motion-audit")] motion_auditor: &mut MotionAuditor,
) {
    if state.frame % MOTION_LOG_EVERY_FRAMES != 0 {
        return;
    }

    let ball_speed = speed3(state.ball.vx, state.ball.vy, state.ball.vz);
    let (step_speed, displacement, suspicious_jump) = previous_state
        .map(|previous| {
            let dt = (state.sim_time - previous.sim_time).max(f64::EPSILON);
            let dx = state.ball.x - previous.ball.x;
            let dy = state.ball.y - previous.ball.y;
            let dz = state.ball.z - previous.ball.z;
            let displacement = (dx * dx + dy * dy + dz * dz).sqrt();
            let step_speed = displacement / dt;
            let suspicious_jump = displacement > 0.03 && step_speed > ball_speed + 1.0;
            (step_speed, displacement, suspicious_jump)
        })
        .unwrap_or((0.0, 0.0, false));

    let nearest = nearest_robots(state, 2);
    let nearest_summary = nearest
        .iter()
        .map(|(robot, distance, forward, lateral)| {
            format!(
                "{}{} d={distance:.3} rel=({forward:.3},{lateral:.3}) v={:.2} av={:.2} ir={}",
                team_label(robot.team),
                robot.id,
                speed2(robot.vx, robot.vy),
                robot.v_angular,
                robot.infrared,
            )
        })
        .collect::<Vec<_>>()
        .join(" | ");

    println!(
        "motion t={:.2} frame={} ball pos=({:.3},{:.3},{:.3}) vel=({:.2},{:.2},{:.2}) speed={:.2} step={:.2} disp={:.3}{}{} nearest=[{}]",
        state.sim_time,
        state.frame,
        state.ball.x,
        state.ball.y,
        state.ball.z,
        state.ball.vx,
        state.ball.vy,
        state.ball.vz,
        ball_speed,
        step_speed,
        displacement,
        if suspicious_jump {
            " suspicious-jump"
        } else {
            ""
        },
        wall_summary(state, config),
        nearest_summary,
    );

    #[cfg(feature = "motion-audit")]
    {
        let findings = motion_auditor.audit(state, previous_state, config);
        if !findings.is_empty() {
            let fastest = state
                .blue_robots
                .iter()
                .chain(state.yellow_robots.iter())
                .max_by(|left, right| {
                    speed2(left.vx, left.vy).total_cmp(&speed2(right.vx, right.vy))
                });
            if let Some(robot) = fastest {
                println!("motion-audit fastest={}", robot_motion_summary(robot));
            }
            for finding in findings {
                println!("motion-audit {} {}", finding.kind, finding.detail);
            }
        }
    }
}

fn nearest_robots(state: &WorldState, count: usize) -> Vec<(&RobotState, f64, f64, f64)> {
    let mut robots = state
        .blue_robots
        .iter()
        .chain(state.yellow_robots.iter())
        .map(|robot| {
            let dx = state.ball.x - robot.x;
            let dy = state.ball.y - robot.y;
            let distance = (dx * dx + dy * dy).sqrt();
            let forward = dx * robot.orientation.cos() + dy * robot.orientation.sin();
            let lateral = -dx * robot.orientation.sin() + dy * robot.orientation.cos();
            (robot, distance, forward, lateral)
        })
        .collect::<Vec<_>>();

    robots.sort_by(|left, right| left.1.total_cmp(&right.1));
    robots.truncate(count);
    robots
}

fn team_label(team: TeamColor) -> &'static str {
    match team {
        TeamColor::Blue => "B",
        TeamColor::Yellow => "Y",
    }
}

fn speed2(x: f64, y: f64) -> f64 {
    (x * x + y * y).sqrt()
}

fn speed3(x: f64, y: f64, z: f64) -> f64 {
    (x * x + y * y + z * z).sqrt()
}

fn wall_summary(state: &WorldState, config: &WorldConfig) -> String {
    let touchline =
        config.field.field_width * 0.5 + config.field.margin_touch_line - config.ball.radius;
    let goal_line =
        config.field.field_length * 0.5 + config.field.margin_goal_line - config.ball.radius;
    if state.ball.y.abs() >= touchline - 0.03 {
        return format!(" wall=touchline tangential={:.2}", state.ball.vx);
    }
    if state.ball.x.abs() >= goal_line - 0.03 {
        return format!(" wall=goalline tangential={:.2}", state.ball.vy);
    }
    String::new()
}
