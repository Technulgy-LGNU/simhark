use std::time::{SystemTime, UNIX_EPOCH};

use core_dump::proto::{CpBall, CpCommand, CpRobot, CpTrackedRobot, CpVector2};
use crashpilot::core_dump;
use crashpilot::core_dump::proto::{
    self as cp_proto, RobotCp, SslDetectionBall, SslDetectionFrame, SslDetectionRobot,
    SslGeometryData, SslGeometryFieldSize, SslWrapperPacket, Team, TrackedBall, TrackedFrame,
    TrackedRobot, TrackerWrapperPacket, Vector2, Vector3,
};
use simhark::state::{KickStatus, RobotState};
use simhark::{WorldConfig, WorldState};
use tf_jetsoncode::TeensyRecMSG;

pub fn world_state_to_cp_events(events: &mut ::crashpilot::Events, state: &WorldState) {
    events.raw = world_state_to_ssl_wrapper(state);
    events.tracked = world_state_to_tracker_wrapper(state);
}

pub fn world_state_to_ssl_wrapper(state: &WorldState) -> Option<SslWrapperPacket> {
    let now = unix_time_seconds();

    let detection = SslDetectionFrame {
        frame_number: state.frame as u32,
        t_capture: now,
        t_sent: now,
        t_capture_camera: None,
        camera_id: 0,
        balls: vec![SslDetectionBall {
            confidence: 1.0,
            area: None,
            x: (state.ball.x * 1000.0) as f32,
            y: (state.ball.y * 1000.0) as f32,
            z: Some((state.ball.z * 1000.0) as f32),
            pixel_x: 0.0,
            pixel_y: 0.0,
        }],
        robots_yellow: detection_robots(&state.yellow_robots),
        robots_blue: detection_robots(&state.blue_robots),
    };

    Some(SslWrapperPacket {
        detection: Some(detection),
        geometry: Some(geometry_from_config(&WorldConfig::division_b())),
        source: None,
    })
}

fn geometry_from_config(config: &WorldConfig) -> SslGeometryData {
    let field = &config.field;
    let ball = &config.ball;
    let robot = &config.yellow_robots;

    SslGeometryData {
        field: SslGeometryFieldSize {
            field_length: meters_to_mm(field.field_length),
            field_width: meters_to_mm(field.field_width),
            goal_width: meters_to_mm(field.goal_width),
            goal_depth: meters_to_mm(field.goal_depth),
            boundary_width: meters_to_mm(field.margin_touch_line),
            boundary_width_goal_line: Some(meters_to_mm(field.margin_goal_line)),
            field_lines: Vec::new(),
            field_arcs: Vec::new(),
            penalty_area_depth: Some(meters_to_mm(field.penalty_depth)),
            penalty_area_width: Some(meters_to_mm(field.penalty_width)),
            center_circle_radius: Some(meters_to_mm(field.field_center_radius)),
            line_thickness: Some(meters_to_mm(field.field_line_width)),
            goal_center_to_penalty_mark: Some(meters_to_mm(field.penalty_point)),
            goal_height: Some(meters_to_mm(field.goal_height)),
            ball_radius: Some((ball.radius * 1000.0) as f32),
            max_robot_radius: Some((robot.radius * 1000.0) as f32),
            goal_substitution_area_width: Some(meters_to_mm(field.goal_substitution_area_width)),
        },
        calib: Vec::new(),
        models: None,
    }
}

fn meters_to_mm(value: f64) -> i32 {
    (value * 1000.0).round() as i32
}

pub fn world_state_to_tracker_wrapper(state: &WorldState) -> Option<TrackerWrapperPacket> {
    let mut robots = Vec::with_capacity(state.yellow_robots.len() + state.blue_robots.len());
    tracked_robots(&mut robots, &state.yellow_robots, Team::Yellow);
    tracked_robots(&mut robots, &state.blue_robots, Team::Blue);

    let frame = TrackedFrame {
        frame_number: state.frame as u32,
        timestamp: unix_time_seconds(),
        balls: vec![TrackedBall {
            pos: Vector3 {
                x: state.ball.x as f32,
                y: state.ball.y as f32,
                z: state.ball.z as f32,
            },
            vel: Some(Vector3 {
                x: state.ball.vx as f32,
                y: state.ball.vy as f32,
                z: state.ball.vz as f32,
            }),
            visibility: Some(1.0),
        }],
        robots,
        kicked_ball: None,
        capabilities: Vec::new(),
    };

    Some(TrackerWrapperPacket {
        uuid: String::from("simhark"),
        source_name: None,
        tracked_frame: Some(frame),
    })
}

fn detection_robots(robots: &[RobotState]) -> Vec<SslDetectionRobot> {
    robots
        .iter()
        .filter(|robot| robot.is_on)
        .map(|robot| SslDetectionRobot {
            confidence: 1.0,
            robot_id: Some(robot.id as u32),
            x: (robot.x * 1000.0) as f32,
            y: (robot.y * 1000.0) as f32,
            orientation: Some(robot.orientation as f32),
            pixel_x: 0.0,
            pixel_y: 0.0,
            height: Some((robot.z * 1000.0) as f32),
        })
        .collect()
}

fn tracked_robots(out: &mut Vec<TrackedRobot>, robots: &[RobotState], team: Team) {
    out.extend(
        robots
            .iter()
            .filter(|robot| robot.is_on)
            .map(|robot| TrackedRobot {
                robot_id: cp_proto::RobotId {
                    id: Some(robot.id as u32),
                    team: Some(team as i32),
                },
                pos: Vector2 {
                    x: robot.x as f32,
                    y: robot.y as f32,
                },
                orientation: robot.orientation as f32,
                vel: Some(Vector2 {
                    x: robot.vx as f32,
                    y: robot.vy as f32,
                }),
                vel_angular: Some(robot.v_angular as f32),
                visibility: Some(1.0),
            }),
    );
}

pub fn robot_events(
    robot: u32,
    cp_data: crashpilot::RobotData,
    state: &WorldState,
) -> tf_jetsoncode::Events {
    let Some(robot) = state.yellow_robots.iter().find(|r| r.id == robot as usize) else {
        panic!("Robot with id {} not found in world state", robot);
    };

    let mut flags = 0;

    // Bitflags:
    // Bit 0: Error
    // Bit 1: Has Ball
    // Bit 2: Kick Ready
    // Bit 3: Chip Ready

    if robot.infrared {
        flags = set_bit(flags, 1);
    }

    if robot.kick_status == KickStatus::NoKick {
        flags = set_bit(flags, 2);
        flags = set_bit(flags, 3);
    }

    tf_jetsoncode::Events {
        cp: Some(cp_robot_msg(cp_data.msg)),
        vis: None,
        teensy: Some(TeensyRecMSG {
            flags,
            batt_level: 0,
            current: 0,
        }),
    }
}

fn cp_robot_msg(msg: core_dump::proto::CpRobot) -> CpRobot {
    CpRobot {
        robot_id: msg.robot_id,
        timestamp: msg.timestamp,
        packet_id: msg.packet_id,
        ball: cp_ball(msg.ball),
        robots_yellow: msg
            .robots_yellow
            .into_iter()
            .map(cp_tracked_robot)
            .collect(),
        robots_blue: msg.robots_blue.into_iter().map(cp_tracked_robot).collect(),
        cmd: cp_command(msg.cmd),
        infos: Default::default(),
    }
}

#[inline]
fn cp_vec2(v: core_dump::proto::CpVector2) -> CpVector2 {
    CpVector2 { x: v.x, y: v.y }
}

#[inline]
fn cp_ball(b: core_dump::proto::CpBall) -> CpBall {
    CpBall {
        pos: cp_vec2(b.pos),
        vel: b.vel.map(cp_vec2),
    }
}

#[inline]
fn cp_tracked_robot(r: core_dump::proto::CpTrackedRobot) -> CpTrackedRobot {
    CpTrackedRobot {
        robot_id: r.robot_id,
        pos: cp_vec2(r.pos),
        orientation: r.orientation,
        vel: r.vel.map(cp_vec2),
        visibility: r.visibility,
    }
}

#[inline]
fn cp_command(c: core_dump::proto::CpCommand) -> CpCommand {
    CpCommand {
        state: c.state,
        task: c.task,
        pos: c.pos.map(cp_vec2),
        speed: c.speed,
        orientation: c.orientation,
        kick_orient: c.kick_orient,
        kick_speed: c.kick_speed,
        enemy_id: c.enemy_id,
    }
}

pub fn robot_cp(cp: RobotCp) -> RobotCp {
    core_dump::proto::RobotCp {
        robot_id: cp.robot_id,
        battery_voltage: cp.battery_voltage,
        current: cp.current,
        kicker_ready: cp.kicker_ready,
        has_ball: cp.has_ball,
        has_error: cp.has_error,
        acting: cp.acting,
        last_rec_packet: cp.last_rec_packet,
        timestamp: 0.0,
    }
}

fn set_bit(flags: u32, bit: u8) -> u32 {
    flags | (1 << bit)
}

fn unix_time_seconds() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
