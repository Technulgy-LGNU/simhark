use std::collections::{HashMap, HashSet};
use std::io::{ErrorKind, Read, Result, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use prost::Message;

use crate::command::{MoveCommand, RobotCommand, WorldCommand};
use crate::engine::SimulationEngine;
use crate::state::{TeamColor, WorldState};

use crate::proto::sumatra_sim_net::DriveMode;
use crate::proto::sumatra_sim_net::{
    BotId, SimBallState, SimBotAction, SimBotState, SimReferee, SimRegister, SimRequest,
    SimResponse, Vector2, Vector3, sim_referee,
};

#[derive(Debug, Clone)]
pub struct SumatraSimNetConfig {
    pub bind_addr: SocketAddr,
}

impl Default for SumatraSimNetConfig {
    fn default() -> Self {
        Self {
            bind_addr: "127.0.0.1:14242".parse().expect("valid default bind addr"),
        }
    }
}

pub struct SumatraSimNetServer {
    listener: TcpListener,
    clients: Vec<SumatraClient>,
}

struct SumatraClient {
    stream: TcpStream,
    teams: HashSet<TeamColor>,
    read_buf: Vec<u8>,
}

impl SumatraSimNetServer {
    pub fn bind(config: SumatraSimNetConfig) -> Result<Self> {
        let listener = TcpListener::bind(config.bind_addr)?;
        listener.set_nonblocking(true)?;
        Ok(Self {
            listener,
            clients: Vec::new(),
        })
    }

    pub fn step(&mut self, engine: &mut SimulationEngine) -> Result<Vec<WorldState>> {
        self.accept_new_clients()?;
        let states = engine.get_all_states();
        if let Some(state) = states.first() {
            self.publish_state(state)?;
            let actions = self.receive_actions(state)?;
            let commands = vec![merge_actions_into_world_command(actions); engine.count()];
            return Ok(engine.step_with_commands(&commands));
        }
        let commands = vec![WorldCommand::default(); engine.count()];
        Ok(engine.step_with_commands(&commands))
    }

    pub fn step_with_local_commands(
        &mut self,
        engine: &mut SimulationEngine,
        local_commands: &[WorldCommand],
    ) -> Result<Vec<WorldState>> {
        self.accept_new_clients()?;
        let states = engine.get_all_states();
        if let Some(state) = states.first() {
            self.publish_state(state)?;
            let actions = self.receive_actions(state)?;
            let mut commands = local_commands.to_vec();
            if let Some(command) = commands.first_mut() {
                merge_actions_into_existing_world_command(command, actions);
            }
            return Ok(engine.step_with_commands(&commands));
        }
        let commands = local_commands.to_vec();
        Ok(engine.step_with_commands(&commands))
    }

    pub fn has_clients(&mut self) -> Result<bool> {
        self.accept_new_clients()?;
        Ok(!self.clients.is_empty())
    }

    fn accept_new_clients(&mut self) -> Result<()> {
        loop {
            match self.listener.accept() {
                Ok((mut stream, _)) => {
                    stream.set_nodelay(true)?;
                    stream.set_nonblocking(false)?;
                    let register = read_length_delimited::<SimRegister>(&mut stream)?;
                    let teams = register
                        .team_color
                        .into_iter()
                        .filter_map(proto_team_to_local)
                        .collect::<HashSet<_>>();
                    stream.set_nonblocking(true)?;
                    self.clients.push(SumatraClient {
                        stream,
                        teams,
                        read_buf: Vec::new(),
                    });
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(err) => return Err(err),
            }
        }
    }

    fn publish_state(&mut self, state: &WorldState) -> Result<()> {
        let request = build_request(state);
        self.clients
            .retain_mut(|client| write_length_delimited(&mut client.stream, &request).is_ok());
        Ok(())
    }

    fn receive_actions(
        &mut self,
        state: &WorldState,
    ) -> Result<HashMap<(TeamColor, usize), RobotCommand>> {
        let mut actions = HashMap::new();
        self.clients.retain_mut(|client| {
            match read_available(&mut client.stream, &mut client.read_buf) {
                Ok(false) => false,
                Ok(true) => loop {
                    match try_take_length_delimited::<SimResponse>(&mut client.read_buf) {
                        Ok(Some(response)) => {
                            for action in response.action {
                                if let Some((key, command)) =
                                    decode_action(action, &client.teams, state)
                                {
                                    actions.insert(key, command);
                                }
                            }
                        }
                        Ok(None) => return true,
                        Err(_) => return false,
                    }
                },
                Err(err) if err.kind() == ErrorKind::WouldBlock => true,
                Err(_) => false,
            }
        });
        Ok(actions)
    }
}

fn build_request(state: &WorldState) -> SimRequest {
    SimRequest {
        timestamp: (state.sim_time * 1_000_000.0) as i64,
        frame_id: state.frame as i64,
        bot_state: state
            .blue_robots
            .iter()
            .map(|robot| bot_state(robot, TeamColor::Blue))
            .chain(
                state
                    .yellow_robots
                    .iter()
                    .map(|robot| bot_state(robot, TeamColor::Yellow)),
            )
            .collect(),
        ball_state: Some(SimBallState {
            pose: Some(Vector3 {
                x: state.ball.x * 1000.0,
                y: state.ball.y * 1000.0,
                z: state.ball.z * 1000.0,
            }),
            vel: Some(Vector3 {
                x: state.ball.vx * 1000.0,
                y: state.ball.vy * 1000.0,
                z: state.ball.vz * 1000.0,
            }),
            acc: Some(Vector3::default()),
            spin: Some(Vector2::default()),
        }),
        last_kick_event: None,
        referee_message: Some(default_referee(state)),
    }
}

fn bot_state(robot: &crate::state::RobotState, team: TeamColor) -> SimBotState {
    SimBotState {
        bot_id: Some(BotId {
            id: robot.id as i32,
            color: local_team_to_proto(team),
        }),
        pose: Some(Vector3 {
            x: robot.x * 1000.0,
            y: robot.y * 1000.0,
            z: robot.orientation,
        }),
        vel: Some(Vector3 {
            x: robot.vx,
            y: robot.vy,
            z: robot.v_angular,
        }),
        barrier_interrupted: robot.infrared,
    }
}

fn default_referee(state: &WorldState) -> SimReferee {
    SimReferee {
        packet_timestamp: (state.sim_time * 1_000_000.0) as u64,
        stage: sim_referee::Stage::NormalFirstHalf as i32,
        stage_time_left: Some(0),
        command: sim_referee::Command::ForceStart as i32,
        command_counter: state.frame as u32,
        command_timestamp: (state.sim_time * 1_000_000.0) as u64,
        yellow: sim_referee::TeamInfo {
            name: "Yellow".into(),
            score: 0,
            red_cards: 0,
            yellow_card_times: Vec::new(),
            yellow_cards: 0,
            timeouts: 0,
            timeout_time: 0,
            goalie: 0,
        },
        blue: sim_referee::TeamInfo {
            name: "Blue".into(),
            score: 0,
            red_cards: 0,
            yellow_card_times: Vec::new(),
            yellow_cards: 0,
            timeouts: 0,
            timeout_time: 0,
            goalie: 0,
        },
        designated_position: None,
        blue_team_on_positive_half: Some(false),
    }
}

fn decode_action(
    action: SimBotAction,
    registered_teams: &HashSet<TeamColor>,
    state: &WorldState,
) -> Option<((TeamColor, usize), RobotCommand)> {
    let bot_id = action.bot_id?;
    let team = proto_team_to_local(bot_id.color)?;
    if !registered_teams.contains(&team) {
        return None;
    }
    let robot = robot_state(state, team, bot_id.id as usize)?;
    let mode_xy = DriveMode::try_from(action.mode_xy).ok()?;
    let mode_w = DriveMode::try_from(action.mode_w).ok()?;
    let move_command = match mode_xy {
        DriveMode::WheelVel => action.target_wheel_vel.as_ref().map(|wheel| {
            let mut values = [0.0; 4];
            for (index, value) in wheel.x.iter().copied().take(4).enumerate() {
                values[index] = value;
            }
            MoveCommand::WheelVelocity(values)
        }),
        _ => decode_velocity_action(&action, robot, mode_xy, mode_w),
    };
    Some((
        (team, bot_id.id as usize),
        RobotCommand {
            id: bot_id.id as usize,
            move_command,
            kick_speed: action.kick_speed,
            kick_angle: if action.chip { 45.0 } else { 0.0 },
            dribbler_on: action.dribble_rpm > 0.0,
        },
    ))
}

fn decode_velocity_action(
    action: &SimBotAction,
    robot: &crate::state::RobotState,
    mode_xy: DriveMode,
    mode_w: DriveMode,
) -> Option<MoveCommand> {
    let angular = decode_angular_velocity(action, robot, mode_w)?;
    match mode_xy {
        DriveMode::Off => {
            if angular.abs() > 1e-6 {
                Some(MoveCommand::LocalVelocity {
                    forward: 0.0,
                    left: 0.0,
                    angular,
                })
            } else {
                None
            }
        }
        DriveMode::LocalVel => {
            action
                .target_vel_local
                .as_ref()
                .map(|vel| MoveCommand::LocalVelocity {
                    forward: vel.x * 0.001,
                    left: vel.y * 0.001,
                    angular,
                })
        }
        DriveMode::GlobalPos => {
            let target = action.target_pos.as_ref()?;
            let limits = action.drive_limits.as_ref();
            let vel_limit = limits.map_or(3.0, |limits| limits.vel_max).max(0.1);
            let target_x = target.x * 0.001;
            let target_y = target.y * 0.001;
            Some(MoveCommand::GlobalVelocity {
                vx: clamp((target_x - robot.x) * 4.0, vel_limit),
                vy: clamp((target_y - robot.y) * 4.0, vel_limit),
                angular,
            })
        }
        DriveMode::WheelVel => None,
    }
}

fn clamp(value: f64, limit: f64) -> f64 {
    value.clamp(-limit, limit)
}

fn decode_angular_velocity(
    action: &SimBotAction,
    robot: &crate::state::RobotState,
    mode_w: DriveMode,
) -> Option<f64> {
    match mode_w {
        DriveMode::Off | DriveMode::WheelVel => Some(0.0),
        DriveMode::LocalVel => Some(action.target_vel_local.as_ref()?.z),
        DriveMode::GlobalPos => {
            let target = action.target_pos.as_ref()?;
            let limit = action
                .drive_limits
                .as_ref()
                .map_or(10.0, |limits| limits.vel_max_w)
                .max(0.1);
            Some(clamp(
                normalize_angle(target.z - robot.orientation) * 6.0,
                limit,
            ))
        }
    }
}

fn robot_state(
    state: &WorldState,
    team: TeamColor,
    id: usize,
) -> Option<&crate::state::RobotState> {
    let robots = match team {
        TeamColor::Blue => &state.blue_robots,
        TeamColor::Yellow => &state.yellow_robots,
    };
    robots.iter().find(|robot| robot.id == id)
}

fn normalize_angle(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

fn merge_actions_into_world_command(
    actions: HashMap<(TeamColor, usize), RobotCommand>,
) -> WorldCommand {
    let mut command = WorldCommand::default();
    merge_actions_into_existing_world_command(&mut command, actions);
    command
}

fn merge_actions_into_existing_world_command(
    command: &mut WorldCommand,
    actions: HashMap<(TeamColor, usize), RobotCommand>,
) {
    for ((team, _), robot_command) in actions {
        match team {
            TeamColor::Blue => command.blue.push(robot_command),
            TeamColor::Yellow => command.yellow.push(robot_command),
        }
    }
}

fn proto_team_to_local(value: i32) -> Option<TeamColor> {
    match crate::proto::sumatra_sim_net::TeamColor::try_from(value).ok()? {
        crate::proto::sumatra_sim_net::TeamColor::Yellow => Some(TeamColor::Yellow),
        crate::proto::sumatra_sim_net::TeamColor::Blue => Some(TeamColor::Blue),
    }
}

fn local_team_to_proto(value: TeamColor) -> i32 {
    match value {
        TeamColor::Yellow => 0,
        TeamColor::Blue => 1,
    }
}

fn decode_error(err: impl std::error::Error) -> std::io::Error {
    std::io::Error::new(ErrorKind::InvalidData, err.to_string())
}

fn read_length_delimited<M: Message + Default>(stream: &mut TcpStream) -> Result<M> {
    let len = read_varint(stream)? as usize;
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf)?;
    M::decode(buf.as_slice()).map_err(decode_error)
}

fn read_available(stream: &mut TcpStream, buffer: &mut Vec<u8>) -> Result<bool> {
    let mut temp = [0u8; 4096];
    loop {
        match stream.read(&mut temp) {
            Ok(0) => return Ok(false),
            Ok(read) => buffer.extend_from_slice(&temp[..read]),
            Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(true),
            Err(err) if err.kind() == ErrorKind::Interrupted => continue,
            Err(err) => return Err(err),
        }
    }
}

fn try_take_length_delimited<M: Message + Default>(buffer: &mut Vec<u8>) -> Result<Option<M>> {
    let Some((len, header_len)) = try_read_varint_from_slice(buffer.as_slice())? else {
        return Ok(None);
    };
    let total_len = header_len + len as usize;
    if buffer.len() < total_len {
        return Ok(None);
    }

    let message = M::decode(&buffer[header_len..total_len]).map_err(decode_error)?;
    buffer.drain(..total_len);
    Ok(Some(message))
}

fn write_length_delimited<M: Message>(stream: &mut TcpStream, message: &M) -> Result<()> {
    let mut payload = Vec::new();
    message.encode(&mut payload).map_err(decode_error)?;
    write_varint(stream, payload.len() as u64)?;
    stream.write_all(&payload)?;
    Ok(())
}

fn read_varint(stream: &mut TcpStream) -> Result<u64> {
    let mut value = 0u64;
    let mut shift = 0u32;
    loop {
        let mut byte = [0u8; 1];
        stream.read_exact(&mut byte)?;
        value |= ((byte[0] & 0x7f) as u64) << shift;
        if byte[0] & 0x80 == 0 {
            return Ok(value);
        }
        shift += 7;
        if shift >= 64 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "varint too long",
            ));
        }
    }
}

fn try_read_varint_from_slice(buf: &[u8]) -> Result<Option<(u64, usize)>> {
    let mut value = 0u64;
    let mut shift = 0u32;
    for (index, byte) in buf.iter().copied().enumerate() {
        value |= ((byte & 0x7f) as u64) << shift;
        if byte & 0x80 == 0 {
            return Ok(Some((value, index + 1)));
        }
        shift += 7;
        if shift >= 64 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "varint too long",
            ));
        }
    }
    Ok(None)
}

fn write_varint(stream: &mut TcpStream, mut value: u64) -> Result<()> {
    let mut buf = [0u8; 10];
    let mut index = 0usize;
    loop {
        let mut byte = (value & 0x7f) as u8;
        value >>= 7;
        if value != 0 {
            byte |= 0x80;
        }
        buf[index] = byte;
        index += 1;
        if value == 0 {
            break;
        }
    }
    stream.write_all(&buf[..index])
}
