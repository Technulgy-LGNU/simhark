//! Writes a standard SSL log file (vision + tracker + referee) that Loguna can
//! replay. Uses `loguna`'s proto types so the on-disk format matches the viewer.

use loguna::proto as p;
use loguna::{LogMessage, LogWriter, MessageId};
use prost::Message;
use simhark::{RobotState, TeamColor, WorldConfig, WorldState};

use crate::director::Score;

pub struct GameLog {
  writer: LogWriter,
  base_ns: i64,
  blue_name: String,
  yellow_name: String,
}

impl GameLog {
  pub fn create(
    path: &str,
    cfg: &WorldConfig,
    blue_name: &str,
    yellow_name: &str,
  ) -> std::io::Result<Self> {
    let writer = LogWriter::create(path)
      .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))?;
    let base_ns = 1_600_000_000_000_000_000; // arbitrary fixed epoch base
    let mut log = Self {
      writer,
      base_ns,
      blue_name: blue_name.to_string(),
      yellow_name: yellow_name.to_string(),
    };
    log.write_geometry(cfg)?;
    Ok(log)
  }

  fn ts(&self, sim_time: f64) -> i64 {
    self.base_ns + (sim_time * 1e9) as i64
  }

  fn put(&mut self, id: MessageId, sim_time: f64, payload: Vec<u8>) -> std::io::Result<()> {
    self
      .writer
      .write_message(&LogMessage {
        timestamp_ns: self.ts(sim_time),
        message_id: id,
        payload,
      })
      .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))
  }

  fn write_geometry(&mut self, cfg: &WorldConfig) -> std::io::Result<()> {
    let f = &cfg.field;
    let field = p::SslGeometryFieldSize {
      field_length: (f.field_length * 1000.0) as i32,
      field_width: (f.field_width * 1000.0) as i32,
      goal_width: (f.goal_width * 1000.0) as i32,
      goal_depth: (f.goal_depth * 1000.0) as i32,
      boundary_width: (f.margin_touch_line * 1000.0) as i32,
      penalty_area_depth: Some((f.penalty_depth * 1000.0) as i32),
      penalty_area_width: Some((f.penalty_width * 1000.0) as i32),
      center_circle_radius: Some((f.field_center_radius * 1000.0) as i32),
      line_thickness: Some((f.field_line_width * 1000.0) as i32),
      goal_height: Some((f.goal_height * 1000.0) as i32),
      ball_radius: Some((cfg.ball.radius * 1000.0) as f32),
      max_robot_radius: Some((cfg.blue_robots.radius * 1000.0) as f32),
      ..Default::default()
    };
    let geo = p::SslGeometryData {
      field,
      ..Default::default()
    };
    let wrapper = p::SslWrapperPacket {
      geometry: Some(geo),
      ..Default::default()
    };
    self.put(MessageId::Vision2014, 0.0, wrapper.encode_to_vec())
  }

  pub fn write_frame(
    &mut self,
    state: &WorldState,
    score: Score,
    command: i32,
    command_counter: u32,
  ) -> std::io::Result<()> {
    let detection = p::SslDetectionFrame {
      frame_number: state.frame as u32,
      t_capture: state.sim_time,
      t_sent: state.sim_time,
      camera_id: 0,
      balls: vec![p::SslDetectionBall {
        confidence: 1.0,
        x: (state.ball.x * 1000.0) as f32,
        y: (state.ball.y * 1000.0) as f32,
        z: Some((state.ball.z * 1000.0) as f32),
        pixel_x: 0.0,
        pixel_y: 0.0,
        ..Default::default()
      }],
      robots_yellow: det_robots(&state.yellow_robots),
      robots_blue: det_robots(&state.blue_robots),
      ..Default::default()
    };
    let wrapper = p::SslWrapperPacket {
      detection: Some(detection),
      ..Default::default()
    };
    self.put(
      MessageId::Vision2014,
      state.sim_time,
      wrapper.encode_to_vec(),
    )?;

    let stamp = (self.ts(state.sim_time) / 1000) as u64;
    let referee = p::Referee {
      source_identifier: Some("match-runner".into()),
      packet_timestamp: stamp,
      stage: p::referee::Stage::NormalFirstHalf as i32,
      command,
      command_counter,
      command_timestamp: stamp,
      yellow: team_info(&self.yellow_name, score.yellow),
      blue: team_info(&self.blue_name, score.blue),
      blue_team_on_positive_half: Some(false),
      ..Default::default()
    };
    self.put(
      MessageId::Referee2013,
      state.sim_time,
      referee.encode_to_vec(),
    )?;
    Ok(())
  }

  pub fn close(self) -> std::io::Result<()> {
    self
      .writer
      .close()
      .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))
  }
}

fn det_robots(robots: &[RobotState]) -> Vec<p::SslDetectionRobot> {
  robots
    .iter()
    .filter(|r| r.is_on)
    .map(|r| p::SslDetectionRobot {
      confidence: 1.0,
      robot_id: Some(r.id as u32),
      x: (r.x * 1000.0) as f32,
      y: (r.y * 1000.0) as f32,
      orientation: Some(r.orientation as f32),
      pixel_x: 0.0,
      pixel_y: 0.0,
      height: Some((r.z * 1000.0) as f32),
      ..Default::default()
    })
    .collect()
}

fn team_info(name: &str, score: u32) -> p::referee::TeamInfo {
  p::referee::TeamInfo {
    name: name.to_string(),
    score,
    timeouts: 4,
    ..Default::default()
  }
}

#[allow(dead_code)]
pub fn team_color_str(c: TeamColor) -> &'static str {
  match c {
    TeamColor::Blue => "blue",
    TeamColor::Yellow => "yellow",
  }
}
