//! World state types returned after each simulation step.

use serde::{Deserialize, Serialize};

/// Which team a robot belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TeamColor {
  Blue = 0,
  Yellow = 1,
}

/// Kick type that is in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum KickStatus {
  NoKick,
  FlatKick,
  ChipKick,
}

impl Default for KickStatus {
  fn default() -> Self {
    KickStatus::NoKick
  }
}

/// Complete snapshot of a single robot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
  pub id: usize,
  pub team: TeamColor,
  pub x: f64,
  pub y: f64,
  pub z: f64,
  /// Orientation in radians.
  pub orientation: f64,
  /// Velocity in world frame.
  pub vx: f64,
  pub vy: f64,
  pub vz: f64,
  /// Angular velocity around Z (rad/s).
  pub v_angular: f64,
  /// Whether the infrared sensor detects the ball near the kicker.
  pub infrared: bool,
  /// Whether the dribbler is currently commanded on.
  pub dribbler_on: bool,
  /// Current kick status.
  pub kick_status: KickStatus,
  /// Whether the robot is active/on.
  pub is_on: bool,
  /// Individual wheel angular speeds (rad/s):
  /// [front_right, front_left, back_left, back_right].
  pub wheel_speeds: [f64; 4],
}

/// Complete snapshot of the ball.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BallState {
  pub x: f64,
  pub y: f64,
  pub z: f64,
  pub vx: f64,
  pub vy: f64,
  pub vz: f64,
}

/// Complete snapshot of an entire world at a given time step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldState {
  /// World index within the parallel batch.
  pub world_id: usize,
  /// Simulation time in seconds.
  pub sim_time: f64,
  /// Frame number.
  pub frame: u64,
  /// Ball state.
  pub ball: BallState,
  /// Blue team robots.
  pub blue_robots: Vec<RobotState>,
  /// Yellow team robots.
  pub yellow_robots: Vec<RobotState>,
  /// Whether a goal was scored (simple detection: ball crossed goal line).
  pub goal_blue: bool,
  pub goal_yellow: bool,
}
