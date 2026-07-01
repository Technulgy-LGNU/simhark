use crate::command::{RobotCommand, WorldCommand};
use crate::state::{TeamColor, WorldState};

pub trait TeamController: Send {
  fn reset(&mut self) {}

  fn command_team(
    &mut self,
    world_index: usize,
    team: TeamColor,
    state: &WorldState,
  ) -> Vec<RobotCommand>;
}

pub struct NoopController;

impl TeamController for NoopController {
  fn command_team(
    &mut self,
    _world_index: usize,
    _team: TeamColor,
    _state: &WorldState,
  ) -> Vec<RobotCommand> {
    Vec::new()
  }
}

pub struct FnTeamController<F> {
  f: F,
}

impl<F> FnTeamController<F> {
  pub fn new(f: F) -> Self {
    Self { f }
  }
}

impl<F> TeamController for FnTeamController<F>
where
  F: FnMut(usize, TeamColor, &WorldState) -> Vec<RobotCommand> + Send,
{
  fn command_team(
    &mut self,
    world_index: usize,
    team: TeamColor,
    state: &WorldState,
  ) -> Vec<RobotCommand> {
    (self.f)(world_index, team, state)
  }
}

pub struct ControlledTeams {
  pub blue: Box<dyn TeamController>,
  pub yellow: Box<dyn TeamController>,
}

impl ControlledTeams {
  pub fn new(blue: Box<dyn TeamController>, yellow: Box<dyn TeamController>) -> Self {
    Self { blue, yellow }
  }

  pub fn reset(&mut self) {
    self.blue.reset();
    self.yellow.reset();
  }

  pub fn build_command(&mut self, world_index: usize, state: &WorldState) -> WorldCommand {
    WorldCommand {
      blue: self.blue.command_team(world_index, TeamColor::Blue, state),
      yellow: self
        .yellow
        .command_team(world_index, TeamColor::Yellow, state),
      teleport_ball: None,
      teleport_robots: Vec::new(),
    }
  }
}
