use core_dump::proto::{
  CpMode, InterfaceCommandCp, InterfaceGameCp, InterfaceManualCp, InterfaceTestCp,
  InterfaceWrapperCp,
};
use simhark::TeamColor;

pub fn force_start_referee() -> core_dump::proto::Referee {
  core_dump::proto::Referee {
    command: 3, // FORCE_START
    ..Default::default()
  }
}

pub fn interface_command(team: TeamColor) -> InterfaceWrapperCp {
  let (team_color, side) = match team {
    TeamColor::Yellow => (true, false),
    TeamColor::Blue => (false, true),
  };
  InterfaceWrapperCp {
    robot_commands: Vec::new(),
    interface_command: InterfaceCommandCp {
      mode: CpMode::ModeGame as i32,
      manual: InterfaceManualCp {
        ball_tracked: true,
        ..Default::default()
      },
      game: InterfaceGameCp {
        running: true,
        side,
        team_color,
        goalkeeper_id: 0,
        max_speed: 0,
      },
      test: InterfaceTestCp::default(),
    },
  }
}
