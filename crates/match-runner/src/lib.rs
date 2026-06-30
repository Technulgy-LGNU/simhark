//! Library core for the SSL match runner: a reusable `run_match` plus the
//! controller / director / evaluator / logging building blocks.

pub mod controller;
pub mod director;
pub mod evaluator;
pub mod logio;
#[cfg(feature = "referris")]
pub mod referris_autoref;
#[cfg(feature = "sumatra")]
pub mod sumatra_match;

use controller::{TeamKind, build_controller};
use director::MatchDirector;
use evaluator::{Evaluator, MatchReport};
use logio::GameLog;
use simhark::{SimulationEngine, TeamColor, WorldConfig};

/// Everything needed to play one match.
#[derive(Clone)]
pub struct MatchConfig {
  pub blue: TeamKind,
  pub yellow: TeamKind,
  pub seconds: f64,
  pub div: char,
  pub seed: u64,
  pub log: Option<String>,
  pub log_every: u64,
  pub quiet: bool,
  /// Open the live web viewer (requires the `viewer` build feature).
  pub viewer: bool,
  /// Pace the simulation to ~60 Hz wall-clock (implied by `viewer`).
  pub realtime: bool,
}

impl Default for MatchConfig {
  fn default() -> Self {
    Self {
      blue: TeamKind::Bangka,
      yellow: TeamKind::Bangka,
      seconds: 60.0,
      div: 'b',
      seed: 1,
      log: None,
      log_every: 2,
      quiet: false,
      viewer: false,
      realtime: false,
    }
  }
}

pub fn world_config(div: char, seed: u64) -> WorldConfig {
  let mut cfg = match div {
    'a' | 'A' => WorldConfig::division_a(),
    _ => WorldConfig::division_b(),
  };
  cfg.seed = seed;
  cfg
}

/// Play one match start-to-finish and return its evaluation.
///
/// If either side is an external AI (e.g. the real Sumatra), the match is run
/// through the SimNet hybrid path; otherwise both sides are driven in-process.
pub fn run_match(mc: &MatchConfig) -> MatchReport {
  if mc.blue.is_external() || mc.yellow.is_external() {
    #[cfg(feature = "sumatra")]
    return sumatra_match::run(mc);
    #[cfg(not(feature = "sumatra"))]
    {
      eprintln!(
        "sumatra matches require building match-runner with `--features sumatra` and setting SIMHARK_SUMATRA_REPO_ROOT"
      );
      let cfg = world_config(mc.div, mc.seed);
      return Evaluator::new(
        cfg,
        format!("blue:{}", mc.blue.label()),
        format!("yellow:{}", mc.yellow.label()),
      )
      .finish(0.0);
    }
  }
  let cfg = world_config(mc.div, mc.seed);
  let mut engine = SimulationEngine::new(1, cfg.clone());

  let n = cfg.robots_per_team as u8;
  let mut blue_ctrl = build_controller(&mc.blue, TeamColor::Blue, n);
  let mut yellow_ctrl = build_controller(&mc.yellow, TeamColor::Yellow, n);
  let blue_name = format!("blue:{}", blue_ctrl.name());
  let yellow_name = format!("yellow:{}", yellow_ctrl.name());

  let mut director = MatchDirector::new(cfg.clone(), mc.seconds);
  let mut evaluator = Evaluator::new(cfg.clone(), blue_name.clone(), yellow_name.clone());
  #[cfg(feature = "referris")]
  let mut referris = referris_autoref::ReferrisAutoref::new();
  let mut pickup_validator = PickupValidator::default();

  let mut log = match &mc.log {
    Some(path) => GameLog::create(path, &cfg, &blue_name, &yellow_name).ok(),
    None => None,
  };

  #[cfg(feature = "viewer")]
  let viewer = if mc.viewer {
    let vc = simhark::viewer::ViewerConfig::default();
    match simhark::viewer::ViewerServer::bind(vc, 1, &cfg) {
      Ok(v) => {
        println!("viewer: {}", vc.http_url());
        Some(v)
      }
      Err(e) => {
        eprintln!("viewer bind failed: {e}");
        None
      }
    }
  } else {
    None
  };
  let pace = mc.realtime || mc.viewer;

  let kickoff = director.kickoff_reset();
  let mut state = engine.step_with_commands(&[kickoff]).remove(0);

  let mut command_counter: u32 = 1;
  while !director.is_over(&state) {
    let gc_blue = director.command_for(TeamColor::Blue);
    let gc_yellow = director.command_for(TeamColor::Yellow);
    let blue_cmds = blue_ctrl.act(&state, &cfg, TeamColor::Blue, gc_blue);
    let yellow_cmds = yellow_ctrl.act(&state, &cfg, TeamColor::Yellow, gc_yellow);

    let mut wc = director.update(&state);
    if let Some(scorer) = director.take_goal() {
      evaluator.record_goal(scorer);
      command_counter = command_counter.wrapping_add(1);
      if !mc.quiet {
        if let Some(ev) = &director.last_event {
          println!("  [{:6.1}s] {}", state.sim_time, ev);
        }
      }
    }
    wc.blue = blue_cmds;
    wc.yellow = yellow_cmds;

    let new_state = engine.step_with_commands(&[wc]).remove(0);
    evaluator.tick(&new_state, Some(&state));

    #[cfg(feature = "referris")]
    let referris_tick = referris.step(
      &new_state,
      &cfg,
      director.score,
      director.referee_command_code(),
      mc.quiet,
    );

    if std::env::var("MATCH_DEBUG").is_ok() && new_state.frame % 120 == 0 {
      let bh = new_state.blue_robots.iter().filter(|r| r.infrared).count();
      let yh = new_state
        .yellow_robots
        .iter()
        .filter(|r| r.infrared)
        .count();
      let bd = new_state
        .blue_robots
        .iter()
        .filter(|r| r.dribbler_on)
        .count();
      let (bx, by) = (new_state.ball.x, new_state.ball.y);
      let nearest = |rs: &[simhark::RobotState]| {
        rs.iter()
          .map(|r| ((r.x - bx).powi(2) + (r.y - by).powi(2)).sqrt())
          .fold(f64::INFINITY, f64::min)
      };
      eprintln!(
        "[match] t={:.1} ball=({:.2},{:.2}) v=({:.2},{:.2}) blue_ir={bh} blue_drib={bd} yel_ir={yh} near_b={:.2} near_y={:.2}",
        new_state.sim_time,
        bx,
        by,
        new_state.ball.vx,
        new_state.ball.vy,
        nearest(&new_state.blue_robots),
        nearest(&new_state.yellow_robots),
      );
    }

    if let Some(log) = log.as_mut() {
      if new_state.frame % mc.log_every == 0 {
        #[cfg(feature = "referris")]
        let (referee_command_code, command_counter) =
          (referris_tick.command_code, referris_tick.command_counter);
        #[cfg(not(feature = "referris"))]
        let (referee_command_code, command_counter) =
          (director.referee_command_code(), command_counter);
        let _ = log.write_frame(
          &new_state,
          director.score,
          director.referee_command_code(),
          command_counter,
        );
      }
    }
    #[cfg(feature = "viewer")]
    if let Some(v) = &viewer {
      #[cfg(feature = "referris")]
      v.set_game_state(simhark::viewer::GameStateInfo {
        command: referris_tick.command_label.to_string(),
        command_counter: referris_tick.command_counter,
        stage: None,
        blue_name: Some(blue_name.clone()),
        yellow_name: Some(yellow_name.clone()),
      });
      v.publish(&new_state);
    }
    if pace {
      std::thread::sleep(std::time::Duration::from_millis(16));
    }

    state = new_state;
  }

  if let Some(log) = log {
    let _ = log.close();
  }
  evaluator.finish(state.sim_time)
}
