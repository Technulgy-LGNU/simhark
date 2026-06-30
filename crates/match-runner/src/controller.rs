//! Team controllers. Every AI runs *inside CrashPilot* and reaches simhark only
//! through the `simhark_faabs` binding (CrashPilot -> tf_jetsoncode firmware ->
//! simhark). A controller just wraps a `Faabs<A>` for one team colour.

use core_dump::proto::Referee;
use core_dump::types::Ai;
use simhark::{RobotCommand, TeamColor, WorldCommand, WorldConfig, WorldState};
use simhark_faabs::synth::force_start_referee;
use simhark_faabs::Faabs;

/// Referee state resolved relative to a team, as decided by the match director.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GameCommand {
  Halt,
  Stop,
  Running,
  FreeKickUs,
  FreeKickThem,
  PrepareKickoffUs,
  PrepareKickoffThem,
}

pub trait Controller {
  fn name(&self) -> &str;
  fn act(
    &mut self,
    state: &WorldState,
    cfg: &WorldConfig,
    color: TeamColor,
    gc: GameCommand,
  ) -> Vec<RobotCommand>;
}

fn referee_for(gc: GameCommand) -> Option<Referee> {
  match gc {
    GameCommand::Halt => Some(Referee {
      command: 0, // HALT
      ..Default::default()
    }),
    // Everything else: keep CrashPilot in the Running phase so it plays. The
    // match director handles kickoff positioning via teleports.
    _ => Some(force_start_referee()),
  }
}

/// Wraps a `Faabs<A>` (a CrashPilot AI bound into simhark) as a `Controller`.
pub struct FaabsController<A: Ai> {
  faabs: Faabs<A>,
  name: String,
}

impl<A: Ai + Send> Controller for FaabsController<A> {
  fn name(&self) -> &str {
    &self.name
  }

  fn act(
    &mut self,
    state: &WorldState,
    _cfg: &WorldConfig,
    color: TeamColor,
    gc: GameCommand,
  ) -> Vec<RobotCommand> {
    let mut scratch = WorldCommand::default();
    self.faabs.step(state, &mut scratch, referee_for(gc));
    match color {
      TeamColor::Blue => scratch.blue,
      TeamColor::Yellow => scratch.yellow,
    }
  }
}

/// Identifies which AI a side should use.
#[derive(Debug, Clone)]
pub enum TeamKind {
  /// Bangka — the current non-ML role/skill AI, run inside CrashPilot.
  Bangka,
  /// Bongka — the tuned/legacy Bangka-line AI, run inside CrashPilot.
  Bongka { params: Option<String> },
  /// Ungabunga — a sibling Bangka-line AI, run inside CrashPilot.
  Ungabunga { params: Option<String> },
  /// Frozen snapshot of Bangka at Pass 5 (goal-shadow wall + far-post striker),
  /// used as a fixed sparring partner for deterministic benchmarking.
  Bangka1,
  /// Frozen snapshot of the original Bangka, used as a fixed sparring partner
  /// for deterministic benchmarking of new Bangka versions.
  BangkaLegacy,
  /// CrashPilot's machine-learning AI.
  CrashPilot { model: Option<String> },
  /// The real Sumatra (external Java AI), driven over the SimNet protocol.
  /// This side is *not* a faabs controller; `run_match` handles it specially.
  Sumatra,
}

impl TeamKind {
  pub fn parse(s: &str) -> Result<Self, String> {
    let (name, arg) = match s.split_once(':') {
      Some((n, a)) => (n, Some(a.to_string())),
      None => (s, None),
    };
    match name.to_ascii_lowercase().as_str() {
      "bangka" | "us" | "new" => Ok(TeamKind::Bangka),
      "bongka" => Ok(TeamKind::Bongka { params: arg }),
      "ungabunga" => Ok(TeamKind::Ungabunga { params: arg }),
      "bangka1" => Ok(TeamKind::Bangka1),
      "legacy" | "bangka0" | "baseline" => Ok(TeamKind::BangkaLegacy),
      "crashpilot" | "cp" | "ml" | "ai" => Ok(TeamKind::CrashPilot { model: arg }),
      "sumatra" | "real" | "tigers" => Ok(TeamKind::Sumatra),
      other => Err(format!("unknown team kind: {other}")),
    }
  }

  pub fn label(&self) -> &'static str {
    match self {
      TeamKind::Bangka => "bangka",
      TeamKind::Bongka { .. } => "bongka",
      TeamKind::Ungabunga { .. } => "ungabunga",
      TeamKind::Bangka1 => "bangka1",
      TeamKind::BangkaLegacy => "legacy",
      TeamKind::CrashPilot { .. } => "crashpilot",
      TeamKind::Sumatra => "sumatra",
    }
  }

  /// True for AIs that run externally (over SimNet) rather than as a faabs
  /// controller inside this process.
  pub fn is_external(&self) -> bool {
    matches!(self, TeamKind::Sumatra)
  }
}

/// Build a faabs controller for an in-process side. Panics for external kinds
/// (e.g. [`TeamKind::Sumatra`]); `run_match` must route those separately.
pub fn build_controller(kind: &TeamKind, color: TeamColor, num_robots: u8) -> Box<dyn Controller> {
  match kind {
    #[cfg(not(feature = "bangka"))]
    TeamKind::Bangka => panic!("Bangka is disabled; build with `--features bangka` to enable"),
    #[cfg(feature = "bangka")]
    TeamKind::Bangka => {
      let faabs = Faabs::with_ai(num_robots, color, bangka::Bangka::new());
      Box::new(FaabsController {
        faabs,
        name: "bangka".to_string(),
      })
    }
    #[cfg(not(feature = "bongka"))]
    TeamKind::Bongka => panic!("Bongka is disabled; build with `--features bongka` to enable"),
    #[cfg(feature = "bongka")]
    TeamKind::Bongka { params } => {
      let p = params
        .as_ref()
        .and_then(|path| std::fs::read_to_string(path).ok())
        .and_then(|s| bongka::Params::from_json_str(&s))
        .unwrap_or_default();
      let faabs = Faabs::with_ai(num_robots, color, bongka::Bangka::with_params(p));
      Box::new(FaabsController {
        faabs,
        name: "bongka".to_string(),
      })
    }
    #[cfg(not(feature = "ungabunga"))]
    TeamKind::Ungabunga { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::Ungabunga { params } => {
      let p = params
        .as_ref()
        .and_then(|path| std::fs::read_to_string(path).ok())
        .and_then(|s| ungabunga::Params::from_json_str(&s))
        .unwrap_or_default();
      let faabs = Faabs::with_ai(num_robots, color, ungabunga::Bangka::with_params(p));
      Box::new(FaabsController {
        faabs,
        name: "ungabunga".to_string(),
      })
    }

    #[cfg(not(feature = "ungabunga"))]
    TeamKind::Bangka1 { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::Bangka1 => {
      let faabs = Faabs::with_ai(num_robots, color, ungabunga::Bangka1::new());
      Box::new(FaabsController {
        faabs,
        name: "bangka1".to_string(),
      })
    }

    #[cfg(not(feature = "ungabunga"))]
    TeamKind::BangkaLegacy { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::BangkaLegacy => {
      let faabs = Faabs::with_ai(num_robots, color, ungabunga::LegacyBangka::new());
      Box::new(FaabsController {
        faabs,
        name: "legacy".to_string(),
      })
    }
    #[cfg(not(feature = "artificial_incompetence"))]
    TeamKind::CrashPilot { .. } => {
      panic!("CrashPilot is disabled; build with `--features artificial_incompetence` to enable")
    }
    #[cfg(feature = "artificial_incompetence")]
    TeamKind::CrashPilot { model } => {
      let path = model
        .as_deref()
        .unwrap_or(artificial_incompetence::DEFAULT_MODEL_PATH);
      let ai = MlAi::from_safetensors(path).unwrap_or_else(|err| {
        panic!("failed to load CrashPilot model from {path}: {err}");
      });
      let faabs = Faabs::with_ai(num_robots, color, ai);
      Box::new(FaabsController {
        faabs,
        name: "crashpilot".to_string(),
      })
    }
    TeamKind::Sumatra => {
      unreachable!("Sumatra is external; run_match drives it over SimNet")
    }
  }
}
