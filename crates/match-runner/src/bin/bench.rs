//! `bench` — the self-improvement yardstick.
//!
//! Plays the challenger (bangka, optionally with a params file) against a
//! fixed opponent (default: the real `sumatra`) over several seeds, on BOTH
//! sides to cancel kickoff/side bias, then prints an aggregate and appends a
//! one-line record to the iterations log.
//!
//! Note: `--opponent sumatra` launches the real Sumatra JVM per match and runs
//! in real time, so it is slow; use `--opponent bangka` for fast self-play
//! parameter sweeps.
//!
//! ```text
//! bench --seeds 6 --seconds 45 --label "iter 3: better keeper"
//! ```

use match_runner::controller::TeamKind;
use match_runner::evaluator::MatchReport;
use match_runner::{MatchConfig, run_match};
use std::io::Write;

struct Cfg {
  seeds: u64,
  seconds: f64,
  div: char,
  opponent: String,
  challenger_params: Option<String>,
  log: String,
  label: String,
}

fn parse() -> Cfg {
  let mut c = Cfg {
    seeds: 6,
    seconds: 45.0,
    div: 'b',
    opponent: "sumatra".to_string(),
    challenger_params: None,
    log: "eval/iterations.log".to_string(),
    label: String::new(),
  };
  let mut it = std::env::args().skip(1);
  while let Some(a) = it.next() {
    let mut next = || it.next().expect("missing value");
    match a.as_str() {
      "--seeds" => c.seeds = next().parse().unwrap(),
      "--seconds" => c.seconds = next().parse().unwrap(),
      "--div" => c.div = next().chars().next().unwrap_or('b'),
      "--opponent" => c.opponent = next(),
      "--params" => c.challenger_params = Some(next()),
      "--log" => c.log = next(),
      "--label" => c.label = next(),
      other => panic!("unknown arg {other}"),
    }
  }
  c
}

fn challenger(params: &Option<String>) -> TeamKind {
  if params.is_some() {
    eprintln!("warning: --params is ignored for bangka challenger");
  }
  TeamKind::Bangka
}

#[derive(Default)]
struct Agg {
  chal_goals: u32,
  opp_goals: u32,
  chal_score: f64,
  wins: u32,
  draws: u32,
  losses: u32,
  chal_poss: f64,
  chal_shots: u32,
  chal_on_target: u32,
  matches: u32,
}

impl Agg {
  /// Fold in one match given which color the challenger played.
  fn add(&mut self, r: &MatchReport, chal_is_blue: bool) {
    let (chal, opp) = if chal_is_blue {
      (&r.blue, &r.yellow)
    } else {
      (&r.yellow, &r.blue)
    };
    self.chal_goals += chal.metrics.goals_for;
    self.opp_goals += opp.metrics.goals_for;
    self.chal_score += chal.score;
    self.chal_poss += chal.possession_pct;
    self.chal_shots += chal.metrics.shots;
    self.chal_on_target += chal.metrics.shots_on_target;
    self.matches += 1;
    match chal.metrics.goals_for.cmp(&opp.metrics.goals_for) {
      std::cmp::Ordering::Greater => self.wins += 1,
      std::cmp::Ordering::Equal => self.draws += 1,
      std::cmp::Ordering::Less => self.losses += 1,
    }
  }
}

fn git_rev() -> String {
  std::process::Command::new("git")
    .args(["rev-parse", "--short", "HEAD"])
    .output()
    .ok()
    .filter(|o| o.status.success())
    .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    .unwrap_or_else(|| "nogit".to_string())
}

fn main() {
  let c = parse();
  let opp = TeamKind::parse(&c.opponent).expect("bad opponent");
  let chal = challenger(&c.challenger_params);

  let mut agg = Agg::default();
  for s in 0..c.seeds {
    // Challenger as blue.
    let r1 = run_match(&MatchConfig {
      blue: chal.clone(),
      yellow: opp.clone(),
      seconds: c.seconds,
      div: c.div,
      seed: 1000 + s,
      quiet: true,
      ..Default::default()
    });
    agg.add(&r1, true);
    // Challenger as yellow.
    let r2 = run_match(&MatchConfig {
      blue: opp.clone(),
      yellow: chal.clone(),
      seconds: c.seconds,
      div: c.div,
      seed: 2000 + s,
      quiet: true,
      ..Default::default()
    });
    agg.add(&r2, false);
  }

  let m = agg.matches.max(1) as f64;
  let net = agg.chal_goals as i64 - agg.opp_goals as i64;
  let avg_score = agg.chal_score / m;
  let winrate = 100.0 * agg.wins as f64 / m;
  let avg_poss = agg.chal_poss / m;

  println!("\n========== BENCH: bangka vs {} ==========", c.opponent);
  println!(
    "  matches={}  W-D-L = {}-{}-{}  winrate={:.0}%",
    agg.matches, agg.wins, agg.draws, agg.losses, winrate
  );
  println!(
    "  goals: challenger {} - {} opponent   (net {:+})",
    agg.chal_goals, agg.opp_goals, net
  );
  println!(
    "  avg score {:+.2}  avg poss {:.0}%  shots {} ({} on target)",
    avg_score, avg_poss, agg.chal_shots, agg.chal_on_target
  );

  // Append to the cumulative iterations log.
  let rev = git_rev();
  let ts = now_string();
  if let Some(parent) = std::path::Path::new(&c.log).parent() {
    let _ = std::fs::create_dir_all(parent);
  }
  if let Ok(mut f) = std::fs::OpenOptions::new()
    .create(true)
    .append(true)
    .open(&c.log)
  {
    let _ = writeln!(
      f,
      "{ts} | {rev:>8} | vs {opp:<10} | W-D-L {w}-{d}-{l} ({wr:>3.0}%) | goals {cg}-{og} net {net:+} | avg_score {asc:+.2} | poss {pos:.0}% | shots {sh}/{ot} | {label}",
      opp = c.opponent,
      w = agg.wins,
      d = agg.draws,
      l = agg.losses,
      wr = winrate,
      cg = agg.chal_goals,
      og = agg.opp_goals,
      asc = avg_score,
      pos = avg_poss,
      sh = agg.chal_shots,
      ot = agg.chal_on_target,
      label = c.label,
    );
  }
  println!("  -> appended to {}", c.log);
}

fn now_string() -> String {
  // Avoid extra deps: use SystemTime seconds since epoch.
  let secs = std::time::SystemTime::now()
    .duration_since(std::time::UNIX_EPOCH)
    .map(|d| d.as_secs())
    .unwrap_or(0);
  format!("t={secs}")
}
