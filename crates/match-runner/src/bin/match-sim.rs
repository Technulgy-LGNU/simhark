//! `match-sim` — run an AI-vs-AI RoboCup SSL match inside simhark, score it
//! RL-style, and optionally write an SSL log for Loguna.
//!
//! Examples:
//! ```text
//! match-sim --blue bangka --yellow sumatra --div b --seconds 60 --log game.log
//! match-sim --blue bangka --yellow bangka --div b --matches 3
//! ```

use match_runner::controller::TeamKind;
use match_runner::evaluator::MatchReport;
use match_runner::{run_match, MatchConfig};

struct Args {
  mc: MatchConfig,
  matches: usize,
  summary: Option<String>,
}

fn parse() -> Result<Args, String> {
  let mut mc = MatchConfig::default();
  let mut matches = 1;
  let mut summary = None;
  let log_base: Option<String>;
  let mut log = None;

  let mut it = std::env::args().skip(1);
  while let Some(a) = it.next() {
    let mut next = || it.next().ok_or_else(|| format!("missing value for {a}"));
    match a.as_str() {
      "--blue" => mc.blue = TeamKind::parse(&next()?)?,
      "--yellow" => mc.yellow = TeamKind::parse(&next()?)?,
      "--seconds" => mc.seconds = next()?.parse().map_err(|_| "bad --seconds")?,
      "--div" => mc.div = next()?.chars().next().unwrap_or('b'),
      "--seed" => mc.seed = next()?.parse().map_err(|_| "bad --seed")?,
      "--matches" => matches = next()?.parse().map_err(|_| "bad --matches")?,
      "--log" => log = Some(next()?),
      "--summary" => summary = Some(next()?),
      "--log-every" => mc.log_every = next()?.parse().map_err(|_| "bad --log-every")?,
      "--print-commands" => mc.print_commands = true,
      "--print-commands-every" => {
        mc.print_commands_every = next()?.parse().map_err(|_| "bad --print-commands-every")?
      }
      "--validate-pickup" => mc.validate_pickup = true,
      "--viewer" => mc.viewer = true,
      "--realtime" => mc.realtime = true,
      "--quiet" => mc.quiet = true,
      "-h" | "--help" => {
        print_help();
        std::process::exit(0);
      }
      other => return Err(format!("unknown argument: {other}")),
    }
  }
  log_base = log;
  mc.log = log_base;
  Ok(Args {
    mc,
    matches,
    summary,
  })
}

fn print_help() {
  println!(
    "match-sim — AI vs AI RoboCup SSL match in simhark\n\
\n\
Options:\n\
  --blue   <kind>   team controlling blue   (default bangka)\n\
  --yellow <kind>   team controlling yellow (default bangka)\n\
  --seconds <f>     match length in sim seconds (default 60)\n\
  --div <a|b>       division / field+robot count (default b)\n\
  --seed <u>        RNG seed (default 1)\n\
  --matches <n>     play n matches (seeds seed..seed+n) and aggregate\n\
  --log <path>      write SSL log file (Loguna-compatible)\n\
  --summary <path>  append one JSON summary line per run\n\
  --log-every <n>   log every n-th frame (default 2)\n\
  --print-commands  print simulator robot commands to stderr\n\
  --print-commands-every <n> frame interval for command printing (default 60)\n\
  --validate-pickup warn if close slow pickup or fast predicted pickup is neglected\n\
  --viewer          open the live web viewer (build with --features viewer)\n\
  --realtime        pace the sim to ~60Hz wall-clock\n\
  --quiet           less stdout\n\
\n\
Team kinds: bangka | bongka[:params.json] | ungabunga[:params.json] | crashpilot[:model.safetensors] | sumatra (real, external JVM)\n\
\n\
'crashpilot' defaults to /run/media/shark/data/dev/robocup/ai/crashpilot.safetensors.\n\
'sumatra' launches the real Sumatra over SimNet and runs in real time. Use\n\
--div b: the in-process AI (CrashPilot) supports at most 8 robots/team."
  );
}

fn print_report(report: &MatchReport, quiet: bool) {
  println!(
    "\n=== Final: {} {} - {} {}  | winner: {} ===",
    report.blue.name,
    report.blue.metrics.goals_for,
    report.yellow.metrics.goals_for,
    report.yellow.name,
    report.winner,
  );
  if quiet {
    return;
  }
  for t in [&report.blue, &report.yellow] {
    println!(
      "  {:<22} score={:+7.2}  poss={:>4.0}%  shots={}({} on target)  progress={:.1}m",
      t.name,
      t.score,
      t.possession_pct,
      t.metrics.shots,
      t.metrics.shots_on_target,
      t.metrics.ball_progress,
    );
    for n in &t.notes {
      println!("      - {n}");
    }
  }
}

fn append_summary(path: &str, report: &MatchReport) {
  use std::io::Write;
  if let Ok(mut f) = std::fs::OpenOptions::new()
    .create(true)
    .append(true)
    .open(path)
  {
    if let Ok(line) = serde_json::to_string(report) {
      let _ = writeln!(f, "{line}");
    }
  }
}

#[tokio::main]
async fn main() {
  let args = match parse() {
    Ok(a) => a,
    Err(e) => {
      eprintln!("error: {e}\n");
      print_help();
      std::process::exit(2);
    }
  };

  let (mut blue_total, mut yellow_total) = (0.0, 0.0);
  let (mut blue_goals, mut yellow_goals) = (0u32, 0u32);

  for i in 0..args.matches {
    if !args.mc.quiet && args.matches > 1 {
      println!("--- match {}/{} ---", i + 1, args.matches);
    }
    let mut mc = args.mc.clone();
    mc.seed = args.mc.seed.wrapping_add(i as u64);
    if let (Some(base), true) = (&args.mc.log, args.matches > 1) {
      mc.log = Some(base.replace(".log", &format!("_{i}.log")));
    }
    let report = run_match(&mc);
    blue_total += report.blue.score;
    yellow_total += report.yellow.score;
    blue_goals += report.blue.metrics.goals_for;
    yellow_goals += report.yellow.metrics.goals_for;
    print_report(&report, args.mc.quiet);
    if let Some(path) = &args.summary {
      append_summary(path, &report);
    }
  }

  if args.matches > 1 {
    let n = args.matches as f64;
    println!(
      "\n=== Aggregate over {} matches ===\n  blue   avg {:+.2} (goals {})\n  yellow avg {:+.2} (goals {})",
      args.matches,
      blue_total / n,
      blue_goals,
      yellow_total / n,
      yellow_goals,
    );
  }
}
