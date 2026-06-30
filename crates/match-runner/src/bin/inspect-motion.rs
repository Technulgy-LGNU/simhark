//! inspect-motion — measure how much each robot actually moves over a match.
//!
//! Reads an SSL log written by `match-sim --log <path>` and, for every robot of
//! a team, reports the total path length it travelled, the bounding box of its
//! positions, and its largest single-frame step. This is a quick sanity check
//! for "is robot N actually moving?" — e.g. validating that the goalie (id 0)
//! tracks the ball instead of standing still.
//!
//! Usage:
//!   inspect-motion <log-path> [--team blue|yellow] [--id N]
//!
//! Example:
//!   match-sim --blue bangka --yellow sumatra --div b --seconds 30 \
//!       --seed 1 --log runs/motion.log
//!   inspect-motion runs/motion.log --team blue --id 0

use loguna::proto::SslWrapperPacket;
use loguna::{LogReader, MessageId};
use prost::Message;
use std::collections::BTreeMap;

#[derive(Default, Clone)]
struct Track {
  frames: u64,
  path_mm: f64,
  max_step_mm: f64,
  min_x: f64,
  max_x: f64,
  min_y: f64,
  max_y: f64,
  last: Option<(f64, f64)>,
}

impl Track {
  fn update(&mut self, x: f64, y: f64) {
    if let Some((px, py)) = self.last {
      let step = ((x - px).powi(2) + (y - py).powi(2)).sqrt();
      self.path_mm += step;
      if step > self.max_step_mm {
        self.max_step_mm = step;
      }
    } else {
      self.min_x = x;
      self.max_x = x;
      self.min_y = y;
      self.max_y = y;
    }
    self.min_x = self.min_x.min(x);
    self.max_x = self.max_x.max(x);
    self.min_y = self.min_y.min(y);
    self.max_y = self.max_y.max(y);
    self.last = Some((x, y));
    self.frames += 1;
  }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
  let mut args = std::env::args().skip(1);
  let Some(path) = args.next() else {
    eprintln!("usage: inspect-motion <log-path> [--team blue|yellow] [--id N]");
    std::process::exit(2);
  };
  let mut team = "blue".to_string();
  let mut focus: Option<u32> = Some(0);
  while let Some(a) = args.next() {
    match a.as_str() {
      "--team" => team = args.next().unwrap_or_else(|| "blue".into()),
      "--id" => focus = args.next().and_then(|s| s.parse().ok()),
      "--all" => focus = None,
      other => {
        eprintln!("unknown argument: {other}");
        std::process::exit(2);
      }
    }
  }
  let want_blue = team.eq_ignore_ascii_case("blue");

  let mut reader = LogReader::open(&path)?;
  // id -> motion track, for the selected team.
  let mut tracks: BTreeMap<u32, Track> = BTreeMap::new();

  while let Some(msg) = reader.next_message()? {
    if msg.message_id != MessageId::Vision2014 {
      continue;
    }
    let Ok(wrapper) = SslWrapperPacket::decode(msg.payload.as_slice()) else {
      continue;
    };
    let Some(det) = wrapper.detection else {
      continue;
    };
    let robots = if want_blue {
      &det.robots_blue
    } else {
      &det.robots_yellow
    };
    for r in robots {
      let id = r.robot_id.unwrap_or_default();
      tracks.entry(id).or_default().update(r.x as f64, r.y as f64);
    }
  }

  if tracks.is_empty() {
    eprintln!("no {team} robot detections found in {path}");
    std::process::exit(1);
  }

  println!("Motion report ({team} team) — {path}");
  println!(
    "{:>3}  {:>6}  {:>10}  {:>10}  {:>10}  {:>10}",
    "id", "frames", "path(m)", "x-span(m)", "y-span(m)", "max-step(mm)"
  );
  for (id, t) in &tracks {
    if let Some(f) = focus {
      if *id != f {
        continue;
      }
    }
    let marker = if *id == 0 { " <- goalie" } else { "" };
    println!(
      "{:>3}  {:>6}  {:>10.2}  {:>10.2}  {:>10.2}  {:>10.0}{}",
      id,
      t.frames,
      t.path_mm / 1000.0,
      (t.max_x - t.min_x) / 1000.0,
      (t.max_y - t.min_y) / 1000.0,
      t.max_step_mm,
      marker,
    );
  }

  // A clear verdict for the focused robot (default: goalie id 0).
  if let Some(f) = focus {
    if let Some(t) = tracks.get(&f) {
      let path_m = t.path_mm / 1000.0;
      let span_m = ((t.max_x - t.min_x).hypot(t.max_y - t.min_y)) / 1000.0;
      let verdict = if path_m < 0.10 {
        "STATIONARY (did not move)"
      } else if span_m < 0.30 {
        "jitter only (<0.3 m span)"
      } else {
        "MOVING"
      };
      println!(
        "\nrobot {f}: travelled {path_m:.2} m, ranged {span_m:.2} m across the field -> {verdict}"
      );
    }
  }

  Ok(())
}
