//! # RoboCup SSL Simulator
//!
//! A high-performance, massively parallel RoboCup Small Size League simulator
//! inspired by grSim. Designed for AI training with support for thousands of
//! simultaneous worlds.
//!
//! ## Key Features
//! - Run 512, 1024, 2048+ worlds in parallel via rayon
//! - Pure Rust API (no protobuf needed)
//! - Domain randomization: per-world physics constant mutations for sim-to-real transfer
//! - Deterministic simulation with seeded RNG
//! - Headless mode for maximum throughput
//! - Optional visualization for debugging
//!
//! ## Quick Start
//! ```rust
//! use simhark::{SimulationEngine, WorldConfig, RobotCommand};
//!
//! // Create engine with 1024 parallel worlds
//! let mut engine = SimulationEngine::new(1024, WorldConfig::division_a());
//!
//! // Step all worlds
//! let states = engine.step_all();
//! ```

pub mod config;
pub mod geometry;
pub mod physics;
pub mod robot;
pub mod world;
pub mod engine;
pub mod command;
pub mod state;
pub mod domain_randomization;
pub mod proto;
pub mod grsim;

// Re-export main types
pub use config::{WorldConfig, RobotConfig, BallConfig, FieldConfig};
pub use engine::SimulationEngine;
pub use command::{RobotCommand, TeamCommand, WorldCommand, MoveCommand, TeleportBall, TeleportRobot};
pub use state::{WorldState, RobotState, BallState, TeamColor};
pub use world::World;
pub use domain_randomization::{DomainRandomizer, RandomizationConfig};
pub use grsim::{GrSimCompatConfig, GrSimCompatServer};
