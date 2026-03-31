//! Domain randomization for sim-to-real transfer.
//!
//! Each parallel world can have slightly different physics constants,
//! making trained policies more robust when deployed on real robots.

use rand::Rng;
use rand_chacha::ChaCha8Rng;
use rand::SeedableRng;
use serde::{Deserialize, Serialize};

use crate::config::WorldConfig;

/// Configuration for how much to randomize each parameter.
/// Each field is a fraction: 0.0 = no randomization, 0.1 = +/- 10%.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RandomizationConfig {
    // Ball
    pub ball_mass: f64,
    pub ball_friction: f64,
    pub ball_bounce: f64,
    pub ball_radius: f64,
    pub ball_damping: f64,

    // Robot
    pub robot_mass: f64,
    pub robot_radius: f64,
    pub wheel_friction: f64,
    pub wheel_motor_fmax: f64,
    pub kicker_damp_factor: f64,
    pub kicker_friction: f64,
    pub acc_limits: f64,
    pub vel_limits: f64,

    // World
    pub gravity: f64,
}

impl Default for RandomizationConfig {
    fn default() -> Self {
        Self {
            ball_mass: 0.0,
            ball_friction: 0.0,
            ball_bounce: 0.0,
            ball_radius: 0.0,
            ball_damping: 0.0,
            robot_mass: 0.0,
            robot_radius: 0.0,
            wheel_friction: 0.0,
            wheel_motor_fmax: 0.0,
            kicker_damp_factor: 0.0,
            kicker_friction: 0.0,
            acc_limits: 0.0,
            vel_limits: 0.0,
            gravity: 0.0,
        }
    }
}

impl RandomizationConfig {
    /// Moderate randomization suitable for sim-to-real training.
    pub fn moderate() -> Self {
        Self {
            ball_mass: 0.05,
            ball_friction: 0.15,
            ball_bounce: 0.1,
            ball_radius: 0.02,
            ball_damping: 0.2,
            robot_mass: 0.05,
            robot_radius: 0.01,
            wheel_friction: 0.15,
            wheel_motor_fmax: 0.1,
            kicker_damp_factor: 0.1,
            kicker_friction: 0.1,
            acc_limits: 0.1,
            vel_limits: 0.05,
            gravity: 0.005,
        }
    }

    /// Aggressive randomization for robust training.
    pub fn aggressive() -> Self {
        Self {
            ball_mass: 0.15,
            ball_friction: 0.3,
            ball_bounce: 0.25,
            ball_radius: 0.05,
            ball_damping: 0.4,
            robot_mass: 0.15,
            robot_radius: 0.03,
            wheel_friction: 0.3,
            wheel_motor_fmax: 0.2,
            kicker_damp_factor: 0.2,
            kicker_friction: 0.2,
            acc_limits: 0.2,
            vel_limits: 0.1,
            gravity: 0.01,
        }
    }
}

/// Applies domain randomization to a WorldConfig.
pub struct DomainRandomizer {
    pub randomization: RandomizationConfig,
}

impl DomainRandomizer {
    pub fn new(randomization: RandomizationConfig) -> Self {
        Self { randomization }
    }

    /// Apply randomization to create a mutated config.
    /// `world_index` is used to seed the RNG deterministically.
    pub fn randomize(&self, base: &WorldConfig, world_index: usize) -> WorldConfig {
        let mut config = base.clone();
        let mut rng = ChaCha8Rng::seed_from_u64(base.seed.wrapping_add(world_index as u64).wrapping_mul(0x9E3779B97F4A7C15));

        let r = &self.randomization;

        // Ball
        config.ball.mass = mutate(&mut rng, config.ball.mass, r.ball_mass);
        config.ball.friction = mutate(&mut rng, config.ball.friction, r.ball_friction);
        config.ball.bounce = mutate_clamped(&mut rng, config.ball.bounce, r.ball_bounce, 0.0, 1.0);
        config.ball.radius = mutate(&mut rng, config.ball.radius, r.ball_radius);
        config.ball.linear_damping = mutate(&mut rng, config.ball.linear_damping, r.ball_damping);
        config.ball.angular_damping = mutate(&mut rng, config.ball.angular_damping, r.ball_damping);

        // Blue robots
        randomize_robot_config(&mut rng, &mut config.blue_robots, r);
        // Yellow robots
        randomize_robot_config(&mut rng, &mut config.yellow_robots, r);

        // Physics
        config.physics.gravity = mutate(&mut rng, config.physics.gravity, r.gravity);

        // Update seed so this world is still deterministic but different
        config.seed = base.seed.wrapping_add(world_index as u64);

        config
    }
}

fn randomize_robot_config(rng: &mut ChaCha8Rng, cfg: &mut crate::config::RobotConfig, r: &RandomizationConfig) {
    cfg.body_mass = mutate(rng, cfg.body_mass, r.robot_mass);
    cfg.radius = mutate(rng, cfg.radius, r.robot_radius);
    cfg.wheel_tangent_friction = mutate(rng, cfg.wheel_tangent_friction, r.wheel_friction);
    cfg.wheel_perpendicular_friction = mutate(rng, cfg.wheel_perpendicular_friction, r.wheel_friction);
    cfg.wheel_motor_fmax = mutate(rng, cfg.wheel_motor_fmax, r.wheel_motor_fmax);
    cfg.kicker_damp_factor = mutate(rng, cfg.kicker_damp_factor, r.kicker_damp_factor);
    cfg.kicker_friction = mutate(rng, cfg.kicker_friction, r.kicker_friction);
    cfg.acc_speedup_absolute_max = mutate(rng, cfg.acc_speedup_absolute_max, r.acc_limits);
    cfg.acc_speedup_angular_max = mutate(rng, cfg.acc_speedup_angular_max, r.acc_limits);
    cfg.acc_brake_absolute_max = mutate(rng, cfg.acc_brake_absolute_max, r.acc_limits);
    cfg.acc_brake_angular_max = mutate(rng, cfg.acc_brake_angular_max, r.acc_limits);
    cfg.vel_absolute_max = mutate(rng, cfg.vel_absolute_max, r.vel_limits);
    cfg.vel_angular_max = mutate(rng, cfg.vel_angular_max, r.vel_limits);
}

/// Apply a uniform random mutation: value * (1.0 + uniform(-fraction, +fraction)).
fn mutate(rng: &mut ChaCha8Rng, value: f64, fraction: f64) -> f64 {
    if fraction <= 0.0 {
        return value;
    }
    let factor: f64 = 1.0 + rng.random_range(-fraction..=fraction);
    (value * factor).max(0.0)
}

/// Apply mutation clamped to [lo, hi].
fn mutate_clamped(rng: &mut ChaCha8Rng, value: f64, fraction: f64, lo: f64, hi: f64) -> f64 {
    mutate(rng, value, fraction).clamp(lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WorldConfig;

    #[test]
    fn test_no_randomization_preserves_config() {
        let base = WorldConfig::division_a();
        let randomizer = DomainRandomizer::new(RandomizationConfig::default());
        let mutated = randomizer.randomize(&base, 0);

        assert_eq!(base.ball.mass, mutated.ball.mass);
        assert_eq!(base.ball.friction, mutated.ball.friction);
        assert_eq!(base.blue_robots.body_mass, mutated.blue_robots.body_mass);
    }

    #[test]
    fn test_randomization_changes_values() {
        let base = WorldConfig::division_a();
        let randomizer = DomainRandomizer::new(RandomizationConfig::moderate());
        let mutated = randomizer.randomize(&base, 1);

        // At least some values should differ
        let any_different = base.ball.mass != mutated.ball.mass
            || base.ball.friction != mutated.ball.friction
            || base.blue_robots.body_mass != mutated.blue_robots.body_mass;
        assert!(any_different, "moderate randomization should change at least some values");
    }

    #[test]
    fn test_randomization_is_deterministic() {
        let base = WorldConfig::division_a();
        let randomizer = DomainRandomizer::new(RandomizationConfig::moderate());
        let m1 = randomizer.randomize(&base, 5);
        let m2 = randomizer.randomize(&base, 5);

        assert_eq!(m1.ball.mass, m2.ball.mass);
        assert_eq!(m1.ball.friction, m2.ball.friction);
        assert_eq!(m1.blue_robots.body_mass, m2.blue_robots.body_mass);
    }

    #[test]
    fn test_different_worlds_get_different_values() {
        let base = WorldConfig::division_a();
        let randomizer = DomainRandomizer::new(RandomizationConfig::moderate());
        let m1 = randomizer.randomize(&base, 0);
        let m2 = randomizer.randomize(&base, 1);

        let any_different = m1.ball.mass != m2.ball.mass
            || m1.ball.friction != m2.ball.friction;
        assert!(any_different, "different world indices should produce different configs");
    }

    #[test]
    fn test_values_stay_positive() {
        let base = WorldConfig::division_a();
        let randomizer = DomainRandomizer::new(RandomizationConfig::aggressive());
        for i in 0..100 {
            let m = randomizer.randomize(&base, i);
            assert!(m.ball.mass > 0.0);
            assert!(m.ball.radius > 0.0);
            assert!(m.ball.bounce >= 0.0 && m.ball.bounce <= 1.0);
            assert!(m.blue_robots.body_mass > 0.0);
            assert!(m.physics.gravity > 0.0);
        }
    }
}
