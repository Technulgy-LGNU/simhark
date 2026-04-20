//! Robot simulation logic, mirroring grSim's Robot class.
//!
//! Handles velocity conversion (local/global → wheel speeds), acceleration
//! limiting, kicker and dribbler logic.

use crate::config::RobotConfig;
use crate::geometry::deg2rad;

/// Per-robot mutable simulation state (not physics handles, but game state).
#[derive(Debug, Clone)]
pub struct RobotSim {
    pub id: usize,
    pub is_on: bool,
    pub wheel_speeds: [f64; 4],
    pub dribbler_on: bool,
    pub kick_countdown: i32,
    pub kick_type: KickType,
    pub holding_ball: bool,
    first_time: bool,
    dir_sign: f64,
    // Cached limits from config
    acc_speedup_abs: f64,
    acc_speedup_ang: f64,
    acc_brake_abs: f64,
    acc_brake_ang: f64,
    vel_abs_max: f64,
    vel_ang_max: f64,
    wheel_angles_rad: [f64; 4],
    wheel_radius: f64,
    robot_radius: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KickType {
    None,
    Flat,
    Chip,
}

impl RobotSim {
    pub fn new(id: usize, config: &RobotConfig, dir_sign: f64) -> Self {
        Self {
            id,
            is_on: true,
            wheel_speeds: [0.0; 4],
            dribbler_on: false,
            kick_countdown: 0,
            kick_type: KickType::None,
            holding_ball: false,
            first_time: true,
            dir_sign,
            acc_speedup_abs: config.acc_speedup_absolute_max,
            acc_speedup_ang: config.acc_speedup_angular_max,
            acc_brake_abs: config.acc_brake_absolute_max,
            acc_brake_ang: config.acc_brake_angular_max,
            vel_abs_max: config.vel_absolute_max,
            vel_ang_max: config.vel_angular_max,
            wheel_angles_rad: [
                deg2rad(config.wheel_angles[0]),
                deg2rad(config.wheel_angles[1]),
                deg2rad(config.wheel_angles[2]),
                deg2rad(config.wheel_angles[3]),
            ],
            wheel_radius: config.wheel_radius,
            robot_radius: config.radius,
        }
    }

    /// Convert local velocity (vx=forward, vy=left, vw=angular) to wheel speeds,
    /// applying acceleration and velocity limits.
    /// `current_speed` is the current robot speed (linear, m/s).
    /// `current_angular` is the current angular velocity (rad/s).
    /// `dt` is the time step.
    ///
    /// This mirrors grSim's Robot::setSpeed(vx, vy, vw).
    pub fn set_local_velocity(
        &mut self,
        mut vx: f64,
        mut vy: f64,
        mut vw: f64,
        current_speed: f64,
        current_angvel: f64,
        dt: f64,
    ) {
        // Clamp velocity
        let v = (vx * vx + vy * vy).sqrt();
        if v > self.vel_abs_max {
            let scale = self.vel_abs_max / v;
            vx *= scale;
            vy *= scale;
        }
        if vw.abs() > self.vel_ang_max {
            vw = vw.signum() * self.vel_ang_max;
        }

        // Apply acceleration limits (linear)
        let target_v = (vx * vx + vy * vy).sqrt();
        let a = (target_v - current_speed) / dt / 2.0;
        let a_limit = if a > 0.0 {
            self.acc_speedup_abs
        } else {
            self.acc_brake_abs
        };
        if a.abs() > a_limit {
            let clamped_a = a.signum() * a_limit;
            let new_v = current_speed + clamped_a * dt * 2.0;
            if target_v > 0.0 {
                let scale = new_v / target_v;
                vx *= scale;
                vy *= scale;
            }
        }

        // Apply acceleration limits (angular)
        let aw = (vw - current_angvel) / dt / 2.0;
        let aw_limit = if aw > 0.0 {
            self.acc_speedup_ang
        } else {
            self.acc_brake_ang
        };
        if aw.abs() > aw_limit {
            let clamped_aw = aw.signum() * aw_limit;
            vw = current_angvel + clamped_aw * dt * 2.0;
        }

        // Convert to wheel speeds (grSim's inverse kinematics)
        for i in 0..4 {
            let alpha = self.wheel_angles_rad[i];
            self.wheel_speeds[i] = (1.0 / self.wheel_radius)
                * (self.robot_radius * vw - vx * alpha.sin() + vy * alpha.cos());
        }
    }

    /// Set individual wheel speeds directly.
    pub fn set_wheel_speeds(&mut self, speeds: [f64; 4]) {
        self.wheel_speeds = speeds;
    }

    /// Reset all speeds to zero.
    pub fn reset_speeds(&mut self) {
        self.wheel_speeds = [0.0; 4];
    }

    /// Step kicker countdown.
    pub fn step_kicker(&mut self) {
        if self.kick_countdown > 0 {
            self.kick_countdown -= 1;
            if self.kick_countdown <= 0 {
                self.kick_type = KickType::None;
            }
        }
    }

    /// Whether this is the first simulation step (for initial orientation).
    pub fn consume_first_time(&mut self) -> bool {
        if self.first_time {
            self.first_time = false;
            true
        } else {
            false
        }
    }

    pub fn initial_dir_deg(&self) -> f64 {
        if self.dir_sign < 0.0 { 180.0 } else { 0.0 }
    }
}

/// Check if the ball is touching a kicker, similar to grSim's isTouchingBall.
pub fn is_ball_touching_kicker(
    ball_pos: [f64; 3],
    kicker_pos: [f64; 3],
    robot_dir: [f64; 2], // unit direction vector of chassis
    kicker_thickness: f64,
    kicker_width: f64,
    kicker_height: f64,
    ball_radius: f64,
) -> bool {
    let dx = robot_dir[0];
    let dy = robot_dir[1];
    let kx = kicker_pos[0] + dx * kicker_thickness * 0.5;
    let ky = kicker_pos[1] + dy * kicker_thickness * 0.5;

    let bx = ball_pos[0];
    let by = ball_pos[1];
    let bz = ball_pos[2];

    let xx = ((kx - bx) * dx + (ky - by) * dy).abs();
    let yy = (-(kx - bx) * dy + (ky - by) * dx).abs();
    let zz = (kicker_pos[2] - bz).abs();

    xx < kicker_thickness * 2.0 + ball_radius && yy < kicker_width * 0.5 && zz < kicker_height * 0.5
}
