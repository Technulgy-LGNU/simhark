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
    /// applying only hard velocity limits.
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
        current_vx: f64,
        current_vy: f64,
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

        if dt > f64::EPSILON {
            let current_speed = (current_vx * current_vx + current_vy * current_vy).sqrt();
            let target_speed = (vx * vx + vy * vy).sqrt();
            let accel_limit = if target_speed >= current_speed {
                self.acc_speedup_abs
            } else {
                self.acc_brake_abs
            };
            let max_delta_v = accel_limit * dt;
            let delta_vx = vx - current_vx;
            let delta_vy = vy - current_vy;
            let delta_speed = (delta_vx * delta_vx + delta_vy * delta_vy).sqrt();
            if delta_speed > max_delta_v && delta_speed > f64::EPSILON {
                let scale = max_delta_v / delta_speed;
                vx = current_vx + delta_vx * scale;
                vy = current_vy + delta_vy * scale;
            }

            let angular_delta = vw - current_angvel;
            let angular_limit = if angular_delta >= 0.0 {
                self.acc_speedup_ang
            } else {
                self.acc_brake_ang
            };
            let max_angular_delta = angular_limit * dt;
            if angular_delta.abs() > max_angular_delta {
                vw = current_angvel + angular_delta.signum() * max_angular_delta;
            }
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

/// Check if the ball is close enough to the chassis kicker line to count as
/// contact, mirroring Sumatra's `BallContactCalculator` geometry.
pub fn is_ball_touching_kicker(
    ball_pos: [f64; 3],
    robot_pos: [f64; 3],
    robot_dir: [f64; 2], // unit direction vector of chassis
    center_from_kicker: f64,
    robot_radius: f64,
    kicker_height: f64,
    ball_radius: f64,
    tolerance: f64,
) -> bool {
    let dx = robot_dir[0];
    let dy = robot_dir[1];
    let bx = ball_pos[0];
    let by = ball_pos[1];
    let bz = ball_pos[2];
    let kicker_center_x = robot_pos[0] + dx * center_from_kicker;
    let kicker_center_y = robot_pos[1] + dy * center_from_kicker;
    let half_width = (robot_radius * robot_radius - center_from_kicker * center_from_kicker)
        .max(0.0)
        .sqrt();
    let left_x = kicker_center_x - dy * half_width;
    let left_y = kicker_center_y + dx * half_width;
    let right_x = kicker_center_x + dy * half_width;
    let right_y = kicker_center_y - dx * half_width;

    let seg_x = right_x - left_x;
    let seg_y = right_y - left_y;
    let ball_x = bx - left_x;
    let ball_y = by - left_y;
    let seg_len_sq = seg_x * seg_x + seg_y * seg_y;
    if seg_len_sq <= f64::EPSILON {
        return false;
    }

    let t = ((ball_x * seg_x + ball_y * seg_y) / seg_len_sq).clamp(0.0, 1.0);
    let closest_x = left_x + seg_x * t;
    let closest_y = left_y + seg_y * t;
    let dist_x = bx - closest_x;
    let dist_y = by - closest_y;
    let line_distance = (dist_x * dist_x + dist_y * dist_y).sqrt();
    let zz = (robot_pos[2] - bz).abs();

    line_distance <= ball_radius + tolerance
        && zz <= kicker_height * 0.5 + ball_radius
}
