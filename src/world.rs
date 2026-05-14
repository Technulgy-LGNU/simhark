//! Single simulation world, combining physics + robot logic + game state.

use rapier3d::prelude::Vector;

use crate::command::{MoveCommand, RobotCommand, TeleportBall, TeleportRobot, WorldCommand};
use crate::config::{BALL_COLLISION_SUBSTEPS, WorldConfig};
use crate::geometry::deg2rad;
use crate::physics::PhysicsWorld;
use crate::robot::{KickType, RobotSim, is_ball_touching_kicker};
use crate::state::{BallState, KickStatus, RobotState, TeamColor, WorldState};

/// A single simulation world with its own physics and robot state.
pub struct World {
    pub id: usize,
    pub config: WorldConfig,
    pub physics: PhysicsWorld,
    pub blue_sims: Vec<RobotSim>,
    pub yellow_sims: Vec<RobotSim>,
    pub sim_time: f64,
    pub frame: u64,
    /// Current dribble holder, if any. Possession is sticky: a robot keeps
    /// the ball until it turns off its dribbler, the ball physically leaves
    /// the kicker pocket, or it kicks. This prevents the per-frame
    /// oscillation that happens when two robots are both close to the ball.
    holder: Option<(TeamColor, usize)>,
}

impl World {
    /// Create a new world from configuration.
    pub fn new(id: usize, config: WorldConfig) -> Self {
        let physics = PhysicsWorld::new(&config);

        let blue_sims = (0..config.robots_per_team)
            .map(|i| RobotSim::new(i, &config.blue_robots, 1.0))
            .collect();
        let yellow_sims = (0..config.robots_per_team)
            .map(|i| RobotSim::new(i, &config.yellow_robots, -1.0))
            .collect();

        Self {
            id,
            config,
            physics,
            blue_sims,
            yellow_sims,
            sim_time: 0.0,
            frame: 0,
            holder: None,
        }
    }

    /// Apply commands and step the simulation forward by one time step.
    pub fn step(&mut self, commands: &WorldCommand) -> WorldState {
        self.advance(commands);

        // Extract state
        self.get_state()
    }

    /// Advance the simulation by one time step without extracting a state snapshot.
    pub fn advance(&mut self, commands: &WorldCommand) {
        // Apply teleportation commands
        if let Some(ref tb) = commands.teleport_ball {
            self.teleport_ball(tb);
        }
        for tr in &commands.teleport_robots {
            self.teleport_robot(tr);
        }

        // Apply robot commands
        self.apply_robot_commands(&commands.blue, TeamColor::Blue);
        self.apply_robot_commands(&commands.yellow, TeamColor::Yellow);

        // Push wheel speeds into the physics engine (just storage; the
        // forces are applied per-substep below).
        self.sync_wheel_speeds();

        // Physics substeps (grSim does 5 substeps with ball friction each).
        // Drive forces and dribble pull are re-applied EVERY substep so that
        // contact forces between robots can actually accumulate against
        // them. The previous design set linvel only at the start of the
        // frame, so contacts from the 4 inner substeps were silently
        // overwritten and robot clusters froze rigid around the ball.
        let dt_substep = self.physics.substep_dt();
        for _ in 0..BALL_COLLISION_SUBSTEPS {
            self.physics.apply_ball_friction();
            self.physics.apply_drive_forces(dt_substep);
            self.apply_dribbler_control();
            self.physics.substep();
        }

        // Step robot game logic (kicker countdown, etc.)
        for sim in &mut self.blue_sims {
            sim.step_kicker();
        }
        for sim in &mut self.yellow_sims {
            sim.step_kicker();
        }

        self.sim_time += self.config.physics.delta_time;
        self.frame += 1;
    }

    /// Step without any commands (useful for free-running simulation).
    pub fn step_empty(&mut self) -> WorldState {
        self.step(&WorldCommand::default())
    }

    /// Advance without commands and without extracting a state snapshot.
    pub fn advance_empty(&mut self) {
        self.advance(&WorldCommand::default())
    }

    fn apply_robot_commands(&mut self, commands: &[RobotCommand], team: TeamColor) {
        // Clone handles out to avoid borrow conflict with self.physics
        let (robot_cfg, handle_copies) = match team {
            TeamColor::Blue => (
                self.config.blue_robots.clone(),
                self.physics.blue_robots.clone(),
            ),
            TeamColor::Yellow => (
                self.config.yellow_robots.clone(),
                self.physics.yellow_robots.clone(),
            ),
        };
        let dt = self.config.physics.delta_time;
        let ball_radius = self.config.ball.radius;

        let sims = match team {
            TeamColor::Blue => &mut self.blue_sims,
            TeamColor::Yellow => &mut self.yellow_sims,
        };

        for cmd in commands {
            if cmd.id >= sims.len() {
                continue;
            }

            let sim = &mut sims[cmd.id];
            if !sim.is_on {
                continue;
            }

            let handle = &handle_copies[cmd.id];

            sim.dribbler_on = cmd.dribbler_on;

            if let Some(ref mc) = cmd.move_command {
                let linvel = self.physics.get_body_linvel(handle.chassis_body);
                let angvel = self.physics.get_body_angvel(handle.chassis_body);
                let current_speed = (linvel.x * linvel.x + linvel.y * linvel.y).sqrt() as f64;
                let current_angvel = angvel.z as f64;

                match mc {
                    MoveCommand::LocalVelocity {
                        forward,
                        left,
                        angular,
                    } => {
                        sim.set_local_velocity(
                            *forward,
                            *left,
                            *angular,
                            current_speed,
                            current_angvel,
                            dt,
                        );
                    }
                    MoveCommand::GlobalVelocity { vx, vy, angular } => {
                        let yaw = self.physics.get_body_yaw(handle.chassis_body) as f64;
                        let local_vx = vx * (-yaw).cos() - vy * (-yaw).sin();
                        let local_vy = vy * (-yaw).cos() + vx * (-yaw).sin();
                        sim.set_local_velocity(
                            local_vx,
                            local_vy,
                            *angular,
                            current_speed,
                            current_angvel,
                            dt,
                        );
                    }
                    MoveCommand::WheelVelocity(speeds) => sim.set_wheel_speeds(*speeds),
                }
            }

            if cmd.kick_speed > 0.0001 {
                let ball_body = self.physics.ball_body;
                let ball_pos = self.physics.get_body_position(ball_body);
                let kicker_pos = self.physics.get_body_position(handle.kicker_body);
                let yaw = self.physics.get_body_yaw(handle.chassis_body) as f64;
                let dir = [yaw.cos(), yaw.sin()];

                let touching = is_ball_touching_kicker(
                    [ball_pos.x as f64, ball_pos.y as f64, ball_pos.z as f64],
                    [
                        kicker_pos.x as f64,
                        kicker_pos.y as f64,
                        kicker_pos.z as f64,
                    ],
                    dir,
                    robot_cfg.kicker_thickness,
                    robot_cfg.kicker_width,
                    robot_cfg.kicker_height,
                    ball_radius,
                );

                if touching {
                    let mut kick_speed = cmd.kick_speed;
                    let limit = if cmd.kick_angle > 0.0 {
                        robot_cfg.max_chip_kick_speed
                    } else {
                        robot_cfg.max_linear_kick_speed
                    };
                    if kick_speed > limit {
                        kick_speed = limit;
                    }

                    let kick_angle_rad = deg2rad(cmd.kick_angle);
                    let speed_xy = kick_angle_rad.cos() * kick_speed;
                    let speed_z = kick_angle_rad.sin() * kick_speed;

                    let ball = &mut self.physics.rigid_body_set[ball_body];
                    let old_vel = ball.linvel();

                    let damp = robot_cfg.kicker_damp_factor as f32;
                    let vn = -(old_vel.x * dir[0] as f32 + old_vel.y * dir[1] as f32) * damp;
                    let vt = -(old_vel.x * dir[1] as f32 - old_vel.y * dir[0] as f32);

                    let vx =
                        dir[0] as f32 * speed_xy as f32 + vn * dir[0] as f32 - vt * dir[1] as f32;
                    let vy =
                        dir[1] as f32 * speed_xy as f32 + vn * dir[1] as f32 + vt * dir[0] as f32;
                    let vz = speed_z as f32;

                    ball.set_linvel(Vector::new(vx, vy, vz), true);

                    sim.kick_type = if speed_z >= 1.0 {
                        KickType::Chip
                    } else {
                        KickType::Flat
                    };
                    sim.kick_countdown = 10;

                    // Kicking releases possession — otherwise the dribble
                    // pull would yank the ball straight back next frame.
                    if self.holder == Some((team, cmd.id)) {
                        self.holder = None;
                    }
                }
            }
        }
    }

    fn sync_wheel_speeds(&mut self) {
        // Clone handles to avoid borrow conflict
        let blue_handles = self.physics.blue_robots.clone();
        for (sim, handle) in self.blue_sims.iter().zip(blue_handles.iter()) {
            if sim.is_on {
                for i in 0..4 {
                    self.physics
                        .set_wheel_speed(handle, i, sim.wheel_speeds[i] as f32);
                }
            }
        }

        let yellow_handles = self.physics.yellow_robots.clone();
        for (sim, handle) in self.yellow_sims.iter().zip(yellow_handles.iter()) {
            if sim.is_on {
                for i in 0..4 {
                    self.physics
                        .set_wheel_speed(handle, i, sim.wheel_speeds[i] as f32);
                }
            }
        }
    }

    fn apply_dribbler_control(&mut self) {
        let ball_body = self.physics.ball_body;
        let ball_pos = self.physics.get_body_position(ball_body);

        for sim in &mut self.blue_sims {
            sim.holding_ball = false;
        }
        for sim in &mut self.yellow_sims {
            sim.holding_ball = false;
        }

        // Validate the existing holder. They keep possession until their
        // dribbler turns off or the ball falls out of the kicker pocket.
        // (Kicks clear the holder back in apply_robot_commands.)
        if let Some((team, idx)) = self.holder {
            if !self.is_robot_holding(team, idx, ball_pos) {
                self.holder = None;
            }
        }

        // No holder? Promote the closest valid candidate. Possession is
        // sticky from this point forward, so two robots fighting for the
        // ball can't ping-pong it back and forth every frame.
        if self.holder.is_none() {
            let blue = self.find_team_holder_candidate(TeamColor::Blue, ball_pos);
            let yellow = self.find_team_holder_candidate(TeamColor::Yellow, ball_pos);
            self.holder = match (blue, yellow) {
                (Some(b), Some(y)) => Some(if b.1 <= y.1 { (TeamColor::Blue, b.0) } else { (TeamColor::Yellow, y.0) }),
                (Some(b), None) => Some((TeamColor::Blue, b.0)),
                (None, Some(y)) => Some((TeamColor::Yellow, y.0)),
                (None, None) => None,
            };
        }

        let Some((team, idx)) = self.holder else {
            return;
        };

        match team {
            TeamColor::Blue => self.blue_sims[idx].holding_ball = true,
            TeamColor::Yellow => self.yellow_sims[idx].holding_ball = true,
        }

        let (handle, robot_cfg) = match team {
            TeamColor::Blue => (&self.physics.blue_robots[idx], &self.config.blue_robots),
            TeamColor::Yellow => (&self.physics.yellow_robots[idx], &self.config.yellow_robots),
        };

        let chassis_pos = self.physics.get_body_position(handle.chassis_body);
        let chassis_vel = self.physics.get_body_linvel(handle.chassis_body);
        let yaw = self.physics.get_body_yaw(handle.chassis_body) as f64;
        let dir = [yaw.cos(), yaw.sin()];

        // Target: ball edge just kissing the kicker FRONT face (no
        // penetration). The previous version aimed at a position inside the
        // kicker box, so the PD pull and the kicker collider would fight
        // and pump energy into the ball — launching it across the field.
        let kicker_face_offset =
            robot_cfg.center_from_kicker + robot_cfg.kicker_thickness * 1.5;
        let front_offset = kicker_face_offset + self.config.ball.radius + 0.001;
        let target_x = chassis_pos.x as f64 + dir[0] * front_offset;
        let target_y = chassis_pos.y as f64 + dir[1] * front_offset;

        // Critically-damped horizontal PD pull. No z component — gravity and
        // the ground plane handle the vertical axis cleanly, and any pull
        // there just fights the contact and bounces the ball.
        let mass = self.physics.ball_mass();
        let kp = 40.0_f32;
        let kd = 2.0 * (kp / 1.0).sqrt() * 1.05;
        let ball = &mut self.physics.rigid_body_set[ball_body];
        let current_pos = ball.translation();
        let current_vel = ball.linvel();
        let dx = target_x as f32 - current_pos.x;
        let dy = target_y as f32 - current_pos.y;
        let dvx = chassis_vel.x - current_vel.x;
        let dvy = chassis_vel.y - current_vel.y;
        let force = Vector::new(
            mass * (kp * dx + kd * dvx),
            mass * (kp * dy + kd * dvy),
            0.0,
        );
        ball.add_force(force, true);
    }

    fn find_team_holder_candidate(
        &self,
        team: TeamColor,
        ball_pos: rapier3d::na::Vector3<f32>,
    ) -> Option<(usize, f64)> {
        let (sims, handles, robot_cfg) = match team {
            TeamColor::Blue => (
                &self.blue_sims,
                &self.physics.blue_robots,
                &self.config.blue_robots,
            ),
            TeamColor::Yellow => (
                &self.yellow_sims,
                &self.physics.yellow_robots,
                &self.config.yellow_robots,
            ),
        };

        sims.iter()
            .zip(handles.iter())
            .enumerate()
            .filter(|(_, (sim, _))| sim.is_on && sim.dribbler_on)
            .filter_map(|(index, (_, handle))| {
                let kicker_pos = self.physics.get_body_position(handle.kicker_body);
                let yaw = self.physics.get_body_yaw(handle.chassis_body) as f64;
                let dir = [yaw.cos(), yaw.sin()];
                let touching = is_ball_touching_kicker(
                    [ball_pos.x as f64, ball_pos.y as f64, ball_pos.z as f64],
                    [
                        kicker_pos.x as f64,
                        kicker_pos.y as f64,
                        kicker_pos.z as f64,
                    ],
                    dir,
                    robot_cfg.kicker_thickness,
                    robot_cfg.kicker_width,
                    robot_cfg.kicker_height,
                    self.config.ball.radius,
                );
                if !touching {
                    return None;
                }
                let dx = ball_pos.x as f64 - kicker_pos.x as f64;
                let dy = ball_pos.y as f64 - kicker_pos.y as f64;
                Some((index, dx * dx + dy * dy))
            })
            .min_by(|(_, left), (_, right)| left.total_cmp(right))
    }

    fn is_robot_holding(
        &self,
        team: TeamColor,
        idx: usize,
        ball_pos: rapier3d::na::Vector3<f32>,
    ) -> bool {
        let (sims, handles, robot_cfg) = match team {
            TeamColor::Blue => (
                &self.blue_sims,
                &self.physics.blue_robots,
                &self.config.blue_robots,
            ),
            TeamColor::Yellow => (
                &self.yellow_sims,
                &self.physics.yellow_robots,
                &self.config.yellow_robots,
            ),
        };
        let (Some(sim), Some(handle)) = (sims.get(idx), handles.get(idx)) else {
            return false;
        };
        if !sim.is_on || !sim.dribbler_on {
            return false;
        }
        let kicker_pos = self.physics.get_body_position(handle.kicker_body);
        let yaw = self.physics.get_body_yaw(handle.chassis_body) as f64;
        let dir = [yaw.cos(), yaw.sin()];
        is_ball_touching_kicker(
            [ball_pos.x as f64, ball_pos.y as f64, ball_pos.z as f64],
            [
                kicker_pos.x as f64,
                kicker_pos.y as f64,
                kicker_pos.z as f64,
            ],
            dir,
            robot_cfg.kicker_thickness,
            robot_cfg.kicker_width,
            robot_cfg.kicker_height,
            self.config.ball.radius,
        )
    }

    fn teleport_ball(&mut self, tb: &TeleportBall) {
        let ball_body = self.physics.ball_body;
        let pos = self.physics.get_body_position(ball_body);
        let vel = self.physics.get_body_linvel(ball_body);

        let x = tb.x.unwrap_or(pos.x as f64) as f32;
        let y = tb.y.unwrap_or(pos.y as f64) as f32;
        let z =
            tb.z.map(|z| (self.config.ball.radius + 0.005 + z) as f32)
                .unwrap_or(pos.z);
        let vx = tb.vx.unwrap_or(vel.x as f64) as f32;
        let vy = tb.vy.unwrap_or(vel.y as f64) as f32;
        let vz = tb.vz.unwrap_or(vel.z as f64) as f32;

        self.physics.teleport_body(ball_body, x, y, z);
        let ball = &mut self.physics.rigid_body_set[ball_body];
        ball.set_linvel(Vector::new(vx, vy, vz), true);
        ball.set_angvel(Vector::ZERO, true);
        // The ball just got moved out from under any holder.
        self.holder = None;
    }

    fn teleport_robot(&mut self, tr: &TeleportRobot) {
        let robot_cfg = match tr.team {
            TeamColor::Blue => &self.config.blue_robots,
            TeamColor::Yellow => &self.config.yellow_robots,
        };
        let start_z = robot_cfg.start_z() as f32;

        // Clone handle to avoid borrow conflict
        let handle = match tr.team {
            TeamColor::Blue => {
                if tr.id >= self.physics.blue_robots.len() {
                    return;
                }
                self.physics.blue_robots[tr.id].clone()
            }
            TeamColor::Yellow => {
                if tr.id >= self.physics.yellow_robots.len() {
                    return;
                }
                self.physics.yellow_robots[tr.id].clone()
            }
        };

        let sims = match tr.team {
            TeamColor::Blue => &mut self.blue_sims,
            TeamColor::Yellow => &mut self.yellow_sims,
        };
        if tr.id >= sims.len() {
            return;
        }
        let sim = &mut sims[tr.id];

        let pos = self.physics.get_body_position(handle.chassis_body);
        let x = tr.x.unwrap_or(pos.x as f64) as f32;
        let y = tr.y.unwrap_or(pos.y as f64) as f32;

        self.physics
            .teleport_body(handle.chassis_body, x, y, start_z);
        self.physics.reset_body_velocity(handle.chassis_body);

        if let Some(orientation) = tr.orientation {
            self.physics
                .set_body_yaw(handle.chassis_body, orientation as f32);
        }

        if let Some(present) = tr.present {
            sim.is_on = present;
            if !present {
                let off_x = 1e6 * tr.id as f32;
                let off_y = 1e6 * tr.team as u8 as f32;
                self.physics
                    .teleport_body(handle.chassis_body, off_x, off_y, start_z);
            }
        }

        sim.reset_speeds();
    }

    /// Extract the current state snapshot.
    pub fn get_state(&self) -> WorldState {
        let ball_pos = self.physics.get_body_position(self.physics.ball_body);
        let ball_vel = self.physics.get_body_linvel(self.physics.ball_body);

        let ball = BallState {
            x: ball_pos.x as f64,
            y: ball_pos.y as f64,
            z: ball_pos.z as f64,
            vx: ball_vel.x as f64,
            vy: ball_vel.y as f64,
            vz: ball_vel.z as f64,
        };

        let mut blue_robots = self.extract_robot_states(TeamColor::Blue);
        let mut yellow_robots = self.extract_robot_states(TeamColor::Yellow);

        // The geometric IR check (matching grSim's `isTouchingBall`) is wide
        // enough that two robots facing each other across the ball can both
        // light up at once. The dribbler logic already snaps the ball to the
        // single closest dribbling robot, so two simultaneous "I have the
        // ball" reports are a lie — and downstream AIs (Sumatra in
        // particular) get stuck because both teams act on it. Squash all
        // candidate IRs down to the single robot whose kicker is closest to
        // the ball.
        keep_only_closest_infrared(&ball_pos, &mut blue_robots, &mut yellow_robots);

        // Simple goal detection: ball crossed goal line
        let half_length = self.config.field.field_length / 2.0;
        let half_goal_width = self.config.field.goal_width / 2.0;
        let goal_blue =
            ball_pos.x as f64 > half_length && (ball_pos.y as f64).abs() < half_goal_width;
        let goal_yellow =
            (ball_pos.x as f64) < -half_length && (ball_pos.y as f64).abs() < half_goal_width;

        WorldState {
            world_id: self.id,
            sim_time: self.sim_time,
            frame: self.frame,
            ball,
            blue_robots,
            yellow_robots,
            goal_blue,
            goal_yellow,
        }
    }

    fn extract_robot_states(&self, team: TeamColor) -> Vec<RobotState> {
        let ball_pos = self.physics.get_body_position(self.physics.ball_body);
        let (sims, handles, robot_cfg) = match team {
            TeamColor::Blue => (
                &self.blue_sims,
                &self.physics.blue_robots,
                &self.config.blue_robots,
            ),
            TeamColor::Yellow => (
                &self.yellow_sims,
                &self.physics.yellow_robots,
                &self.config.yellow_robots,
            ),
        };

        sims.iter()
            .zip(handles.iter())
            .map(|(sim, handle)| {
                let pos = self.physics.get_body_position(handle.chassis_body);
                let vel = self.physics.get_body_linvel(handle.chassis_body);
                let angvel = self.physics.get_body_angvel(handle.chassis_body);
                let yaw = self.physics.get_body_yaw(handle.chassis_body);

                // Check infrared (ball near kicker)
                let kicker_pos = self.physics.get_body_position(handle.kicker_body);
                let dir = [yaw.cos() as f64, yaw.sin() as f64];
                let infrared = is_ball_touching_kicker(
                    [ball_pos.x as f64, ball_pos.y as f64, ball_pos.z as f64],
                    [
                        kicker_pos.x as f64,
                        kicker_pos.y as f64,
                        kicker_pos.z as f64,
                    ],
                    dir,
                    robot_cfg.kicker_thickness,
                    robot_cfg.kicker_width,
                    robot_cfg.kicker_height,
                    self.config.ball.radius,
                );

                let kick_status = match sim.kick_type {
                    KickType::None => KickStatus::NoKick,
                    KickType::Flat => KickStatus::FlatKick,
                    KickType::Chip => KickStatus::ChipKick,
                };

                RobotState {
                    id: sim.id,
                    team,
                    x: pos.x as f64,
                    y: pos.y as f64,
                    z: pos.z as f64,
                    orientation: yaw as f64,
                    vx: vel.x as f64,
                    vy: vel.y as f64,
                    vz: vel.z as f64,
                    v_angular: angvel.z as f64,
                    infrared,
                    kick_status,
                    is_on: sim.is_on,
                    wheel_speeds: sim.wheel_speeds,
                }
            })
            .collect()
    }

    /// Reset the world to initial state.
    pub fn reset(&mut self) {
        *self = World::new(self.id, self.config.clone());
    }

    /// Rebuild the world with a new configuration.
    pub fn reconfigure(&mut self, config: WorldConfig) {
        *self = World::new(self.id, config);
    }
}

/// Among all robots whose IR currently fires, keep only the one whose kicker
/// front face is closest to the ball; clear the rest. If no robot's IR was
/// firing, this is a no-op.
fn keep_only_closest_infrared(
    ball_pos: &rapier3d::na::Vector3<f32>,
    blue: &mut [RobotState],
    yellow: &mut [RobotState],
) {
    let bx = ball_pos.x as f64;
    let by = ball_pos.y as f64;
    let mut winner: Option<(TeamColor, usize, f64)> = None;

    let mut consider = |team: TeamColor, robots: &[RobotState]| {
        for (index, robot) in robots.iter().enumerate() {
            if !robot.infrared {
                continue;
            }
            // Distance from ball to a point in front of the robot (where
            // the kicker face sits). We don't need the kicker's full pose
            // here — direction × forward-offset is enough to pick a winner.
            let fx = robot.x + robot.orientation.cos() * 0.078;
            let fy = robot.y + robot.orientation.sin() * 0.078;
            let dx = bx - fx;
            let dy = by - fy;
            let d2 = dx * dx + dy * dy;
            match winner {
                Some((_, _, best)) if best <= d2 => {}
                _ => winner = Some((team, index, d2)),
            }
        }
    };
    consider(TeamColor::Blue, blue);
    consider(TeamColor::Yellow, yellow);

    let Some((winning_team, winning_index, _)) = winner else {
        return;
    };

    for (index, robot) in blue.iter_mut().enumerate() {
        if !(winning_team == TeamColor::Blue && winning_index == index) {
            robot.infrared = false;
        }
    }
    for (index, robot) in yellow.iter_mut().enumerate() {
        if !(winning_team == TeamColor::Yellow && winning_index == index) {
            robot.infrared = false;
        }
    }
}
