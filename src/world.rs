//! Single simulation world, combining physics + robot logic + game state.

use nalgebra::Vector3 as NVec3;

use crate::command::{MoveCommand, RobotCommand, TeleportBall, TeleportRobot, WorldCommand};
use crate::config::{WorldConfig, BALL_COLLISION_SUBSTEPS};
use crate::geometry::deg2rad;
use crate::physics::PhysicsWorld;
use crate::robot::{is_ball_touching_kicker, KickType, RobotSim};
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
        }
    }

    /// Apply commands and step the simulation forward by one time step.
    pub fn step(&mut self, commands: &WorldCommand) -> WorldState {
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

        // Set wheel motor speeds in physics
        self.sync_wheel_speeds();

        // Physics substeps (grSim does 5 substeps with ball friction each)
        for _ in 0..BALL_COLLISION_SUBSTEPS {
            self.physics.apply_ball_friction();
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

        // Extract state
        self.get_state()
    }

    /// Step without any commands (useful for free-running simulation).
    pub fn step_empty(&mut self) -> WorldState {
        self.step(&WorldCommand::default())
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
            let handle = &handle_copies[cmd.id];

            if !sim.is_on {
                continue;
            }

            // Dribbler
            sim.dribbler_on = cmd.dribbler_on;

            // Movement
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
                    MoveCommand::WheelVelocity(speeds) => {
                        sim.set_wheel_speeds(*speeds);
                    }
                }
            }

            // Kick
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

                    // Apply kick velocity to ball
                    let ball = &mut self.physics.rigid_body_set[ball_body];
                    let old_vel = *ball.linvel();

                    // Damp existing velocity component along kick direction
                    let damp = robot_cfg.kicker_damp_factor as f32;
                    let vn = -(old_vel.x * dir[0] as f32 + old_vel.y * dir[1] as f32) * damp;
                    let vt = -(old_vel.x * dir[1] as f32 - old_vel.y * dir[0] as f32);

                    let vx =
                        dir[0] as f32 * speed_xy as f32 + vn * dir[0] as f32 - vt * dir[1] as f32;
                    let vy =
                        dir[1] as f32 * speed_xy as f32 + vn * dir[1] as f32 + vt * dir[0] as f32;
                    let vz = speed_z as f32;

                    ball.set_linvel(NVec3::new(vx, vy, vz), true);

                    sim.kick_type = if speed_z >= 1.0 {
                        KickType::Chip
                    } else {
                        KickType::Flat
                    };
                    sim.kick_countdown = 10;
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
        ball.set_linvel(NVec3::new(vx, vy, vz), true);
        ball.set_angvel(NVec3::zeros(), true);
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

        let blue_robots = self.extract_robot_states(TeamColor::Blue);
        let yellow_robots = self.extract_robot_states(TeamColor::Yellow);

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
                let ball_pos = self.physics.get_body_position(self.physics.ball_body);
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
