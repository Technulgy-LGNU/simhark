//! Physics simulation layer using rapier3d.
//!
//! Uses a simplified robot model: each robot is a single cylinder body.
//! Wheel kinematics are computed analytically and applied as forces/torques
//! to the chassis. This avoids the instability of modeling individual wheel
//! bodies while maintaining behavioral fidelity to grSim's movement model.

use nalgebra::{Isometry3, Point3, SMatrix, SVector, Unit, UnitQuaternion, Vector3 as NVec3};
use rapier3d::prelude::*;

use crate::config::{BALL_COLLISION_SUBSTEPS, RobotConfig, WALL_COUNT, WHEEL_COUNT, WorldConfig};
use crate::geometry::deg2rad;

/// Handle indices for one robot in the physics world.
#[derive(Debug, Clone)]
pub struct RobotHandles {
    pub chassis_body: RigidBodyHandle,
    pub chassis_collider: ColliderHandle,
    pub kicker_body: RigidBodyHandle,
    pub kicker_collider: ColliderHandle,
    pub kicker_joint: ImpulseJointHandle,
}

#[derive(Clone)]
struct DriveKinematics {
    wheel_radius: f32,
    body_from_wheels: SMatrix<f32, 3, WHEEL_COUNT>,
}

impl DriveKinematics {
    fn new(robot_cfg: &RobotConfig) -> Self {
        let mut wheel_model = [0.0; WHEEL_COUNT * 3];

        for (i, angle_deg) in robot_cfg.wheel_angles.iter().enumerate() {
            let angle = deg2rad(*angle_deg) as f32;
            wheel_model[i * 3] = -angle.sin();
            wheel_model[i * 3 + 1] = angle.cos();
            wheel_model[i * 3 + 2] = robot_cfg.radius as f32;
        }

        let wheel_model = SMatrix::<f32, WHEEL_COUNT, 3>::from_row_slice(&wheel_model);
        let body_from_wheels = (wheel_model.transpose() * wheel_model)
            .try_inverse()
            .expect("robot wheel model should be invertible")
            * wheel_model.transpose();

        Self {
            wheel_radius: robot_cfg.wheel_radius as f32,
            body_from_wheels,
        }
    }

    fn body_velocity(&self, wheel_speeds: [f32; WHEEL_COUNT]) -> (f32, f32, f32) {
        let wheel_speeds = SVector::<f32, WHEEL_COUNT>::from_row_slice(&wheel_speeds);
        let body_velocity = self.body_from_wheels * (wheel_speeds * self.wheel_radius);
        (body_velocity[0], body_velocity[1], body_velocity[2])
    }
}

/// The complete physics simulation for one world.
pub struct PhysicsWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub gravity: NVec3<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,

    pub ball_body: RigidBodyHandle,
    pub ball_collider: ColliderHandle,
    pub blue_robots: Vec<RobotHandles>,
    pub yellow_robots: Vec<RobotHandles>,
    pub wall_colliders: Vec<ColliderHandle>,
    pub ground_collider: ColliderHandle,
    blue_drive: DriveKinematics,
    yellow_drive: DriveKinematics,
    blue_wheel_speeds: Vec<[f32; WHEEL_COUNT]>,
    yellow_wheel_speeds: Vec<[f32; WHEEL_COUNT]>,

    // Cached config values
    ball_friction: f32,
    ball_mass: f32,
    ball_radius: f32,
    gravity_val: f32,
}

impl PhysicsWorld {
    /// Create a new physics world from configuration.
    pub fn new(config: &WorldConfig) -> Self {
        let gravity = NVec3::new(0.0, 0.0, -config.physics.gravity as f32);
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let mut impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();

        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt =
            (config.physics.delta_time / BALL_COLLISION_SUBSTEPS as f64) as f32;

        // Ground plane
        let ground_body = rigid_body_set.insert(RigidBodyBuilder::fixed().build());
        let ground_collider = collider_set.insert_with_parent(
            ColliderBuilder::halfspace(Unit::new_unchecked(Vector::new(0.0, 0.0, 1.0)))
                .friction(0.5)
                .restitution(0.0)
                .build(),
            ground_body,
            &mut rigid_body_set,
        );

        // Ball
        let ball_pos = NVec3::new(0.0, 0.0, config.ball.radius as f32 * 1.2);
        let ball_rb = RigidBodyBuilder::dynamic()
            .translation(to_rapier_vec(ball_pos))
            .linear_damping(config.ball.linear_damping as f32)
            .angular_damping(config.ball.angular_damping as f32)
            .ccd_enabled(true)
            .build();
        let ball_body = rigid_body_set.insert(ball_rb);
        let ball_collider = collider_set.insert_with_parent(
            ColliderBuilder::ball(config.ball.radius as f32)
                .density(
                    config.ball.mass as f32
                        / (4.0 / 3.0 * std::f32::consts::PI * (config.ball.radius as f32).powi(3)),
                )
                .friction(config.ball.friction as f32)
                .restitution(config.ball.bounce as f32)
                .restitution_combine_rule(CoefficientCombineRule::Max)
                .build(),
            ball_body,
            &mut rigid_body_set,
        );

        // Walls
        let wall_colliders = Self::create_walls(config, &mut rigid_body_set, &mut collider_set);

        // Robots
        let blue_robots = (0..config.robots_per_team)
            .map(|i| {
                let (x, y) = default_blue_position(i, config);
                Self::create_robot(
                    &config.blue_robots,
                    x,
                    y,
                    std::f64::consts::PI,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut impulse_joint_set,
                )
            })
            .collect();

        let yellow_robots = (0..config.robots_per_team)
            .map(|i| {
                let (x, y) = default_yellow_position(i, config);
                Self::create_robot(
                    &config.yellow_robots,
                    x,
                    y,
                    0.0,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut impulse_joint_set,
                )
            })
            .collect();

        Self {
            rigid_body_set,
            collider_set,
            gravity,
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver: CCDSolver::new(),
            ball_body,
            ball_collider,
            blue_robots,
            yellow_robots,
            wall_colliders,
            ground_collider,
            blue_drive: DriveKinematics::new(&config.blue_robots),
            yellow_drive: DriveKinematics::new(&config.yellow_robots),
            blue_wheel_speeds: vec![[0.0; WHEEL_COUNT]; config.robots_per_team],
            yellow_wheel_speeds: vec![[0.0; WHEEL_COUNT]; config.robots_per_team],
            ball_friction: config.ball.friction as f32,
            ball_mass: config.ball.mass as f32,
            ball_radius: config.ball.radius as f32,
            gravity_val: config.physics.gravity as f32,
        }
    }

    fn create_walls(
        config: &WorldConfig,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Vec<ColliderHandle> {
        let f = &config.field;
        let thick = f.wall_thickness;
        let inc_x = f.margin_goal_line + f.referee_margin + thick / 2.0;
        let inc_y = f.margin_touch_line + f.referee_margin + thick / 2.0;
        let pos_x = f.field_length / 2.0 + inc_x;
        let pos_y = f.field_width / 2.0 + inc_y;
        let siz_x = 2.0 * pos_x;
        let siz_y = 2.0 * pos_y;
        let siz_z = 0.4;

        let mut walls = Vec::with_capacity(WALL_COUNT);
        let wall_bounce = config.ball.bounce as f32;

        let mut add_wall = |cx: f64, cy: f64, cz: f64, hx: f64, hy: f64, hz: f64| {
            let body = bodies.insert(
                RigidBodyBuilder::fixed()
                    .translation(Vector::new(cx as f32, cy as f32, cz as f32))
                    .build(),
            );
            let ch = colliders.insert_with_parent(
                ColliderBuilder::cuboid(hx as f32 / 2.0, hy as f32 / 2.0, hz as f32 / 2.0)
                    .friction(1.0)
                    .restitution(wall_bounce)
                    .build(),
                body,
                bodies,
            );
            walls.push(ch);
        };

        // Bounding walls [0..3]
        add_wall(thick / 2.0, pos_y, 0.0, siz_x, thick, siz_z);
        add_wall(-thick / 2.0, -pos_y, 0.0, siz_x, thick, siz_z);
        add_wall(pos_x, -thick / 2.0, 0.0, thick, siz_y, siz_z);
        add_wall(-pos_x, thick / 2.0, 0.0, thick, siz_y, siz_z);

        // Goal walls [4..9]
        let gthick = f.goal_thickness;
        let gpos_x = (f.field_length + gthick) / 2.0 + f.goal_depth;
        let gpos_y = (f.goal_width + gthick) / 2.0;
        let gpos_z = f.goal_height / 2.0;
        let gsiz_x = f.margin_goal_line + f.referee_margin;
        let gsiz_z = f.goal_height;

        add_wall(gpos_x, 0.0, gpos_z, gthick, f.goal_width, gsiz_z);
        let gpos2_x = (f.field_length + gsiz_x) / 2.0;
        add_wall(gpos2_x, -gpos_y, gpos_z, gsiz_x, gthick, gsiz_z);
        add_wall(gpos2_x, gpos_y, gpos_z, gsiz_x, gthick, gsiz_z);
        add_wall(-gpos_x, 0.0, gpos_z, gthick, f.goal_width, gsiz_z);
        add_wall(-gpos2_x, -gpos_y, gpos_z, gsiz_x, gthick, gsiz_z);
        add_wall(-gpos2_x, gpos_y, gpos_z, gsiz_x, gthick, gsiz_z);

        walls
    }

    fn create_robot(
        robot_cfg: &RobotConfig,
        x: f64,
        y: f64,
        orientation: f64,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
    ) -> RobotHandles {
        // Place robot on the ground (bottom of cylinder at z=0)
        let z = (robot_cfg.height / 2.0 + 0.001) as f32;
        let rot = UnitQuaternion::from_axis_angle(&NVec3::z_axis(), orientation as f32);
        let chassis_iso = Isometry3::from_parts(NVec3::new(x as f32, y as f32, z).into(), rot);

        // Chassis (cylinder) - the entire robot body
        let chassis_rb = RigidBodyBuilder::dynamic()
            .pose(chassis_iso.into())
            .linear_damping(0.5)
            .angular_damping(1.0)
            // Lock Z rotation to keep robot upright (only rotate around Z axis for yaw)
            .enabled_rotations(false, false, true)
            // Lock Z translation to keep on ground plane
            .lock_translations()
            .build();
        let chassis_body = bodies.insert(chassis_rb);
        // Unlock X and Y translations (lock_translations locks all, we only want Z locked)
        // Actually, let's just use a kinematic approach: lock nothing, but set high damping
        // Re-do: make it fully dynamic but constrained to ground
        bodies.remove(
            chassis_body,
            &mut IslandManager::new(),
            colliders,
            impulse_joints,
            &mut MultibodyJointSet::new(),
            true,
        );

        let chassis_rb = RigidBodyBuilder::dynamic()
            .pose(chassis_iso.into())
            .linear_damping(0.5)
            .angular_damping(1.0)
            .enabled_rotations(false, false, true)
            .build();
        let chassis_body = bodies.insert(chassis_rb);
        let chassis_collider = colliders.insert_with_parent(
            ColliderBuilder::cylinder(robot_cfg.height as f32 / 2.0, robot_cfg.radius as f32)
                .mass(robot_cfg.body_mass as f32)
                .friction(0.3)
                .restitution(0.1)
                .build(),
            chassis_body,
            bodies,
        );

        // Kicker (box) - fixed to chassis
        let kicker_offset_x = robot_cfg.center_from_kicker + robot_cfg.kicker_thickness;
        let kicker_offset_z = robot_cfg.kicker_z;
        let kicker_local = NVec3::new(kicker_offset_x as f32, 0.0, kicker_offset_z as f32);
        let kicker_world = chassis_iso * Point3::from(kicker_local);

        let kicker_rb = RigidBodyBuilder::dynamic()
            .translation(to_rapier_vec(kicker_world.coords))
            .rotation(to_rapier_vec(rot.scaled_axis()))
            .enabled_rotations(false, false, true)
            .build();
        let kicker_body = bodies.insert(kicker_rb);
        let kicker_collider = colliders.insert_with_parent(
            ColliderBuilder::cuboid(
                robot_cfg.kicker_thickness as f32 / 2.0,
                robot_cfg.kicker_width as f32 / 2.0,
                robot_cfg.kicker_height as f32 / 2.0,
            )
            .mass(robot_cfg.kicker_mass as f32)
            .friction(robot_cfg.kicker_friction as f32)
            .build(),
            kicker_body,
            bodies,
        );

        let kicker_joint = FixedJointBuilder::new()
            .local_anchor1(to_rapier_vec(kicker_local))
            .local_anchor2(Vector::ZERO);
        let kicker_joint = impulse_joints.insert(chassis_body, kicker_body, kicker_joint, true);

        RobotHandles {
            chassis_body,
            chassis_collider,
            kicker_body,
            kicker_collider,
            kicker_joint,
        }
    }

    /// Perform one physics sub-step.
    pub fn substep(&mut self) {
        self.physics_pipeline.step(
            to_rapier_vec(self.gravity),
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    /// Apply ball rolling friction (matches grSim's ball friction model).
    pub fn apply_ball_friction(&mut self) {
        let ball = &self.rigid_body_set[self.ball_body];
        let vel = ball.linvel();
        let speed = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt();

        if speed > 0.01 {
            let fk = self.ball_friction * self.ball_mass * self.gravity_val;
            let friction_force = Vector::new(
                -fk * vel.x / speed,
                -fk * vel.y / speed,
                -fk * vel.z / speed,
            );
            let torque = Vector::new(
                -friction_force.y * self.ball_radius,
                friction_force.x * self.ball_radius,
                0.0,
            );
            let ball = &mut self.rigid_body_set[self.ball_body];
            ball.add_force(friction_force, true);
            ball.add_torque(torque, true);
        } else {
            let ball = &mut self.rigid_body_set[self.ball_body];
            ball.set_linvel(Vector::ZERO, true);
            ball.set_angvel(Vector::ZERO, true);
        }
    }

    /// Apply robot velocity directly by setting linear/angular velocity.
    /// This is the simplified model: instead of wheel motors, we directly
    /// set the chassis velocity based on the computed wheel kinematics.
    /// `vx`, `vy` are in the robot's local frame, `vw` is angular velocity.
    pub fn apply_robot_velocity(&mut self, robot: &RobotHandles, vx: f32, vy: f32, vw: f32) {
        let body = &self.rigid_body_set[robot.chassis_body];
        let facing = body.rotation().mul_vec3(Vector::X);
        let yaw = facing.y.atan2(facing.x);

        // Convert local velocity to world frame
        let world_vx = vx * yaw.cos() - vy * yaw.sin();
        let world_vy = vx * yaw.sin() + vy * yaw.cos();

        let body = &mut self.rigid_body_set[robot.chassis_body];
        body.set_linvel(Vector::new(world_vx, world_vy, body.linvel().z), true);
        body.set_angvel(Vector::new(0.0, 0.0, vw), true);
    }

    /// Update one wheel speed and immediately project the full wheel set to chassis motion.
    pub fn set_wheel_speed(&mut self, robot: &RobotHandles, wheel_index: usize, speed: f32) {
        if wheel_index >= WHEEL_COUNT {
            return;
        }

        if let Some(index) = self
            .blue_robots
            .iter()
            .position(|handles| handles.chassis_body == robot.chassis_body)
        {
            self.blue_wheel_speeds[index][wheel_index] = speed;
            let wheel_speeds = self.blue_wheel_speeds[index];
            let (vx, vy, vw) = self.blue_drive.body_velocity(wheel_speeds);
            let handle = self.blue_robots[index].clone();
            self.apply_robot_velocity(&handle, vx, vy, vw);
            return;
        }

        if let Some(index) = self
            .yellow_robots
            .iter()
            .position(|handles| handles.chassis_body == robot.chassis_body)
        {
            self.yellow_wheel_speeds[index][wheel_index] = speed;
            let wheel_speeds = self.yellow_wheel_speeds[index];
            let (vx, vy, vw) = self.yellow_drive.body_velocity(wheel_speeds);
            let handle = self.yellow_robots[index].clone();
            self.apply_robot_velocity(&handle, vx, vy, vw);
        }
    }

    /// Get position of a rigid body.
    pub fn get_body_position(&self, handle: RigidBodyHandle) -> NVec3<f32> {
        to_nalgebra_vec(self.rigid_body_set[handle].translation())
    }

    /// Get linear velocity of a rigid body.
    pub fn get_body_linvel(&self, handle: RigidBodyHandle) -> NVec3<f32> {
        to_nalgebra_vec(self.rigid_body_set[handle].linvel())
    }

    /// Get angular velocity of a rigid body.
    pub fn get_body_angvel(&self, handle: RigidBodyHandle) -> NVec3<f32> {
        to_nalgebra_vec(self.rigid_body_set[handle].angvel())
    }

    /// Get orientation (yaw) angle in radians.
    pub fn get_body_yaw(&self, handle: RigidBodyHandle) -> f32 {
        let rot = self.rigid_body_set[handle].rotation();
        let facing = rot.mul_vec3(Vector::X);
        facing.y.atan2(facing.x)
    }

    /// Set body position.
    pub fn teleport_body(&mut self, handle: RigidBodyHandle, x: f32, y: f32, z: f32) {
        let body = &mut self.rigid_body_set[handle];
        let rot = *body.rotation();
        body.set_position(Pose::from_parts(Vector::new(x, y, z), rot), true);
    }

    /// Set body orientation (yaw).
    pub fn set_body_yaw(&mut self, handle: RigidBodyHandle, yaw: f32) {
        let body = &mut self.rigid_body_set[handle];
        let pos = body.translation();
        let rot = Rotation::from_rotation_z(yaw);
        body.set_position(Pose::from_parts(pos, rot), true);
    }

    /// Reset all velocities for a body.
    pub fn reset_body_velocity(&mut self, handle: RigidBodyHandle) {
        let body = &mut self.rigid_body_set[handle];
        body.set_linvel(Vector::ZERO, true);
        body.set_angvel(Vector::ZERO, true);
    }
}

fn to_rapier_vec(v: NVec3<f32>) -> Vector {
    Vector::new(v.x, v.y, v.z)
}

fn to_nalgebra_vec(v: Vector) -> NVec3<f32> {
    NVec3::new(v.x, v.y, v.z)
}

/// Default blue team positions (spread on the negative-x half).
fn default_blue_position(index: usize, config: &WorldConfig) -> (f64, f64) {
    let n = config.robots_per_team;
    let spacing = config.field.field_width / (n as f64 + 1.0);
    let x = -config.field.field_length / 4.0;
    let y = -config.field.field_width / 2.0 + spacing * (index as f64 + 1.0);
    (x, y)
}

/// Default yellow team positions (spread on the positive-x half).
fn default_yellow_position(index: usize, config: &WorldConfig) -> (f64, f64) {
    let n = config.robots_per_team;
    let spacing = config.field.field_width / (n as f64 + 1.0);
    let x = config.field.field_length / 4.0;
    let y = -config.field.field_width / 2.0 + spacing * (index as f64 + 1.0);
    (x, y)
}
