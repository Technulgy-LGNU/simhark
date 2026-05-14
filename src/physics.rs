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

// Collision group bits, mirroring grSim's per-surface collision setup.
//
// In grSim, each pair of geoms must have a `PSurface` created for them to
// generate contacts. Rapier instead uses bitmask filters: a contact is
// generated when both colliders' membership intersects the other's filter.
//
// Layout:
//   - BALL: the soccer ball
//   - CHASSIS: each robot's visible cylinder body
//   - DUMMY: small invisible sphere at robot center used as the ball-collision
//     proxy (so the ball can slip past the cylinder edges into the kicker mouth)
//   - KICKER: each robot's kicker box
//   - WORLD: walls and ground plane
const GROUP_BALL: Group = Group::GROUP_1;
const GROUP_CHASSIS: Group = Group::GROUP_2;
const GROUP_DUMMY: Group = Group::GROUP_3;
const GROUP_KICKER: Group = Group::GROUP_4;
const GROUP_WORLD: Group = Group::GROUP_5;

fn ball_groups() -> InteractionGroups {
    InteractionGroups::new(
        GROUP_BALL,
        GROUP_DUMMY.union(GROUP_KICKER).union(GROUP_WORLD),
        InteractionTestMode::And,
    )
}

fn chassis_groups() -> InteractionGroups {
    // Chassis collides with other chassis (robot-robot bumping), kickers
    // (mouth/back contact between robots), and the world. Own-kicker
    // contacts are filtered out via the kicker joint having
    // `contacts_enabled(false)`.
    InteractionGroups::new(
        GROUP_CHASSIS,
        GROUP_CHASSIS
            .union(GROUP_KICKER)
            .union(GROUP_WORLD),
        InteractionTestMode::And,
    )
}

fn dummy_groups() -> InteractionGroups {
    // Dummies collide with the ball and with other robots' dummies. They are
    // intentionally inert vs. walls/ground (the chassis handles those).
    InteractionGroups::new(
        GROUP_DUMMY,
        GROUP_BALL.union(GROUP_DUMMY),
        InteractionTestMode::And,
    )
}

fn kicker_groups() -> InteractionGroups {
    // Kicker collides with the ball and with other robots' chassis (so a
    // robot's kicker can push another robot's body). Own-chassis contacts
    // are suppressed by the kicker joint.
    InteractionGroups::new(
        GROUP_KICKER,
        GROUP_BALL.union(GROUP_CHASSIS),
        InteractionTestMode::And,
    )
}

fn world_groups() -> InteractionGroups {
    InteractionGroups::new(
        GROUP_WORLD,
        GROUP_BALL.union(GROUP_CHASSIS),
        InteractionTestMode::And,
    )
}

/// Handle indices for one robot in the physics world.
#[derive(Debug, Clone)]
pub struct RobotHandles {
    pub chassis_body: RigidBodyHandle,
    pub chassis_collider: ColliderHandle,
    pub dummy_collider: ColliderHandle,
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
    blue_drive_params: DriveParams,
    yellow_drive_params: DriveParams,

    // Cached config values
    ball_friction: f32,
    ball_mass: f32,
    ball_radius: f32,
    gravity_val: f32,
}

/// Per-team mass / inertia / acceleration limits used by the
/// torque-limited velocity controller in `apply_drive_forces`.
#[derive(Clone, Copy)]
struct DriveParams {
    mass: f32,
    inertia_z: f32,
    max_force: f32,
    max_torque: f32,
}

impl DriveParams {
    fn from_config(cfg: &RobotConfig) -> Self {
        let mass = cfg.body_mass as f32;
        let radius = cfg.radius as f32;
        // Solid cylinder spinning about its symmetry axis: I = 0.5 m r^2.
        let inertia_z = 0.5 * mass * radius * radius;
        Self {
            mass,
            inertia_z,
            max_force: mass * cfg.acc_speedup_absolute_max as f32,
            max_torque: inertia_z * cfg.acc_speedup_angular_max as f32,
        }
    }
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
                .collision_groups(world_groups())
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
                .collision_groups(ball_groups())
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
            blue_drive_params: DriveParams::from_config(&config.blue_robots),
            yellow_drive_params: DriveParams::from_config(&config.yellow_robots),
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
                    .collision_groups(world_groups())
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
        // Mirrors grSim: chassis sits with its bottom one wheel-radius above the
        // ground (the wheels live in this gap, but we don't model them as
        // separate bodies).
        let z = robot_cfg.start_z() as f32;
        let rot = UnitQuaternion::from_axis_angle(&NVec3::z_axis(), orientation as f32);
        let chassis_iso = Isometry3::from_parts(NVec3::new(x as f32, y as f32, z).into(), rot);

        // Chassis: a simple cylinder, exactly like grSim's PCylinder. The ball
        // never sees this cylinder (collision groups exclude it); only other
        // robots and the world do.
        let chassis_rb = RigidBodyBuilder::dynamic()
            .pose(chassis_iso.into())
            .linear_damping(0.5)
            .angular_damping(1.0)
            .enabled_rotations(false, false, true)
            // Pin the chassis at start_z so we don't need to model wheels.
            .enabled_translations(true, true, false)
            .build();
        let chassis_body = bodies.insert(chassis_rb);

        let half_height = robot_cfg.height as f32 / 2.0;
        let chassis_collider = colliders.insert_with_parent(
            ColliderBuilder::cylinder(half_height, robot_cfg.radius as f32)
                .mass(robot_cfg.body_mass as f32 * 0.99)
                .friction(0.3)
                .restitution(0.1)
                .collision_groups(chassis_groups())
                .build(),
            chassis_body,
            bodies,
        );

        // Dummy: small invisible sphere at the chassis center, used as the
        // ball-collision proxy. Sized to `center_from_kicker` so the ball can
        // slip around its lower hemisphere and reach the kicker mouth (the
        // dummy bottom sits well above the ball at this radius / start_z).
        let dummy_collider = colliders.insert_with_parent(
            ColliderBuilder::ball(robot_cfg.center_from_kicker as f32)
                .mass(robot_cfg.body_mass as f32 * 0.01)
                .friction(0.3)
                .restitution(0.1)
                .collision_groups(dummy_groups())
                .build(),
            chassis_body,
            bodies,
        );

        // Kicker: a flat box at the front, joined rigidly to the chassis.
        let kicker_offset_x = robot_cfg.center_from_kicker + robot_cfg.kicker_thickness;
        let kicker_offset_z = -robot_cfg.height * 0.5 + robot_cfg.wheel_radius
            - robot_cfg.bottom_height
            + robot_cfg.kicker_z;
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
            .collision_groups(kicker_groups())
            .build(),
            kicker_body,
            bodies,
        );

        // Suppress contacts between the chassis and its own kicker — the joint
        // already glues them together, and otherwise their colliders overlap
        // (the kicker box sits inside the cylinder envelope).
        let kicker_joint = FixedJointBuilder::new()
            .local_anchor1(to_rapier_vec(kicker_local))
            .local_anchor2(Vector::ZERO)
            .contacts_enabled(false);
        let kicker_joint = impulse_joints.insert(chassis_body, kicker_body, kicker_joint, true);

        RobotHandles {
            chassis_body,
            chassis_collider,
            dummy_collider,
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

    /// Update one wheel speed. The actual chassis motion is driven by
    /// `apply_drive_forces`, which runs once per substep so contact forces
    /// can resist commanded motion (rather than being wiped out by a
    /// per-frame `set_linvel`).
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
            return;
        }
        if let Some(index) = self
            .yellow_robots
            .iter()
            .position(|handles| handles.chassis_body == robot.chassis_body)
        {
            self.yellow_wheel_speeds[index][wheel_index] = speed;
        }
    }

    /// Push each robot toward its commanded velocity using force/torque,
    /// limited to the chassis's max acceleration. Called once per substep.
    /// This replaces the previous `set_linvel`-every-frame approach, which
    /// silently discarded contact-resolution forces and let perfectly
    /// symmetric robot clusters lock the ball in place.
    pub fn apply_drive_forces(&mut self, dt_substep: f32) {
        let blue_handles = self.blue_robots.clone();
        let blue_speeds = self.blue_wheel_speeds.clone();
        for (handle, speeds) in blue_handles.iter().zip(blue_speeds.iter()) {
            let (vx, vy, vw) = self.blue_drive.body_velocity(*speeds);
            self.apply_drive_force(handle, vx, vy, vw, self.blue_drive_params, dt_substep);
        }
        let yellow_handles = self.yellow_robots.clone();
        let yellow_speeds = self.yellow_wheel_speeds.clone();
        for (handle, speeds) in yellow_handles.iter().zip(yellow_speeds.iter()) {
            let (vx, vy, vw) = self.yellow_drive.body_velocity(*speeds);
            self.apply_drive_force(handle, vx, vy, vw, self.yellow_drive_params, dt_substep);
        }
    }

    fn apply_drive_force(
        &mut self,
        robot: &RobotHandles,
        target_vx_local: f32,
        target_vy_local: f32,
        target_vw: f32,
        params: DriveParams,
        dt_substep: f32,
    ) {
        let body = &self.rigid_body_set[robot.chassis_body];
        let facing = body.rotation().mul_vec3(Vector::X);
        let yaw = facing.y.atan2(facing.x);
        let target_world_vx = target_vx_local * yaw.cos() - target_vy_local * yaw.sin();
        let target_world_vy = target_vx_local * yaw.sin() + target_vy_local * yaw.cos();
        let current_lin = body.linvel();
        let current_ang = body.angvel().z;

        // F = m * (target_v - current_v) / dt_substep, magnitude-clamped.
        let dvx = target_world_vx - current_lin.x;
        let dvy = target_world_vy - current_lin.y;
        let mut force_x = params.mass * dvx / dt_substep;
        let mut force_y = params.mass * dvy / dt_substep;
        let force_mag = (force_x * force_x + force_y * force_y).sqrt();
        if force_mag > params.max_force {
            let scale = params.max_force / force_mag;
            force_x *= scale;
            force_y *= scale;
        }

        let dvw = target_vw - current_ang;
        let raw_torque = params.inertia_z * dvw / dt_substep;
        let torque_z = raw_torque.clamp(-params.max_torque, params.max_torque);

        let body = &mut self.rigid_body_set[robot.chassis_body];
        body.add_force(Vector::new(force_x, force_y, 0.0), true);
        body.add_torque(Vector::new(0.0, 0.0, torque_z), true);
    }

    pub fn substep_dt(&self) -> f32 {
        self.integration_parameters.dt
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

    pub fn ball_mass(&self) -> f32 {
        self.ball_mass
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
