use std::borrow::Cow;

use crashpilot::core_dump::proto::CpInfos;
use simhark::{RobotState, WorldCommand, WorldConfig, WorldState};
use simhark_faabs::run_sim_action;
use simhark_testing::{
    BallInit, InitialWorld, RobotInit, SimTest, TeamCommands, TestCase, TestOutcome, TestPlan,
    TestRunner, TestSuite,
};
use tf_jetsoncode::{
    Config, CpBall, CpCommand, CpRobot, CpState, CpTrackedRobot, CpVector2, Events, Robot,
    TeensyRecMSG,
};

const GOALIE_ID: usize = 0;
const MAIN_DISTANCE: f64 = 2.8;
const NEAR_DISTANCE: f64 = 2.0;
const SHOT_SPEED: f64 = 4.0;
const ANGLE_MIN_DEG: i32 = -35;
const ANGLE_MAX_DEG: i32 = 35;
const ANGLE_STEP_DEG: usize = 5;
const PASS_FRAME: u64 = 260;

fn main() -> std::io::Result<()> {
    let runner = TestRunner::with_config(WorldConfig::division_b())
        .max_ticks(360)
        .concurrent_root_groups(1);

    #[cfg(not(feature = "viewer"))]
    let report = runner.run_plan(make_plan());

    #[cfg(feature = "viewer")]
    let report = runner.run_plan_with_viewer(
        make_plan,
        simhark::viewer::ViewerConfig::default(),
        std::time::Duration::from_millis(16),
    )?;

    println!(
        "goalie testing finished: {} passed, {} failed",
        report.passed(),
        report.failed()
    );

    Ok(())
}

fn make_plan() -> TestPlan<GoalieShotTest> {
    let config = WorldConfig::division_b();
    let half_goal_width = config.field.goal_width * 0.5;
    let targets = [
        ("miss lower", -half_goal_width - 0.25),
        ("lower post", -half_goal_width + 0.04),
        ("lower lane", -0.25),
        ("center", 0.0),
        ("upper lane", 0.25),
        ("upper post", half_goal_width - 0.04),
        ("miss upper", half_goal_width + 0.25),
    ];

    let mut suite = TestSuite::new("goalie");
    for (name, target_y) in targets {
        suite = suite.subtests(
            format!("{name} main arc"),
            arc_cases(&config, target_y, MAIN_DISTANCE),
        );
    }
    suite = suite.subtests("center near arc", arc_cases(&config, 0.0, NEAR_DISTANCE));

    TestPlan::new().suite(suite)
}

fn arc_cases(
    config: &WorldConfig,
    target_y: f64,
    distance: f64,
) -> impl IntoIterator<Item = TestCase<GoalieShotTest>> {
    let target_x = -config.field.field_length * 0.5 - config.ball.radius;

    (ANGLE_MIN_DEG..=ANGLE_MAX_DEG)
        .step_by(ANGLE_STEP_DEG)
        .map(move |angle_deg| {
            TestCase::new(
                format!("{angle_deg:+03}deg d={distance:.1}m"),
                GoalieShotTest::new(target_x, target_y, distance, angle_deg as f64),
            )
        })
        .collect::<Vec<_>>()
}

struct GoalieShotTest {
    target_x: f64,
    target_y: f64,
    distance: f64,
    angle_deg: f64,
    goalie: Robot<()>,
}

impl GoalieShotTest {
    fn new(target_x: f64, target_y: f64, distance: f64, angle_deg: f64) -> Self {
        let mut robot_config = Config::default();
        robot_config.robot_id = GOALIE_ID as u8;

        Self {
            target_x,
            target_y,
            distance,
            angle_deg,
            goalie: Robot::new(robot_config),
        }
    }
}

impl SimTest for GoalieShotTest {
    fn name(&self) -> Cow<'_, str> {
        Cow::Borrowed("goalie shot")
    }

    fn initial_state(&self) -> InitialWorld {
        let config = WorldConfig::division_b();
        let goal_x = -config.field.field_length * 0.5;
        let angle = self.angle_deg.to_radians();
        let ball_x = self.target_x + self.distance * angle.cos();
        let ball_y = self.target_y + self.distance * angle.sin();
        let dx = self.target_x - ball_x;
        let dy = self.target_y - ball_y;
        let len = (dx * dx + dy * dy).sqrt().max(f64::EPSILON);

        let mut ball = BallInit::at(ball_x, ball_y);
        ball.vx = SHOT_SPEED * dx / len;
        ball.vy = SHOT_SPEED * dy / len;

        InitialWorld::new(
            [],
            [RobotInit::at(GOALIE_ID, goal_x + 0.22, 0.0, 0.0)],
            ball,
        )
    }

    fn drive(&mut self, state: &WorldState) -> TeamCommands {
        let (teensy, _) = self.goalie.step_with_data(goalie_events(state));
        let mut command = WorldCommand::default();
        run_sim_action(GOALIE_ID as u32, teensy, &mut command);

        TeamCommands {
            blue: command.blue,
            yellow: command.yellow,
        }
    }

    fn validate(&mut self, state: &WorldState) -> TestOutcome {
        if state.yellow_robots[0].infrared {
            return TestOutcome::Passed;
        }

        if state.goal_yellow || state.ball.x < -4.6 {
            TestOutcome::Failed(format!(
                "goal conceded for target y={:.2}, angle={:+.0}deg, distance={:.1}m",
                self.target_y, self.angle_deg, self.distance
            ))
        } else {
            TestOutcome::Running
        }
    }
}

fn goalie_events(state: &WorldState) -> Events {
    Events {
        cp: Some(cp_robot(state)),
        vis: None,
        teensy: Some(TeensyRecMSG {
            flags: (1 << 2) | (1 << 3),
            batt_level: 0,
            current: 0,
        }),
    }
}

fn cp_robot(state: &WorldState) -> CpRobot {
    let config = WorldConfig::division_b();

    CpRobot {
        robot_id: GOALIE_ID as u32,
        timestamp: state.sim_time,
        packet_id: state.frame as u32,
        ball: CpBall {
            pos: cp_vec(state.ball.x, state.ball.y),
            vel: Some(cp_vec(state.ball.vx, state.ball.vy)),
        },
        robots_yellow: state
            .yellow_robots
            .iter()
            .map(cp_tracked_robot)
            .collect::<Vec<_>>(),
        robots_blue: state
            .blue_robots
            .iter()
            .map(cp_tracked_robot)
            .collect::<Vec<_>>(),
        cmd: CpCommand {
            state: CpState::StateGoalie as i32,
            ..CpCommand::default()
        },
        infos: CpInfos {
            team_color: false,
            team_site: true,
            width: meters_to_mm_u32(config.field.field_length),
            height: meters_to_mm_u32(config.field.field_width),
            runoff_width: meters_to_mm_u32(config.field.margin_touch_line),
            penalty_area_width: meters_to_mm_u32(config.field.penalty_width),
            penalty_area_height: meters_to_mm_u32(config.field.penalty_depth),
            goal_width: meters_to_mm_u32(config.field.goal_width),
        },
    }
}

fn cp_tracked_robot(robot: &RobotState) -> CpTrackedRobot {
    CpTrackedRobot {
        robot_id: robot.id as u32,
        pos: cp_vec(robot.x, robot.y),
        orientation: robot.orientation.to_degrees() as i32,
        vel: Some(cp_vec(robot.vx, robot.vy)),
        visibility: if robot.is_on { 255 } else { 0 },
    }
}

fn cp_vec(x: f64, y: f64) -> CpVector2 {
    CpVector2 {
        x: meters_to_mm_i32(x),
        y: meters_to_mm_i32(y),
    }
}

fn meters_to_mm_i32(value: f64) -> i32 {
    (value * 1000.0).round() as i32
}

fn meters_to_mm_u32(value: f64) -> u32 {
    meters_to_mm_i32(value).max(0) as u32
}
