use std::borrow::Cow;
use std::time::Duration;

use anyhow::Result;
use simhark::viewer::ViewerConfig;
use simhark::{MoveCommand, RobotCommand, WorldConfig, WorldState};
use simhark_testing::{
    BallInit, InitialWorld, RobotInit, SimTest, TeamCommands, TestCase, TestOutcome, TestPlan,
    TestRunner, TestSuite,
};

fn main() -> Result<()> {
    let config = WorldConfig::division_b();
    let report = TestRunner::with_config(config)
        .max_ticks(500)
        .concurrent_root_groups(1)
        .run_plan_with_viewer(
            make_plan,
            ViewerConfig::default(),
            Duration::from_millis(16),
        )?;

    println!(
        "testing finished: {} passed, {} failed",
        report.passed(),
        report.failed()
    );
    Ok(())
}

fn make_plan() -> TestPlan<DriveBallTest> {
    TestPlan::new()
        .suite(
            TestSuite::new("shoot")
                .subtests(
                    "straight shots",
                    [
                        TestCase::new("center", DriveBallTest::new(0.0, 1.8, true)),
                        TestCase::new("upper lane", DriveBallTest::new(0.8, 1.8, true)),
                        TestCase::new("lower lane", DriveBallTest::new(-0.8, 1.8, true)),
                    ],
                )
                .subtests(
                    "blocked shot",
                    [TestCase::new(
                        "defender blocks lane",
                        DriveBallTest::new(0.0, 1.8, false),
                    )],
                ),
        )
        .suite(TestSuite::new("defend").subtests(
            "clear ball",
            [
                TestCase::new("near post", DriveBallTest::new(1.0, -1.2, true)),
                TestCase::new("far post", DriveBallTest::new(-1.0, -1.2, true)),
            ],
        ))
}

struct DriveBallTest {
    lane_y: f64,
    target_x: f64,
    should_pass: bool,
}

impl DriveBallTest {
    fn new(lane_y: f64, target_x: f64, should_pass: bool) -> Self {
        Self {
            lane_y,
            target_x,
            should_pass,
        }
    }
}

impl SimTest for DriveBallTest {
    fn name(&self) -> Cow<'_, str> {
        Cow::Borrowed("drive ball")
    }

    fn initial_state(&self) -> InitialWorld {
        let mut yellow = Vec::new();
        if !self.should_pass {
            yellow.push(RobotInit::at(0, 0.4, self.lane_y, std::f64::consts::PI));
        }

        InitialWorld::new(
            [RobotInit::at(0, -1.2, self.lane_y, 0.0)],
            yellow,
            BallInit::at(-0.75, self.lane_y),
        )
    }

    fn drive(&mut self, state: &WorldState) -> TeamCommands {
        let Some(robot) = state.blue_robots.iter().find(|robot| robot.id == 0) else {
            return TeamCommands::empty();
        };

        let dx = state.ball.x - robot.x;
        let dy = state.ball.y - robot.y;
        let distance_to_ball = (dx * dx + dy * dy).sqrt();
        let forward = if distance_to_ball > 0.18 { 1.5 } else { 0.8 };

        TeamCommands {
            blue: vec![RobotCommand {
                id: 0,
                move_command: Some(MoveCommand::LocalVelocity {
                    forward,
                    left: dy.clamp(-0.4, 0.4),
                    angular: 0.0,
                }),
                kick_speed: if distance_to_ball < 0.18 { 2.5 } else { 0.0 },
                kick_angle: 0.0,
                dribbler_on: true,
            }],
            yellow: Vec::new(),
        }
    }

    fn validate(&mut self, state: &WorldState) -> TestOutcome {
        if self.should_pass && state.ball.x >= self.target_x {
            TestOutcome::Passed
        } else if !self.should_pass && state.ball.x >= self.target_x {
            TestOutcome::Failed("blocked shot reached the target".to_string())
        } else if !self.should_pass && state.frame > 240 {
            TestOutcome::Passed
        } else {
            TestOutcome::Running
        }
    }
}
