mod conv;
mod run;

use std::mem;
use tf_jetsoncode::Robot;
use ::CrashPilot::{CrashPilot, core_dump};
use simhark::{World, WorldCommand, WorldState};
use crate::conv::world_state_to_cp_events;
use crate::run::run_sim_action;

pub struct Faabs {
    pub robots: Vec<Robot<()>>,
    pub crash_pilot: CrashPilot<()>,
    pub feedback_robot: u32,
    pub events: ::CrashPilot::Events,

}

impl Faabs {
    pub fn new(num_robots: u8) -> Self {
        let mut robots = Vec::with_capacity(num_robots as usize);

        for i in 0..num_robots {
            let mut config = tf_jetsoncode::Config::default();
            config.robot_id = i + 1;


            robots.push(Robot::new(config));

        }


        Self {
            robots,
            crash_pilot: CrashPilot::new(::CrashPilot::Config::default()),
            feedback_robot: 0,
            events: ::CrashPilot::Events::default(),
        }
    }

    pub fn step(&mut self, state: &WorldState, command: &mut WorldCommand) {
        world_state_to_cp_events(&mut self.events, state);

        let (_interface, robots)= self.crash_pilot.step_with_data(mem::take(&mut self.events));



        for (id, data) in robots {
            let Some(robot) = self.robots.get_mut((id - 1) as usize) else {
                continue;
            };

            let events = conv::robot_events(id, data, state);

            let (teensy, robot_cp) = robot.step_with_data(events);

            run_sim_action(id, teensy, command);

            if self.feedback_robot == id {
                self.events.rf = Some(conv::robot_cp(robot_cp));
            }

        }

        self.feedback_robot += 1;
        self.feedback_robot %= self.robots.len() as u32;



    }
}