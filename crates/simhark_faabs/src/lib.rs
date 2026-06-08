mod conv;
#[cfg(feature = "interface")]
mod interface;
mod run;

use crate::conv::world_state_to_cp_events;
#[cfg(feature = "interface")]
use crate::interface::EventShare;
use crate::run::run_sim_action;
use simhark::{WorldCommand, WorldState};
use std::collections::HashMap;
use std::mem;
use std::net::Ipv4Addr;
use tf_jetsoncode::Robot;
use ::CrashPilot::config::{LoggingConfig, RobotConfig, ServerConfig, SslConfig};
use ::CrashPilot::CrashPilot;

pub struct Faabs {
    pub robots: Vec<Robot<()>>,
    pub crash_pilot: CrashPilot<()>,
    pub feedback_robot: u32,
    pub events: ::CrashPilot::Events,
    #[cfg(feature = "interface")]
    pub interface: EventShare,
    #[cfg(feature = "interface")]
    pub ws_out: ::CrashPilot::communication::WebsocketOut,
}

impl Faabs {
    #[cfg(feature = "interface")]
    pub fn with_interface(num_robots: u8) -> Self {
        let faabs = Self::new(num_robots);

        #[cfg(feature = "interface")]
        {
            let cfg = get_config();
            let tx = faabs.interface.clone();
            let ws_out = faabs.ws_out.clone();

            ::CrashPilot::interface::spawn_interface();

            tokio::spawn(async move {
                crate::interface::spawn_websocket(&cfg, tx, ws_out).await;
            });
        }

        faabs
    }

    pub fn new(num_robots: u8) -> Self {
        let mut robots = Vec::with_capacity(num_robots as usize);

        for i in 0..num_robots {
            let mut config = tf_jetsoncode::Config::default();
            config.robot_id = i;

            robots.push(Robot::new(config));
        }

        Self {
            robots,
            crash_pilot: CrashPilot::new(get_config()),
            feedback_robot: 0,
            events: ::CrashPilot::Events::default(),
            #[cfg(feature = "interface")]
            interface: EventShare::default(),
            #[cfg(feature = "interface")]
            ws_out: ::CrashPilot::communication::WebsocketOut::new(),
        }
    }

    pub fn step(&mut self, state: &WorldState, command: &mut WorldCommand, referee: Option<::CrashPilot::core_dump::proto::Referee>) {
        world_state_to_cp_events(&mut self.events, state);
        self.events.gc = referee;

        #[cfg(feature = "interface")]
        {
            self.events.ws = self.interface.blocking_lock().clone();
        }

        let ws = self.events.ws.clone();

        let (interface, robots) = self.crash_pilot.step_with_data(mem::take(&mut self.events));

        #[cfg(feature = "interface")]
        {
            self.ws_out.publish_sync(interface);
        }

        self.events.ws = ws;

        for (id, data) in robots {
            let Some(robot) = self.robots.get_mut(id as usize) else {
                panic!(
                    "Received data for robot with id {}, but only {} robots are configured",
                    id,
                    self.robots.len()
                );
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

fn get_config() -> ::CrashPilot::Config {
    let mut robots = HashMap::new();

    robots.insert(
        0,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 101),
            substitution_pos: Default::default(),
        },
    );
    robots.insert(
        1,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 101),
            substitution_pos: Default::default(),
        },
    );
    robots.insert(
        2,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 102),
            substitution_pos: Default::default(),
        },
    );
    robots.insert(
        3,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 103),
            substitution_pos: Default::default(),
        },
    );
    robots.insert(
        4,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 104),
            substitution_pos: Default::default(),
        },
    );
    robots.insert(
        5,
        RobotConfig {
            ip: Ipv4Addr::new(10, 0, 64, 104),
            substitution_pos: Default::default(),
        },
    );

    ::CrashPilot::Config {
        ssl: SslConfig::default(),
        server: ServerConfig::default(),
        logging: LoggingConfig::default(),
        robots,
    }
}
