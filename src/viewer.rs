//! Optional browser-based debug viewer.

use std::collections::HashMap;
use std::io::{Error, Result};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use parking_lot::Mutex;
use serde::Serialize;
use tiny_http::{Header, Method, Response, Server, StatusCode};
use tungstenite::{Message, accept};

use crate::config::{FieldConfig, WorldConfig};
use crate::state::WorldState;

#[derive(Debug, Clone, Copy)]
pub struct ViewerConfig {
    pub host: IpAddr,
    pub http_port: u16,
}

impl Default for ViewerConfig {
    fn default() -> Self {
        Self {
            host: IpAddr::V4(Ipv4Addr::UNSPECIFIED),
            http_port: 8315,
        }
    }
}

impl ViewerConfig {
    pub fn websocket_port(self) -> u16 {
        self.http_port.saturating_add(1)
    }

    pub fn http_url(self) -> String {
        match self.host {
            IpAddr::V4(ip) if ip.is_unspecified() => format!("http://127.0.0.1:{}", self.http_port),
            IpAddr::V6(ip) if ip.is_unspecified() => format!("http://[::1]:{}", self.http_port),
            host => format!("http://{}:{}", host, self.http_port),
        }
    }
}

/// Game-state info pushed to the viewer alongside world state.
///
/// Mirrors the shape of `referris::RefereeSnapshot` without taking on the
/// dependency: callers may translate from any source (referris, SSL
/// game-controller, sumatra default referee, etc.) into this struct.
#[derive(Debug, Clone, Default, Serialize)]
pub struct GameStateInfo {
    /// Current command, normalised to UPPER_SNAKE_CASE (e.g. `FORCE_START`).
    pub command: String,
    /// Monotonic command counter from the game controller.
    pub command_counter: u32,
    /// Optional stage label, e.g. `NORMAL_FIRST_HALF`.
    pub stage: Option<String>,
    pub blue_name: Option<String>,
    pub yellow_name: Option<String>,
}

#[derive(Default)]
struct GoalTracker {
    blue: u32,
    yellow: u32,
    last_blue: bool,
    last_yellow: bool,
}

impl GoalTracker {
    fn observe(&mut self, state: &WorldState) {
        if state.goal_blue && !self.last_blue {
            self.blue += 1;
        }
        if state.goal_yellow && !self.last_yellow {
            self.yellow += 1;
        }
        self.last_blue = state.goal_blue;
        self.last_yellow = state.goal_yellow;
    }
}

#[derive(Serialize)]
struct GoalSummary {
    blue: u32,
    yellow: u32,
    blue_active: bool,
    yellow_active: bool,
}

/// Shared run-state handle used when an application opts in to web-driven
/// start/stop/restart via [`ViewerServer::enable_web_control`].
#[derive(Default)]
struct WebControlState {
    enabled: AtomicBool,
    running: AtomicBool,
    restart_requested: AtomicBool,
}

#[derive(Serialize)]
struct ControlSnapshot {
    web_enabled: bool,
    running: bool,
}

#[derive(Default)]
struct GameStateTracker {
    info: Option<GameStateInfo>,
    counts: HashMap<String, u32>,
    last_command: Option<String>,
    last_counter: Option<u32>,
}

impl GameStateTracker {
    fn update(&mut self, info: GameStateInfo) {
        let command_changed = self.last_command.as_deref() != Some(info.command.as_str());
        let counter_advanced = self
            .last_counter
            .is_none_or(|previous| info.command_counter != previous);
        if command_changed || counter_advanced {
            *self.counts.entry(info.command.clone()).or_insert(0) += 1;
        }
        self.last_command = Some(info.command.clone());
        self.last_counter = Some(info.command_counter);
        self.info = Some(info);
    }

    fn snapshot(&self) -> Option<PublishedGameState<'_>> {
        self.info.as_ref().map(|info| PublishedGameState {
            command: &info.command,
            command_counter: info.command_counter,
            stage: info.stage.as_deref(),
            blue_name: info.blue_name.as_deref(),
            yellow_name: info.yellow_name.as_deref(),
            state_counts: &self.counts,
        })
    }
}

#[derive(Serialize)]
struct PublishedGameState<'a> {
    command: &'a str,
    command_counter: u32,
    stage: Option<&'a str>,
    blue_name: Option<&'a str>,
    yellow_name: Option<&'a str>,
    state_counts: &'a HashMap<String, u32>,
}

pub struct ViewerServer {
    world_count: usize,
    field: FieldConfig,
    robot_radius: f64,
    ball_radius: f64,
    selected_world: Arc<AtomicUsize>,
    latest_frame: Arc<Mutex<Option<String>>>,
    game_state: Arc<Mutex<GameStateTracker>>,
    goal_tracker: Arc<Mutex<GoalTracker>>,
    control: Arc<WebControlState>,
    _http_thread: thread::JoinHandle<()>,
    _ws_thread: thread::JoinHandle<()>,
}

#[derive(Serialize)]
struct ViewerFrame<'a> {
    world_count: usize,
    selected_world: usize,
    field: &'a FieldConfig,
    robot_radius: f64,
    ball_radius: f64,
    state: &'a WorldState,
    game_state: Option<PublishedGameState<'a>>,
    goals: GoalSummary,
    control: ControlSnapshot,
}

impl ViewerServer {
    pub fn bind(
        config: ViewerConfig,
        world_count: usize,
        world_config: &WorldConfig,
    ) -> Result<Self> {
        let http_addr = SocketAddr::new(config.host, config.http_port);
        let ws_addr = SocketAddr::new(config.host, config.websocket_port());

        let http_server = Server::http(http_addr).map_err(|err| Error::other(err.to_string()))?;
        let ws_listener = TcpListener::bind(ws_addr)?;

        let selected_world = Arc::new(AtomicUsize::new(0));
        let latest_frame = Arc::new(Mutex::new(None));
        let game_state = Arc::new(Mutex::new(GameStateTracker::default()));
        let goal_tracker = Arc::new(Mutex::new(GoalTracker::default()));
        let control = Arc::new(WebControlState::default());
        // When web control is disabled the simulator is considered always
        // running, so callers that don't opt in see the legacy behaviour.
        control.running.store(true, Ordering::Relaxed);

        let http_thread = {
            let ws_port = config.websocket_port();
            thread::spawn(move || run_http_server(http_server, ws_port))
        };

        let ws_thread = {
            let latest_frame = Arc::clone(&latest_frame);
            let selected_world = Arc::clone(&selected_world);
            let control_for_ws = Arc::clone(&control);
            thread::spawn(move || {
                run_websocket_server(ws_listener, latest_frame, selected_world, control_for_ws)
            })
        };

        Ok(Self {
            world_count,
            field: world_config.field.clone(),
            robot_radius: world_config.blue_robots.radius,
            ball_radius: world_config.ball.radius,
            selected_world,
            latest_frame,
            game_state,
            goal_tracker,
            control,
            _http_thread: http_thread,
            _ws_thread: ws_thread,
        })
    }

    /// Opt in to web-driven start/stop/restart. The simulator starts in the
    /// stopped state; the application is expected to gate stepping on
    /// [`Self::is_running`] and react to [`Self::take_restart_request`].
    pub fn enable_web_control(&self) {
        self.control.enabled.store(true, Ordering::Relaxed);
        self.control.running.store(false, Ordering::Relaxed);
        self.control.restart_requested.store(false, Ordering::Relaxed);
    }

    /// True when the simulator should keep stepping. Always true when web
    /// control is disabled.
    pub fn is_running(&self) -> bool {
        self.control.running.load(Ordering::Relaxed)
    }

    /// Returns true once when the web UI has asked for a restart, then resets
    /// the flag. Always false when web control is disabled.
    pub fn take_restart_request(&self) -> bool {
        self.control
            .restart_requested
            .swap(false, Ordering::Relaxed)
    }

    pub fn selected_world(&self) -> usize {
        self.selected_world
            .load(Ordering::Relaxed)
            .min(self.world_count.saturating_sub(1))
    }

    /// Push a new referee snapshot. The viewer accumulates per-command counts
    /// so the UI can show "how many times have we entered each game state".
    pub fn set_game_state(&self, info: GameStateInfo) {
        self.game_state.lock().update(info);
    }

    pub fn publish(&self, state: &WorldState) {
        let game_state_guard = self.game_state.lock();
        let mut goal_guard = self.goal_tracker.lock();
        goal_guard.observe(state);
        let frame = ViewerFrame {
            world_count: self.world_count,
            selected_world: self.selected_world(),
            field: &self.field,
            robot_radius: self.robot_radius,
            ball_radius: self.ball_radius,
            state,
            game_state: game_state_guard.snapshot(),
            goals: GoalSummary {
                blue: goal_guard.blue,
                yellow: goal_guard.yellow,
                blue_active: state.goal_blue,
                yellow_active: state.goal_yellow,
            },
            control: ControlSnapshot {
                web_enabled: self.control.enabled.load(Ordering::Relaxed),
                running: self.control.running.load(Ordering::Relaxed),
            },
        };

        if let Ok(json) = serde_json::to_string(&frame) {
            *self.latest_frame.lock() = Some(json);
        }
    }

    /// Reset the accumulated goal counters (useful when restarting a match).
    pub fn reset_goals(&self) {
        *self.goal_tracker.lock() = GoalTracker::default();
    }
}

fn run_http_server(server: Server, ws_port: u16) {
    let html_type = Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).ok();

    for request in server.incoming_requests() {
        let response = match (request.method(), request.url()) {
            (&Method::Get, "/") | (&Method::Get, "/index.html") => {
                let body = render_index(ws_port);
                let mut response = Response::from_string(body).with_status_code(StatusCode(200));
                if let Some(header) = html_type.clone() {
                    response = response.with_header(header);
                }
                response
            }
            _ => Response::from_string("not found").with_status_code(StatusCode(404)),
        };

        let _ = request.respond(response);
    }
}

fn run_websocket_server(
    listener: TcpListener,
    latest_frame: Arc<Mutex<Option<String>>>,
    selected_world: Arc<AtomicUsize>,
    control: Arc<WebControlState>,
) {
    for stream in listener.incoming() {
        let Ok(stream) = stream else {
            continue;
        };

        let latest_frame = Arc::clone(&latest_frame);
        let selected_world = Arc::clone(&selected_world);
        let control = Arc::clone(&control);
        thread::spawn(move || {
            let Ok(mut websocket) = accept(stream) else {
                return;
            };
            let _ = websocket.get_mut().set_nonblocking(true);

            let mut last_sent = String::new();

            loop {
                match websocket.read() {
                    Ok(Message::Text(text)) => {
                        handle_client_message(text.as_str(), &selected_world, &control)
                    }
                    Ok(Message::Close(_)) => break,
                    Ok(_) => {}
                    Err(tungstenite::Error::Io(err))
                        if err.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(
                        tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed,
                    ) => {
                        break;
                    }
                    Err(_) => break,
                }

                if let Some(frame) = latest_frame.lock().clone() {
                    if frame != last_sent {
                        if websocket.send(Message::Text(frame.clone().into())).is_err() {
                            break;
                        }
                        last_sent = frame;
                    }
                }

                thread::sleep(Duration::from_millis(16));
            }
        });
    }
}

fn handle_client_message(
    message: &str,
    selected_world: &AtomicUsize,
    control: &WebControlState,
) {
    if let Some(value) = message.strip_prefix("world:") {
        if let Ok(index) = value.trim().parse::<usize>() {
            selected_world.store(index, Ordering::Relaxed);
        }
        return;
    }

    if let Some(action) = message.strip_prefix("control:") {
        // Control commands are silently ignored when web control wasn't
        // opted in, so a buggy/old UI can't restart a headless training job.
        if !control.enabled.load(Ordering::Relaxed) {
            return;
        }
        match action.trim() {
            "start" => control.running.store(true, Ordering::Relaxed),
            "stop" => control.running.store(false, Ordering::Relaxed),
            "restart" => {
                control.restart_requested.store(true, Ordering::Relaxed);
                control.running.store(true, Ordering::Relaxed);
            }
            _ => {}
        }
    }
}

const FRONTEND_HTML: &str = include_str!("../frontend/dist/index.html");

fn render_index(ws_port: u16) -> String {
    let injected = format!("<script>window.__SIMHARK_WS_PORT__={ws_port};</script>");
    if let Some((head, tail)) = FRONTEND_HTML.split_once("</head>") {
        format!("{head}{injected}</head>{tail}")
    } else {
        // No </head> tag — fall back to prepending the script to the body.
        format!("{injected}{FRONTEND_HTML}")
    }
}
