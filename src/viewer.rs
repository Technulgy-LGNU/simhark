//! Optional browser-based debug viewer.

use std::io::{Error, Result};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use parking_lot::Mutex;
use serde::Serialize;
use tiny_http::{Header, Method, Response, Server, StatusCode};
use tungstenite::{accept, Message};

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

pub struct ViewerServer {
    world_count: usize,
    field: FieldConfig,
    robot_radius: f64,
    ball_radius: f64,
    selected_world: Arc<AtomicUsize>,
    latest_frame: Arc<Mutex<Option<String>>>,
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

        let http_thread = {
            let ws_port = config.websocket_port();
            thread::spawn(move || run_http_server(http_server, ws_port))
        };

        let ws_thread = {
            let latest_frame = Arc::clone(&latest_frame);
            let selected_world = Arc::clone(&selected_world);
            thread::spawn(move || run_websocket_server(ws_listener, latest_frame, selected_world))
        };

        Ok(Self {
            world_count,
            field: world_config.field.clone(),
            robot_radius: world_config.blue_robots.radius,
            ball_radius: world_config.ball.radius,
            selected_world,
            latest_frame,
            _http_thread: http_thread,
            _ws_thread: ws_thread,
        })
    }

    pub fn selected_world(&self) -> usize {
        self.selected_world
            .load(Ordering::Relaxed)
            .min(self.world_count.saturating_sub(1))
    }

    pub fn publish(&self, state: &WorldState) {
        let frame = ViewerFrame {
            world_count: self.world_count,
            selected_world: self.selected_world(),
            field: &self.field,
            robot_radius: self.robot_radius,
            ball_radius: self.ball_radius,
            state,
        };

        if let Ok(json) = serde_json::to_string(&frame) {
            *self.latest_frame.lock() = Some(json);
        }
    }
}

fn run_http_server(server: Server, ws_port: u16) {
    let html_type = Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).ok();

    for request in server.incoming_requests() {
        let response = match (request.method(), request.url()) {
            (&Method::Get, "/") | (&Method::Get, "/index.html") => {
                let body = INDEX_HTML.replace("__WS_PORT__", &ws_port.to_string());
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
) {
    for stream in listener.incoming() {
        let Ok(stream) = stream else {
            continue;
        };

        let latest_frame = Arc::clone(&latest_frame);
        let selected_world = Arc::clone(&selected_world);
        thread::spawn(move || {
            let Ok(mut websocket) = accept(stream) else {
                return;
            };
            let _ = websocket.get_mut().set_nonblocking(true);

            let mut last_sent = String::new();

            loop {
                match websocket.read() {
                    Ok(Message::Text(text)) => {
                        handle_client_message(text.as_str(), &selected_world)
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

fn handle_client_message(message: &str, selected_world: &AtomicUsize) {
    if let Some(value) = message.strip_prefix("world:") {
        if let Ok(index) = value.trim().parse::<usize>() {
            selected_world.store(index, Ordering::Relaxed);
        }
    }
}

const INDEX_HTML: &str = r#"<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>simhark viewer</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #08111a;
      --panel: rgba(8, 17, 26, 0.88);
      --line: #dceef2;
      --blue: #4db4ff;
      --yellow: #ffd447;
      --ball: #ff8647;
      --text: #eff9fb;
      --muted: #a5c8cf;
    }

    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background:
        radial-gradient(circle at top, rgba(77, 180, 255, 0.16), transparent 30%),
        linear-gradient(180deg, #0a1621 0%, var(--bg) 100%);
      color: var(--text);
      font: 14px/1.4 Inter, system-ui, sans-serif;
      display: grid;
      grid-template-rows: auto 1fr;
    }

    .toolbar {
      display: flex;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
      padding: 12px 16px;
      background: rgba(6, 11, 17, 0.7);
      border-bottom: 1px solid rgba(220, 238, 242, 0.12);
      backdrop-filter: blur(10px);
    }

    .toolbar-group {
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
    }

    .badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 6px 10px;
      border-radius: 999px;
      background: rgba(220, 238, 242, 0.08);
      color: var(--muted);
    }

    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #ff6b6b;
      box-shadow: 0 0 12px rgba(255, 107, 107, 0.6);
    }

    .dot.live {
      background: #2ee68d;
      box-shadow: 0 0 12px rgba(46, 230, 141, 0.8);
    }

    select {
      border: 1px solid rgba(220, 238, 242, 0.14);
      background: rgba(220, 238, 242, 0.08);
      color: var(--text);
      border-radius: 10px;
      padding: 8px 10px;
    }

    .layout {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 300px;
      gap: 16px;
      padding: 16px;
    }

    .stage,
    .panel {
      background: var(--panel);
      border: 1px solid rgba(220, 238, 242, 0.12);
      border-radius: 18px;
      overflow: hidden;
      box-shadow: 0 12px 40px rgba(0, 0, 0, 0.24);
    }

    .stage {
      position: relative;
      min-height: 60vh;
    }

    canvas {
      width: 100%;
      height: 100%;
      display: block;
    }

    .panel {
      padding: 16px;
      display: grid;
      align-content: start;
      gap: 12px;
    }

    .stat {
      padding: 12px;
      border-radius: 14px;
      background: rgba(220, 238, 242, 0.05);
    }

    .label {
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
      margin-bottom: 6px;
    }

    .value {
      font-size: 22px;
      font-weight: 700;
    }

    .mono {
      font-family: ui-monospace, SFMono-Regular, monospace;
      font-size: 13px;
    }

    @media (max-width: 980px) {
      .layout {
        grid-template-columns: 1fr;
      }

      .stage {
        min-height: 48vh;
      }
    }
  </style>
</head>
<body>
  <div class="toolbar">
    <div class="toolbar-group">
      <strong>simhark viewer</strong>
      <span class="badge"><span id="status-dot" class="dot"></span><span id="status-label">connecting</span></span>
    </div>
    <div class="toolbar-group">
      <label for="world-select">World</label>
      <select id="world-select"></select>
    </div>
  </div>

  <div class="layout">
    <div class="stage"><canvas id="field"></canvas></div>
    <div class="panel">
      <div class="stat">
        <div class="label">Frame</div>
        <div id="frame" class="value mono">-</div>
      </div>
      <div class="stat">
        <div class="label">Sim Time</div>
        <div id="sim-time" class="value mono">-</div>
      </div>
      <div class="stat">
        <div class="label">Ball</div>
        <div id="ball" class="mono">-</div>
      </div>
      <div class="stat">
        <div class="label">Blue Robots</div>
        <div id="blue-count" class="mono">-</div>
      </div>
      <div class="stat">
        <div class="label">Yellow Robots</div>
        <div id="yellow-count" class="mono">-</div>
      </div>
      <div class="stat">
        <div class="label">Goals</div>
        <div id="goals" class="mono">-</div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('field');
    const ctx = canvas.getContext('2d');
    const statusDot = document.getElementById('status-dot');
    const statusLabel = document.getElementById('status-label');
    const worldSelect = document.getElementById('world-select');
    const frameEl = document.getElementById('frame');
    const simTimeEl = document.getElementById('sim-time');
    const ballEl = document.getElementById('ball');
    const blueCountEl = document.getElementById('blue-count');
    const yellowCountEl = document.getElementById('yellow-count');
    const goalsEl = document.getElementById('goals');

    let snapshot = null;
    let socket = null;

    function setStatus(connected) {
      statusDot.classList.toggle('live', connected);
      statusLabel.textContent = connected ? 'live' : 'disconnected';
    }

    function ensureWorldOptions(worldCount, selectedWorld) {
      if (worldSelect.options.length !== worldCount) {
        worldSelect.innerHTML = '';
        for (let i = 0; i < worldCount; i += 1) {
          const option = document.createElement('option');
          option.value = String(i);
          option.textContent = `world ${i}`;
          worldSelect.appendChild(option);
        }
      }
      worldSelect.value = String(selectedWorld);
    }

    function connect() {
      const wsPort = Number('__WS_PORT__');
      const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
      socket = new WebSocket(`${protocol}://${location.hostname}:${wsPort}`);

      socket.addEventListener('open', () => {
        setStatus(true);
        if (worldSelect.value) {
          socket.send(`world:${worldSelect.value}`);
        }
      });

      socket.addEventListener('message', (event) => {
        snapshot = JSON.parse(event.data);
        ensureWorldOptions(snapshot.world_count, snapshot.selected_world);
        updateStats(snapshot);
        draw();
      });

      socket.addEventListener('close', () => {
        setStatus(false);
        window.setTimeout(connect, 1000);
      });

      socket.addEventListener('error', () => {
        setStatus(false);
      });
    }

    function updateStats(data) {
      const state = data.state;
      frameEl.textContent = `${state.frame}`;
      simTimeEl.textContent = `${state.sim_time.toFixed(3)} s`;
      ballEl.textContent = `${state.ball.x.toFixed(3)}, ${state.ball.y.toFixed(3)}, ${state.ball.z.toFixed(3)}`;
      blueCountEl.textContent = `${state.blue_robots.filter((robot) => robot.is_on).length} active`;
      yellowCountEl.textContent = `${state.yellow_robots.filter((robot) => robot.is_on).length} active`;
      goalsEl.textContent = `blue=${state.goal_blue} yellow=${state.goal_yellow}`;
    }

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.floor(rect.width * dpr);
      canvas.height = Math.floor(rect.height * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }

    function worldToScreen(field, x, y, width, height) {
      const halfWidth = field.field_length / 2 + field.margin_goal_line + field.goal_depth + 0.2;
      const halfHeight = field.field_width / 2 + field.margin_touch_line + 0.2;
      const scale = Math.min(width / (halfWidth * 2), height / (halfHeight * 2));
      return {
        px: width / 2 + x * scale,
        py: height / 2 - y * scale,
        scale,
      };
    }

    function drawField(field, width, height) {
      const gradient = ctx.createLinearGradient(0, 0, 0, height);
      gradient.addColorStop(0, '#168554');
      gradient.addColorStop(1, '#0d5e3d');
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, width, height);

      const center = worldToScreen(field, 0, 0, width, height);
      const scale = center.scale;
      const fieldW = field.field_length * scale;
      const fieldH = field.field_width * scale;
      const left = width / 2 - fieldW / 2;
      const top = height / 2 - fieldH / 2;

      ctx.strokeStyle = 'rgba(220, 238, 242, 0.92)';
      ctx.lineWidth = Math.max(2, field.field_line_width * scale * 100);
      ctx.strokeRect(left, top, fieldW, fieldH);

      ctx.beginPath();
      ctx.moveTo(width / 2, top);
      ctx.lineTo(width / 2, top + fieldH);
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(width / 2, height / 2, field.field_center_radius * scale, 0, Math.PI * 2);
      ctx.stroke();

      const penaltyW = field.penalty_depth * scale;
      const penaltyH = field.penalty_width * scale;
      ctx.strokeRect(left, height / 2 - penaltyH / 2, penaltyW, penaltyH);
      ctx.strokeRect(left + fieldW - penaltyW, height / 2 - penaltyH / 2, penaltyW, penaltyH);

      const goalW = field.goal_depth * scale;
      const goalH = field.goal_width * scale;
      ctx.strokeRect(left - goalW, height / 2 - goalH / 2, goalW, goalH);
      ctx.strokeRect(left + fieldW, height / 2 - goalH / 2, goalW, goalH);
    }

    function drawRobot(field, robot, radius, color, width, height) {
      if (!robot.is_on) {
        return;
      }

      const { px, py, scale } = worldToScreen(field, robot.x, robot.y, width, height);
      const r = Math.max(radius * scale, 6);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fill();

      ctx.strokeStyle = 'rgba(8, 17, 26, 0.9)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(robot.orientation) * r, py - Math.sin(robot.orientation) * r);
      ctx.stroke();
    }

    function drawBall(field, ball, radius, width, height) {
      const { px, py, scale } = worldToScreen(field, ball.x, ball.y, width, height);
      const r = Math.max(radius * scale * 1.8, 5);
      ctx.fillStyle = '#ff8647';
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fill();

      if (ball.z > radius * 1.2) {
        ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
        ctx.font = '12px ui-monospace, monospace';
        ctx.fillText(`${ball.z.toFixed(2)}m`, px + r + 4, py - r - 4);
      }
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      const width = rect.width;
      const height = rect.height;
      ctx.clearRect(0, 0, width, height);

      if (!snapshot) {
        ctx.fillStyle = '#0f2131';
        ctx.fillRect(0, 0, width, height);
        ctx.fillStyle = '#eff9fb';
        ctx.font = '16px Inter, system-ui, sans-serif';
        ctx.fillText('Waiting for simulation frames...', 24, 40);
        return;
      }

      drawField(snapshot.field, width, height);
      for (const robot of snapshot.state.blue_robots) {
        drawRobot(snapshot.field, robot, snapshot.robot_radius, '#4db4ff', width, height);
      }
      for (const robot of snapshot.state.yellow_robots) {
        drawRobot(snapshot.field, robot, snapshot.robot_radius, '#ffd447', width, height);
      }
      drawBall(snapshot.field, snapshot.state.ball, snapshot.ball_radius, width, height);
    }

    worldSelect.addEventListener('change', () => {
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(`world:${worldSelect.value}`);
      }
    });

    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
    connect();
  </script>
</body>
</html>
"#;
