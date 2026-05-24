import { useRef, useEffect, useCallback } from "react";
import type { FieldConfig, RobotState, ViewerFrame } from "../hooks/useViewerSocket";

interface FieldCanvasProps {
  frame: ViewerFrame | null;
}

const FIELD_GREEN_LIGHT = "#1a5c34";
const FIELD_GREEN_DARK = "#0d3320";
const LINE_COLOR = "#ffffff";
const LINE_GLOW_COLOR = "rgba(255, 255, 255, 0.15)";
const BALL_COLOR = "#ff8c00";
const BALL_GLOW = "rgba(255, 140, 0, 0.4)";
const BLUE_COLOR = "#3b82f6";
const YELLOW_COLOR = "#f59e0b";
const PADDING = 36;

export default function FieldCanvas({ frame }: FieldCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const frameRef = useRef<ViewerFrame | null>(frame);

  // Keep latest frame in a ref so the resize observer can redraw without
  // re-binding.
  frameRef.current = frame;

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = container.getBoundingClientRect();
    canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    canvas.style.width = `${rect.width}px`;
    canvas.style.height = `${rect.height}px`;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;

    ctx.fillStyle = "#070d15";
    ctx.fillRect(0, 0, w, h);

    const snapshot = frameRef.current;
    if (!snapshot) {
      ctx.fillStyle = "#475569";
      ctx.font = '14px "Inter", system-ui';
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("Waiting for simulation frames…", w / 2, h / 2);
      return;
    }

    const field = snapshot.field;
    const robotRadius = snapshot.robot_radius;
    const ballRadius = snapshot.ball_radius;

    drawField(ctx, w, h, field);

    const fieldLength = field.field_length;
    const fieldWidth = field.field_width;
    const halfBoundsX =
      fieldLength / 2 + field.margin_goal_line + field.goal_depth + 0.2;
    const halfBoundsY = fieldWidth / 2 + field.margin_touch_line + 0.2;
    const scale = Math.min(
      (w - 2 * PADDING) / (halfBoundsX * 2),
      (h - 2 * PADDING) / (halfBoundsY * 2)
    );
    const offsetX = w / 2;
    const offsetY = h / 2;
    const toCanvas = (fx: number, fy: number): [number, number] => [
      offsetX + fx * scale,
      offsetY - fy * scale,
    ];

    drawFieldLines(ctx, toCanvas, scale, field);

    for (const robot of snapshot.state.blue_robots) {
      if (!robot.is_on) continue;
      drawRobot(ctx, toCanvas, scale, robotRadius, robot, BLUE_COLOR);
    }
    for (const robot of snapshot.state.yellow_robots) {
      if (!robot.is_on) continue;
      drawRobot(ctx, toCanvas, scale, robotRadius, robot, YELLOW_COLOR);
    }

    drawBall(
      ctx,
      toCanvas,
      scale,
      ballRadius,
      snapshot.state.ball
    );

    ctx.fillStyle = "rgba(71, 85, 105, 0.7)";
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(
      `${fieldLength.toFixed(2)}m × ${fieldWidth.toFixed(2)}m`,
      PADDING,
      h - 10
    );
  }, []);

  useEffect(() => {
    draw();
  }, [frame, draw]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver(() => draw());
    observer.observe(container);
    return () => observer.disconnect();
  }, [draw]);

  return (
    <div ref={containerRef} className="w-full h-full relative">
      <canvas ref={canvasRef} className="absolute inset-0" />
    </div>
  );
}

function drawField(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  field: FieldConfig
) {
  const gradient = ctx.createLinearGradient(0, 0, w, h);
  gradient.addColorStop(0, FIELD_GREEN_DARK);
  gradient.addColorStop(0.5, FIELD_GREEN_LIGHT);
  gradient.addColorStop(1, FIELD_GREEN_DARK);
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);
  // Suppress unused-arg warning for `field`; the param is kept for parity
  // with the original implementation should we need it later.
  void field;
}

function drawFieldLines(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  field: FieldConfig
) {
  const drawGlowLine = (
    x1: number,
    y1: number,
    x2: number,
    y2: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  };

  const drawGlowArc = (
    cx: number,
    cy: number,
    r: number,
    a1: number,
    a2: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.arc(cx, cy, r, a1, a2);
    ctx.stroke();
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, r, a1, a2);
    ctx.stroke();
  };

  const drawGlowRect = (
    rx: number,
    ry: number,
    rw: number,
    rh: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.strokeRect(rx, ry, rw, rh);
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(rx, ry, rw, rh);
  };

  const fieldLength = field.field_length;
  const fieldWidth = field.field_width;
  const [lx, ty] = toCanvas(-fieldLength / 2, fieldWidth / 2);
  drawGlowRect(lx, ty, fieldLength * scale, fieldWidth * scale);

  const [cx1, cy1] = toCanvas(0, fieldWidth / 2);
  const [cx2, cy2] = toCanvas(0, -fieldWidth / 2);
  drawGlowLine(cx1, cy1, cx2, cy2);

  const [cc, ccY] = toCanvas(0, 0);
  drawGlowArc(cc, ccY, field.field_center_radius * scale, 0, Math.PI * 2);

  // Penalty boxes
  const penaltyW = field.penalty_depth * scale;
  const penaltyH = field.penalty_width * scale;
  const [plx, ply] = toCanvas(-fieldLength / 2, field.penalty_width / 2);
  drawGlowRect(plx, ply, penaltyW, penaltyH);
  const [prx, pry] = toCanvas(
    fieldLength / 2 - field.penalty_depth,
    field.penalty_width / 2
  );
  drawGlowRect(prx, pry, penaltyW, penaltyH);

  // Goals
  const goalW = field.goal_depth * scale;
  const goalH = field.goal_width * scale;
  const [glx, gly] = toCanvas(
    -fieldLength / 2 - field.goal_depth,
    field.goal_width / 2
  );
  drawGlowRect(glx, gly, goalW, goalH);
  const [grx, gry] = toCanvas(fieldLength / 2, field.goal_width / 2);
  drawGlowRect(grx, gry, goalW, goalH);
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  robotRadius: number,
  robot: RobotState,
  color: string
) {
  const [rx, ry] = toCanvas(robot.x, robot.y);
  const r = Math.max(robotRadius * scale, 8);

  const glowGrad = ctx.createRadialGradient(rx, ry, r, rx, ry, r * 2.5);
  glowGrad.addColorStop(
    0,
    color === BLUE_COLOR ? "rgba(59,130,246,0.18)" : "rgba(245,158,11,0.18)"
  );
  glowGrad.addColorStop(1, "rgba(0,0,0,0)");
  ctx.fillStyle = glowGrad;
  ctx.beginPath();
  ctx.arc(rx, ry, r * 2.5, 0, Math.PI * 2);
  ctx.fill();

  const bodyGrad = ctx.createRadialGradient(
    rx - r * 0.3,
    ry - r * 0.3,
    0,
    rx,
    ry,
    r
  );
  if (color === BLUE_COLOR) {
    bodyGrad.addColorStop(0, "#60a5fa");
    bodyGrad.addColorStop(1, "#2563eb");
  } else {
    bodyGrad.addColorStop(0, "#fbbf24");
    bodyGrad.addColorStop(1, "#d97706");
  }
  ctx.beginPath();
  ctx.arc(rx, ry, r, 0, Math.PI * 2);
  ctx.fillStyle = bodyGrad;
  ctx.fill();
  ctx.strokeStyle = "rgba(255,255,255,0.6)";
  ctx.lineWidth = 1.5;
  ctx.stroke();

  const dirLen = r + 8;
  const dx = Math.cos(robot.orientation) * dirLen;
  const dy = -Math.sin(robot.orientation) * dirLen;
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(rx + dx, ry + dy);
  ctx.strokeStyle = color === BLUE_COLOR ? "#93c5fd" : "#fcd34d";
  ctx.lineWidth = 2.5;
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(rx + dx, ry + dy, 2.5, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();

  const fontSize = Math.max(r * 0.85, 9);
  ctx.font = `bold ${fontSize}px "Inter", system-ui`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.strokeStyle = "rgba(0, 0, 0, 0.7)";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.strokeText(String(robot.id), rx, ry);
  ctx.fillStyle = "#ffffff";
  ctx.fillText(String(robot.id), rx, ry);
}

function drawBall(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  ballRadius: number,
  ball: { x: number; y: number; z: number }
) {
  const [bx, by] = toCanvas(ball.x, ball.y);
  const r = Math.max(ballRadius * scale * 1.8, 5);

  const glowGrad = ctx.createRadialGradient(bx, by, r * 0.5, bx, by, r * 4);
  glowGrad.addColorStop(0, BALL_GLOW);
  glowGrad.addColorStop(1, "rgba(255, 140, 0, 0)");
  ctx.fillStyle = glowGrad;
  ctx.beginPath();
  ctx.arc(bx, by, r * 4, 0, Math.PI * 2);
  ctx.fill();

  const ballGrad = ctx.createRadialGradient(
    bx - r * 0.3,
    by - r * 0.3,
    0,
    bx,
    by,
    r
  );
  ballGrad.addColorStop(0, "#ffb347");
  ballGrad.addColorStop(0.7, BALL_COLOR);
  ballGrad.addColorStop(1, "#cc6600");
  ctx.beginPath();
  ctx.arc(bx, by, r, 0, Math.PI * 2);
  ctx.fillStyle = ballGrad;
  ctx.fill();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.5)";
  ctx.lineWidth = 1;
  ctx.stroke();

  if (ball.z > ballRadius * 1.5) {
    ctx.fillStyle = "rgba(255,255,255,0.9)";
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(`${ball.z.toFixed(2)}m`, bx + r + 4, by - r - 2);
  }
}
