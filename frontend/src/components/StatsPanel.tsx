import type { ViewerFrame } from "../hooks/useViewerSocket";

interface StatsPanelProps {
  frame: ViewerFrame | null;
}

export default function StatsPanel({ frame }: StatsPanelProps) {
  const state = frame?.state;

  const blueActive = state?.blue_robots.filter((r) => r.is_on).length ?? 0;
  const yellowActive = state?.yellow_robots.filter((r) => r.is_on).length ?? 0;

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2">
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path d="M3 3v18h18" />
          <path d="M7 14l4-4 4 4 6-6" />
        </svg>
        Simulation
      </h2>

      <div className="grid grid-cols-2 gap-2">
        <Stat label="Frame" value={state ? state.frame.toString() : "—"} />
        <Stat
          label="Sim Time"
          value={state ? `${state.sim_time.toFixed(2)} s` : "—"}
        />
      </div>

      <div className="grid grid-cols-2 gap-2">
        <Stat
          label="Blue"
          value={`${blueActive} active`}
          accent="text-blue-400"
        />
        <Stat
          label="Yellow"
          value={`${yellowActive} active`}
          accent="text-amber-400"
        />
      </div>

      <div className="rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5 py-2">
        <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500 mb-1">
          Ball
        </div>
        <div className="font-mono text-xs text-slate-200">
          {state
            ? `${state.ball.x.toFixed(2)}, ${state.ball.y.toFixed(2)}, ${state.ball.z.toFixed(2)}`
            : "—"}
        </div>
        <div className="font-mono text-[10px] text-slate-500 mt-0.5">
          v ={" "}
          {state
            ? `${state.ball.vx.toFixed(2)}, ${state.ball.vy.toFixed(2)}, ${state.ball.vz.toFixed(2)}`
            : "—"}
        </div>
      </div>
    </div>
  );
}

function Stat({
  label,
  value,
  accent,
}: {
  label: string;
  value: string;
  accent?: string;
}) {
  return (
    <div className="rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5 py-1.5">
      <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500">
        {label}
      </div>
      <div className={`font-mono text-sm ${accent ?? "text-slate-100"}`}>
        {value}
      </div>
    </div>
  );
}
