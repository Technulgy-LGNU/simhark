import type { GameStateInfo, GoalSummary } from "../hooks/useViewerSocket";

interface GameStatePanelProps {
  gameState: GameStateInfo | null;
  goals: GoalSummary;
}

const COMMAND_COLORS: Record<string, string> = {
  HALT: "bg-red-600/30 text-red-300 border-red-500/40",
  STOP: "bg-amber-500/25 text-amber-200 border-amber-400/40",
  NORMAL_START: "bg-emerald-500/25 text-emerald-200 border-emerald-400/40",
  FORCE_START: "bg-emerald-500/25 text-emerald-200 border-emerald-400/40",
  RUNNING: "bg-emerald-500/25 text-emerald-200 border-emerald-400/40",
  PREPARE_KICKOFF_BLUE:
    "bg-blue-500/25 text-blue-200 border-blue-400/40",
  PREPARE_KICKOFF_YELLOW:
    "bg-amber-500/25 text-amber-200 border-amber-400/40",
  PREPARE_PENALTY_BLUE:
    "bg-blue-500/25 text-blue-200 border-blue-400/40",
  PREPARE_PENALTY_YELLOW:
    "bg-amber-500/25 text-amber-200 border-amber-400/40",
  DIRECT_FREE_BLUE: "bg-blue-500/25 text-blue-200 border-blue-400/40",
  DIRECT_FREE_YELLOW: "bg-amber-500/25 text-amber-200 border-amber-400/40",
  INDIRECT_FREE_BLUE: "bg-blue-500/25 text-blue-200 border-blue-400/40",
  INDIRECT_FREE_YELLOW:
    "bg-amber-500/25 text-amber-200 border-amber-400/40",
  TIMEOUT_BLUE: "bg-slate-500/30 text-slate-200 border-slate-400/40",
  TIMEOUT_YELLOW: "bg-slate-500/30 text-slate-200 border-slate-400/40",
  BALL_PLACEMENT_BLUE: "bg-blue-500/25 text-blue-200 border-blue-400/40",
  BALL_PLACEMENT_YELLOW:
    "bg-amber-500/25 text-amber-200 border-amber-400/40",
  UNKNOWN: "bg-slate-700/40 text-slate-300 border-slate-600/40",
};

function commandClass(cmd: string) {
  return COMMAND_COLORS[cmd] ?? COMMAND_COLORS.UNKNOWN;
}

export default function GameStatePanel({
  gameState,
  goals,
}: GameStatePanelProps) {
  const counts = gameState?.state_counts ?? {};
  const sortedCounts = Object.entries(counts).sort(
    ([, a], [, b]) => b - a
  );

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2 shrink-0 overflow-hidden">
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <circle cx="12" cy="12" r="9" />
          <path d="M12 7v5l3 2" />
        </svg>
        Game State
      </h2>

      <div className="grid grid-cols-2 gap-2">
        <ScoreBox
          name={gameState?.blue_name ?? "Blue"}
          score={goals.blue}
          flashing={goals.blue_active}
          accent="text-blue-300"
          border="border-blue-500/30"
          bg="bg-blue-500/10"
        />
        <ScoreBox
          name={gameState?.yellow_name ?? "Yellow"}
          score={goals.yellow}
          flashing={goals.yellow_active}
          accent="text-amber-300"
          border="border-amber-500/30"
          bg="bg-amber-500/10"
        />
      </div>

      {gameState ? (
        <>
          <div className="flex items-center justify-between gap-2">
            <span
              className={`inline-flex items-center px-2 py-1 rounded-md text-[11px] font-mono uppercase tracking-wide border ${commandClass(
                gameState.command
              )}`}
            >
              {gameState.command}
            </span>
            <span className="font-mono text-[10px] text-slate-500">
              #{gameState.command_counter}
            </span>
          </div>

          {gameState.stage && (
            <div className="text-[10px] text-slate-400 font-mono">
              stage: {gameState.stage}
            </div>
          )}

          {sortedCounts.length > 0 && (
            <div className="rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5 py-2">
              <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500 mb-1">
                Command counts
              </div>
              <div className="space-y-0.5 max-h-40 overflow-y-auto">
                {sortedCounts.map(([cmd, count]) => (
                  <div
                    key={cmd}
                    className="flex justify-between items-center font-mono text-[10px]"
                  >
                    <span className="text-slate-400 truncate pr-2">{cmd}</span>
                    <span className="text-slate-200">{count}</span>
                  </div>
                ))}
              </div>
            </div>
          )}
        </>
      ) : (
        <div className="text-[10px] text-slate-500 italic">
          No referee data — showing goals tracked from world state
        </div>
      )}
    </div>
  );
}

function ScoreBox({
  name,
  score,
  flashing,
  accent,
  border,
  bg,
}: {
  name: string;
  score: number;
  flashing: boolean;
  accent: string;
  border: string;
  bg: string;
}) {
  return (
    <div
      className={`rounded-lg ${bg} border ${border} px-2.5 py-1.5 transition-shadow ${
        flashing ? "ring-2 ring-emerald-400/70 shadow-[0_0_12px_rgba(52,211,153,0.4)]" : ""
      }`}
    >
      <div className="text-[9px] uppercase tracking-[0.15em] text-slate-400 truncate">
        {name}
      </div>
      <div className={`font-mono text-xl font-bold ${accent}`}>{score}</div>
    </div>
  );
}
