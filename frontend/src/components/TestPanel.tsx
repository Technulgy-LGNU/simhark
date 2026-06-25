import type { TestStatus, TestSuiteSnapshot } from "../hooks/useViewerSocket";

interface TestPanelProps {
  suite: TestSuiteSnapshot | null;
  selectedWorld: number;
  onSelect: (index: number) => void;
}

export default function TestPanel({ suite, selectedWorld, onSelect }: TestPanelProps) {
  if (!suite) {
    return null;
  }

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2 min-h-0">
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path d="M9 11l3 3L22 4" />
          <path d="M21 12v7a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11" />
        </svg>
        Tests
      </h2>

      <div className="grid grid-cols-4 gap-1.5">
        <Summary label="Run" value={suite.running} tone="text-slate-300" />
        <Summary label="Pass" value={suite.passed} tone="text-emerald-200" />
        <Summary label="Fail" value={suite.failed} tone="text-red-300" />
        <Summary label="Time" value={suite.timed_out} tone="text-amber-300" />
      </div>

      <div className="max-h-40 overflow-y-auto space-y-1 pr-2">
        {suite.tests.map((test) => (
          <TestRow
            key={test.world_id}
            test={test}
            selected={test.world_id === selectedWorld}
            onSelect={onSelect}
          />
        ))}
      </div>
    </div>
  );
}

function Summary({
  label,
  value,
  tone,
}: {
  label: string;
  value: number;
  tone: string;
}) {
  return (
    <div className="rounded-md bg-slate-900/40 border border-slate-700/30 px-1.5 py-1">
      <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500">
        {label}
      </div>
      <div className={`font-mono text-xs ${tone}`}>{value}</div>
    </div>
  );
}

function TestRow({
  test,
  selected,
  onSelect,
}: {
  test: TestStatus;
  selected: boolean;
  onSelect: (index: number) => void;
}) {
  return (
    <button
      onClick={() => onSelect(test.world_id)}
      className={`w-full text-left rounded-md border px-2 py-1.5 transition ${
        selected
          ? "border-cyan-500/60 bg-cyan-500/10"
          : "border-slate-700/30 bg-slate-900/40 hover:bg-slate-800/50"
      }`}
    >
      <div className="flex items-center justify-between gap-2">
        <span className="text-xs text-slate-200 truncate">{test.name}</span>
        <span className={`text-[9px] font-mono uppercase ${statusTone(test.outcome)}`}>
          {statusLabel(test.outcome)}
        </span>
      </div>
      <div className="flex items-center justify-between gap-2 mt-0.5">
        <span className="text-[10px] font-mono text-slate-500">
          world {test.world_id} · frame {test.frame}
        </span>
      </div>
      {test.message ? (
        <div className="text-[10px] text-red-300 truncate mt-0.5">{test.message}</div>
      ) : null}
    </button>
  );
}

function statusLabel(status: TestStatus["outcome"]): string {
  switch (status) {
    case "passed":
      return "passed";
    case "failed":
      return "failed";
    case "timed_out":
      return "timeout";
    case "running":
      return "running";
  }
}

function statusTone(status: TestStatus["outcome"]): string {
  switch (status) {
    case "passed":
      return "text-emerald-300";
    case "failed":
      return "text-red-300";
    case "timed_out":
      return "text-amber-300";
    case "running":
      return "text-slate-400";
  }
}
