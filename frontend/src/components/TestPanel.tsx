import { useState } from "react";
import type { TestStatus, TestSuiteSnapshot } from "../hooks/useViewerSocket";

interface TestPanelProps {
  suite: TestSuiteSnapshot | null;
  selectedWorld: number;
  onSelect: (index: number) => void;
}

export default function TestPanel({ suite, selectedWorld, onSelect }: TestPanelProps) {
  const [filter, setFilter] = useState<TestStatus["outcome"] | "all">("all");

  if (!suite) {
    return null;
  }

  const tests =
    filter === "all"
      ? suite.tests
      : suite.tests.filter((test) => test.outcome === filter);

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
          <path d="M9 11l3 3L22 4" />
          <path d="M21 12v7a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11" />
        </svg>
        Tests
      </h2>

      <div className="grid grid-cols-4 gap-1.5">
        <Summary
          label="Run"
          value={suite.running}
          tone="text-slate-300"
          active={filter === "running"}
          onClick={() => setFilter(toggleFilter(filter, "running"))}
        />
        <Summary
          label="Pass"
          value={suite.passed}
          tone="text-emerald-200"
          active={filter === "passed"}
          onClick={() => setFilter(toggleFilter(filter, "passed"))}
        />
        <Summary
          label="Fail"
          value={suite.failed}
          tone="text-red-300"
          active={filter === "failed"}
          onClick={() => setFilter(toggleFilter(filter, "failed"))}
        />
        <Summary
          label="Time"
          value={suite.timed_out}
          tone="text-amber-300"
          active={filter === "timed_out"}
          onClick={() => setFilter(toggleFilter(filter, "timed_out"))}
        />
      </div>

      <div className="h-36 overflow-y-auto space-y-1 pr-2">
        {tests.map((test) => (
          <TestRow
            key={test.world_id}
            test={test}
            selected={test.world_id === selectedWorld}
            onSelect={onSelect}
          />
        ))}
        {tests.length === 0 ? (
          <div className="rounded-md border border-slate-700/30 bg-slate-900/30 px-2 py-3 text-center text-[10px] font-mono text-slate-500">
            no {filter === "all" ? "" : statusLabel(filter)} tests
          </div>
        ) : null}
      </div>
    </div>
  );
}

function Summary({
  label,
  value,
  tone,
  active,
  onClick,
}: {
  label: string;
  value: number;
  tone: string;
  active: boolean;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={`rounded-md border px-1.5 py-1 text-left transition ${
        active
          ? "border-cyan-500/60 bg-cyan-500/10"
          : "border-slate-700/30 bg-slate-900/40 hover:bg-slate-800/50"
      }`}
    >
      <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500">
        {label}
      </div>
      <div className={`font-mono text-xs ${tone}`}>{value}</div>
    </button>
  );
}

function toggleFilter(
  current: TestStatus["outcome"] | "all",
  next: TestStatus["outcome"]
): TestStatus["outcome"] | "all" {
  return current === next ? "all" : next;
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
  const copyName = async () => {
    try {
      await copyText(test.name);
    } catch (err) {
      console.error("failed to copy test name", err);
    }
  };

  return (
    <div
      className={`block w-full overflow-hidden text-left rounded-md border px-2 py-1.5 transition ${
        selected
          ? "border-cyan-500/60 bg-cyan-500/10"
          : "border-slate-700/30 bg-slate-900/40 hover:bg-slate-800/50"
      }`}
    >
      <div className="flex min-w-0 items-center gap-2">
        <button
          type="button"
          onClick={() => onSelect(test.world_id)}
          title={test.name}
          className="min-w-0 flex-1 text-left"
        >
          <div className="flex min-w-0 items-center justify-between gap-2">
            <span className="min-w-0 flex-1 truncate text-xs text-slate-200">{test.name}</span>
            <span className={`shrink-0 text-[9px] font-mono uppercase ${statusTone(test.outcome)}`}>
              {statusLabel(test.outcome)}
            </span>
          </div>
          <div className="mt-0.5 flex min-w-0 items-center justify-between gap-2">
            <span className="min-w-0 truncate text-[10px] font-mono text-slate-500">
              world {test.world_id} · frame {test.frame}
            </span>
          </div>
        </button>
        <button
          type="button"
          onClick={copyName}
          title="Copy test name"
          className="shrink-0 rounded-md border border-slate-700/30 bg-slate-950/30 p-1 text-slate-400 transition hover:border-cyan-500/50 hover:text-cyan-200"
        >
          <svg
            className="h-3.5 w-3.5"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <rect x="9" y="9" width="13" height="13" rx="2" />
            <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1" />
          </svg>
        </button>
      </div>
      {test.message ? (
        <div className="text-[10px] text-red-300 truncate mt-0.5">{test.message}</div>
      ) : null}
    </div>
  );
}

async function copyText(text: string) {
  if (navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(text);
    return;
  }

  const textarea = document.createElement("textarea");
  textarea.value = text;
  textarea.style.position = "fixed";
  textarea.style.left = "-9999px";
  textarea.style.top = "0";
  document.body.appendChild(textarea);
  textarea.focus();
  textarea.select();
  document.execCommand("copy");
  document.body.removeChild(textarea);
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
