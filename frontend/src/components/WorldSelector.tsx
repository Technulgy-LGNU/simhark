interface WorldSelectorProps {
  worldCount: number;
  selected: number;
  onSelect: (index: number) => void;
}

export default function WorldSelector({
  worldCount,
  selected,
  onSelect,
}: WorldSelectorProps) {
  const safeCount = Math.max(worldCount, 1);
  const disabled = safeCount <= 1;

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30">
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] mb-2 flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <rect x="3" y="3" width="7" height="7" />
          <rect x="14" y="3" width="7" height="7" />
          <rect x="3" y="14" width="7" height="7" />
          <rect x="14" y="14" width="7" height="7" />
        </svg>
        World
      </h2>
      <div className="flex items-center gap-2">
        <select
          value={selected}
          onChange={(e) => onSelect(Number(e.target.value))}
          disabled={disabled}
          className="bg-slate-900/60 border border-slate-700/40 rounded-md text-xs text-slate-100 font-mono px-2 py-1.5 flex-1 focus:outline-none focus:border-cyan-500/60 disabled:opacity-60"
        >
          {Array.from({ length: safeCount }, (_, i) => (
            <option key={i} value={i}>
              world {i}
            </option>
          ))}
        </select>
        <span className="text-[10px] text-slate-500 font-mono">
          of {safeCount}
        </span>
      </div>
    </div>
  );
}
