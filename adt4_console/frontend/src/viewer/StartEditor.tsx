// An agent's start in the roster (API.md §15.1): typed map x / y (metres) or
// lat / lon (through the GPS anchor), "live pose" for a bound robot, pick on the
// map, and what is wrong with it (outside the map, off the places graph — with a
// snap to the nearest place). The start shown and edited is the open mission's
// when the agent is in it (what is previewed and simulated), else the roster's;
// a change goes through moveStart (a new revision of the open mission).
import {useEffect, useRef, useState} from "react";
import {inputOf, moveResetsRun, startFromInput, startLockReason, startWarning, type StartInput} from "../lib/agentStart";
import {moveStart, useMission, type Agent} from "../stores/mission";
import {armMissionTool} from "../stores/tools";

export type CoordMode = "xy" | "lonlat";

const sameInput = (a: StartInput, b: StartInput) => JSON.stringify(a) === JSON.stringify(b);

export function StartEditor({agent, mode}: {agent: Agent; mode: CoordMode}) {
  const geo = useMission(s => s.geo);
  const bounds = useMission(s => s.world?.bounds ?? null);
  const run = useMission(s => s.run);
  const missionAgent = useMission(s => s.current?.agents.find(a => a.id === agent.id) ?? null);
  const previewStart = useMission(s => s.preview?.agents[agent.id]?.start ?? null);
  const offgraph = useMission(s => s.preview?.issues.find(i => i.code === "start_offgraph" && i.agent === agent.id) ?? null);
  const picking = useMission(s => s.tool.kind === "start" && s.tool.agent === agent.id);
  const inMission = !!missionAgent;
  const start = inMission ? (missionAgent.start ?? null) : (agent.start ?? null);
  const binding = (inMission ? missionAgent.binding : agent.binding) ?? agent.binding ?? null;
  const live = !start && !!binding;
  const lock = inMission ? startLockReason(run) : null;
  const resets = inMission && moveResetsRun(run);
  const m: CoordMode = mode === "lonlat" && geo ? "lonlat" : "xy";
  const shown = inputOf(start, m, geo);
  const [fields, setFields] = useState<StartInput>(shown);
  const [error, setError] = useState<string | null>(null);
  const dirty = useRef(false);
  // follow the start (a drag, a map pick, a revision) unless the operator is typing
  const key = JSON.stringify(shown);
  useEffect(() => {
    if (!dirty.current) {
      setFields(shown);
      setError(null);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key]);

  const commit = () => {
    if (!dirty.current) return;
    const r = startFromInput(fields, geo);
    if ("error" in r) {
      setError(r.error);
      return;
    }
    dirty.current = false;
    setError(null);
    if (start && r.start[0] === start[0] && r.start[1] === start[1]) {
      setFields(shown);
      return;
    }
    void moveStart(agent.id, r.start);
  };
  const revert = () => {
    dirty.current = false;
    setError(null);
    setFields(shown);
  };
  const edit = (patch: Partial<Record<"x" | "y" | "lat" | "lon", string>>) => {
    dirty.current = true;
    setError(null);
    setFields(f => ({...f, ...patch}) as StartInput);
  };
  const onKey = (e: React.KeyboardEvent) => {
    if (e.key === "Enter") {
      e.preventDefault();
      commit();
    } else if (e.key === "Escape") {
      e.preventDefault();
      revert();
    }
  };
  const warn = start ? startWarning(start, bounds) : null;
  const pending = dirty.current && !sameInput(fields, shown);
  const field = (k: "x" | "y" | "lat" | "lon", label: string, title: string) => (
    <label className="start-field">
      <span className="muted">{label}</span>
      <input
        className={`input input-xs mono ${error ? "invalid" : ""}`}
        inputMode="decimal"
        enterKeyHint="done"
        aria-label={`${agent.id} start ${title}`}
        aria-invalid={!!error}
        title={title}
        disabled={!!lock || live}
        value={live ? "" : ((fields as Record<string, string>)[k] ?? "")}
        placeholder={live ? (previewStart ? (k === "x" ? previewStart[0].toFixed(1) : k === "y" ? previewStart[1].toFixed(1) : "live") : "live") : ""}
        onChange={e => edit({[k]: e.target.value})}
        onKeyDown={onKey}
      />
    </label>
  );
  return (
    <div
      className="start-edit"
      data-testid={`start-edit-${agent.id}`}
      onBlur={e => {
        // commit when the focus leaves the start fields (not when it moves x → y)
        if (!e.currentTarget.contains(e.relatedTarget as Node | null)) commit();
      }}
    >
      <div className="start-row">
        <span className="muted small start-label">start</span>
        {m === "xy" ? (
          <>
            {field("x", "x", "map x (m)")}
            {field("y", "y", "map y (m)")}
          </>
        ) : (
          <>
            {field("lat", "lat", "latitude (°)")}
            {field("lon", "lon", "longitude (°)")}
          </>
        )}
        {pending && (
          <button className="btn btn-xs btn-primary" onMouseDown={e => e.preventDefault()} onClick={commit} title="Set this start (Enter)">
            Set
          </button>
        )}
        <button
          className={`btn btn-xs ${picking ? "on" : ""}`}
          disabled={!!lock}
          aria-pressed={picking}
          title="Click the map to put the start there (or drag the agent on the map)"
          onClick={() => armMissionTool(picking ? {kind: "none"} : {kind: "start", agent: agent.id})}
        >
          ⌖
        </button>
        {binding && (
          <label className="mini-check" title={`Start where ${binding} is when the mission runs (its live pose)`}>
            <input
              type="checkbox"
              checked={live}
              disabled={!!lock}
              onChange={e => {
                dirty.current = false;
                if (e.target.checked) void moveStart(agent.id, null);
                else {
                  const p = previewStart ?? useMission.getState().world?.start_hint ?? null;
                  if (p) void moveStart(agent.id, [p[0], p[1]]);
                }
              }}
            />
            live pose
          </label>
        )}
      </div>
      {error && (
        <div className="small start-msg err" role="alert">
          ✖ {error}
        </div>
      )}
      {!error && lock && <div className="small start-msg muted">{lock}</div>}
      {!error && !lock && resets && <div className="small start-msg muted">Moving a start resets the simulation.</div>}
      {!error && warn && <div className="small start-msg warn">⚠ {warn}</div>}
      {!error && offgraph && (
        <div className="small start-msg warn" data-testid={`start-offgraph-${agent.id}`}>
          ⚠ {offgraph.offgraph_m != null ? `${Number(offgraph.offgraph_m).toFixed(0)} m off the places graph` : "off the places graph"}
          {Array.isArray(offgraph.snap) && !lock && (
            <button className="btn btn-xs" onClick={() => void moveStart(agent.id, offgraph.snap as number[])} title="Move the start to the nearest mapped place">
              Snap to nearest place
            </button>
          )}
        </div>
      )}
    </div>
  );
}
