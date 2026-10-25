// The Mission tab (API.md §15): agent roster, areas (search areas and no-go
// zones, AreaTools.tsx), mock objects, the selected mission's timeline with the
// run's transport controls, issues (with fixes for targets the map no longer
// has), and quad detections to approve. Missions are written in the chat
// (propose_mission); omniplanner can plan ground-only ones. Map editing is the
// Map tab (MapEditPanel.tsx).
import {useEffect, useMemo, useRef, useState} from "react";
import {Badge, Empty, Segmented, Spinner, Toggle} from "../components/ui";
import {withToken} from "../lib/auth";
import {fmtLonLat, mapToLonLat, mapToUtm, utmZone} from "../lib/geo";
import {
  abort,
  addAgent,
  annotate,
  closeMission,
  decideCandidate,
  deleteMission,
  dropTask,
  execute,
  fitMission,
  MAX_AGENTS,
  openMission,
  planWithOmniplanner,
  rebaseMission,
  refreshList,
  removeAgent,
  replanText,
  rosterDiff,
  saveMission,
  simCmd,
  undoStart,
  updateAgent,
  useMission,
  withOperatorInputs,
  type AgentKind,
  type Candidate,
  type Issue,
  type Mission,
  type MissionSummary,
  type RunState,
} from "../stores/mission";
import {useRobots} from "../stores/robots";
import {useRobotColor3D} from "./background";
import {agentColors} from "./MissionOverlay";
import {useViewer} from "../stores/viewer";
import {useViewStyle} from "../stores/viewStyle";
import {prefillChat, useChat} from "../stores/chat";
import {armMissionTool} from "../stores/tools";
import {undoLabel} from "../stores/mapEdit";
import {AreasSection, MockObjects} from "./AreaTools";
import {StartEditor, type CoordMode} from "./StartEditor";
import {startLockReason} from "../lib/agentStart";
import type {ChatItem} from "./ChatPanel";
import type {TranscriptItem} from "../lib/chatTranscript";

const TASK_ICON: Record<string, string> = {goto: "→", inspect: "◎", explore_area: "▦", request_image: "📷", wait_for: "⏸", return_home: "⌂", hold: "◼"};

function fmtS(s: number | null | undefined): string {
  if (s == null) return "–";
  const m = Math.floor(s / 60);
  return m ? `${m}:${String(Math.round(s % 60)).padStart(2, "0")}` : `${s.toFixed(0)} s`;
}

function Roster() {
  const {roster, current, geo, snapStarts, startUndo, run} = useMission();
  const live = Object.keys(useRobots(s => s.state));
  const robotColor = useRobotColor3D();
  const colors = useMemo(() => agentColors(roster, robotColor), [roster, robotColor]);
  const [kind, setKind] = useState<AgentKind>("spot");
  const [coord, setCoord] = useState<CoordMode>("xy");
  const inMission = new Set(current?.agents.map(a => a.id) ?? []);
  const lastUndo = startUndo[startUndo.length - 1];
  const undoLocked = !!lastUndo?.mission && lastUndo.mission === current?.id && !!startLockReason(run);
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>
          Agents <span className="muted small">{roster.length}/{MAX_AGENTS}</span>
        </h4>
        <span className="grow" />
        {lastUndo && (
          <button className="btn btn-xs" disabled={undoLocked} onClick={() => void undoStart()} title={`Put ${lastUndo.agent}'s start back where it was (Ctrl+Z when it is the newest change)`}>
            ↶ {undoLabel(`move ${lastUndo.agent}`)}
          </button>
        )}
      </header>
      {roster.length === 0 && <div className="muted small">Add the agents the mission may use; the chat plans with these ids and starts.</div>}
      {roster.length > 0 && (
        <div className="row gap wrap start-opts">
          {geo && (
            <Segmented
              ariaLabel="Start coordinates"
              value={coord}
              onChange={setCoord}
              options={[
                {value: "xy", label: "map x/y", title: "Starts in map metres"},
                {value: "lonlat", label: "lat/lon", title: "Starts as latitude / longitude (through the map's GPS anchor)"},
              ]}
            />
          )}
          <Toggle checked={snapStarts} onChange={v => useMission.getState().set({snapStarts: v})} label="snap to places" title="A dragged start lands on the nearest place node (within 4 m); Shift inverts it while dragging" />
          <span className="muted small">Drag an agent on the map to move its start.</span>
        </div>
      )}
      <ul className="mission-roster">
        {roster.map(a => (
          <li key={a.id}>
            <div className="roster-main">
              <span className="roster-id">
                <span className="swatch" style={{background: colors[a.id]}} />
                <AgentIdInput id={a.id} taken={roster.map(r => r.id)} />
                {inMission.has(a.id) && <Badge tone="info">in mission</Badge>}
                <span className="grow" />
                <button className="btn btn-xs" aria-label={`Remove ${a.id}`} onClick={() => removeAgent(a.id)}>
                  ×
                </button>
              </span>
              <span className="roster-selects">
              <select className="input input-xs" value={a.kind} aria-label="Kind" onChange={e => updateAgent(a.id, {kind: e.target.value as AgentKind})}>
                <option value="spot">spot</option>
                <option value="husky">husky</option>
                <option value="quad">quad</option>
              </select>
              <select className="input input-xs" value={a.binding ?? ""} aria-label="Real robot" title="Bind to a real robot for execution (simulated otherwise)" onChange={e => updateAgent(a.id, {binding: e.target.value || null})}>
                <option value="">sim only</option>
                {a.kind === "quad" ? <option value="penn_quad">Penn quad</option> : live.map(r => <option key={r} value={r}>{r}</option>)}
                {a.binding && a.binding !== "penn_quad" && !live.includes(a.binding) && <option value={a.binding}>{a.binding}</option>}
              </select>
              </span>
            </div>
            <StartEditor agent={a} mode={coord} />
          </li>
        ))}
      </ul>
      {roster.length < MAX_AGENTS && (
        <div className="row gap">
          <select className="input input-xs" value={kind} onChange={e => setKind(e.target.value as AgentKind)} aria-label="New agent kind">
            <option value="spot">Spot</option>
            <option value="husky">Husky</option>
            <option value="quad">Quad</option>
          </select>
          <button className="btn btn-xs" onClick={() => addAgent(kind)}>
            Add agent
          </button>
          {live
            .filter(r => !roster.some(a => a.binding === r))
            .slice(0, MAX_AGENTS - roster.length)
            .map(r => (
              <button key={r} className="btn btn-xs" title={`Add the live robot ${r} (bound; starts at its pose)`} onClick={() => addAgent("spot", r, null)}>
                + {r}
              </button>
            ))}
        </div>
      )}
    </section>
  );
}

function Timeline({run}: {run: RunState | null}) {
  const {current, preview} = useMission();
  const robotColor = useRobotColor3D();
  const colors = useMemo(() => agentColors(current?.agents ?? [], robotColor), [current, robotColor]);
  const ref = useRef<HTMLDivElement>(null);
  if (!current || !preview) return null;
  const span = Math.max(preview.makespan ?? 0, run?.t ?? 0, ...Object.values(run?.tasks ?? {}).map(t => t.times?.[1] ?? 0), 10);
  const pct = (s: number) => `${(100 * s) / span}%`;
  const seek = (e: React.MouseEvent) => {
    const el = ref.current;
    if (!el || run?.kind === "live") return;
    const r = el.getBoundingClientRect();
    void simCmd("seek", Math.max(0, ((e.clientX - r.left) / r.width) * span));
  };
  return (
    <div className="mission-timeline" ref={ref} onClick={seek} title={run?.kind === "live" ? undefined : "Click to jump the simulation to that time"}>
      {current.agents.map(a => (
        <div key={a.id} className="tl-row">
          <span className="tl-agent" style={{color: colors[a.id]}}>
            {a.id}
          </span>
          <div className="tl-track">
            {current.tasks
              .filter(t => t.agent === a.id)
              .map(t => {
                const tp = preview.tasks[t.id];
                const rs = run?.tasks[t.id];
                const t0 = rs?.times?.[0] ?? tp?.t_start ?? 0;
                const t1 = rs?.times?.[1] ?? (rs?.state === "active" ? (run?.t ?? t0) : tp?.t_end ?? t0);
                const state = rs?.state ?? "planned";
                return (
                  <div
                    key={t.id}
                    className={`tl-bar tl-${state}`}
                    style={{left: pct(t0), width: `max(4px, ${pct(Math.max(0, t1 - t0))})`, background: colors[a.id]}}
                    title={`${t.id}: ${t.type}${t.target ? ` ${JSON.stringify(t.target)}` : ""} · ${state}${rs?.note ? ` · ${rs.note}` : ""} · ${fmtS(t0)}–${fmtS(t1)}`}
                  >
                    {TASK_ICON[t.type]}
                  </div>
                );
              })}
          </div>
        </div>
      ))}
      {run && <div className="tl-cursor" style={{left: `calc(64px + (100% - 64px) * ${run.t / span})`}} />}
      <div className="tl-axis muted small">
        <span>0</span>
        <span>{fmtS(span)}</span>
      </div>
    </div>
  );
}

function Transport({run}: {run: RunState | null}) {
  const [auto, setAuto] = useState(false);
  const {current, preview} = useMission();
  const live = run?.kind === "live";
  const errors = preview?.issues.filter(i => i.level === "error").length ?? 0;
  const bound = current?.agents.filter(a => a.binding) ?? [];
  const [confirming, setConfirming] = useState(false);
  const why = disabledWhy(errors, bound.length, !!run && !live);
  return (
    <>
    <div className="mission-transport">
      {!run || live ? (
        <button className="btn btn-sm btn-primary" disabled={errors > 0} onClick={() => void simCmd("new", undefined, auto).then(() => simCmd("play")).then(() => fitMission())} title={errors ? "Fix the errors first" : "Simulate the mission (Space)"}>
          ▶ Simulate
        </button>
      ) : run.playing ? (
        <button className="btn btn-sm" onClick={() => void simCmd("pause")}>
          ❚❚ Pause
        </button>
      ) : (
        <button className="btn btn-sm btn-primary" onClick={() =>
            void simCmd(run.finished ? "reset" : "play").then(() => {
              if (run.finished) void simCmd("play");
            })
          }>
          {run.finished ? "↻ Replay" : "▶ Play"}
        </button>
      )}
      {run && !live && (
        <>
          <select className="input input-xs" value={run.speed} onChange={e => void simCmd("speed", Number(e.target.value))} aria-label="Speed">
            {[1, 2, 5, 10, 20].map(s => (
              <option key={s} value={s}>
                {s}×
              </option>
            ))}
          </select>
          <button className="btn btn-xs" onClick={() => void simCmd("step", 5)} title="Advance 5 s">
            +5 s
          </button>
          <button className="btn btn-xs" onClick={() => void simCmd("reset")}>
            Reset
          </button>
        </>
      )}
      <Toggle
        checked={auto}
        onChange={v => {
          setAuto(v);
          if (run && !live) void simCmd("auto_approve", v);
        }}
        label="auto-approve"
        title="Approve every detection automatically (rehearsals)"
      />
      <span className="grow" />
      {run && (
        <span className="mono small">
          {live ? "LIVE " : ""}t={fmtS(run.t)}
          {run.finished ? " ✓" : ""}
        </span>
      )}
      {live ? (
        <button className="btn btn-sm btn-danger" onClick={() => void abort()}>
          Abort
        </button>
      ) : confirming ? (
        <span className="row gap">
          <span className="small">Send to {bound.map(a => a.binding).join(", ")}?</span>
          <button
            className="btn btn-xs btn-danger"
            onClick={() => {
              setConfirming(false);
              void execute();
            }}
          >
            Execute
          </button>
          <button className="btn btn-xs" onClick={() => setConfirming(false)}>
            Cancel
          </button>
        </span>
      ) : (
        <button className="btn btn-sm" disabled={!bound.length || errors > 0} onClick={() => setConfirming(true)} title={bound.length ? "Run it on the bound robots (unbound agents stay simulated)" : "Bind agents to real robots in the roster first"}>
          Execute…
        </button>
      )}
    </div>
    {why && (
      <div className="small muted disabled-why" data-testid="transport-why">
        {why}
      </div>
    )}
    </>
  );
}

/** "Update mission from roster": the roster's starts / robots / kinds, the no-go
 *  zones and mock objects into the open mission (a new revision); off, saying
 *  so, when the mission already has them. */
function UpdateFromRoster({mission}: {mission: Mission}) {
  const diff = rosterDiff(mission, useMission());
  const busy = useMission(s => s.busy === "save");
  const why = diff.length ? `Differs: ${diff.join(", ")}` : "Nothing to update: the mission already has the roster's starts and robots, the no-go zones and the mock objects.";
  return (
    <>
      <button
        className="btn btn-xs"
        disabled={!diff.length || busy}
        title={diff.length ? `Put into this mission (new revision): ${diff.join(", ")}` : why}
        onClick={() => void saveMission(withOperatorInputs(mission), true)}
        data-testid="update-from-roster"
      >
        Update mission from roster{diff.length ? ` (${diff.length})` : ""}
      </button>
      <span className="small muted roster-diff" data-testid="roster-diff">
        {diff.length ? why : "matches the roster"}
      </span>
    </>
  );
}

/** A mission's time in the picker: "14:05" today, else "Oct 23 14:05". */
export function fmtWhen(sec: number, now: Date = new Date()): string {
  const d = new Date(sec * 1000);
  if (!Number.isFinite(d.getTime())) return "";
  const time = d.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
  return d.toDateString() === now.toDateString() ? time : `${d.toLocaleDateString([], {month: "short", day: "numeric"})} ${time}`;
}

/** Why Simulate / Execute are disabled, shown under the transport row (null: both usable). */
export function disabledWhy(errors: number, bound: number, simulating: boolean): string | null {
  if (errors > 0) return `Simulate and Execute are off until the ${errors === 1 ? "error" : `${errors} errors`} below ${errors === 1 ? "is" : "are"} fixed.`;
  if (!bound && !simulating) return "Execute is off: bind agents to real robots in Agents first (simulation works without).";
  return null;
}

function CandidateRow({c, mission, canCommit}: {c: Candidate; mission: string; canCommit: boolean}) {
  const busy = useMission(s => s.busy === `cand:${c.id}`);
  return (
    <li className={`mission-cand ${c.status}`} id={`cand-${c.id}`} data-cand={c.id}>
      <button className="thumb" onClick={() => useMission.getState().set({openImage: {mission, image: c.image, candidate: c.id}})} title="Open the image">
        <img src={withToken(`api/missions/${encodeURIComponent(mission)}/image/${c.image}`)} alt={c.label} loading="lazy" />
      </button>
      <div className="grow">
        <div>
          <b>{c.label}</b> <span className="muted small">by {c.agent} · t={fmtS(c.t)}</span>
        </div>
        <div className="mono small muted">
          ({c.pos[0].toFixed(1)}, {c.pos[1].toFixed(1)}){c.sym ? ` → ${c.sym}` : ""}
        </div>
        {c.status === "pending" ? (
          <div className="row gap">
            <button className="btn btn-xs btn-primary" disabled={busy} onClick={() => void decideCandidate(c.id, true, false)} title="Accept for this run only">
              Accept
            </button>
            {canCommit && (
              <button className="btn btn-xs" disabled={busy} onClick={() => void decideCandidate(c.id, true, true)} title="Accept and add it to the Neo4j map (ground robots can go to it)">
                Add to map
              </button>
            )}
            <button className="btn btn-xs" disabled={busy} onClick={() => void decideCandidate(c.id, false)}>
              Reject
            </button>
            {busy && <Spinner />}
          </div>
        ) : (
          <div className="row gap">
            <Badge tone={c.status === "approved" ? "ok" : "off"}>{c.status === "approved" && inMap(c) ? `in map ${c.sym}` : c.status === "approved" ? "accepted (this run)" : c.status}</Badge>
            {c.status === "approved" && !inMap(c) && canCommit && (
              <button className="btn btn-xs" disabled={busy} onClick={() => void decideCandidate(c.id, true, true)} title="Add it to the Neo4j map too (ground robots can go to it)">
                Add to map
              </button>
            )}
            {busy && <Spinner />}
          </div>
        )}
      </div>
    </li>
  );
}

/** An accepted detection that is in the Neo4j map (not only in the run). */
export const inMap = (c: Pick<Candidate, "status" | "sym">) => c.status === "approved" && !!c.sym && !c.sym.startsWith("sim:");

/** The detection viewer: the stamped image, its candidate box, and drag-to-box a new detection. */
export function ImageViewer() {
  const open = useMission(s => s.openImage);
  const run = useMission(s => s.run);
  const [box, setBox] = useState<number[] | null>(null);
  const [label, setLabel] = useState("");
  const drag = useRef<number[] | null>(null);
  const imgRef = useRef<HTMLImageElement>(null);
  useEffect(() => setBox(null), [open]);
  if (!open) return null;
  const cand = run?.candidates.find(c => c.id === open.candidate);
  const toPx = (e: React.PointerEvent): number[] => {
    const el = imgRef.current!;
    const r = el.getBoundingClientRect();
    return [((e.clientX - r.left) / r.width) * el.naturalWidth, ((e.clientY - r.top) / r.height) * el.naturalHeight];
  };
  const W = imgRef.current?.naturalWidth || 640;
  const H = imgRef.current?.naturalHeight || 480;
  const rect = (b: number[], cls: string) => (
    <div className={cls} style={{left: `${(100 * Math.min(b[0], b[2])) / W}%`, top: `${(100 * Math.min(b[1], b[3])) / H}%`, width: `${(100 * Math.abs(b[2] - b[0])) / W}%`, height: `${(100 * Math.abs(b[3] - b[1])) / H}%`}} />
  );
  return (
    <div className="mission-imgview" role="dialog" aria-label="Quad image">
      <header className="row gap">
        <b>Quad image {open.image}</b>
        {cand && <span className="small">candidate {cand.label}</span>}
        <span className="grow" />
        <button className="btn btn-xs" onClick={() => useMission.getState().set({openImage: null})}>
          Close
        </button>
      </header>
      <div
        className="imgwrap"
        onPointerDown={e => {
          const p = toPx(e);
          drag.current = p;
          setBox([...p, ...p]);
          (e.target as HTMLElement).setPointerCapture?.(e.pointerId);
        }}
        onPointerMove={e => drag.current && setBox([...drag.current, ...toPx(e)])}
        onPointerUp={() => (drag.current = null)}
      >
        <img ref={imgRef} draggable={false} src={withToken(`api/missions/${encodeURIComponent(open.mission)}/image/${open.image}`)} alt="quad nadir image" />
        {cand && rect(cand.bbox, "box box-cand")}
        {box && Math.abs(box[2] - box[0]) > 3 && rect(box, "box box-new")}
      </div>
      <div className="row gap">
        <span className="small muted">{!box || Math.abs(box[2] - box[0]) < 4 ? "Drag a box around something to report it." : !label.trim() ? "Type what it is, then Report." : "Report it as a detection."}</span>
        <input className="input input-xs" placeholder="label, e.g. car" value={label} onChange={e => setLabel(e.target.value)} aria-label="Label" />
        <button
          className="btn btn-xs btn-primary"
          disabled={!box || !label.trim() || Math.abs(box[2] - box[0]) < 4}
          onClick={() => {
            const b = [Math.min(box![0], box![2]), Math.min(box![1], box![3]), Math.max(box![0], box![2]), Math.max(box![1], box![3])];
            void annotate(open.image, b, label.trim());
            setBox(null);
          }}
        >
          Report
        </button>
      </div>
    </div>
  );
}

/** "  ⚠ 1 error" after a mission in the picker (targets the map no longer has, missing starts). */
export function errorBadge(m: Pick<MissionSummary, "errors">): string {
  const n = m.errors ?? 0;
  return n ? `  ⚠ ${n} error${n === 1 ? "" : "s"}` : "";
}

/** One issue; a target the map no longer has (e.g. renumbered by a re-ingest)
 *  offers its fixes: pick a new one on the map, drop the task, re-plan in the chat. */
export function IssueRow({issue, mission, problems}: {issue: Issue; mission: Mission; problems: string[]}) {
  const picking = useMission(s => s.tool.kind === "retarget" && s.tool.task === issue.task);
  const busy = useMission(s => s.busy === "save");
  const fixable = issue.code === "missing_target" && !!issue.task && mission.tasks.some(t => t.id === issue.task);
  return (
    <li className={issue.level} data-code={issue.code}>
      {issue.level === "error" ? "✖" : "⚠"} {issue.msg}
      {fixable && (
        <span className="issue-fixes">
          <button
            className={`btn btn-xs ${picking ? "on" : ""}`}
            aria-pressed={picking}
            onClick={() => armMissionTool(picking ? {kind: "none"} : {kind: "retarget", task: issue.task!})}
            title="Click an object, a place or a map point: it becomes this task's target (a new revision)"
          >
            Pick on map
          </button>
          <button className="btn btn-xs" disabled={busy} onClick={() => void dropTask(issue.task!)} title="Remove this task from the mission (a new revision)">
            Drop task
          </button>
          <button
            className="btn btn-xs"
            onClick={() => prefillChat(replanText(mission, problems))}
            title="Put the mission and its problems in the chat box, to re-plan it with the assistant"
          >
            Re-plan in chat
          </button>
        </span>
      )}
    </li>
  );
}

function CursorReadout() {
  const geo = useMission(s => s.geo);
  const [p, setP] = useState<number[] | null>(null);
  useEffect(() => {
    const on = (e: Event) => setP((e as CustomEvent<number[]>).detail);
    window.addEventListener("adt4-map-hover", on);
    return () => window.removeEventListener("adt4-map-hover", on);
  }, []);
  if (!p) return null;
  return (
    <div className="mono small muted">
      map ({p[0].toFixed(1)}, {p[1].toFixed(1)})
      {geo && (
        <>
          {" · "}
          {fmtLonLat(...mapToLonLat(geo, p[0], p[1]))}
          {" · UTM "}
          {utmZone(geo.anchor.epsg)} {mapToUtm(geo.anchor, p[0], p[1]).map(v => v.toFixed(1)).join(" ")}
        </>
      )}
    </div>
  );
}

/** An agent's id, renamed on Enter / blur (Escape reverts). Edited as a draft: the
 *  roster rows are keyed by id, so renaming on every keystroke remounted the row,
 *  dropped the focus after one letter and sent the next keys to the map shortcuts. */
export function AgentIdInput({id, taken}: {id: string; taken: string[]}) {
  const [draft, setDraft] = useState(id);
  useEffect(() => setDraft(id), [id]);
  const commit = () => {
    const next = draft.trim();
    if (next && next !== id && !taken.includes(next)) updateAgent(id, {id: next});
    else setDraft(id);
  };
  return (
    <input
      className="input input-xs mono"
      value={draft}
      aria-label="Agent id"
      title="Rename: Enter or click away to apply, Esc to cancel"
      onChange={e => setDraft(e.target.value.replace(/[^A-Za-z0-9_-]/g, ""))}
      onBlur={commit}
      onKeyDown={e => {
        if (e.key === "Enter") (e.target as HTMLInputElement).blur();
        else if (e.key === "Escape") {
          setDraft(id);
          requestAnimationFrame(() => (e.target as HTMLInputElement).blur());
        }
      }}
    />
  );
}

export function MissionPanel() {
  const s = useMission();
  const {current, preview, run, list, error, geo, layers, satellite, satOpacity} = s;
  const grid = useViewStyle(v => v.grid);
  const patchStyle = useViewStyle(v => v.patch);
  const [omniText, setOmniText] = useState("");
  const [planner, setPlanner] = useState<"chat" | "omniplanner">("chat");
  useEffect(() => {
    void refreshList();
  }, []);
  const neo4j = Object.values(useViewer(v => v.sources)).some(x => x.kind === "neo4j");
  const issues = preview?.issues ?? [];
  const pending = run?.candidates.filter(c => c.status === "pending").length ?? 0;
  return (
    <div className="mission-panel">
      <section className="vsec">
        <header className="vsec-head">
          <h4>Mission</h4>
          <span className="grow" />
          <select
            className="input input-xs"
            value={current?.id ?? ""}
            onChange={e => (e.target.value ? void openMission(e.target.value) : closeMission())}
            aria-label="Mission"
          >
            <option value="">{list.length ? "— pick a mission —" : "no missions yet"}</option>
            {list.some(m => m.saved) && (
              <optgroup label="Saved">
                {list
                  .filter(m => m.saved)
                  .map(m => (
                    <option key={m.id} value={m.id}>
                      ★ {m.name || m.id} · {fmtWhen(m.created_at)} ({m.agents} agents{m.decisions ? ", run saved" : ""}){errorBadge(m)}
                    </option>
                  ))}
              </optgroup>
            )}
            <optgroup label="Drafts">
              {list
                .filter(m => !m.saved)
                .map(m => (
                  <option key={m.id} value={m.id}>
                    {m.name || m.id} · {fmtWhen(m.created_at)} (r{m.revision}, {m.agents} agents){errorBadge(m)}
                  </option>
                ))}
            </optgroup>
          </select>
          {current && (
            <button className="btn btn-xs" onClick={() => fitMission()} title="Frame the mission's agents, paths and areas on the map (F)" data-testid="fit-mission-panel">
              ⌖ Fit
            </button>
          )}
          {current && (
            <button className="btn btn-xs" onClick={() => void deleteMission(current.id)} title="Delete this mission">
              🗑
            </button>
          )}
        </header>
        <Segmented
          ariaLabel="Planner"
          value={planner}
          onChange={setPlanner}
          options={[
            {value: "chat", label: "Chat planner"},
            {value: "omniplanner", label: "omniplanner (ground)"},
          ]}
        />
        {planner === "chat" ? (
          <div className="muted small">
            Describe the mission in the chat, e.g. <span className="mono">quad1 search A1 for a crate; when found spot1 inspects it while spot2 guards the gate</span>. It uses the agents
            and areas below.
          </div>
        ) : (
          <div className="row gap">
            <input className="input input-sm grow" placeholder="e.g. spot1 go to the nearest tree, spot2 visit the sign" value={omniText} onChange={e => setOmniText(e.target.value)} aria-label="omniplanner command" />
            <button className="btn btn-sm" disabled={!omniText.trim() || s.busy === "omni"} onClick={() => void planWithOmniplanner(omniText.trim())}>
              {s.busy === "omni" ? <Spinner /> : "Plan"}
            </button>
          </div>
        )}
        {error && (
          <div className="msg msg-err small" role="alert">
            {error}{" "}
            <button className="btn btn-xs" onClick={() => s.set({error: null})}>
              ×
            </button>
          </div>
        )}
        {current && preview ? (
          <>
            <div className="row gap wrap small">
              <b title={current.text || undefined}>{current.name || current.text.slice(0, 60) || current.id}</b>
              <Badge tone="info">r{current.revision}</Badge>
              <Badge tone="off">{current.planner}</Badge>
              <span className="muted">~{fmtS(preview.makespan)}</span>
              <span className="grow" />
              {current.chat && (
                <button
                  className="btn btn-xs"
                  title="Put this mission's saved chat back in the chat pane, to continue the conversation"
                  onClick={() => useChat.setState({items: current.chat!.items as ChatItem[], transcript: current.chat!.transcript as TranscriptItem[], busy: false, turnStart: 0})}
                >
                  Load its chat
                </button>
              )}
              {(current.decisions?.length ?? 0) > 0 && (
                <button className="btn btn-xs" title="Simulate it again with the saved approvals applied at the same times" onClick={() => void simCmd("replay")}>
                  ↻ Replay saved run
                </button>
              )}
              <UpdateFromRoster mission={current} />
            </div>
            <Transport run={run} />
            <Timeline run={run} />
            {preview.map?.stale && (
              <div className="mission-stale small" role="status" data-testid="mission-map-stale">
                <span>
                  ⚠ {preview.map.unknown ? "This mission may be stale: it was planned before map versions were recorded." : `The map changed since planning (v${preview.map.planned_map_version} → v${preview.map.map_version}${preview.map.edits_since ? `, ${preview.map.edits_since} edit(s)` : ""}).`}{" "}
                  {issues.some(i => i.code === "missing_target") ? "Some targets are no longer in the map: fix them below." : "Check the plan, then keep it."}
                </span>
                <button className="btn btn-xs" onClick={() => void rebaseMission(current.id)} title="Keep this plan on the current map (records its version)">
                  Accept current map
                </button>
              </div>
            )}
            {issues.filter(i => i.code !== "map_stale").length > 0 && (
              <ul className="mission-issues">
                {issues
                  .filter(i => i.code !== "map_stale")
                  .map((i, k) => (
                    <IssueRow key={k} issue={i} mission={current} problems={issues.filter(x => x.level === "error").map(x => x.msg)} />
                  ))}
              </ul>
            )}
            <ul className="mission-tasks small">
              {current.tasks.map(t => {
                const rs = run?.tasks[t.id];
                return (
                  <li key={t.id} className={`ts-${rs?.state ?? "planned"}`}>
                    <span className="mono">{t.agent}</span> {TASK_ICON[t.type]} {t.type.replace("_", " ")}{" "}
                    <span className="mono">{t.target ? (Array.isArray(t.target) ? `(${t.target.map(v => Number(v).toFixed(1)).join(", ")})` : t.target) : t.params?.query ? `“${String(t.params.query)}”` : ""}</span>
                    {t.depends_on?.length ? <span className="muted"> after {t.depends_on.join(", ")}</span> : null}
                    {rs && <span className="muted"> · {rs.state}</span>}
                    {rs?.note && <span className="muted"> · {rs.note}</span>}
                  </li>
                );
              })}
            </ul>
          </>
        ) : (
          <Empty title="No mission selected">Plan one in the chat, or pick a saved mission.</Empty>
        )}
      </section>
      {run && run.candidates.length > 0 && (
        <section className="vsec">
          <header className="vsec-head">
            <h4>
              Detections {pending > 0 && <Badge tone="warn">{pending} to review</Badge>}
            </h4>
          </header>
          <ul className="mission-cands">
            {run.candidates.map(c => (
              <CandidateRow key={c.id} c={c} mission={run.mission} canCommit={neo4j} />
            ))}
          </ul>
        </section>
      )}
      {run && run.images > 0 && (
        <div className="small muted pad">
          {run.images} quad image(s) taken.{" "}
          <button
            className="btn btn-xs"
            onClick={() => {
              const last = [...run.events].reverse().find(e => e.kind === "image");
              if (last) s.set({openImage: {mission: run.mission, image: String(last.image)}});
            }}
          >
            Open the latest
          </button>
        </div>
      )}
      <Roster />
      <AreasSection />
      <MockObjects />
      <section className="vsec">
        <header className="vsec-head">
          <h4>Satellite</h4>
        </header>
        {geo ? (
          <>
            <div className="row gap">
              <Toggle checked={satellite} onChange={v => s.set({satellite: v})} label="show" />
              <Toggle checked={grid} onChange={v => patchStyle({grid: v})} label="grid" title="The ground grid over the imagery" />
              <input type="range" min={0.2} max={1} step={0.05} value={satOpacity} onChange={e => s.set({satOpacity: Number(e.target.value)})} aria-label="Satellite opacity" />
            </div>
            <div className="small muted">
              GPS anchor {utmZone(geo.anchor.epsg)}
              {geo.anchor.rms != null ? `, fit RMS ${geo.anchor.rms.toFixed(2)} m` : ""} · {layers.map(l => `${l.id}${l.cached ? ` (${l.cached} tiles)` : ""}${l.live ? " live" : ""}`).join(", ") || "no tiles cached"}
              {layers[0]?.attribution ? ` · imagery © ${layers[0].attribution}` : ""}
            </div>
            <CursorReadout />
          </>
        ) : (
          <div className="small muted">This map has no GPS anchor (scripts/geo/anchor_from_gps.py), so there is no satellite view.</div>
        )}
      </section>
    </div>
  );
}

/** Scroll the Mission tab to a detection and flash it (the panel may still be
 *  opening: retried for a moment). */
export function revealCandidate(id: string, tries = 10) {
  const el = document.getElementById(`cand-${id}`);
  if (!el) {
    if (tries > 0) window.setTimeout(() => revealCandidate(id, tries - 1), 80);
    return;
  }
  el.scrollIntoView({block: "center", behavior: "smooth"});
  el.classList.add("flash");
  window.setTimeout(() => el.classList.remove("flash"), 1600);
}

/** Over the map while detections wait for review: "quad1 found “crate” —
 *  review". Stays until reviewed, decided or dismissed; a new detection brings
 *  it back. ``onReview`` brings the Mission tab forward. */
export function DetectionToast({onReview}: {onReview: () => void}) {
  const run = useMission(s => s.run);
  const [seen, setSeen] = useState<ReadonlySet<string>>(new Set());
  const pending = run?.candidates.filter(c => c.status === "pending" && !seen.has(c.id)) ?? [];
  if (!pending.length) return null;
  const c = pending[pending.length - 1];
  const hide = () => setSeen(new Set([...seen, ...pending.map(p => p.id)]));
  return (
    <div className="toast detect-toast" role="alert" data-testid="detect-toast">
      <span>
        <b>{c.agent}</b> found “{c.label}” — review{pending.length > 1 ? ` (+${pending.length - 1} more)` : ""}
      </span>
      <button
        className="btn btn-xs btn-primary"
        onClick={() => {
          hide();
          onReview();
          revealCandidate(c.id);
        }}
      >
        Review
      </button>
      <button className="btn btn-xs" aria-label="Dismiss" title="Hide until the next detection (they stay in the Mission tab)" onClick={hide}>
        ×
      </button>
    </div>
  );
}
