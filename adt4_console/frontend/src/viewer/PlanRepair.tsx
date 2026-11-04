// Plan repair v3 in the Plan panel: is goal_manager running, the robot's
// persistent constraints (sent with every goal_manager goal), and goal_manager
// / planner-failure alerts. API.md §9.  Plan repair v5: scene changes and the
// console's replan recommendations (SceneRepairPanel). API.md §20.
import {useEffect, useState} from "react";
import {errorText, getJSON, postJSON, sendJSON, useApi} from "../lib/api";
import {fmtTime} from "../lib/fuzzy";
import type {
  ConstraintFact,
  GoalSendResult,
  RepairAlert,
  RepairStatus,
  SceneChangeBatch,
  SceneChangeEntry,
  SceneRecommendation,
  SceneRepairState,
} from "../types";
import {Badge, type Tone} from "../components/ui";
import {ws} from "../ws";

export type SendTone = "ok" | "warn" | "err";

/** What to tell the operator after a goal was sent. */
export function goalSendMessage(label: string, r: Partial<GoalSendResult> | undefined): {tone: SendTone; text: string} {
  if (!r) return {tone: "ok", text: `${label}: ok`};
  if (r.subscribers === 0)
    // published, but nothing matched within the discovery window: the planner is not running
    return {tone: "err", text: `No planner is listening on ${r.topic} — is omniplanner running?`};
  if (r.warning) return {tone: "warn", text: `Sent to omniplanner (${r.topic}). ${capitalize(r.warning)}`};
  if (r.via === "goal_manager") {
    const n = r.constraints?.length ?? 0;
    return {tone: "ok", text: `Sent through plan repair (goal_manager${n ? `, ${n} constraint${n > 1 ? "s" : ""}` : ""})`};
  }
  return {tone: r.ok === false ? "err" : "ok", text: r.topic ? `Sent on ${r.topic}` : `${label}: ok`};
}

const capitalize = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);

/** "forbidden-poi O(14)" / "forbidden-edge p1 p2" -> a constraint (null if malformed). */
export function parseConstraintInput(text: string): ConstraintFact | null {
  const [pred, ...syms] = text.trim().split(/[\s,]+/).filter(Boolean);
  if (!pred) return null;
  const p = pred.toLowerCase();
  const want = p === "forbidden-poi" ? 1 : p === "forbidden-edge" ? 2 : 0;
  if (!want || syms.length !== want) return null;
  return {predicate: p, symbols: syms.map(x => x.replace(/[()]/g, "").toLowerCase())};
}

export const factText = (c: ConstraintFact) => `(${c.predicate} ${c.symbols.join(" ")})`;

export function RepairBadge() {
  const st = useApi<RepairStatus>("api/plan/repair", 10_000);
  if (!st.data) return null;
  return st.data.goal_manager ? (
    <Badge tone="ok" title={`goal_manager on ${st.data.topic}: goals get constraints and replanning`}>
      plan repair on
    </Badge>
  ) : (
    <Badge tone="warn" title={st.data.available ? `nothing subscribes to ${st.data.topic}: PDDL goals go straight to omniplanner` : st.data.error ?? "ROS unavailable"}>
      plan repair off
    </Badge>
  );
}

export function ConstraintsBox({robot, constraints}: {robot: string; constraints: ConstraintFact[]}) {
  const [text, setText] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const path = `api/robots/${encodeURIComponent(robot)}/constraints`;
  const act = async (fn: () => Promise<unknown>) => {
    setErr(null);
    try {
      await fn(); // the WS plan update redraws the list
    } catch (e) {
      setErr(errorText(e));
    }
  };
  const add = () => {
    const c = parseConstraintInput(text);
    if (!c) {
      setErr("Use “forbidden-poi o14” or “forbidden-edge p1 p2”");
      return;
    }
    void act(async () => {
      await postJSON(path, {constraints: [c]});
      setText("");
    });
  };
  return (
    <div className="repair-constraints">
      <div className="row gap between">
        <h4 className="sub-title">Constraints</h4>
        {constraints.length > 0 && (
          <button className="btn btn-xs" onClick={() => act(() => sendJSON("DELETE", path))}>
            Clear all
          </button>
        )}
      </div>
      <p className="muted small">Sent with every plan-repair goal until cleared.</p>
      {constraints.length === 0 ? (
        <div className="muted small">None.</div>
      ) : (
        <ul className="constraint-list">
          {constraints.map(c => (
            <li key={factText(c)}>
              <span className="mono small">{factText(c)}</span>
              <button className="icon-btn sm" aria-label={`Remove ${factText(c)}`} onClick={() => act(() => sendJSON("DELETE", path, {constraints: [c]}))}>
                ×
              </button>
            </li>
          ))}
        </ul>
      )}
      <div className="row gap-sm">
        <input
          className="input mono"
          value={text}
          placeholder="forbidden-poi o14"
          aria-label="New constraint"
          onChange={e => setText(e.target.value)}
          onKeyDown={e => e.key === "Enter" && add()}
        />
        <button className="btn btn-sm" disabled={!text.trim()} onClick={add}>
          Add
        </button>
      </div>
      {err && <div className="inline-msg tone-err small">{err}</div>}
    </div>
  );
}

export function RepairAlerts({alerts}: {alerts: RepairAlert[]}) {
  if (!alerts.length) return null;
  return (
    <>
      <h4 className="sub-title">Plan repair</h4>
      <ul className="result-list small repair-alerts">
        {alerts.slice(-5).reverse().map(a => (
          <li key={a.seq} className={a.kind === "plan_failed" ? "tone-err" : "tone-warn"}>
            <span className="mono muted">{fmtTime(a.t)}</span> {a.kind === "plan_failed" ? "planner failed: " : ""}
            {a.text}
          </li>
        ))}
      </ul>
    </>
  );
}

// ---------------------------------------------------------------- plan repair v5

export const SCENE_REPAIR_TOPIC = "plan/scene_repair";

/** The console's scene change / recommendation state: initial GET + hub pushes. */
export function useSceneRepair(): [SceneRepairState | null, (s: SceneRepairState) => void] {
  const [st, setSt] = useState<SceneRepairState | null>(null);
  useEffect(() => {
    let live = true;
    getJSON<SceneRepairState>("api/plan/scene_repair").then(d => live && setSt(d), () => undefined);
    const off = ws.on(SCENE_REPAIR_TOPIC, d => {
      if (d && typeof d === "object") setSt(d as SceneRepairState);
    });
    return () => {
      live = false;
      off();
    };
  }, []);
  return [st, setSt];
}

/** "awcd/hamilton" -> "awcd" (the chip's class and text). */
export function sourceKind(source: string): string {
  const head = (source || "").split("/")[0].toLowerCase();
  return ["awcd", "sget", "console", "executor"].includes(head) ? head : "other";
}

export function changeText(c: SceneChangeEntry): string {
  return [c.kind, c.symbol, c.label].filter(Boolean).join(" ");
}

const PRIORITY_TONE: Record<SceneRecommendation["priority"], Tone> = {high: "err", normal: "warn", low: "info"};

/** The pending recommendation for ``robot`` (null: every robot's) and the recent
 *  scene change feed. Shown with the plan repair alerts. */
export function SceneRepairPanel({robot}: {robot: string | null}) {
  const [st, setSt] = useSceneRepair();
  // an older console without §20 (or a test stub) answers something else
  if (!st || !Array.isArray(st.recommendations) || !Array.isArray(st.changes)) return null;
  const pending = st.recommendations.filter(r => r.status === "pending" || r.status === "confirming").filter(r => !robot || r.robot === robot);
  const recent = st.changes.slice(-6).reverse();
  const staged = st.staged && st.staged.count > 0 ? st.staged : null;
  if (!pending.length && !recent.length && !staged) return null;
  const refresh = () => getJSON<SceneRepairState>("api/plan/scene_repair").then(setSt, () => undefined);
  return (
    <div className="scene-repair">
      {staged && (
        <div className="inline-msg tone-warn small scene-staged" role="status">
          {staged.count} staged map edit{staged.count > 1 ? "s" : ""} not published yet
          {staged.summary ? ` (${staged.summary})` : ""}: the robots still plan on the old map. Publish them to
          get a replan recommendation.
        </div>
      )}
      {pending.map(r => (
        <RecommendationCard key={r.id} rec={r} onDone={refresh} />
      ))}
      {recent.length > 0 && <SceneChangeFeed batches={recent} mapVersion={st.map_version} />}
    </div>
  );
}

export function RecommendationCard({rec, onDone}: {rec: SceneRecommendation; onDone?: () => void}) {
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [resume, setResume] = useState(false);
  const base = `api/plan/scene_repair/${encodeURIComponent(rec.id)}`;
  const act = async (label: string, path: string, body: unknown) => {
    setBusy(label);
    setErr(null);
    try {
      await postJSON(path, body);
      onDone?.();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };
  const rewritten = rec.rewrites.length > 0 && rec.proposed_goal !== rec.goal.pddl;
  const newSyms = rec.rewrites.map(w => w.to.toUpperCase()).join(", ");
  const disabled = !!busy || rec.status !== "pending";
  return (
    <div className={`scene-rec prio-${rec.priority}`} role="alert" aria-label={`Replan recommendation for ${rec.robot}`}>
      <div className="row gap between">
        <h4 className="sub-title">Replan recommended · {rec.robot}</h4>
        <Badge tone={PRIORITY_TONE[rec.priority] ?? "warn"}>{rec.priority}</Badge>
      </div>
      <p className="small scene-rec-reason">{rec.reason}</p>
      {rec.changes.length > 0 && (
        <ul className="result-list small scene-rec-changes">
          {rec.changes.slice(-5).map((c, i) => (
            <li key={i}>
              <span className={`src-chip src-${sourceKind(c.source ?? rec.source)}`}>{sourceKind(c.source ?? rec.source)}</span> {changeText(c)}
            </li>
          ))}
        </ul>
      )}
      <div className="scene-rec-goals small">
        <div>
          <span className="muted">goal </span>
          <code className="mono">{rec.goal.pddl}</code>
        </div>
        {rewritten && (
          <div>
            <span className="muted">proposed </span>
            <code className="mono">{rec.proposed_goal}</code>
          </div>
        )}
      </div>
      <div className="row gap-sm wrap">
        {rewritten && (
          <button className="btn btn-sm btn-primary" disabled={disabled} onClick={() => act("proposed", `${base}/confirm`, {variant: "proposed"})}>
            Replan with {newSyms}
          </button>
        )}
        <button className={`btn btn-sm${rewritten ? "" : " btn-primary"}`} disabled={disabled} onClick={() => act("original", `${base}/confirm`, {variant: "original"})}>
          Replan same goal
        </button>
        <button className="btn btn-sm" disabled={disabled} onClick={() => act("decline", `${base}/decline`, {resume})}>
          Dismiss
        </button>
        <label className="small muted row gap-sm" title="goal_manager may have paused the robot for this change">
          <input type="checkbox" checked={resume} onChange={e => setResume(e.target.checked)} aria-label="Resume robot on dismiss" /> resume robot
        </label>
      </div>
      {err && <div className="inline-msg tone-err small">{err}</div>}
    </div>
  );
}

export function SceneChangeFeed({batches, mapVersion}: {batches: SceneChangeBatch[]; mapVersion: number | null}) {
  return (
    <>
      <h4 className="sub-title">
        Scene changes{mapVersion != null && <span className="mono small muted"> map v{mapVersion}</span>}
      </h4>
      <ul className="result-list small scene-feed">
        {batches.map(b => (
          <li key={b.id}>
            <span className="mono muted">{fmtTime(b.t)}</span> <span className={`src-chip src-${sourceKind(b.source)}`} title={b.source}>{sourceKind(b.source)}</span>{" "}
            {b.changes.length ? b.changes.slice(0, 3).map(changeText).join(", ") : b.summary || "map edited"}
            {b.changes.length > 3 && <span className="muted"> +{b.changes.length - 3}</span>}
            {b.map_version != null && <span className="mono muted"> v{b.map_version}</span>}
          </li>
        ))}
      </ul>
    </>
  );
}
