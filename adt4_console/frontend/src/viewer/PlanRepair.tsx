// Plan repair v3 in the Plan panel: is goal_manager running, the robot's
// persistent constraints (sent with every goal_manager goal), and goal_manager
// / planner-failure alerts. API.md §9.
import {useState} from "react";
import {errorText, postJSON, sendJSON, useApi} from "../lib/api";
import {fmtTime} from "../lib/fuzzy";
import type {ConstraintFact, GoalSendResult, RepairAlert, RepairStatus} from "../types";
import {Badge} from "../components/ui";

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
