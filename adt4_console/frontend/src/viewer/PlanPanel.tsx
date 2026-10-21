import {useEffect, useState} from "react";
import {useViewer} from "../stores/viewer";
import {useRobots} from "../stores/robots";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {fmtTime} from "../lib/fuzzy";
import type {GoalSendResult, PlanRobot, RobotPlan} from "../types";
import {Badge, Empty, ErrorBox, Loading, Segmented, Spinner} from "../components/ui";
import {currentActionIndex, useRobotColorOf} from "./planUtil";
import {parseSym} from "../codec";
import {LeaseControl, LeaseRefusalNotice} from "./LeaseControl";
import {FocusChip} from "./RobotFocusPicker";
import {useFocusedRobot} from "./useRobotFocus";
import {ConstraintsBox, goalSendMessage, RepairAlerts, RepairBadge, type SendTone} from "./PlanRepair";

/** "O(12)" -> "o12" (omniplanner lower-cases NodeSymbol names in PDDL). */
export function pddlSymbol(sym: string): string {
  const {prefix, index} = parseSym(sym);
  return index < 0 ? sym.toLowerCase() : `${prefix.toLowerCase()}${index}`;
}

export function visitGoal(syms: string[]): string {
  const facts = syms.map(s => `(visited-${parseSym(s).prefix.toUpperCase() === "O" ? "object" : "place"} ${pddlSymbol(s)})`);
  return facts.length === 1 ? facts[0] : `(and ${facts.join(" ")})`;
}

type Tool = "goto" | "pddl" | "language";

/** POST a goal; PDDL goals go through plan repair v3's goal_manager when it
 *  runs (``via``), else straight to omniplanner with a ``warning`` (API.md §9). */
export async function sendGoal(robot: string, kind: string, body: unknown) {
  return postJSON<GoalSendResult>(`api/robots/${encodeURIComponent(robot)}/goal/${kind}`, body);
}

export function PlanPanel() {
  const robots = useApi<PlanRobot[]>("api/plan/robots");
  const {planRobot: chosen, set, setFocus, gotoDraft, pickMode, pddlDraft} = useViewer();
  // a robot focus decides the plan robot; "all robots" keeps the panel's own pick
  const focused = useFocusedRobot();
  const planRobot = focused ?? chosen;
  const plan = useRobots(s => (planRobot ? s.plan[planRobot] : undefined));
  const [tool, setTool] = useState<Tool>("goto");
  const [lang, setLang] = useState("");
  const [langDomain, setLangDomain] = useState<"default" | "Pddl" | "goto_points">("default");
  const [pddlPlugin, setPddlPlugin] = useState("");
  const [busy, setBusy] = useState<string | null>(null);
  const [msg, setMsg] = useState<{tone: SendTone; text: string} | null>(null);

  const list = robots.data ?? [];
  useEffect(() => {
    if (!focused && !planRobot && list.length) set({planRobot: list[0].robot});
    // keep the store in step (chat sends to planRobot)
    else if (focused && chosen !== focused) set({planRobot: focused});
  }, [list, planRobot, focused, chosen, set]);
  const current = list.find(r => r.robot === planRobot);
  const kinds = new Set(current?.plugins.map(p => p.kind) ?? []);
  const pddlPlugins = current?.plugins.filter(p => p.kind === "pddl") ?? [];
  useEffect(() => {
    if (pddlPlugins.length && !pddlPlugins.some(p => p.name === pddlPlugin)) setPddlPlugin(pddlPlugins[0].name);
  }, [pddlPlugins, pddlPlugin]);
  useEffect(() => {
    if (pddlDraft) setTool("pddl");
  }, [pddlDraft]);

  // initial plan fetch (WS keeps it current)
  useEffect(() => {
    if (!planRobot) return;
    getJSON<RobotPlan>(`api/robots/${encodeURIComponent(planRobot)}/plan`)
      .then(p => useRobots.setState(s => ({plan: {...s.plan, [planRobot]: p}})))
      .catch(() => undefined);
  }, [planRobot]);

  const run = async (label: string, fn: () => Promise<unknown>) => {
    setBusy(label);
    setMsg(null);
    try {
      setMsg(goalSendMessage(label, (await fn()) as Partial<GoalSendResult> | undefined));
    } catch (e) {
      setMsg({tone: "err", text: errorText(e)});
    } finally {
      setBusy(null);
    }
  };

  if (robots.loading && !robots.data) return <Loading label="Loading planners" />;
  if (robots.error && !robots.data) return <ErrorBox error={robots.error} onRetry={robots.reload} />;
  if (!list.length) return <Empty title="No planning robots">The console exposes planning for robots given with --robots.</Empty>;
  if (focused && !current)
    return (
      <Empty title={`No planner for ${focused}`}>
        The console exposes planning for robots given with --robots ({list.map(r => r.robot).join(", ")}).{" "}
        <button className="btn btn-xs" onClick={() => setFocus("all")}>
          All robots
        </button>
      </Empty>
    );

  return (
    <div className="plan-panel">
      <div className="row gap between">
        {focused ? (
          <FocusChip robot={focused} />
        ) : (
          <select className="input" value={planRobot ?? ""} onChange={e => set({planRobot: e.target.value})} aria-label="Plan robot">
            {list.map(r => (
              <option key={r.robot} value={r.robot}>
                {r.robot}
              </option>
            ))}
          </select>
        )}
        <div className="row gap-sm">
          <RepairBadge />
          {planRobot && <LeaseControl robot={planRobot} />}
          <button className="btn btn-sm" disabled={!!busy} onClick={() => run("Pause", () => postJSON(`api/robots/${planRobot}/executor/pause`))}>
            Pause
          </button>
          <button className="btn btn-sm" disabled={!!busy} onClick={() => run("Resume", () => postJSON(`api/robots/${planRobot}/executor/resume`))}>
            Resume
          </button>
        </div>
      </div>

      <Segmented<Tool>
        ariaLabel="Goal tool"
        value={tool}
        onChange={setTool}
        options={[
          {value: "goto", label: "Goto points"},
          {value: "pddl", label: "PDDL"},
          {value: "language", label: "Language"},
        ]}
      />

      {tool === "goto" && (
        <div className="goal-tool">
          <div className="row gap between">
            <button className={`btn btn-sm ${pickMode === "goto" ? "btn-primary" : ""}`} onClick={() => set({pickMode: pickMode === "goto" ? "inspect" : "goto"})}>
              {pickMode === "goto" ? "Picking: click map (on)" : "Pick on map"}
            </button>
            {gotoDraft.length > 0 && (
              <button className="btn btn-xs" onClick={() => set({gotoDraft: []})}>
                Clear
              </button>
            )}
          </div>
          <p className="muted small">Click a place, object or the mesh; each pick snaps to the nearest place (or object) in the planner's map.</p>
          {gotoDraft.length === 0 ? (
            <div className="muted small pad">No points yet.</div>
          ) : (
            <ol className="goto-list">
              {gotoDraft.map((g, i) => (
                <li key={g.sym + i}>
                  <span className="goto-n">{i + 1}</span>
                  <span className="mono">{g.sym}</span>
                  <span className="muted small ellipsis">{g.label !== g.sym ? g.label : ""}</span>
                  <button className="icon-btn sm" aria-label="Remove point" onClick={() => set({gotoDraft: gotoDraft.filter((_, j) => j !== i)})}>
                    ×
                  </button>
                </li>
              ))}
            </ol>
          )}
          <div className="row gap-sm">
            <button
              className="btn btn-primary"
              disabled={!gotoDraft.length || !!busy || (!kinds.has("goto") && kinds.size > 0)}
              onClick={() => run("Goto", () => sendGoal(planRobot!, "goto", {points: gotoDraft.map(g => g.sym)}))}
            >
              {busy === "Goto" && <Spinner />} Send goto
            </button>
            {kinds.has("tsp") && (
              <button className="btn" disabled={!gotoDraft.length || !!busy} onClick={() => run("TSP", () => sendGoal(planRobot!, "tsp", {points: gotoDraft.map(g => g.sym)}))}>
                Send as TSP
              </button>
            )}
          </div>
        </div>
      )}

      {tool === "pddl" && (
        <div className="goal-tool">
          {pddlPlugins.length > 1 && (
            <select className="input" value={pddlPlugin} onChange={e => setPddlPlugin(e.target.value)} aria-label="PDDL plugin">
              {pddlPlugins.map(p => (
                <option key={p.name} value={p.name}>
                  {p.name}
                </option>
              ))}
            </select>
          )}
          <textarea
            className="input mono"
            rows={4}
            value={pddlDraft}
            placeholder="(visited-object o12)"
            onChange={e => set({pddlDraft: e.target.value})}
            aria-label="PDDL goal"
          />
          <p className="muted small">Select an object and press "Visit (PDDL)" to fill a template. Edit freely.</p>
          <button
            className="btn btn-primary"
            disabled={!pddlDraft.trim() || !!busy || !pddlPlugins.length}
            onClick={() => run("PDDL", () => sendGoal(planRobot!, "pddl", {plugin: pddlPlugin, goal: pddlDraft.trim()}))}
          >
            {busy === "PDDL" && <Spinner />} Send PDDL goal
          </button>
          {!pddlPlugins.length && <div className="muted small">No PDDL plugin for this robot.</div>}
          {planRobot && <ConstraintsBox robot={planRobot} constraints={plan?.constraints ?? []} />}
        </div>
      )}

      {tool === "language" && (
        <div className="goal-tool">
          <select className="input" value={langDomain} onChange={e => setLangDomain(e.target.value as typeof langDomain)} aria-label="Language domain">
            <option value="default">LLM → plan (planner default)</option>
            <option value="Pddl">LLM → PDDL</option>
            <option value="goto_points">Node symbols, no LLM (e.g. O(12) P(850))</option>
          </select>
          <textarea
            className="input"
            rows={3}
            value={lang}
            placeholder={langDomain === "goto_points" ? "O(12) P(850)" : "go to the red sign near the door"}
            onChange={e => setLang(e.target.value)}
            aria-label="Language goal"
          />
          <button className="btn btn-primary" disabled={!lang.trim() || !!busy} onClick={() => run("Language", () => sendGoal(planRobot!, "language", {command: lang.trim(), domain_type: langDomain}))}>
            {busy === "Language" && <Spinner />} Send command
          </button>
        </div>
      )}

      {msg && <div className={`inline-msg tone-${msg.tone}`} role={msg.tone === "ok" ? "status" : "alert"}>{msg.text}</div>}
      <LeaseRefusalNotice robot={planRobot} />

      <CurrentPlan robot={planRobot} plan={plan} />
    </div>
  );
}

function CurrentPlan({robot, plan}: {robot: string | null; plan: RobotPlan | undefined}) {
  const colorOf = useRobotColorOf();
  if (!robot) return null;
  const cur = currentActionIndex(plan);
  const color = colorOf(robot);
  return (
    <div className="current-plan">
      <h4 className="sub-title">
        Current plan
        {plan?.plan && <span className="mono small muted"> {plan.plan.plan_id}</span>}
      </h4>
      {!plan?.plan ? (
        <div className="muted small">No plan received yet.</div>
      ) : (
        <ol className="action-list">
          {plan.plan.actions.map((a, i) => (
            <li key={i} className={i < cur ? "done" : i === cur ? "current" : ""} style={i === cur ? {borderColor: color} : undefined}>
              <span className="act-type mono">{a.type}</span>
              <span className="small">
                {a.object_class || a.object_id || ""}
                {a.path?.length ? <span className="muted"> · {a.path.length} pts</span> : null}
              </span>
              {i === cur && <Badge tone="info">now</Badge>}
              {i < cur && <Badge tone="ok">done</Badge>}
            </li>
          ))}
        </ol>
      )}
      {plan?.results && plan.results.length > 0 && (
        <>
          <h4 className="sub-title">Results</h4>
          <ul className="result-list small">
            {plan.results.slice(-6).reverse().map((r, i) => (
              <li key={i}>
                <span className="mono muted">{fmtTime(r.t)}</span> #{r.action_index} <span className={/fail|abort|error/i.test(r.status ?? "") ? "tone-err" : "tone-ok"}>{r.status}</span> {r.detail}
              </li>
            ))}
          </ul>
        </>
      )}
      <RepairAlerts alerts={plan?.repair_alerts ?? []} />
      {plan?.status && plan.status.length > 0 && (
        <>
          <h4 className="sub-title">Planner</h4>
          <ul className="result-list small">
            {plan.status.slice(-4).reverse().map((st, i) => (
              <li key={i}>
                <span className="mono muted">{fmtTime(st.t)}</span> <span className="mono">{st.plugin}</span>{" "}
                {st.ok ? <span className="tone-ok">planned</span> : <span className="tone-err">failed: {st.error}</span>}
              </li>
            ))}
          </ul>
        </>
      )}
      {plan?.llm && plan.llm.length > 0 && (
        <>
          <h4 className="sub-title">LLM responses</h4>
          <div className="llm-list">
            {plan.llm.slice(-3).reverse().map((t, i) => (
              <pre key={i} className="llm-msg">
                {t}
              </pre>
            ))}
          </div>
        </>
      )}
      {plan?.goals && plan.goals.length > 0 && (
        <>
          <h4 className="sub-title">Sent goals</h4>
          <ul className="result-list small">
            {plan.goals.slice(-5).reverse().map((g, i) => (
              <li key={i}>
                <span className="mono muted">{fmtTime(g.t)}</span> <Badge>{g.kind}</Badge>
                {g.via === "goal_manager" && <Badge tone="ok">plan repair</Badge>}
                {g.warning && <Badge tone="warn" title={g.warning}>no plan repair</Badge>} <span className="mono ellipsis">{typeof g.msg === "string" ? g.msg : JSON.stringify(g.msg)}</span>
              </li>
            ))}
          </ul>
        </>
      )}
    </div>
  );
}
