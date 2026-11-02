// Data management (API.md §18): experiments and their phases, the live run's
// recorders, every run's metrics / sanity check / events, CSV export, and
// offload of bags + run folders to an external disk or a network host.
import {useEffect, useMemo, useState} from "react";
import {deleteJSON, errorText, getJSON, postJSON, useApi} from "../lib/api";
import {withToken} from "../lib/auth";
import {useFleet} from "../stores/fleet";
import {trackJob} from "../stores/jobs";
import {api, fmtBytes, fmtDuration, useExperiment, type Experiment, type Outcome, type Run, type RunDetail} from "../stores/experiment";
import {RecordingPanel} from "../components/ExperimentRun";
import {Badge, Card, Empty, ErrorBox, Field, Loading, PageHeader, Toggle, type Tone} from "../components/ui";

interface Target {
  id: string;
  kind: "disk" | "network";
  path: string;
  label: string;
  free_bytes?: number | null;
  size_bytes?: number | null;
  writable?: boolean;
  transport?: string | null;
  host?: string;
}

const SANITY_TONE: Record<string, Tone> = {ok: "ok", warn: "warn", err: "err", none: "off"};
const OUTCOME_TONE: Record<string, Tone> = {success: "ok", partial: "warn", fail: "err", aborted: "off"};

export function DataPage() {
  const st = useExperiment(s => s.status);
  const meta = useExperiment(s => s.meta);
  const loadMeta = useExperiment(s => s.loadMeta);
  useEffect(() => void loadMeta(), [loadMeta]);
  const exp = st?.experiment ?? null;
  return (
    <div className="page data-page">
      <PageHeader title="Data" sub="Experiments, runs, bags, metrics and offload" />
      <ExperimentCard exp={exp} all={meta?.experiments ?? []} />
      {exp && (
        <Card title="Current run">
          <RecordingPanel embedded />
        </Card>
      )}
      {exp && <RunsCard exp={exp} />}
      <StorageCard />
    </div>
  );
}

// ------------------------------------------------------------------ experiments
function ExperimentCard({exp, all}: {exp: Experiment | null; all: Experiment[]}) {
  const [creating, setCreating] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const run = useExperiment(s => s.status?.run);
  const act = async (fn: () => Promise<unknown>) => {
    setErr(null);
    try {
      await fn();
    } catch (e) {
      setErr(errorText(e));
    }
  };
  return (
    <Card title="Experiment" actions={<button className="btn btn-sm" onClick={() => setCreating(c => !c)}>{creating ? "Cancel" : "New experiment"}</button>}>
      <div className="row gap wrap">
        <Field label="Active experiment">
          <select value={exp?.id ?? ""} disabled={!!run} title={run ? "end the open run first" : ""}
            onChange={e => act(() => api.activate(e.target.value || null))} data-testid="exp-select">
            <option value="">— none (nothing is logged or bagged) —</option>
            {all.map(e => <option key={e.id} value={e.id}>{e.name} ({e.runs ?? 0} runs)</option>)}
          </select>
        </Field>
      </div>
      {creating && <NewExperiment onDone={() => setCreating(false)} />}
      {exp && <ExperimentSettings exp={exp} act={act} />}
      {err && <div className="banner banner-err">{err}</div>}
    </Card>
  );
}

function useRobotNames(): string[] {
  const machines = useFleet(s => s.machines);
  return (machines?.machines ?? []).filter(m => m.role === "robot").map(m => m.name);
}

function RobotPicker({value, onChange}: {value: string[]; onChange: (v: string[]) => void}) {
  const robots = useRobotNames();
  return (
    <div className="row gap-sm wrap">
      {robots.map(r => (
        <label key={r} className="chip-check">
          <input type="checkbox" checked={value.includes(r)} onChange={e => onChange(e.target.checked ? [...value, r] : value.filter(x => x !== r))} /> {r}
        </label>
      ))}
      {robots.length === 0 && <span className="muted small">no robots in the topology</span>}
    </div>
  );
}

function NewExperiment({onDone}: {onDone: () => void}) {
  const [name, setName] = useState("");
  const [robots, setRobots] = useState<string[]>([]);
  const [profile, setProfile] = useState<"full" | "light">("full");
  const [notes, setNotes] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const go = async () => {
    try {
      await api.create({name, robots, bag_profile: profile, auto_record: true, notes});
      onDone();
    } catch (e) {
      setErr(errorText(e));
    }
  };
  return (
    <div className="data-form">
      <Field label="Name" hint="e.g. lang2mission_penn"><input value={name} onChange={e => setName(e.target.value)} data-testid="exp-name" /></Field>
      <Field label="Robots"><RobotPicker value={robots} onChange={setRobots} /></Field>
      <Field label="Bag profile" hint="full: sensors + metrics (replayable); light: nav, metrics, compressed RGB">
        <select value={profile} onChange={e => setProfile(e.target.value as "full" | "light")}><option value="full">full</option><option value="light">light</option></select>
      </Field>
      <Field label="Notes"><input value={notes} onChange={e => setNotes(e.target.value)} /></Field>
      <button className="btn btn-primary" disabled={!name.trim()} onClick={go} data-testid="exp-create">Create + activate</button>
      {err && <div className="banner banner-err">{err}</div>}
    </div>
  );
}

function ExperimentSettings({exp, act}: {exp: Experiment; act: (fn: () => Promise<unknown>) => void}) {
  const [note, setNote] = useState("");
  const [source, setSource] = useState("");
  const patch = (p: Partial<Experiment>) => act(() => api.patch(exp.id, p));
  return (
    <div className="data-settings">
      <div className="row gap wrap">
        <Toggle checked={exp.auto_record && !exp.test_mode} onChange={v => patch(v ? {test_mode: false, auto_record: true} : {test_mode: true})}
          label={exp.test_mode ? "Testing mode (no bags)" : "Record bags automatically before each plan"} />
        <Field label="Bag profile">
          <select value={exp.bag_profile} onChange={e => patch({bag_profile: e.target.value as "full" | "light"})}>
            <option value="full">full</option><option value="light">light</option>
          </select>
        </Field>
      </div>
      <Field label="Robots"><RobotPicker value={exp.robots} onChange={robots => patch({robots})} /></Field>
      <div className="data-phases">
        <PhaseRow name="1. Mapping" rec={exp.phases.mapping} hint="snapshots the active launch (run dirs, map)" />
        <PhaseRow name="2. Map editing" rec={exp.phases.editing} hint="snapshots the map's edit history + revision" />
        <div className="row gap wrap">
          <input placeholder="note" value={note} onChange={e => setNote(e.target.value)} />
          <input placeholder="graph source (e.g. neo4j:prior)" value={source} onChange={e => setSource(e.target.value)} />
          <button className="btn btn-sm" onClick={() => act(() => api.phase("mapping", note, source || null))}>Mark mapping done</button>
          <button className="btn btn-sm" onClick={() => act(() => api.phase("editing", note, source || null))}>Mark editing done</button>
        </div>
        <p className="muted small">3. Trials: a run opens when you send a plan (Mission / chat / robot panel); its bags start before the plan goes out.</p>
      </div>
    </div>
  );
}

function PhaseRow({name, rec, hint}: {name: string; rec: Record<string, unknown> | null; hint: string}) {
  return (
    <div className="row gap-sm">
      <Badge tone={rec ? "ok" : "off"}>{rec ? "done" : "open"}</Badge>
      <span className="strong">{name}</span>
      {rec ? <span className="muted small">{new Date((rec.t as number) * 1000).toLocaleString()}{rec.note ? ` — ${rec.note}` : ""}</span> : <span className="muted small">{hint}</span>}
    </div>
  );
}

// ------------------------------------------------------------------ runs
function RunsCard({exp}: {exp: Experiment}) {
  const live = useExperiment(s => s.status?.run);
  const [runs, setRuns] = useState<Run[] | null>(null);
  const [err, setErr] = useState<unknown>(null);
  const [sel, setSel] = useState<string | null>(null);
  const [includeTest, setIncludeTest] = useState(false);
  const load = () => api.runs(exp.id).then(r => setRuns(r.runs), setErr);
  useEffect(() => {
    void load();
    const id = window.setInterval(() => void load(), 10_000);
    return () => window.clearInterval(id);
  }, [exp.id, live?.status, live?.id]); // eslint-disable-line react-hooks/exhaustive-deps
  const shown = (runs ?? []).filter(r => includeTest || !r.test_mode).slice().reverse();
  return (
    <Card title={`Runs (${runs?.length ?? 0})`}
      actions={<span className="row gap-sm">
        <Toggle checked={includeTest} onChange={setIncludeTest} label="show test runs" />
        <CsvButton exp={exp.id} table="runs" includeTest={includeTest} />
        <CsvButton exp={exp.id} table="robots" includeTest={includeTest} />
      </span>}>
      {err ? <ErrorBox error={err} /> : runs === null ? <Loading /> : shown.length === 0 ? <Empty title="No runs yet">Send a plan with this experiment active.</Empty> : (
        <div className="table-wrap">
          <table className="table compact runs-table">
            <thead>
              <tr>
                <th>Run</th><th>Condition</th><th>Status</th><th>Outcome</th><th className="num">Mission</th><th className="num">Path</th>
                <th className="num">Interv.</th><th className="num">Bags</th><th>Sanity</th><th>Offload</th>
              </tr>
            </thead>
            <tbody>
              {shown.map(r => {
                const bytes = Object.values(r.bag_bytes ?? {}).reduce<number>((a, b) => a + (b ?? 0), 0);
                return (
                  <tr key={r.id} className={sel === r.id ? "selected" : ""} onClick={() => setSel(sel === r.id ? null : r.id)} style={{cursor: "pointer"}} data-testid={`run-${r.id}`}>
                    <td className="mono">{r.id}{r.test_mode && <Badge tone="warn">test</Badge>}</td>
                    <td>{r.labels?.condition ?? "—"}</td>
                    <td><Badge tone={r.status === "recording" ? "ok" : "off"}>{r.status}</Badge></td>
                    <td>{r.outcome ? <Badge tone={OUTCOME_TONE[r.outcome.label]}>{r.outcome.label}</Badge> : <span className="muted">—</span>}</td>
                    <td className="num mono">{fmtDuration(r.metrics?.mission_time_s ?? r.metrics?.run_time_s)}</td>
                    <td className="num mono">{r.metrics ? `${r.metrics.total_path_m.toFixed(0)} m` : "—"}</td>
                    <td className="num mono">{r.metrics?.interventions ?? "—"}</td>
                    <td className="num mono">{r.record ? fmtBytes(bytes) : "off"}</td>
                    <td>{r.sanity ? <Badge tone={SANITY_TONE[r.sanity.level]}>{r.sanity.level}</Badge> : <span className="muted">—</span>}</td>
                    <td>{r.offload ? <Badge tone={r.offload.ok ? "ok" : "err"}>{r.offload.ok ? "done" : "failed"}</Badge> : <span className="muted">—</span>}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
      {sel && <RunDetailView exp={exp.id} runId={sel} onChanged={load} />}
    </Card>
  );
}

function CsvButton({exp, table, includeTest}: {exp: string; table: "runs" | "robots"; includeTest: boolean}) {
  return (
    <a className="btn btn-sm" href={withToken(api.exportUrl(exp, table, includeTest))} download={`${exp}_${table}.csv`}>
      CSV ({table})
    </a>
  );
}

function RunDetailView({exp, runId, onChanged}: {exp: string; runId: string; onChanged: () => void}) {
  const [d, setD] = useState<RunDetail | null>(null);
  const [err, setErr] = useState<unknown>(null);
  const [tab, setTab] = useState<"summary" | "events" | "sanity">("summary");
  const load = () => api.run(exp, runId).then(setD, setErr);
  useEffect(() => void load(), [exp, runId]); // eslint-disable-line react-hooks/exhaustive-deps
  if (err) return <ErrorBox error={err} />;
  if (!d) return <Loading />;
  const r = d.run;
  const m = d.metrics;
  const open = r.status === "open" || r.status === "recording";
  return (
    <div className="run-detail" data-testid="run-detail">
      <div className="row gap wrap">
        <span className="mono strong">{r.id}</span>
        {r.instruction && <span className="small">“{r.instruction}”</span>}
        <span className="grow" />
        {["summary", "events", "sanity"].map(t => (
          <button key={t} className={`btn btn-sm ${tab === t ? "btn-primary" : ""}`} onClick={() => setTab(t as typeof tab)}>{t}</button>
        ))}
      </div>
      {tab === "summary" && m && (
        <>
          <div className="kv-grid">
            <KVs label="Mission time" v={fmtDuration(m.mission_time_s)} />
            <KVs label="Run time" v={fmtDuration(m.run_time_s)} />
            <KVs label="Planning" v={m.planning_s != null ? `${m.planning_s.toFixed(1)} s` : "—"} />
            <KVs label="Plans sent / replans / failures" v={`${m.plans_sent} / ${m.replans} / ${m.plan_failures}`} />
            <KVs label="Interventions" v={`${m.interventions} (${m.intervention_s.toFixed(0)} s)`} />
            <KVs label="LLM" v={`${m.llm_calls} calls, $${m.llm_cost_usd.toFixed(3)}`} />
          </div>
          <table className="table compact">
            <thead><tr><th>Robot</th><th>Reached</th><th className="num">Path</th><th className="num">Moving</th><th className="num">Actions ok/all</th><th className="num">Interv.</th><th className="num">Link down</th></tr></thead>
            <tbody>
              {Object.entries(m.robots).map(([name, v]) => (
                <tr key={name}>
                  <td className="strong">{name}</td>
                  <td>{v.reached ? <Badge tone="ok">yes</Badge> : v.failed ? <Badge tone="err">failed</Badge> : "—"}</td>
                  <td className="num mono">{v.path_m != null ? `${(v.path_m as number).toFixed(1)} m` : "—"}</td>
                  <td className="num mono">{fmtDuration(v.moving_s as number | null)}</td>
                  <td className="num mono">{`${v.actions_succeeded ?? 0}/${v.actions_attempted ?? 0}`}</td>
                  <td className="num mono">{v.interventions ?? 0}</td>
                  <td className="num mono">{v.link_outage_s != null ? `${v.link_outage_s} s` : "—"}</td>
                </tr>
              ))}
            </tbody>
          </table>
          {!open && <OutcomeEditor exp={exp} run={r} onSaved={() => { void load(); onChanged(); }} />}
          {!open && <OffloadForm exp={exp} run={r} />}
        </>
      )}
      {tab === "events" && <EventList events={d.events} />}
      {tab === "sanity" && (
        <div>
          {!open && <button className="btn btn-sm" onClick={async () => trackJob((await api.check(exp, runId)).job_id, `Check ${runId}`)}>Re-check bags</button>}
          {!d.sanity ? <p className="muted">Not checked yet (runs when the run ends).</p> : d.sanity.machines.map(s => (
            <div key={s.machine} className="sanity-machine">
              <div className="row gap-sm">
                <Badge tone={SANITY_TONE[s.level]}>{s.level}</Badge>
                <span className="strong">{s.machine}</span>
                <span className="muted small">
                  {s.stats.duration_s != null && `${fmtDuration(s.stats.duration_s)} · `}
                  {fmtBytes(s.stats.size_bytes)} · {s.stats.messages ?? 0} msgs · {s.stats.topics_recorded ?? 0}/{s.stats.topics_expected ?? 0} topics
                </span>
              </div>
              <ul className="small">{s.findings.map((f, i) => <li key={i} className={f.level === "err" ? "text-err" : f.level === "warn" ? "text-warn" : "muted"}>{f.msg}</li>)}</ul>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

function KVs({label, v}: {label: string; v: string}) {
  return (
    <div className="kv-cell">
      <div className="muted small">{label}</div>
      <div className="mono strong">{v}</div>
    </div>
  );
}

const HIDE_KINDS = new Set(["recorder_starting"]);
function EventList({events}: {events: RunDetail["events"]}) {
  const [all, setAll] = useState(false);
  const t0 = events[0]?.t ?? 0;
  const shown = all ? events : events.filter(e => !HIDE_KINDS.has(e.kind));
  return (
    <div className="event-list">
      <Toggle checked={all} onChange={setAll} label="all events" />
      <table className="table compact">
        <tbody>
          {shown.map(e => {
            const {seq, t, kind, robot, run: _r, clock_offset_ms: _c, ros_t: _rt, ...rest} = e as Record<string, unknown> & {seq: number; t: number; kind: string; robot?: string};
            return (
              <tr key={seq}>
                <td className="num mono muted">+{fmtDuration(t - t0)}</td>
                <td className="strong">{kind}</td>
                <td>{robot ?? ""}</td>
                <td className="small mono muted event-data">{JSON.stringify(rest).slice(0, 220)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function OutcomeEditor({exp, run, onSaved}: {exp: string; run: Run; onSaved: () => void}) {
  const meta = useExperiment(s => s.meta);
  const [label, setLabel] = useState<Outcome["label"]>(run.outcome?.label ?? "success");
  const [cat, setCat] = useState(run.outcome?.failure_category ?? "");
  const [notes, setNotes] = useState(run.outcome?.notes ?? "");
  const [msg, setMsg] = useState<string | null>(null);
  const save = async () => {
    try {
      await api.outcome(exp, run.id, {label, failure_category: label === "success" ? null : cat || null, notes});
      setMsg("saved");
      onSaved();
    } catch (e) {
      setMsg(errorText(e));
    }
  };
  return (
    <div className="row gap wrap data-outcome">
      <span className="strong">Outcome</span>
      <select value={label} onChange={e => setLabel(e.target.value as Outcome["label"])}>
        {(meta?.outcomes ?? ["success", "partial", "fail", "aborted"]).map(o => <option key={o} value={o}>{o}</option>)}
      </select>
      {label !== "success" && (
        <select value={cat} onChange={e => setCat(e.target.value)}>
          <option value="">failure category…</option>
          {(meta?.failure_categories ?? []).map(c => <option key={c} value={c}>{c}</option>)}
        </select>
      )}
      <input placeholder="notes" value={notes} onChange={e => setNotes(e.target.value)} />
      <button className="btn btn-sm" onClick={save}>Save</button>
      {msg && <span className="small muted">{msg}</span>}
    </div>
  );
}

function OffloadForm({exp, run}: {exp: string; run: Run}) {
  const targets = useApi<{targets: Target[]}>("api/data/targets", 10_000);
  const [target, setTarget] = useState("");
  const [del, setDel] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const list = targets.data?.targets ?? [];
  useEffect(() => {
    if (!target && list.length) setTarget(list[0].id);
  }, [list, target]);
  const go = async () => {
    setErr(null);
    try {
      trackJob((await api.offload(exp, run.id, target, del)).job_id, `Offload ${run.id}`);
    } catch (e) {
      setErr(errorText(e));
    }
  };
  return (
    <div className="row gap wrap data-offload">
      <span className="strong">Offload</span>
      {run.offload && <Badge tone={run.offload.ok ? "ok" : "err"}>{run.offload.ok ? `→ ${run.offload.dest}` : "last offload failed"}</Badge>}
      <select value={target} onChange={e => setTarget(e.target.value)} data-testid="offload-target">
        {list.length === 0 && <option value="">no target: plug in a disk or add a network target below</option>}
        {list.map(t => <option key={t.id} value={t.id}>{t.kind === "disk" ? "💾 " : "🌐 "}{t.label}{t.free_bytes != null ? ` (${fmtBytes(t.free_bytes)} free)` : ""}</option>)}
      </select>
      <label className="row gap-sm small"><input type="checkbox" checked={del} onChange={e => setDel(e.target.checked)} /> delete from robots after verify (asks to confirm)</label>
      <button className="btn btn-sm btn-primary" disabled={!target} onClick={go}>Copy + verify</button>
      {err && <span className="text-err small">{err}</span>}
    </div>
  );
}

// ------------------------------------------------------------------ storage
function StorageCard() {
  const targets = useApi<{targets: Target[]}>("api/data/targets", 5_000);
  const [disks, setDisks] = useState<{machine: string; role?: string; ok: boolean; free_bytes?: number; experiments_bytes?: number; error?: string}[] | null>(null);
  const [busy, setBusy] = useState(false);
  const [spec, setSpec] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const refreshDisks = async () => {
    setBusy(true);
    try {
      setDisks((await getJSON<{machines: typeof disks}>("api/data/disks")).machines);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const add = async () => {
    setErr(null);
    try {
      await postJSON("api/data/targets", {spec});
      setSpec("");
      targets.reload();
    } catch (e) {
      setErr(errorText(e));
    }
  };
  const list = useMemo(() => targets.data?.targets ?? [], [targets.data]);
  return (
    <Card title="Storage + offload targets" actions={<button className="btn btn-sm" onClick={refreshDisks} disabled={busy}>{busy ? "Checking…" : "Check machine disks"}</button>}>
      <div className="table-wrap">
        <table className="table compact">
          <thead><tr><th>Target</th><th>Kind</th><th className="num">Free</th><th /></tr></thead>
          <tbody>
            {list.length === 0 && <tr><td colSpan={4} className="muted">No external disk detected (plug one in: it appears here within seconds) and no network target saved.</td></tr>}
            {list.map(t => (
              <tr key={t.id} data-testid={`target-${t.kind}`}>
                <td><span className="strong">{t.label}</span> <span className="muted small mono">{t.kind === "disk" ? t.path : ""}</span></td>
                <td>{t.kind === "disk" ? <Badge tone={t.writable === false ? "err" : "ok"}>{t.transport ?? "disk"}{t.writable === false ? " · read-only" : ""}</Badge> : <Badge tone="info">network</Badge>}</td>
                <td className="num mono">{fmtBytes(t.free_bytes)}</td>
                <td>{t.kind === "network" && <button className="btn btn-sm" onClick={async () => { await deleteJSON(`api/data/targets/${encodeURIComponent(t.id)}`); targets.reload(); }}>Remove</button>}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="row gap wrap">
        <input placeholder="user@host:/path/to/bags" value={spec} onChange={e => setSpec(e.target.value)} className="mono" />
        <button className="btn btn-sm" disabled={!spec.includes(":")} onClick={add}>Add network target</button>
      </div>
      {disks && (
        <table className="table compact">
          <thead><tr><th>Machine</th><th className="num">Disk free</th><th className="num">Experiment data</th></tr></thead>
          <tbody>
            {disks.map(d => (
              <tr key={d.machine}>
                <td className="strong">{d.machine}{d.role === "base_station" && <span className="muted small"> base</span>}</td>
                {d.ok ? <><td className="num mono">{fmtBytes(d.free_bytes)}</td><td className="num mono">{fmtBytes(d.experiments_bytes)}</td></>
                  : <td colSpan={2} className="text-err small">{d.error}</td>}
              </tr>
            ))}
          </tbody>
        </table>
      )}
      {err && <div className="banner banner-err">{err}</div>}
    </Card>
  );
}
