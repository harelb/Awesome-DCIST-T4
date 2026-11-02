// Experiment-run UI shown on every page (API.md §18): the REC chip, the
// recording-status panel, the testing-mode / recorder-failure banners, the
// "all robots reached their goals" prompt, the end-of-run form, the
// intervention logger and the "recording is not running" send guard.
import {useEffect, useState} from "react";
import {NavLink} from "react-router";
import {errorText, setRecordingBlockedHandler, type RecordingBlocked} from "../lib/api";
import {trackJob} from "../stores/jobs";
import {api, chipView, fmtBytes, fmtDuration, useExperiment, type Intervention, type MachineRec, type Outcome} from "../stores/experiment";
import {Badge, Dot, Field, Modal, type Tone} from "./ui";

/** Re-render every second (elapsed counters tick between server frames). */
function useTick(ms = 1000) {
  const [, set] = useState(0);
  useEffect(() => {
    const id = window.setInterval(() => set(n => n + 1), ms);
    return () => window.clearInterval(id);
  }, [ms]);
}

const STATE_TONE: Record<string, Tone> = {
  recording: "ok", starting: "info", stopping: "info", stalled: "warn", unreachable: "warn",
  failed: "err", stopped: "off", disabled: "off",
};

function since(rxAt: number) {
  return (Date.now() - rxAt) / 1000;
}

// ------------------------------------------------------------------ REC chip
export function RecChip() {
  useTick();
  const st = useExperiment(s => s.status);
  const rxAt = useExperiment(s => s.rxAt);
  const open = useExperiment(s => s.openPanel);
  const v = chipView(st);
  if (!v.show) return null;
  const run = st?.run;
  const elapsed = run?.t_start && st?.live?.wall_time_s != null ? st.live.wall_time_s + since(rxAt) : null;
  return (
    <button className={`btn rec-chip rec-${v.tone} ${v.pulse ? "rec-pulse" : ""}`} onClick={() => open(true)} title={v.detail}
      aria-label={`Recording status: ${v.label}`} data-testid="rec-chip">
      <span className="rec-dot" aria-hidden />
      <span className="rec-label">{v.label}</span>
      <span className="rec-short">{v.label.split(" · ")[0]}</span>
      {elapsed != null && <span className="rec-time mono">{fmtDuration(elapsed)}</span>}
    </button>
  );
}

export function InterventionButton() {
  const run = useExperiment(s => s.status?.run);
  const [open, setOpen] = useState(false);
  if (!run) return null;
  return (
    <>
      <button className="btn intervention-btn" onClick={() => setOpen(true)} title="Log an operator intervention in this run" data-testid="intervention-btn">
        + Intervention
      </button>
      {open && <InterventionDialog onClose={() => setOpen(false)} />}
    </>
  );
}

// ------------------------------------------------------------------ banners
export function ExperimentBanners() {
  const st = useExperiment(s => s.status);
  const open = useExperiment(s => s.openPanel);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  if (!st?.experiment) return null;
  const run = st.run;
  const testing = run ? !run.record : st.test_mode;
  const failed = run?.record ? st.recording.machines.filter(m => m.state === "failed") : [];
  const enable = async () => {
    setBusy(true);
    setErr(null);
    try {
      if (st.experiment!.test_mode) await api.patch(st.experiment!.id, {test_mode: false});
      if (run && !run.record) await api.setRecord(true);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <>
      {testing && (
        <div className="banner banner-warn" role="status" data-testid="testing-banner">
          <strong>TESTING MODE</strong> — bags are <strong>NOT</strong> recording{run ? ` for run ${run.id}` : ""}. Events and metrics are still logged (the run is marked test).
          <button className="btn btn-sm" onClick={enable} disabled={busy}>
            {busy ? "Enabling…" : "Enable recording"}
          </button>
          {err && <span className="small"> {err}</span>}
        </div>
      )}
      {failed.length > 0 && (
        <div className="banner banner-err" role="alert" data-testid="recorder-failed-banner">
          Recording FAILED on {failed.map(m => `${m.machine}${m.error ? ` (${m.error})` : ""}`).join(", ")}.
          <button className="btn btn-sm" onClick={() => open(true)}>
            Recording status
          </button>
        </div>
      )}
    </>
  );
}

// ------------------------------------------------------------------ status panel
function MachineRow({m, rxAt, busy, act}: {m: MachineRec; rxAt: number; busy: string | null; act: (m: string, a: "restart" | "stop") => void}) {
  const live = m.state === "recording" || m.state === "stalled";
  const elapsed = m.elapsed_s != null ? m.elapsed_s + (live ? since(rxAt) : 0) : null;
  return (
    <tr data-testid={`rec-row-${m.machine}`}>
      <td>
        <span className="row gap-sm">
          <Dot tone={STATE_TONE[m.state] ?? "off"} />
          <span className="strong">{m.machine}</span>
          {m.role === "base_station" && <span className="muted small">base</span>}
        </span>
      </td>
      <td>
        <Badge tone={STATE_TONE[m.state] ?? "off"}>{m.state}</Badge>
        {m.error && <div className="small muted">{m.error}</div>}
      </td>
      <td className="num mono">{fmtDuration(elapsed)}</td>
      <td className="num mono">{fmtBytes(m.size_bytes)}</td>
      <td className="num mono">{m.rate_bps ? `${fmtBytes(m.rate_bps)}/s` : "—"}</td>
      <td className="num mono" title={m.last_write_age_s != null ? `last write ${m.last_write_age_s}s ago` : "nothing flushed to disk yet"}>
        {m.last_write_age_s != null ? `${Math.round(m.last_write_age_s)}s` : "—"}
      </td>
      <td className="num mono">{m.segments.length || "—"}</td>
      <td className={`num mono ${m.low_disk ? "text-warn" : ""}`}>
        {fmtBytes(m.free_bytes)}
        {m.time_left_s != null && <div className="small muted">{fmtDuration(m.time_left_s)} left</div>}
      </td>
      <td className="row gap-sm">
        <button className="btn btn-sm" disabled={!!busy} onClick={() => act(m.machine, "restart")} title="Stop (if running) and start a new bag segment in the same run">
          {busy === `${m.machine}:restart` ? "…" : "Restart"}
        </button>
        <button className="btn btn-sm" disabled={!!busy || !live} onClick={() => act(m.machine, "stop")}>
          {busy === `${m.machine}:stop` ? "…" : "Stop"}
        </button>
      </td>
    </tr>
  );
}

export function RecordingPanel({embedded = false}: {embedded?: boolean}) {
  useTick();
  const st = useExperiment(s => s.status);
  const rxAt = useExperiment(s => s.rxAt);
  const close = useExperiment(s => s.openPanel);
  const openEnd = useExperiment(s => s.openEnd);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [startOpen, setStartOpen] = useState(false);
  const [intOpen, setIntOpen] = useState(false);
  const act = async (machine: string, a: "restart" | "stop") => {
    setBusy(`${machine}:${a}`);
    setErr(null);
    try {
      await api.recorder(machine, a);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };
  const run = st?.run;
  const lg = st?.logging;
  const body = (
    <div className="rec-panel">
      {!st?.experiment && (
        <p className="muted">
          No active experiment. Create or pick one on the <NavLink to="/data" onClick={() => close(false)}>Data</NavLink> page.
        </p>
      )}
      {st?.experiment && !run && (
        <div className="row gap wrap">
          <span className="muted">
            <strong>{st.experiment.name}</strong>: no open run. One opens (and its bags start) automatically when a plan is sent{st.test_mode ? " — testing mode: without bags" : ""}.
          </span>
          <button className="btn btn-sm" onClick={() => setStartOpen(true)}>Open a run now…</button>
        </div>
      )}
      {run && (
        <>
          <div className="row gap wrap rec-run-head">
            <span className="mono strong">{run.id}</span>
            <Badge tone={run.record ? "ok" : "warn"}>{run.record ? `recording · ${run.profile}` : "testing: no bags"}</Badge>
            <span className="muted small">robots {run.robots.join(", ") || "—"}</span>
            {run.instruction && <span className="small rec-instr">“{run.instruction}”</span>}
            <span className="grow" />
            <button className="btn btn-sm" onClick={() => setIntOpen(true)}>+ Intervention</button>
            <button className="btn btn-danger btn-sm" onClick={() => openEnd(true)} data-testid="end-run">End run…</button>
          </div>
          {run.record && (
            <div className="table-wrap">
              <table className="table compact rec-table">
                <thead>
                  <tr>
                    <th>Machine</th><th>State</th><th className="num">Elapsed</th><th className="num">Size</th><th className="num">Rate</th>
                    <th className="num" title="time since the bag last grew on disk (rosbag2 writes in bursts)">Last write</th>
                    <th className="num">Segs</th><th className="num">Disk free</th><th />
                  </tr>
                </thead>
                <tbody>
                  {st!.recording.machines.map(m => <MachineRow key={m.machine} m={m} rxAt={rxAt} busy={busy} act={act} />)}
                </tbody>
              </table>
            </div>
          )}
          {lg && (
            <div className="rec-logging small" data-testid="rec-logging">
              <span className="strong">Event log</span> {lg.events} events · {lg.poses} pose samples
              {lg.last_event_age_s != null && <> · last event {Math.round(lg.last_event_age_s)}s ago</>}
              {" · ROS mirror "}
              {lg.ros_mirror === null ? <span className="muted">off (no ROS)</span> : lg.ros_mirror ? <span className="text-ok">ok</span> : <span className="text-err">failing</span>}
              {Object.entries(lg.pose_age_s).map(([r, a]) => (
                <span key={r} className={lg.pose_stale?.includes(r) ? "text-warn" : "muted"}> · {r} pose {Math.round(a)}s</span>
              ))}
            </div>
          )}
        </>
      )}
      {err && <div className="banner banner-err">{err}</div>}
      {startOpen && <OpenRunDialog onClose={() => setStartOpen(false)} />}
      {intOpen && <InterventionDialog onClose={() => setIntOpen(false)} />}
    </div>
  );
  if (embedded) return body;
  return (
    <Modal title="Recording status" onClose={() => close(false)} wide>
      {body}
    </Modal>
  );
}

function OpenRunDialog({onClose}: {onClose: () => void}) {
  const st = useExperiment(s => s.status);
  const [condition, setCondition] = useState("");
  const [instrId, setInstrId] = useState("");
  const [instruction, setInstruction] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const go = async () => {
    setBusy(true);
    setErr(null);
    try {
      await api.openRun({labels: {condition: condition || undefined, instruction_id: instrId || undefined}, instruction});
      onClose();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <Modal title="Open a run" onClose={onClose}
      footer={<><span className="muted small">{st?.test_mode ? "Testing mode: no bags" : "Bags start now on every robot + the base"}</span>
        <button className="btn" onClick={onClose}>Cancel</button>
        <button className="btn btn-primary" onClick={go} disabled={busy}>{busy ? "Starting bags…" : "Open run"}</button></>}>
      <Field label="Condition / method" hint="e.g. ours, baseline-llm, no-repair"><input value={condition} onChange={e => setCondition(e.target.value)} /></Field>
      <Field label="Instruction id"><input value={instrId} onChange={e => setInstrId(e.target.value)} /></Field>
      <Field label="Instruction (language)"><textarea rows={2} value={instruction} onChange={e => setInstruction(e.target.value)} /></Field>
      {err && <div className="banner banner-err">{err}</div>}
    </Modal>
  );
}

// ------------------------------------------------------------------ prompts
export function AllReachedPrompt() {
  const st = useExperiment(s => s.status);
  const dismissed = useExperiment(s => s.dismissed);
  const dismiss = useExperiment(s => s.dismiss);
  const openEnd = useExperiment(s => s.openEnd);
  const [busy, setBusy] = useState(false);
  const p = st?.prompt;
  if (!p || dismissed === p.t) return null;
  const failed = Object.entries(p.robots).filter(([, s]) => s === "failed").map(([r]) => r);
  const cont = async () => {
    setBusy(true);
    try {
      await api.prompt("continue");
    } finally {
      setBusy(false);
    }
  };
  return (
    <Modal title={failed.length ? "All robots finished (some failed)" : "All robots reached their goals"} tone={failed.length ? "warn" : undefined}
      onClose={() => dismiss(p.t)}
      footer={<>
        <span className="muted small">Bags keep recording until you end the run.</span>
        <button className="btn" onClick={cont} disabled={busy} data-testid="prompt-continue">Continue mission</button>
        <button className="btn btn-primary" onClick={() => { dismiss(p.t); openEnd(true); }} data-testid="prompt-end">End run & stop bags…</button>
      </>}>
      <div data-testid="all-reached">
        <p>
          Mission time <strong className="mono">{fmtDuration(p.mission_time_s)}</strong> (since the first plan was sent).
        </p>
        <table className="table compact">
          <thead><tr><th>Robot</th><th>Result</th><th className="num">Path</th></tr></thead>
          <tbody>
            {Object.entries(p.robots).map(([r, s]) => (
              <tr key={r}>
                <td className="strong">{r}</td>
                <td><Badge tone={s === "reached" ? "ok" : "err"}>{s}</Badge></td>
                <td className="num mono">{p.paths?.[r] != null ? `${p.paths[r]!.toFixed(1)} m` : "—"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </Modal>
  );
}

export function EndRunDialog() {
  const st = useExperiment(s => s.status);
  const meta = useExperiment(s => s.meta);
  const close = useExperiment(s => s.openEnd);
  const run = st?.run;
  const [label, setLabel] = useState<Outcome["label"]>("success");
  const [cat, setCat] = useState("");
  const [notes, setNotes] = useState("");
  const [stopBags, setStopBags] = useState(true);
  const [ints, setInts] = useState<Intervention[] | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    if (!run) return;
    api.run(run.experiment, run.id).then(d => setInts(d.metrics?.intervention_list ?? []), () => setInts([]));
  }, [run?.experiment, run?.id]); // eslint-disable-line react-hooks/exhaustive-deps
  if (!run) return null;
  const retract = async (seq: number) => {
    await api.retract(seq, "operator: not an intervention");
    setInts(xs => (xs ?? []).filter(i => i.seq !== seq));
  };
  const go = async () => {
    setBusy(true);
    setErr(null);
    try {
      const r = await api.endRun(stopBags, {label, failure_category: label === "success" ? null : cat || null, notes});
      trackJob(r.job_id, `End run ${run.id}`);
      close(false);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const outcomes = (meta?.outcomes ?? ["success", "partial", "fail", "aborted"]) as Outcome["label"][];
  return (
    <Modal title={`End run ${run.id}`} onClose={() => close(false)} wide
      footer={<>
        <label className="row gap-sm small"><input type="checkbox" checked={stopBags} onChange={e => setStopBags(e.target.checked)} /> stop the bags (SIGINT, finalize) + sanity check</label>
        <button className="btn" onClick={() => close(false)}>Keep running</button>
        <button className="btn btn-danger" onClick={go} disabled={busy} data-testid="end-run-confirm">{busy ? "Ending…" : "End run"}</button>
      </>}>
      <div className="end-run">
        <Field label="Outcome">
          <div className="row gap-sm wrap" role="radiogroup">
            {outcomes.map(o => (
              <button key={o} className={`btn btn-sm ${label === o ? "btn-primary" : ""}`} onClick={() => setLabel(o)} aria-pressed={label === o}>{o}</button>
            ))}
          </div>
        </Field>
        {label !== "success" && (
          <Field label="Failure category">
            <select value={cat} onChange={e => setCat(e.target.value)}>
              <option value="">—</option>
              {(meta?.failure_categories ?? []).map(c => <option key={c} value={c}>{c}</option>)}
            </select>
          </Field>
        )}
        <Field label="Notes"><textarea rows={2} value={notes} onChange={e => setNotes(e.target.value)} /></Field>
        <Field label="Interventions" hint="Auto-detected (teleop, e-stop, pause/stop, lease takeover, abort, mid-run map edits) + logged. Remove false ones.">
          {ints === null ? <span className="muted small">loading…</span> : ints.length === 0 ? <span className="muted small">none</span> : (
            <table className="table compact">
              <tbody>
                {ints.map((i, k) => (
                  <tr key={`${i.seq}-${k}`}>
                    <td>{i.robot || "fleet"}</td><td>{i.type}</td>
                    <td className="num mono">{i.duration_s ? `${i.duration_s.toFixed(0)} s` : "point"}</td>
                    <td className="muted small">{i.source}{i.note ? `: ${i.note}` : ""}</td>
                    <td>{i.seq != null && <button className="btn btn-sm" onClick={() => retract(i.seq!)}>Not one</button>}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </Field>
        {err && <div className="banner banner-err">{err}</div>}
      </div>
    </Modal>
  );
}

export function InterventionDialog({onClose}: {onClose: () => void}) {
  const st = useExperiment(s => s.status);
  const meta = useExperiment(s => s.meta);
  const robots = st?.run?.robots ?? [];
  const [robot, setRobot] = useState(robots.length === 1 ? robots[0] : "");
  const [cat, setCat] = useState("navigation_assist");
  const [note, setNote] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const go = async () => {
    setBusy(true);
    try {
      await api.intervention(robot || null, cat, note);
      onClose();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <Modal title="Log intervention" onClose={onClose}
      footer={<><button className="btn" onClick={onClose}>Cancel</button><button className="btn btn-primary" onClick={go} disabled={busy} data-testid="intervention-save">Log it</button></>}>
      <Field label="Robot">
        <select value={robot} onChange={e => setRobot(e.target.value)}>
          <option value="">whole fleet</option>
          {robots.map(r => <option key={r} value={r}>{r}</option>)}
        </select>
      </Field>
      <Field label="Category">
        <select value={cat} onChange={e => setCat(e.target.value)}>
          {(meta?.intervention_categories ?? ["navigation_assist", "manipulation_assist", "reset", "safety_stop", "other"]).map(c => <option key={c} value={c}>{c.replace(/_/g, " ")}</option>)}
        </select>
      </Field>
      <Field label="Note"><input value={note} onChange={e => setNote(e.target.value)} placeholder="what happened" /></Field>
      <p className="muted small">Teleop, e-stops, pause/stop/resume, lease takeovers and mission aborts are logged automatically.</p>
      {err && <div className="banner banner-err">{err}</div>}
    </Modal>
  );
}

// ------------------------------------------------------------------ send guard
export function RecordingBlockedHost() {
  const [pending, setPending] = useState<{info: RecordingBlocked; resolve: (c: "retry" | "cancel") => void} | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    setRecordingBlockedHandler(info => new Promise(resolve => setPending({info, resolve})));
    return () => setRecordingBlockedHandler(null);
  }, []);
  if (!pending) return null;
  const names = pending.info.machines.map(m => m.machine);
  const finish = (c: "retry" | "cancel") => {
    pending.resolve(c);
    setPending(null);
    setErr(null);
  };
  const run = async (what: string, fn: () => Promise<unknown>) => {
    setBusy(what);
    setErr(null);
    try {
      await fn();
      finish("retry");
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };
  return (
    <Modal title="Recording is not running — goal NOT sent" tone="err" onClose={() => finish("cancel")}
      footer={<>
        <button className="btn" onClick={() => finish("cancel")} disabled={!!busy}>Cancel</button>
        <button className="btn btn-danger-outline" disabled={!!busy} onClick={() => run("skip", () => api.acceptMissing(names))} data-testid="blocked-skip">
          {busy === "skip" ? "…" : `Send without ${names.join(", ")}`}
        </button>
        <button className="btn btn-primary" disabled={!!busy} onClick={() => run("retry", async () => { for (const n of names) await api.recorder(n, "restart"); })} data-testid="blocked-retry">
          {busy === "retry" ? "Restarting recorders…" : "Restart recorder & send"}
        </button>
      </>}>
      <div data-testid="recording-blocked">
        <p>The experiment run records bags, and these recorders are not up:</p>
        <ul>
          {pending.info.machines.map(m => (
            <li key={m.machine}><span className="strong">{m.machine}</span>: {m.state}{m.error ? ` — ${m.error}` : ""}</li>
          ))}
        </ul>
        {err && <div className="banner banner-err">{err}</div>}
      </div>
    </Modal>
  );
}

/** Everything that floats over the pages (mounted once in Shell). */
export function ExperimentHosts() {
  const panelOpen = useExperiment(s => s.panelOpen);
  const endOpen = useExperiment(s => s.endOpen);
  return (
    <>
      {panelOpen && <RecordingPanel />}
      <AllReachedPrompt />
      {endOpen && <EndRunDialog />}
      <RecordingBlockedHost />
    </>
  );
}
