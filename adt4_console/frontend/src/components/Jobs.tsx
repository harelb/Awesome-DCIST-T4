import {useEffect, useMemo, useRef, useState} from "react";
import {fetchJob, isActive, useJobs} from "../stores/jobs";
import {errorText, postJSON} from "../lib/api";
import {canAnswerYes, confirmBody} from "../lib/confirm";
import {fmtTime} from "../lib/fuzzy";
import {issueKey, preflightIssues} from "../lib/preflight";
import type {Job, JobConfirm, JobState} from "../types";
import {Badge, Empty, Modal, ProgressBar, Spinner, type Tone} from "./ui";

export const stateTone = (s: JobState): Tone =>
  s === "succeeded" ? "ok" : s === "failed" ? "err" : s === "waiting" ? "warn" : s === "running" ? "info" : "off";

export function JobStateBadge({state}: {state: JobState}) {
  return <Badge tone={stateTone(state)}>{state}</Badge>;
}

function JobLog({job}: {job: Job}) {
  const ref = useRef<HTMLDivElement>(null);
  const stick = useRef(true);
  const n = job.log?.length ?? 0;
  useEffect(() => {
    const el = ref.current;
    if (el && stick.current) el.scrollTop = el.scrollHeight;
  }, [n]);
  return (
    <div
      className="joblog mono"
      ref={ref}
      onScroll={e => {
        const el = e.currentTarget;
        stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
      }}
    >
      {n === 0 && <div className="log-dim">No output yet.</div>}
      {job.log?.map((l, i) => (
        <div key={i} className={`log-line log-${l.level}`}>
          <span className="log-t">{fmtTime(l.t)}</span>
          <span className="log-text">{l.text}</span>
        </div>
      ))}
    </div>
  );
}

export function JobDetail({job}: {job: Job}) {
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    if (!job.log) void fetchJob(job.id);
  }, [job.id, job.log]);
  const progress = Object.entries(job.progress ?? {});
  return (
    <div className="jobdetail">
      <div className="jobdetail-head">
        <div>
          <div className="job-title">{job.title || job.kind}</div>
          <div className="muted small mono">
            {job.kind} · {job.id} · started {fmtTime(job.created)}
            {job.finished ? ` · finished ${fmtTime(job.finished)}` : ""}
          </div>
        </div>
        <div className="row gap">
          <JobStateBadge state={job.state} />
          {isActive(job) && (
            <button
              className="btn btn-sm btn-danger-outline"
              disabled={busy}
              onClick={async () => {
                setBusy(true);
                try {
                  await postJSON(`api/jobs/${job.id}/cancel`);
                } catch (e) {
                  setErr(errorText(e));
                } finally {
                  setBusy(false);
                }
              }}
            >
              Cancel
            </button>
          )}
        </div>
      </div>
      {err && <div className="inline-err">{err}</div>}
      {progress.length > 0 && (
        <div className="progress-list">
          {progress.map(([k, v]) => (
            <ProgressBar key={k} label={k} pct={v} tone={v >= 100 ? "ok" : "info"} />
          ))}
        </div>
      )}
      <JobLog job={job} />
      {job.result !== undefined && job.result !== null && !isActive(job) && <JobResult result={job.result} />}
    </div>
  );
}

function JobResult({result}: {result: unknown}) {
  const r = result as {rows?: {path: string; consistent: boolean; cells: Record<string, string>}[]; machines?: string[]};
  if (r && Array.isArray(r.rows) && Array.isArray(r.machines)) return <ChecksumMatrix rows={r.rows} machines={r.machines} />;
  return (
    <details className="job-result">
      <summary>Result</summary>
      <pre className="mono">{JSON.stringify(result, null, 2)}</pre>
    </details>
  );
}

function majority(cells: Record<string, string>, machines: string[]): string {
  const counts = new Map<string, number>();
  for (const m of machines) {
    const v = cells[m] ?? "missing";
    counts.set(v, (counts.get(v) ?? 0) + 1);
  }
  return [...counts.entries()].sort((a, b) => b[1] - a[1])[0]?.[0] ?? "";
}

export function ChecksumMatrix({rows, machines}: {rows: {path: string; consistent: boolean; cells: Record<string, string>}[]; machines: string[]}) {
  const bad = rows.filter(r => !r.consistent).length;
  return (
    <div className="checksum">
      <div className="row gap">
        <strong>Checksum verify</strong>
        <Badge tone={bad ? "err" : "ok"}>{bad ? `${bad} mismatched` : "all consistent"}</Badge>
        <span className="muted small">{rows.length} paths</span>
      </div>
      <div className="table-wrap">
        <table className="table compact">
          <thead>
            <tr>
              <th>path</th>
              {machines.map(m => (
                <th key={m}>{m}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            {rows.map(r => (
              <tr key={r.path} className={r.consistent ? "" : "row-err"}>
                <td className="mono">{r.path}</td>
                {machines.map(m => {
                  const v = r.cells[m] ?? "missing";
                  const odd = !r.consistent && v !== majority(r.cells, machines);
                  return (
                    <td key={m} className={`mono small ${odd ? "cell-odd" : ""}`}>
                      {v}
                    </td>
                  );
                })}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}

export function JobsDrawer() {
  const {drawerOpen, close, order, jobs, focusId} = useJobs();
  const focus = focusId ? jobs[focusId] : null;
  if (!drawerOpen) return null;
  return (
    <aside className="drawer" aria-label="Jobs">
      <header className="drawer-head">
        <h2>Jobs</h2>
        <button className="icon-btn" onClick={close} aria-label="Close jobs">
          ×
        </button>
      </header>
      <div className="drawer-body">
        <ul className="joblist">
          {order.length === 0 && <li className="muted pad">No jobs yet. Launches, transfers and verifies show up here.</li>}
          {order.map(id => {
            const j = jobs[id];
            return (
              <li key={id} className={id === focusId ? "on" : ""} onClick={() => useJobs.setState({focusId: id})}>
                <div className="row gap between">
                  <span className="job-title-sm">{j.title || j.kind}</span>
                  {isActive(j) ? <Spinner /> : null}
                </div>
                <div className="row gap between">
                  <span className="muted small mono">{fmtTime(j.created)}</span>
                  <JobStateBadge state={j.state} />
                </div>
              </li>
            );
          })}
        </ul>
        <div className="drawer-detail">{focus ? <JobDetail job={focus} /> : <Empty title="Select a job" />}</div>
      </div>
    </aside>
  );
}

/** Global modal for any job waiting on a yes/no or typed confirmation. */
export function JobConfirmHost() {
  const jobs = useJobs(s => s.jobs);
  const [answered, setAnswered] = useState<Record<string, string>>({});
  const pending = useMemo(
    () => Object.values(jobs).filter(j => j.confirm && j.state === "waiting" && answered[j.id] !== j.confirm.message),
    [jobs, answered],
  );
  // `waiting` jobs from the summary list may lack the confirm payload: fetch them
  useEffect(() => {
    for (const j of Object.values(jobs)) if (j.state === "waiting" && !j.confirm) void fetchJob(j.id);
  }, [jobs]);
  const job = pending[0];
  if (!job || !job.confirm) return null;
  return (
    <ConfirmDialog
      key={job.id + job.confirm.message}
      job={job}
      confirm={job.confirm}
      onDone={() => setAnswered(a => ({...a, [job.id]: job.confirm!.message}))}
    />
  );
}

export function ConfirmDialog({job, confirm, onDone}: {job: Job; confirm: JobConfirm; onDone: () => void}) {
  const [typed, setTyped] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  // a preflight NO-GO: each issue must be ticked before "Launch anyway"
  const issues = preflightIssues(confirm);
  const [acked, setAcked] = useState<Set<string>>(new Set());
  const yesOk = canAnswerYes(confirm, typed) && issues.every(i => acked.has(issueKey(i)));
  const toggleAck = (k: string) =>
    setAcked(cur => {
      const next = new Set(cur);
      if (next.has(k)) next.delete(k);
      else next.add(k);
      return next;
    });
  const answer = async (yes: boolean) => {
    setBusy(true);
    setErr(null);
    try {
      await postJSON(`api/jobs/${job.id}/confirm`, confirmBody(confirm, yes, typed));
      onDone();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const danger = Boolean(confirm.required_text);
  return (
    <Modal
      title={<>Confirm: {job.title || job.kind}</>}
      tone={danger ? "err" : "warn"}
      footer={
        <>
          <button className="btn" disabled={busy} onClick={() => answer(false)}>
            {confirm.no_label || "No"}
          </button>
          <button className={`btn ${danger ? "btn-danger" : "btn-primary"}`} disabled={busy || !yesOk} onClick={() => answer(true)} data-testid="confirm-yes">
            {confirm.yes_label || "Yes"}
          </button>
        </>
      }
    >
      {issues.length > 0 ? (
        <div className="preflight-ack">
          <p className="confirm-msg">
            {(confirm.detail as {stage?: string} | null)?.stage ?? "Preflight"} found {issues.length} NO-GO issue{issues.length > 1 ? "s" : ""}. Tick each one to launch anyway.
          </p>
          <ul className="preflight-issues">
            {issues.map(i => {
              const k = issueKey(i);
              return (
                <li key={k}>
                  <label className={`preflight-issue ${acked.has(k) ? "on" : ""}`}>
                    <input type="checkbox" checked={acked.has(k)} onChange={() => toggleAck(k)} aria-label={`Acknowledge ${i.machine} ${i.name}`} />
                    <span>
                      <strong className="mono">
                        {i.machine} / {i.name}
                      </strong>
                      : {i.detail}
                      {i.hint && <span className="muted small preflight-hint">Fix: {i.hint}</span>}
                    </span>
                  </label>
                </li>
              );
            })}
          </ul>
        </div>
      ) : (
        <p className="confirm-msg">{confirm.message}</p>
      )}
      {issues.length === 0 && confirm.detail !== undefined && confirm.detail !== null && !(typeof confirm.detail === "object" && Object.keys(confirm.detail as object).length === 0) && (
        <pre className="confirm-detail mono">{typeof confirm.detail === "string" ? confirm.detail : JSON.stringify(confirm.detail, null, 2)}</pre>
      )}
      {confirm.required_text && (
        <label className="field">
          <span className="field-label">
            Type <code className="mono req-text">{confirm.required_text}</code> to confirm
          </span>
          <input
            className="input mono"
            autoFocus
            value={typed}
            onChange={e => setTyped(e.target.value)}
            onKeyDown={e => e.key === "Enter" && yesOk && !busy && answer(true)}
            aria-label="Confirmation text"
          />
        </label>
      )}
      {err && <div className="inline-err">{err}</div>}
    </Modal>
  );
}
