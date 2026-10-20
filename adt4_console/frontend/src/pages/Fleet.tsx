import {useState} from "react";
import {useFleet} from "../stores/fleet";
import {trackJob} from "../stores/jobs";
import {errorText, postJSON, useApi} from "../lib/api";
import {fmtAgo} from "../lib/fuzzy";
import type {ConsoleConfig, Machine, NodeStatus, RemoteStatus, RobotNodes} from "../types";
import {TerminalModal} from "../components/Terminal";
import {Badge, Card, Dot, Empty, ErrorBox, Loading, Modal, PageHeader, Spinner, type Tone} from "../components/ui";

export const jobIdOf = (r: unknown): string | null => {
  const o = r as {job_id?: string; id?: string};
  return o?.job_id ?? o?.id ?? null;
};

const NODE_TONE: Record<NodeStatus, Tone> = {NOMINAL: "ok", WARNING: "warn", ERROR: "err", NO_HB: "err", STARTUP: "info"};

function readiness(r: RobotNodes["readiness"]): {tone: Tone; label: string} {
  let color = "";
  let label = "";
  if (Array.isArray(r)) [color, label] = r;
  else if (r && typeof r === "object") ({color, label} = r);
  else if (typeof r === "string") label = r;
  const tone: Tone = color === "green" ? "ok" : color === "yellow" ? "warn" : color === "red" ? "err" : "off";
  return {tone, label: label || "Unknown"};
}

function pingText(p: Machine["ping"]): string {
  if (p === null || p === undefined) return "-";
  if (typeof p === "number") return `${p.toFixed(0)} ms`;
  return p ? "ok" : "fail";
}

function boolTone(v: unknown): Tone {
  if (v === null || v === undefined) return "off";
  if (typeof v === "number") return v >= 0 ? "ok" : "err";
  return v ? "ok" : "err";
}

const na = (v: unknown) => (v === undefined || v === null || v === "" || v === "N/A" ? <span className="muted">-</span> : String(v));

function batteryTone(b?: string): Tone {
  const m = /(\d+)%/.exec(b ?? "");
  if (!m) return "off";
  const v = Number(m[1]);
  return v < 20 ? "err" : v < 40 ? "warn" : "ok";
}

function diskTone(d?: string): Tone {
  const m = /\((\d+)% used\)/.exec(d ?? "");
  if (!m) return "off";
  const v = Number(m[1]);
  return v > 90 ? "err" : v > 75 ? "warn" : "ok";
}

type SessionAction = "stop" | "kill";
interface SessionTarget {
  m: Machine;
  session: string;
  action: SessionAction;
}
interface TermTarget {
  m: Machine;
  session: string | null;
}

function MachineRow({
  m,
  st,
  terminal,
  onStop,
  onSession,
  onTerminal,
}: {
  m: Machine;
  st?: RemoteStatus;
  terminal: boolean;
  onStop: (m: Machine) => void;
  onSession: (t: SessionTarget) => void;
  onTerminal: (t: TermTarget) => void;
}) {
  const canShell = terminal && !!m.online && (!!m.ssh || !!m.local);
  const tmux = st?.tmux_sessions && st.tmux_sessions !== "none" && st.tmux_sessions !== "N/A" ? st.tmux_sessions.split(",").map(s => s.trim()) : [];
  return (
    <tr className={m.online ? "" : "row-dim"}>
      <td className="cell-title">
        <div className="row gap">
          <Dot tone={m.online ? "ok" : "err"} title={m.online ? "online" : "offline"} />
          <strong>{m.name}</strong>
          {m.local && <Badge tone="info">this machine</Badge>}
        </div>
        <div className="muted small">
          {m.role.replace("_", " ")}
          {m.platform_id ? ` · ${m.platform_id}` : ""}
          {m.desc ? ` · ${m.desc}` : ""}
        </div>
      </td>
      <td data-label="ip" className="mono">{m.ip}</td>
      <td data-label="ping">
        <span className="row gap-sm">
          <Dot tone={boolTone(m.ping)} /> <span className="mono small">{pingText(m.ping)}</span>
        </span>
      </td>
      <td data-label="ssh">
        <Dot tone={boolTone(m.ssh)} title={m.ssh ? "ssh ok" : "no ssh"} />
      </td>
      <td data-label="tmux">
        {tmux.length ? (
          <div className="chips">
            {tmux.map(name => {
              return (
                <span key={name} className="chip mono session-chip">
                  {name}
                  {canShell && (
                    <button className="chip-btn" title={`attach to ${name} in a terminal`} onClick={() => onTerminal({m, session: name})}>
                      ⌨
                    </button>
                  )}
                  <button className="chip-btn" title={`Ctrl-C every pane of ${name}`} disabled={!m.online} onClick={() => onSession({m, session: name, action: "stop"})}>
                    ■
                  </button>
                  <button className="chip-btn chip-btn-danger" title={`kill tmux session ${name}`} disabled={!m.online} onClick={() => onSession({m, session: name, action: "kill"})}>
                    ✕
                  </button>
                </span>
              );
            })}
          </div>
        ) : (
          <span className="muted">none</span>
        )}
      </td>
      <td data-label="load" className="mono small">{na(st?.load)}</td>
      <td data-label="mem" className="mono small">{na(st?.mem)}</td>
      <td data-label="gpu" className="mono small">{na(st?.gpu)}</td>
      <td data-label="disk free" className="mono small">
        <span className="row gap-sm">
          {st?.disk && st.disk !== "N/A" && <Dot tone={diskTone(st.disk)} />}
          {na(st?.disk)}
        </span>
      </td>
      <td data-label="battery" className="mono small">
        <span className="row gap-sm">
          {st?.battery && st.battery !== "N/A" && <Dot tone={batteryTone(st.battery)} />}
          {na(st?.battery)}
        </span>
      </td>
      <td data-label="radio" className="mono small">{na(st?.radio)}</td>
      <td className="cell-actions">
        {canShell && (
          <button className="btn btn-sm" title={`shell on ${m.name} (ssh ${m.user}@${m.ip})`} onClick={() => onTerminal({m, session: null})}>
            Terminal
          </button>
        )}
        {m.role === "robot" && (
          <button className="btn btn-sm btn-danger-outline" disabled={!m.online} onClick={() => onStop(m)}>
            Stop &amp; Save
          </button>
        )}
      </td>
    </tr>
  );
}

function RobotNodesCard({robot, info}: {robot: string; info: RobotNodes}) {
  const r = readiness(info.readiness);
  const counts = info.nodes.reduce<Record<string, number>>((a, n) => ((a[n.status] = (a[n.status] ?? 0) + 1), a), {});
  return (
    <Card
      className="robot-nodes"
      title={
        <span className="row gap">
          <Dot tone={r.tone} />
          {robot}
        </span>
      }
      actions={<Badge tone={r.tone}>{r.label}</Badge>}
    >
      <div className="row gap small muted wrap">
        {Object.entries(counts).map(([s, c]) => (
          <span key={s}>
            <Dot tone={NODE_TONE[s as NodeStatus] ?? "off"} /> {c} {s.toLowerCase()}
          </span>
        ))}
      </div>
      {info.nodes.length === 0 ? (
        <Empty title="No nodes reporting" />
      ) : (
        <table className="table compact status-table">
          <tbody>
            {info.nodes.map(n => (
              <tr key={n.node_name || n.nickname} className={`st-${(NODE_TONE[n.status] ?? "off")}`}>
                <td className="st-bar" />
                <td>
                  {n.nickname}
                  {n.required === false && <span className="muted small"> (optional)</span>}
                </td>
                <td className="mono small muted">{n.node_name}</td>
                <td>
                  <span className={`st-label tone-${NODE_TONE[n.status] ?? "off"}`}>{n.status}</span>
                </td>
                <td className="small">{n.notes}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </Card>
  );
}

export function FleetPage() {
  const {machines, status, nodes, nodesError, loaded, error, load} = useFleet();
  const [refreshing, setRefreshing] = useState(false);
  const [stopTarget, setStopTarget] = useState<Machine | null>(null);
  const [actionErr, setActionErr] = useState<string | null>(null);
  const [sessionTarget, setSessionTarget] = useState<SessionTarget | null>(null);
  const [termTarget, setTermTarget] = useState<TermTarget | null>(null);
  const [teardown, setTeardown] = useState(true);
  const config = useApi<ConsoleConfig>("api/config");
  const terminal = !!config.data?.terminal;

  const refresh = async () => {
    setRefreshing(true);
    setActionErr(null);
    try {
      const m = await postJSON<typeof machines>("api/fleet/refresh");
      if (m) useFleet.setState({machines: m});
      await postJSON("api/fleet/status/refresh").catch(() => undefined);
      await load();
    } catch (e) {
      setActionErr(errorText(e));
    } finally {
      setRefreshing(false);
    }
  };

  const stopSave = async (m: Machine) => {
    setStopTarget(null);
    try {
      const r = await postJSON(`api/robots/${encodeURIComponent(m.name)}/stop_save`, {teardown});
      const id = jobIdOf(r);
      if (id) trackJob(id, `Stop & save ${m.name}`);
    } catch (e) {
      setActionErr(errorText(e));
    }
  };

  const sessionAction = async ({m, session, action}: SessionTarget) => {
    setSessionTarget(null);
    setActionErr(null);
    try {
      await postJSON(`api/fleet/${encodeURIComponent(m.name)}/tmux/${encodeURIComponent(session)}/${action}`);
      // the session list refreshes with the next status poll; ask for one now
      setTimeout(() => void load(), action === "kill" ? 1500 : 4000);
    } catch (e) {
      setActionErr(errorText(e));
    }
  };

  const list = machines?.machines ?? [];
  const online = list.filter(m => m.online).length;
  const robots = Object.entries(nodes?.robots ?? {});
  return (
    <div className="page">
      <PageHeader
        title="Fleet"
        sub={
          machines ? (
            <>
              network <span className="mono">{machines.network}</span> · {online}/{list.length} online · status {fmtAgo(status?.updated)}
            </>
          ) : undefined
        }
        actions={
          <button className="btn btn-primary" onClick={refresh} disabled={refreshing}>
            {refreshing ? <Spinner /> : null} Refresh
          </button>
        }
      />
      {actionErr && <ErrorBox error={actionErr} compact />}
      {!loaded && <Loading label="Loading fleet" />}
      {loaded && error && !machines && <ErrorBox error={error} onRetry={load} />}
      {loaded && machines && list.length === 0 && <Empty title="No machines">The topology for this network lists no machines.</Empty>}
      {list.length > 0 && (
        <Card title="Machines">
          <div className="table-wrap">
            <table className="table fleet-table stack">
              <thead>
                <tr>
                  <th>machine</th>
                  <th>ip</th>
                  <th>ping</th>
                  <th>ssh</th>
                  <th>tmux</th>
                  <th>load</th>
                  <th>mem</th>
                  <th>gpu</th>
                  <th>disk free</th>
                  <th>battery</th>
                  <th>radio</th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {[...list]
                  .sort((a, b) => Number(b.online) - Number(a.online) || a.role.localeCompare(b.role) || a.name.localeCompare(b.name))
                  .map(m => (
                    <MachineRow
                      key={m.name}
                      m={m}
                      st={status?.machines?.[m.name]}
                      terminal={terminal}
                      onStop={setStopTarget}
                      onSession={setSessionTarget}
                      onTerminal={setTermTarget}
                    />
                  ))}
              </tbody>
            </table>
          </div>
        </Card>
      )}
      <h2 className="section-title">Robot node readiness</h2>
      {nodesError && !nodes && <ErrorBox error={nodesError} compact />}
      {nodes && robots.length === 0 && <Empty title="No status tables received">Nodes report via /global/ros_system_monitor/table_in.</Empty>}
      <div className="grid-cards">
        {robots.map(([r, info]) => (
          <RobotNodesCard key={r} robot={r} info={info} />
        ))}
      </div>
      {sessionTarget && (
        <Modal
          title={`${sessionTarget.action === "kill" ? "Kill" : "Stop"} ${sessionTarget.session} on ${sessionTarget.m.name}?`}
          tone={sessionTarget.action === "kill" ? "err" : "warn"}
          onClose={() => setSessionTarget(null)}
          footer={
            <>
              <button className="btn" onClick={() => setSessionTarget(null)}>
                Cancel
              </button>
              <button className="btn btn-danger" onClick={() => sessionAction(sessionTarget)}>
                {sessionTarget.action === "kill" ? "Kill session" : "Send Ctrl-C"}
              </button>
            </>
          }
        >
          {sessionTarget.action === "kill" ? (
            <p>
              Runs <span className="mono">tmux kill-session -t {sessionTarget.session}</span>: every process in it gets SIGHUP at once, nothing is
              saved.
            </p>
          ) : (
            <p>
              Sends Ctrl-C to every pane of <span className="mono">{sessionTarget.session}</span>, so each node shuts down the way it would in the
              terminal. The panes stay open (shells); kill the session afterwards to close them.
            </p>
          )}
          {sessionTarget.session === "adt4_system" && sessionTarget.m.role === "robot" && (
            <p className="tone-warn">
              Hydra and ROMAN do not save their maps on Ctrl-C or kill. Use <strong>Stop &amp; Save</strong> first if this run's map matters.
            </p>
          )}
        </Modal>
      )}
      {termTarget && <TerminalModal machine={termTarget.m.name} session={termTarget.session} onClose={() => setTermTarget(null)} />}
      {stopTarget && (
        <Modal
          title={`Stop & save ${stopTarget.name}?`}
          tone="warn"
          onClose={() => setStopTarget(null)}
          footer={
            <>
              <button className="btn" onClick={() => setStopTarget(null)}>
                Cancel
              </button>
              <button className="btn btn-danger" onClick={() => stopSave(stopTarget)}>
                Stop &amp; Save
              </button>
            </>
          }
        >
          <p>
            Calls <span className="mono">/{stopTarget.name}/hydra/shutdown</span> then <span className="mono">/{stopTarget.name}/roman/shutdown</span>:
            hydra saves its scene graph and mesh and exits, ROMAN saves its map. If hydra's service does not answer, runs the same calls in the
            robot's core shutdown pane over ssh. Progress is in the job log.
          </p>
          <label className="row gap">
            <input type="checkbox" checked={teardown} onChange={e => setTeardown(e.target.checked)} />
            <span>
              Then shut down the rest of the stack: once hydra has exited, Ctrl-C every pane of <span className="mono">adt4_system</span> and kill the
              session. If hydra is still saving after 10 min the stack is left running.
            </span>
          </label>
        </Modal>
      )}
    </div>
  );
}
