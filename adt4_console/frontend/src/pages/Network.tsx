import {useState} from "react";
import {SilvusMesh, type MeshLink, type MeshRadio} from "./SilvusMesh";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {trackJob, useJobs} from "../stores/jobs";
import {useFleet} from "../stores/fleet";
import {Badge, Card, Dot, Empty, ErrorBox, Field, Loading, Modal, PageHeader, Spinner} from "../components/ui";
import {JobStateBadge} from "../components/Jobs";
import {jobIdOf} from "./Fleet";
import {BandwidthSection} from "../components/Bandwidth";

interface ZenohStatus {
  local_zenohd: {running: boolean; [k: string]: unknown};
  port: number;
  links: unknown[];
  reachable: Record<string, boolean>;
  fleet_link?: "router" | "fleetlink";
  fleetlink?: {base: string | null; port: number; endpoints: string[]; summary: string};
}

type Cell = {rssi_db?: number | null; link_db?: number | null; quality?: number | string | null; linked?: boolean};
interface SilvusStatus {
  matrix: Record<string, Record<string, Cell> | string>;
  routing: unknown;
  radios?: MeshRadio[];
  links?: MeshLink[];
}

/** RSSI (dBm) -> background colour: green strong, amber marginal, red weak. */
export function rssiColor(db: number | null | undefined): string {
  if (db === null || db === undefined || !Number.isFinite(db)) return "transparent";
  const t = Math.max(0, Math.min(1, (db + 90) / 45)); // -90 dBm -> 0, -45 dBm -> 1
  const hue = t * 130; // 0 red .. 130 green
  return `hsl(${hue.toFixed(0)} 70% ${(24 + t * 8).toFixed(0)}%)`;
}

/** Link value (dB, the radio's network_status; StreamScape 5 has no RSSI query)
 *  -> colour: 5 dB red .. 35 dB green. */
export function linkColor(db: number | null | undefined): string {
  if (db === null || db === undefined || !Number.isFinite(db)) return "transparent";
  return rssiColor(-90 + (Math.max(0, Math.min(1, (db - 5) / 30)) * 45));
}

/** What the matrix shows: RSSI when the firmware reports it, else the link value. */
export function matrixMode(matrix: SilvusStatus["matrix"]): "rssi" | "link" {
  const cells = Object.values(matrix).flatMap(r => (typeof r === "object" ? Object.values(r) : []));
  return cells.some(c => c.rssi_db != null) || !cells.some(c => c.link_db != null) ? "rssi" : "link";
}

function linkText(l: unknown): string {
  if (typeof l === "string") return l;
  const o = l as {src?: string; dst?: string; key?: string; value?: unknown};
  if (o?.src || o?.dst) return `${o.src ?? "?"} -> ${o.dst ?? "?"}`;
  if (o?.key) return o.key;
  return JSON.stringify(l);
}

export function NetworkPage() {
  const zenoh = useApi<ZenohStatus>("api/network/zenoh", 10_000);
  const silvus = useApi<SilvusStatus>("api/network/silvus", 15_000);
  const machines = useFleet(s => s.machines)?.machines ?? []; // selector must return a stable value (zustand 5)
  const [busy, setBusy] = useState<string | null>(null);
  const [msg, setMsg] = useState<{ok: boolean; text: string} | null>(null);
  const [deploy, setDeploy] = useState(false);
  const [route, setRoute] = useState<unknown>(null);
  const [iperfMachine, setIperfMachine] = useState("");
  const [iperfSecs, setIperfSecs] = useState(5);
  const [iperfJob, setIperfJob] = useState<string | null>(null);
  const jobs = useJobs(s => s.jobs);

  const zenohd = async (op: "start" | "stop") => {
    setBusy(op);
    setMsg(null);
    try {
      const r = await postJSON<{ok: boolean; message: string}>(`api/network/zenohd/${op}`);
      setMsg({ok: r.ok, text: r.message || (r.ok ? `zenohd ${op}ed` : `zenohd ${op} failed`)});
      zenoh.reload();
    } catch (e) {
      setMsg({ok: false, text: errorText(e)});
    } finally {
      setBusy(null);
    }
  };

  const checkRoute = async () => {
    setBusy("route");
    try {
      setRoute(await getJSON("api/network/silvus/route"));
    } catch (e) {
      setRoute({error: errorText(e)});
    } finally {
      setBusy(null);
    }
  };

  const runIperf = async () => {
    setBusy("iperf");
    try {
      const r = await postJSON("api/network/iperf", {machine: iperfMachine, seconds: iperfSecs});
      const id = jobIdOf(r);
      if (id) {
        setIperfJob(id);
        trackJob(id, `iperf ${iperfMachine}`);
      }
    } catch (e) {
      setMsg({ok: false, text: errorText(e)});
    } finally {
      setBusy(null);
    }
  };

  const z = zenoh.data;
  const nodes = silvus.data ? Object.keys(silvus.data.matrix) : [];
  // columns in the same order as rows so the diagonal is the radio itself
  const mode = silvus.data ? matrixMode(silvus.data.matrix) : "rssi";
  const dsts = [...new Set([...nodes, ...nodes.flatMap(n => (typeof silvus.data!.matrix[n] === "object" ? Object.keys(silvus.data!.matrix[n] as object) : []))])];
  const ij = iperfJob ? jobs[iperfJob] : null;

  return (
    <div className="page">
      <PageHeader title="Network" sub="Zenoh router mesh, Silvus radio links and throughput tests." />
      {msg && <div className={`banner ${msg.ok ? "banner-ok" : "banner-err"}`}>{msg.text}</div>}
      <div className="grid-2">
        <Card
          title="Zenoh"
          actions={
            <>
              <button className="btn btn-sm" onClick={() => setDeploy(true)}>
                Deploy config
              </button>
              <button className="btn btn-sm" onClick={zenoh.reload} disabled={zenoh.loading}>
                Refresh
              </button>
            </>
          }
        >
          {zenoh.loading && !z && <Loading />}
          {zenoh.error && !z && <ErrorBox error={zenoh.error} onRetry={zenoh.reload} />}
          {z && (
            <>
              <div className="row gap between zenohd">
                <div className="row gap">
                  <Dot tone={z.local_zenohd?.running ? "ok" : "err"} />
                  <strong>local zenohd</strong>
                  <Badge tone={z.local_zenohd?.running ? "ok" : "err"}>{z.local_zenohd?.running ? "running" : "stopped"}</Badge>
                  <span className="muted small mono">port {z.port}</span>
                </div>
                <div className="row gap-sm">
                  <button className="btn btn-sm" disabled={!!busy || z.local_zenohd?.running} onClick={() => zenohd("start")}>
                    {busy === "start" && <Spinner />} Start
                  </button>
                  <button className="btn btn-sm btn-danger-outline" disabled={!!busy || !z.local_zenohd?.running} onClick={() => zenohd("stop")}>
                    {busy === "stop" && <Spinner />} Stop
                  </button>
                </div>
              </div>
              {z.fleet_link === "fleetlink" && z.fleetlink ? (
                <>
                  <h4 className="sub-title">Radio link: fleetlink</h4>
                  <div className="mono small">{z.fleetlink.summary}</div>
                  <div className="muted small">
                    No router links cross the radio. Robot agents try, in order:{" "}
                    <span className="mono">{z.fleetlink.endpoints.join(", ") || "(base has no fleetlink address)"}</span>
                  </div>
                </>
              ) : null}
              <h4 className="sub-title">Router links ({z.links.length})</h4>
              {z.links.length === 0 ? (
                <div className="muted small">No links (admin REST on :8000 not reachable or no peers).</div>
              ) : (
                <ul className="link-list mono small">
                  {z.links.map((l, i) => (
                    <li key={i}>{linkText(l)}</li>
                  ))}
                </ul>
              )}
              <h4 className="sub-title">Zenoh port reachable</h4>
              <div className="chips">
                {Object.entries(z.reachable ?? {}).map(([m, ok]) => (
                  <span key={m} className={`chip ${ok ? "chip-ok" : "chip-err"}`}>
                    <Dot tone={ok ? "ok" : "err"} /> {m}
                  </span>
                ))}
                {Object.keys(z.reachable ?? {}).length === 0 && <span className="muted small">no machines checked</span>}
              </div>
            </>
          )}
        </Card>

        <Card title="Throughput test (iperf)">
          <div className="row gap wrap end-align">
            <Field label="Machine">
              <select className="input" value={iperfMachine} onChange={e => setIperfMachine(e.target.value)} aria-label="iperf machine">
                <option value="">select</option>
                {machines.filter(m => !m.local).map(m => (
                  <option key={m.name} value={m.name}>
                    {m.name} {m.online ? "" : "(offline)"}
                  </option>
                ))}
              </select>
            </Field>
            <Field label="Seconds">
              <input className="input num-input" type="number" min={1} max={60} value={iperfSecs} onChange={e => setIperfSecs(Number(e.target.value))} />
            </Field>
            <button className="btn btn-primary" disabled={!iperfMachine || busy === "iperf"} onClick={runIperf}>
              {busy === "iperf" && <Spinner />} Run test
            </button>
          </div>
          {ij && (
            <div className="iperf-result">
              <div className="row gap">
                <JobStateBadge state={ij.state} />
                <span className="mono small">{ij.title}</span>
              </div>
              {ij.result !== undefined && ij.result !== null && <IperfResult result={ij.result} />}
            </div>
          )}
        </Card>
      </div>

      <BandwidthSection />

      <Card
        title="Silvus radio mesh"
        actions={
          <>
            <button className="btn btn-sm" onClick={checkRoute} disabled={busy === "route"}>
              {busy === "route" && <Spinner />} Check route
            </button>
            <button className="btn btn-sm" onClick={silvus.reload} disabled={silvus.loading}>
              Refresh
            </button>
          </>
        }
      >
        {silvus.loading && !silvus.data && <Loading label="Querying radios" />}
        {silvus.error && !silvus.data && <ErrorBox error={silvus.error} onRetry={silvus.reload} />}
        {silvus.data && nodes.length === 0 && <Empty title="No radios">No Silvus radios configured in the topology.</Empty>}
        {silvus.data?.radios && silvus.data.radios.length > 0 && <SilvusMesh radios={silvus.data.radios} links={silvus.data.links ?? []} />}
        {silvus.data && nodes.length > 0 && (
          <details className="mesh-details" open={!silvus.data.radios || route !== null}>
            <summary className="small muted">Details: link matrix and routing</summary>
          <div className="grid-2 silvus">
            <div>
              <h4 className="sub-title">{mode === "rssi" ? "RSSI matrix (dBm, row hears column)" : "Link quality (dB, row hears column)"}</h4>
              <div className="table-wrap">
                <table className="heatmap">
                  <thead>
                    <tr>
                      <th />
                      {dsts.map(d => (
                        <th key={d} className="mono">
                          {d}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {nodes.map(src => {
                      const row = silvus.data!.matrix[src];
                      return (
                        <tr key={src}>
                          <th className="mono">{src}</th>
                          {typeof row === "string" ? (
                            <td colSpan={dsts.length} className="heat-err">
                              {row}
                            </td>
                          ) : (
                            dsts.map(d => {
                              const c = row[d];
                              if (src === d) return <td key={d} className="heat-self" />;
                              const v = mode === "rssi" ? c?.rssi_db : c?.link_db;
                              return (
                                <td
                                  key={d}
                                  style={{background: mode === "rssi" ? rssiColor(v) : linkColor(v)}}
                                  className={c?.linked === false ? "heat-nolink" : ""}
                                  title={c ? `${mode === "rssi" ? `rssi ${c.rssi_db} dBm` : `link ${c.link_db} dB`}, ${c.quality ?? "-"}${c.linked ? ", in routing tree" : ""}` : "no data"}
                                >
                                  {v !== undefined && v !== null ? Number(v).toFixed(0) : "-"}
                                </td>
                              );
                            })
                          )}
                        </tr>
                      );
                    })}
                  </tbody>
                </table>
              </div>
              <div className="heat-legend small muted">
                <span>{mode === "rssi" ? "-90" : "5"}</span>
                <span className="heat-scale" />
                <span>{mode === "rssi" ? "-45 dBm" : "35 dB"}</span>
                <span className="heat-nolink-swatch" /> not linked
              </div>
            </div>
            <div>
              <h4 className="sub-title">Routing</h4>
              <pre className="mono small pre-box">{JSON.stringify(silvus.data.routing, null, 2)}</pre>
              {route !== null && (
                <>
                  <h4 className="sub-title">Route check</h4>
                  <pre className="mono small pre-box">{JSON.stringify(route, null, 2)}</pre>
                </>
              )}
            </div>
          </div>
          </details>
        )}
      </Card>
      {deploy && <DeployDialog onClose={() => setDeploy(false)} machines={machines.map(m => ({name: m.name, role: m.role}))} />}
    </div>
  );
}

function IperfResult({result}: {result: unknown}) {
  const r = result as Record<string, unknown>;
  const entries = Object.entries(r ?? {}).filter(([, v]) => typeof v === "number" || typeof v === "string");
  if (!entries.length) return <pre className="mono small pre-box">{JSON.stringify(result, null, 2)}</pre>;
  return (
    <div className="stat-row">
      {entries.map(([k, v]) => (
        <div key={k} className="stat">
          <div className="stat-v mono">{typeof v === "number" ? (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2)) : String(v)}</div>
          <div className="stat-k">{k.replace(/_/g, " ")}</div>
        </div>
      ))}
    </div>
  );
}

function DeployDialog({onClose, machines}: {onClose: () => void; machines: {name: string; role: string}[]}) {
  const [sel, setSel] = useState<string[]>(machines.filter(m => m.role === "base_station").map(m => m.name));
  const [robots, setRobots] = useState<string[]>(machines.filter(m => m.role === "robot").map(m => m.name));
  const [err, setErr] = useState<string | null>(null);
  const go = async () => {
    try {
      const r = await postJSON("api/network/zenoh/deploy", {machines: sel, robots});
      const id = jobIdOf(r);
      if (id) trackJob(id, "Deploy zenoh config");
      onClose();
    } catch (e) {
      setErr(errorText(e));
    }
  };
  const pick = (list: string[], set: (v: string[]) => void, n: string) => set(list.includes(n) ? list.filter(x => x !== n) : [...list, n]);
  return (
    <Modal
      title="Deploy zenoh router config"
      onClose={onClose}
      footer={
        <>
          <button className="btn" onClick={onClose}>
            Cancel
          </button>
          <button className="btn btn-primary" disabled={!sel.length} onClick={go}>
            Deploy
          </button>
        </>
      }
    >
      <p className="muted">Writes connect endpoints (base stations connect to the selected robots) to each selected machine.</p>
      <Field label="Deploy to machines">
        <div className="robot-pick">
          {machines.map(m => (
            <label key={m.name} className={`pick ${sel.includes(m.name) ? "on" : ""}`}>
              <input type="checkbox" checked={sel.includes(m.name)} onChange={() => pick(sel, setSel, m.name)} />
              <span className="pick-name">{m.name}</span>
              <span className="muted small">{m.role.replace("_", " ")}</span>
            </label>
          ))}
        </div>
      </Field>
      <Field label="Robots base stations connect to">
        <div className="chips">
          {machines
            .filter(m => m.role === "robot")
            .map(m => (
              <label key={m.name} className={`chip chip-toggle ${robots.includes(m.name) ? "on" : ""}`}>
                <input type="checkbox" checked={robots.includes(m.name)} onChange={() => pick(robots, setRobots, m.name)} />
                {m.name}
              </label>
            ))}
        </div>
      </Field>
      {err && <ErrorBox error={err} compact />}
    </Modal>
  );
}
