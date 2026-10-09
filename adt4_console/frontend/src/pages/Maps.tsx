import {useMemo, useState} from "react";
import {useNavigate} from "react-router";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {trackJob, useJobs} from "../stores/jobs";
import {useFleet} from "../stores/fleet";
import type {ConsoleConfig, GraphSource, MapEntry, MapsResponse} from "../types";
import {Badge, Card, Empty, ErrorBox, Field, Loading, Modal, PageHeader, Spinner, Toggle} from "../components/ui";
import {ChecksumMatrix, JobStateBadge} from "../components/Jobs";
import {jobIdOf} from "./Fleet";

type Dialog =
  | {kind: "transfer"; machine: string; map: MapEntry}
  | {kind: "verify"; map: MapEntry}
  | {kind: "ingest"; machine: string; map: MapEntry};

function sizeText(s: MapEntry["size"]): string {
  if (typeof s === "number") {
    const u = ["B", "K", "M", "G", "T"];
    let i = 0;
    let v = s;
    while (v >= 1024 && i < u.length - 1) {
      v /= 1024;
      i++;
    }
    return `${v.toFixed(v >= 10 || i === 0 ? 0 : 1)}${u[i]}`;
  }
  return s || "-";
}

export function MapsPage() {
  const maps = useApi<MapsResponse>("api/maps");
  const config = useApi<ConsoleConfig>("api/config");
  const fleet = useFleet(s => s.machines);
  const [dialog, setDialog] = useState<Dialog | null>(null);
  const [filter, setFilter] = useState("");
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({});
  const [verifyJob, setVerifyJob] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [opening, setOpening] = useState<string | null>(null);
  const jobs = useJobs(s => s.jobs);
  const navigate = useNavigate();

  const local = config.data?.local_machine ?? fleet?.machines.find(m => m.local)?.name ?? "";
  const entries = Object.entries(maps.data?.machines ?? {}).sort(([a], [b]) => (a === local ? -1 : b === local ? 1 : a.localeCompare(b)));
  const machineNames = entries.map(([m]) => m);

  const startJob = async (path: string, body: unknown, title: string) => {
    setErr(null);
    try {
      const r = await postJSON(path, body);
      const id = jobIdOf(r);
      if (id) trackJob(id, title);
      return id;
    } catch (e) {
      setErr(errorText(e));
      return null;
    }
  };

  const openInViewer = async (machine: string, m: MapEntry) => {
    setErr(null);
    setOpening(m.name);
    try {
      // reuse an already registered file source pointing at this run dir, else register one
      const sources = await getJSON<GraphSource[]>("api/graph/sources").catch(() => [] as GraphSource[]);
      let id = (Array.isArray(sources) ? sources : []).find(s => s.id === `file:${m.name}`)?.id;
      if (!id) {
        await postJSON("api/maps/register", {name: m.name, path: m.path});
        id = `file:${m.name}`;
      }
      navigate(`/viewer?src=${encodeURIComponent(id)}`);
    } catch (e) {
      setErr(`${machine}: ${errorText(e)}`);
    } finally {
      setOpening(null);
    }
  };

  const vjob = verifyJob ? jobs[verifyJob] : null;
  const vres = vjob?.result as {rows?: {path: string; consistent: boolean; cells: Record<string, string>}[]; machines?: string[]} | undefined;
  const total = entries.reduce((n, [, l]) => n + (Array.isArray(l) ? l.length : 0), 0);

  return (
    <div className="page">
      <PageHeader
        title="Maps"
        sub={maps.data ? `${total} experiments across ${entries.length} machines` : "Experiment outputs on every machine"}
        actions={
          <>
            <input className="input" placeholder="Filter experiments" value={filter} onChange={e => setFilter(e.target.value)} aria-label="Filter experiments" />
            <button className="btn" onClick={maps.reload} disabled={maps.loading}>
              {maps.loading && <Spinner />} Rescan
            </button>
          </>
        }
      />
      {err && <ErrorBox error={err} compact />}
      {maps.loading && !maps.data && <Loading label="Listing experiments (ssh to each machine)" />}
      {maps.error && !maps.data && <ErrorBox error={maps.error} onRetry={maps.reload} />}
      {maps.data && entries.length === 0 && <Empty title="No machines reported maps" />}

      {vjob && (
        <Card title={<span className="row gap">Verify <JobStateBadge state={vjob.state} /></span>} actions={<button className="btn btn-sm" onClick={() => setVerifyJob(null)}>Dismiss</button>}>
          {vres?.rows && vres.machines ? <ChecksumMatrix rows={vres.rows} machines={vres.machines} /> : <Spinner label="Hashing files on each machine" />}
        </Card>
      )}

      {entries.map(([machine, list]) => (
        <Card
          key={machine}
          className="map-machine"
          title={
            <button className="link-btn" onClick={() => setCollapsed(c => ({...c, [machine]: !c[machine]}))} aria-expanded={!collapsed[machine]}>
              <span className={`caret ${collapsed[machine] ? "" : "open"}`} />
              {machine}
              {machine === local && <Badge tone="info">base station (local)</Badge>}
            </button>
          }
          actions={Array.isArray(list) ? <span className="muted small">{list.length} experiments</span> : <Badge tone="err">unreachable</Badge>}
        >
          {!collapsed[machine] &&
            (Array.isArray(list) ? (
              <MapTable
                list={list.filter(m => !filter || m.name.toLowerCase().includes(filter.toLowerCase()))}
                isLocal={machine === local}
                opening={opening}
                onTransfer={m => setDialog({kind: "transfer", machine, map: m})}
                onVerify={m => setDialog({kind: "verify", map: m})}
                onIngest={m => setDialog({kind: "ingest", machine, map: m})}
                onOpen={m => openInViewer(machine, m)}
              />
            ) : (
              <div className="inline-err mono">{list.error}</div>
            ))}
        </Card>
      ))}

      {dialog?.kind === "transfer" && (
        <TransferDialog
          machine={dialog.machine}
          map={dialog.map}
          local={local}
          onClose={() => setDialog(null)}
          onStart={async body => {
            setDialog(null);
            await startJob("api/maps/transfer", body, `Transfer ${dialog.map.name}`);
          }}
        />
      )}
      {dialog?.kind === "verify" && (
        <VerifyDialog
          map={dialog.map}
          machines={machineNames}
          onClose={() => setDialog(null)}
          onStart={async body => {
            setDialog(null);
            const id = await startJob("api/maps/verify", body, `Verify ${dialog.map.name}`);
            if (id) setVerifyJob(id);
          }}
        />
      )}
      {dialog?.kind === "ingest" && (
        <IngestDialog
          map={dialog.map}
          targets={config.data?.neo4j_targets ?? []}
          onClose={() => setDialog(null)}
          onStart={async body => {
            setDialog(null);
            await startJob("api/maps/ingest", body, `Load ${dialog.map.name} into Neo4j`);
          }}
        />
      )}
    </div>
  );
}

function MapTable({list, isLocal, opening, onTransfer, onVerify, onIngest, onOpen}: {
  list: MapEntry[];
  isLocal: boolean;
  opening: string | null;
  onTransfer: (m: MapEntry) => void;
  onVerify: (m: MapEntry) => void;
  onIngest: (m: MapEntry) => void;
  onOpen: (m: MapEntry) => void;
}) {
  if (!list.length) return <div className="muted pad">No experiments.</div>;
  return (
    <div className="table-wrap">
      <table className="table stack">
        <thead>
          <tr>
            <th>experiment</th>
            <th className="num">size</th>
            <th>contents</th>
            <th className="actions-col">actions</th>
          </tr>
        </thead>
        <tbody>
          {[...list]
            .sort((a, b) => Number(Boolean(b.is_prior)) - Number(Boolean(a.is_prior)) || b.name.localeCompare(a.name))
            .map(m => (
              <tr key={m.path ?? m.name}>
                <td className="cell-title">
                  <div className="row gap">
                    <span className="mono strong">{m.name}</span>
                    {m.is_prior && <Badge tone="info">prior</Badge>}
                  </div>
                  {m.path && <div className="muted small mono ellipsis" title={m.path}>{m.path}</div>}
                </td>
                <td className="num mono" data-label="size">{sizeText(m.size)}</td>
                <td data-label="contents">
                  <div className="chips">
                    {(m.subdirs ?? []).map(s => (
                      <span key={s} className="chip mono">
                        {s}
                      </span>
                    ))}
                  </div>
                </td>
                <td className="actions-col cell-actions">
                  <div className="row gap-sm end wrap">
                    {!isLocal && (
                      <button className="btn btn-sm" onClick={() => onTransfer(m)}>
                        Transfer to base
                      </button>
                    )}
                    <button className="btn btn-sm" onClick={() => onVerify(m)}>
                      Verify
                    </button>
                    {isLocal && (
                      <>
                        <button className="btn btn-sm" onClick={() => onIngest(m)}>
                          Load into Neo4j
                        </button>
                        <button className="btn btn-sm btn-primary" onClick={() => onOpen(m)} disabled={opening === m.name}>
                          {opening === m.name && <Spinner />} Open in viewer
                        </button>
                      </>
                    )}
                  </div>
                </td>
              </tr>
            ))}
        </tbody>
      </table>
    </div>
  );
}

function TransferDialog({machine, map, local, onClose, onStart}: {machine: string; map: MapEntry; local: string; onClose: () => void; onStart: (b: unknown) => void}) {
  const [asName, setAsName] = useState(map.name);
  const [bags, setBags] = useState(false);
  const [limit, setLimit] = useState(false);
  const [kbps, setKbps] = useState(2000);
  return (
    <Modal
      title={`Transfer ${map.name}`}
      onClose={onClose}
      footer={
        <>
          <button className="btn" onClick={onClose}>
            Cancel
          </button>
          <button
            className="btn btn-primary"
            onClick={() =>
              onStart({src: machine, experiment: map.name, as_name: asName && asName !== map.name ? asName : undefined, include_bags: bags, bwlimit_kbps: limit ? kbps : undefined})
            }
          >
            Start transfer
          </button>
        </>
      }
    >
      <p>
        rsync <span className="mono">{machine}</span> to <span className="mono">{local || "this base station"}</span>.
      </p>
      <Field label="Name on base station">
        <input className="input mono" value={asName} onChange={e => setAsName(e.target.value)} />
      </Field>
      <Toggle checked={bags} onChange={setBags} label="Include rosbags (large)" />
      <Toggle checked={limit} onChange={setLimit} label="Limit bandwidth (shared radio)" />
      {limit && (
        <Field label={`Bandwidth limit: ${kbps} kB/s`}>
          <input type="range" min={100} max={20000} step={100} value={kbps} onChange={e => setKbps(Number(e.target.value))} aria-label="Bandwidth limit" />
        </Field>
      )}
    </Modal>
  );
}

function VerifyDialog({map, machines, onClose, onStart}: {map: MapEntry; machines: string[]; onClose: () => void; onStart: (b: unknown) => void}) {
  const [sel, setSel] = useState<string[]>(machines);
  return (
    <Modal
      title={`Verify ${map.name}`}
      onClose={onClose}
      footer={
        <>
          <button className="btn" onClick={onClose}>
            Cancel
          </button>
          <button className="btn btn-primary" disabled={!sel.length} onClick={() => onStart({experiment: map.name, machines: sel})}>
            Compare checksums
          </button>
        </>
      }
    >
      <p>Hashes every file of the experiment on each selected machine and compares them.</p>
      <div className="robot-pick">
        {machines.map(m => (
          <label key={m} className={`pick ${sel.includes(m) ? "on" : ""}`}>
            <input type="checkbox" checked={sel.includes(m)} onChange={() => setSel(s => (s.includes(m) ? s.filter(x => x !== m) : [...s, m]))} />
            <span className="pick-name">{m}</span>
          </label>
        ))}
      </div>
    </Modal>
  );
}

function IngestDialog({map, targets, onClose, onStart}: {map: MapEntry; targets: ConsoleConfig["neo4j_targets"]; onClose: () => void; onStart: (b: unknown) => void}) {
  const sorted = useMemo(() => [...targets].sort((a, b) => Number(b.prior) - Number(a.prior)), [targets]);
  const [target, setTarget] = useState(sorted[0]?.name ?? "");
  return (
    <Modal
      title={`Load ${map.name} into Neo4j`}
      tone="warn"
      onClose={onClose}
      footer={
        <>
          <button className="btn" onClick={onClose}>
            Cancel
          </button>
          <button className="btn btn-primary" disabled={!target} onClick={() => onStart({path: map.path, target})}>
            Continue
          </button>
        </>
      }
    >
      {targets.length === 0 ? (
        <Empty title="No Neo4j targets configured">Start the console with --neo4j NAME=URI.</Empty>
      ) : (
        <>
          <Field label="Target database">
            <select className="input" value={target} onChange={e => setTarget(e.target.value)} aria-label="Neo4j target">
              {sorted.map(t => (
                <option key={t.name} value={t.name}>
                  {t.name} {t.prior ? "(prior)" : ""} {t.uri}
                </option>
              ))}
            </select>
          </Field>
          <p className="muted">The target is wiped first. You will be asked to type a confirmation showing its current node count.</p>
        </>
      )}
    </Modal>
  );
}
