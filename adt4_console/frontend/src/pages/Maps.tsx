import {useEffect, useMemo, useRef, useState, type ReactNode} from "react";
import {useNavigate} from "react-router";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {trackJob, useJobs} from "../stores/jobs";
import {useFleet} from "../stores/fleet";
import type {ConsoleConfig, GraphSource, MapEntry, MapsResponse} from "../types";
import {Badge, Card, Empty, ErrorBox, Field, Loading, Modal, PageHeader, Spinner, Toggle} from "../components/ui";
import {ChecksumMatrix, JobStateBadge} from "../components/Jobs";
import {jobIdOf} from "./Fleet";
import {fmtBytes, matchRanges} from "../lib/fuzzy";
import {loadPrefs, mapTime, savePrefs, searchMaps, sortMaps, SORT_LABELS, type MapRow, type MapsPrefs, type SearchHit, type SortKey} from "../lib/mapList";

type Dialog =
  | {kind: "transfer"; machine: string; map: MapEntry}
  | {kind: "verify"; map: MapEntry}
  | {kind: "ingest"; machine: string; map: MapEntry};

const fmtDate = (epochS: number | null | undefined, withTime = true): string => {
  if (!epochS) return "-";
  const d = new Date(epochS * 1000);
  const pad = (n: number) => String(n).padStart(2, "0");
  const day = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}`;
  return withTime ? `${day} ${pad(d.getHours())}:${pad(d.getMinutes())}` : day;
};

function sizeText(m: MapEntry): string {
  if (typeof m.size_bytes === "number") return fmtBytes(m.size_bytes);
  if (typeof m.size === "number") return fmtBytes(m.size);
  return m.size && m.size !== "?" ? m.size : "-";
}

/** `text` with the fuzzy-matched characters marked. */
export function Highlight({text, indices}: {text: string; indices?: number[]}) {
  if (!indices?.length) return <>{text}</>;
  const parts: ReactNode[] = [];
  let at = 0;
  for (const [a, b] of matchRanges(indices.filter(i => i < text.length))) {
    if (a > at) parts.push(text.slice(at, a));
    parts.push(
      <mark key={a} className="fz-hit">
        {text.slice(a, b)}
      </mark>,
    );
    at = b;
  }
  if (at < text.length) parts.push(text.slice(at));
  return <>{parts}</>;
}

/** "images: 559 kf · 117 MB" (+ check result) or "no images". */
export function ImageStoreBadge({map}: {map: MapEntry}) {
  const st = map.image_store;
  if (!map.has_image_store || !st)
    return (
      <span className="img-badge">
        <Badge tone="off">no images</Badge>
      </span>
    );
  const kf = st.frames?.keyframe ?? 0;
  const size = st.size_bytes != null ? ` · ${fmtBytes(st.size_bytes)}` : map.size_pending ? " · …" : "";
  const kinds = Object.entries(st.frames ?? {})
    .map(([k, n]) => `${n} ${k}`)
    .join(", ");
  const title = [
    `image store: ${kinds || "no frames"}`,
    `${st.observations} object observations, ${st.sensors} sensor(s)`,
    map.sessions ? `${map.sessions} earlier session store(s) under sessions/` : "",
    st.check.ok ? "quick check: ok" : `quick check FAILED: ${st.check.problems.join("; ")}`,
  ]
    .filter(Boolean)
    .join("\n");
  return (
    <span className="img-badge">
      <Badge tone={st.check.ok ? "ok" : "warn"} title={title}>
        images: {kf} kf{size}
        {!st.check.ok && " · check failed"}
      </Badge>
    </span>
  );
}

function FileChips({map}: {map: MapEntry}) {
  if (map.has_dsg === undefined) return null; // older backend
  const files: [string, boolean | undefined][] = [
    ["dsg.json", map.has_dsg],
    ["dsg_with_mesh", map.has_dsg_mesh],
    ["mesh.ply", map.has_mesh],
  ];
  return (
    <>
      {files.map(([label, ok]) => (
        <span key={label} className={`chip mono ${ok ? "chip-ok" : "chip-missing"}`} title={`hydra/backend/${label}${label.endsWith("mesh") ? ".json" : ""} ${ok ? "present" : "missing"}`}>
          {label}
        </span>
      ))}
    </>
  );
}

export function MapsPage() {
  const maps = useApi<MapsResponse>("api/maps");
  const config = useApi<ConsoleConfig>("api/config");
  const fleet = useFleet(s => s.machines);
  const [dialog, setDialog] = useState<Dialog | null>(null);
  const [query, setQuery] = useState("");
  const [prefs, setPrefsState] = useState<MapsPrefs>(loadPrefs);
  const [collapsed, setCollapsed] = useState<Record<string, boolean>>({});
  const [verifyJob, setVerifyJob] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [opening, setOpening] = useState<string | null>(null);
  const jobs = useJobs(s => s.jobs);
  const navigate = useNavigate();
  const setPrefs = (p: Partial<MapsPrefs>) =>
    setPrefsState(old => {
      const next = {...old, ...p};
      savePrefs(next);
      return next;
    });

  // "/" focuses the search box (fzf habit), unless already typing somewhere
  const searchRef = useRef<HTMLInputElement>(null);
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement | null;
      if (e.key !== "/" || e.ctrlKey || e.metaKey || e.altKey) return;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT" || t.isContentEditable)) return;
      e.preventDefault();
      searchRef.current?.focus();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // sizes still being measured in the background: list again shortly (bounded)
  const polls = useRef(0);
  const pending = maps.data?.pending ?? [];
  useEffect(() => {
    if (!pending.length || maps.loading || polls.current >= 40) return;
    const id = window.setTimeout(() => {
      polls.current += 1;
      maps.reload();
    }, 4000);
    return () => window.clearTimeout(id);
  }, [pending.length, maps.loading, maps.reload]);

  // the backend lists this host as "local" when the topology does not name it
  const local = config.data?.local_machine || fleet?.machines.find(m => m.local)?.name || "local";
  const entries = Object.entries(maps.data?.machines ?? {}).sort(([a], [b]) => (a === local ? -1 : b === local ? 1 : a.localeCompare(b)));
  const machineNames = entries.map(([m]) => m);

  const rows = useMemo<MapRow[]>(() => entries.flatMap(([machine, list]) => (Array.isArray(list) ? list.map(map => ({machine, map})) : [])), [maps.data]); // eslint-disable-line react-hooks/exhaustive-deps
  const shown = useMemo(() => sortMaps(searchMaps(rows, query), prefs.sort, query), [rows, query, prefs.sort]);
  const searching = query.trim().length > 0;

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
  const total = rows.length;
  const withImages = rows.filter(r => r.map.has_image_store).length;
  const tableProps = (machine: string | null) => ({
    local,
    opening,
    showMachine: machine === null,
    onTransfer: (r: SearchHit) => setDialog({kind: "transfer", machine: r.machine, map: r.map}),
    onVerify: (r: SearchHit) => setDialog({kind: "verify", map: r.map}),
    onIngest: (r: SearchHit) => setDialog({kind: "ingest", machine: r.machine, map: r.map}),
    onOpen: (r: SearchHit) => openInViewer(r.machine, r.map),
  });

  return (
    <div className="page">
      <PageHeader
        title="Maps"
        sub={
          maps.data ? (
            <span className="row gap wrap">
              {searching ? `${shown.length} of ${total} runs` : `${total} runs`} across {entries.length} machines · {withImages} with an image store
              {pending.length > 0 && (
                <span className="muted small row gap-sm">
                  <Spinner /> measuring sizes on {pending.join(", ")}
                </span>
              )}
            </span>
          ) : (
            "Experiment outputs on every machine"
          )
        }
        actions={
          <>
            <input
              className="input maps-search"
              type="search"
              placeholder="Search maps (fuzzy)  /"
              value={query}
              onChange={e => setQuery(e.target.value)}
              onKeyDown={e => e.key === "Escape" && setQuery("")}
              aria-label="Search maps"
              ref={searchRef}
            />
            <select className="input maps-sort" value={prefs.sort} onChange={e => setPrefs({sort: e.target.value as SortKey})} aria-label="Sort maps">
              {(Object.keys(SORT_LABELS) as SortKey[]).map(k => (
                <option key={k} value={k}>
                  {SORT_LABELS[k]}
                </option>
              ))}
            </select>
            <Toggle checked={prefs.grouped} onChange={v => setPrefs({grouped: v})} label="Group by machine" />
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

      {!prefs.grouped && maps.data && (
        <>
          {entries
            .filter(([, list]) => !Array.isArray(list))
            .map(([machine, list]) => (
              <div key={machine} className="inline-err mono">
                {machine}: unreachable — {(list as {error: string}).error}
              </div>
            ))}
          <Card className="map-machine" title="All machines" actions={<span className="muted small">{shown.length} runs</span>}>
            <MapTable rows={shown} {...tableProps(null)} emptyText={searching ? `No map matches "${query}".` : "No experiments."} />
          </Card>
        </>
      )}

      {prefs.grouped &&
        entries.map(([machine, list]) => {
          const mine = shown.filter(r => r.machine === machine);
          if (searching && Array.isArray(list) && !mine.length) return null;
          return (
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
              actions={
                Array.isArray(list) ? (
                  <span className="muted small">{searching ? `${mine.length} of ${list.length}` : list.length} experiments</span>
                ) : (
                  <Badge tone="err">unreachable</Badge>
                )
              }
            >
              {!collapsed[machine] &&
                (Array.isArray(list) ? <MapTable rows={mine} {...tableProps(machine)} emptyText="No experiments." /> : <div className="inline-err mono">{list.error}</div>)}
            </Card>
          );
        })}
      {prefs.grouped && searching && maps.data && !shown.length && <Empty title={`No map matches "${query}"`}>Matching is fuzzy over run name, machine and path; try fewer letters.</Empty>}

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

const OTHER_SUBDIR_SKIP = new Set(["image_store"]);
const MAX_SUBDIR_CHIPS = 6;

function SubdirChips({subdirs}: {subdirs: string[]}) {
  const all = subdirs.filter(s => !OTHER_SUBDIR_SKIP.has(s));
  const rest = all.slice(MAX_SUBDIR_CHIPS);
  return (
    <>
      {all.slice(0, MAX_SUBDIR_CHIPS).map(s => (
        <span key={s} className="chip mono chip-dim">
          {s}/
        </span>
      ))}
      {rest.length > 0 && (
        <span className="chip chip-dim" title={rest.map(s => `${s}/`).join("\n")}>
          +{rest.length} more
        </span>
      )}
    </>
  );
}

function MapTable({rows, local, opening, showMachine, emptyText, onTransfer, onVerify, onIngest, onOpen}: {
  rows: SearchHit[];
  local: string;
  opening: string | null;
  showMachine: boolean;
  emptyText: string;
  onTransfer: (r: SearchHit) => void;
  onVerify: (r: SearchHit) => void;
  onIngest: (r: SearchHit) => void;
  onOpen: (r: SearchHit) => void;
}) {
  if (!rows.length) return <div className="muted pad">{emptyText}</div>;
  return (
    <div className="table-wrap">
      <table className="table stack maps-table">
        <thead>
          <tr>
            <th>experiment</th>
            <th>date</th>
            <th className="num">size</th>
            <th>contents</th>
            <th className="actions-col">actions</th>
          </tr>
        </thead>
        <tbody>
          {rows.map(r => {
            const m = r.map;
            const isLocal = r.machine === local;
            const t = mapTime(m);
            return (
              <tr key={`${r.machine}:${m.path ?? m.name}`}>
                <td className="cell-title">
                  <div className="row gap wrap">
                    <span className="mono strong">
                      <Highlight text={m.name} indices={r.hits.name} />
                    </span>
                    {m.is_prior && <Badge tone="info">prior</Badge>}
                    {showMachine && (
                      <span className="chip mono">
                        <Highlight text={r.machine} indices={r.hits.machine} />
                      </span>
                    )}
                  </div>
                  {m.path && (
                    <div className="muted small mono ellipsis" title={m.abs_path ?? m.path}>
                      <Highlight text={m.path} indices={r.hits.path} />
                    </div>
                  )}
                  {!showMachine && r.hits.machine && (
                    <div className="muted small">
                      machine: <Highlight text={r.machine} indices={r.hits.machine} />
                    </div>
                  )}
                  {m.error && <div className="inline-err mono small">{m.error}</div>}
                </td>
                <td className="mono small nowrap" data-label="date" title={m.mtime ? `modified ${fmtDate(m.mtime)}` : m.name_date ? "date parsed from the run name" : "unknown"}>
                  {m.mtime ? fmtDate(t) : m.name_date ? `${fmtDate(m.name_date, false)}*` : "-"}
                </td>
                <td className="num mono nowrap" data-label="size">
                  {m.size_pending ? (
                    <span className="muted" title="measuring (du) in the background">
                      <Spinner />
                    </span>
                  ) : (
                    sizeText(m)
                  )}
                </td>
                <td data-label="contents">
                  <div className="chips">
                    <ImageStoreBadge map={m} />
                    <FileChips map={m} />
                    <SubdirChips subdirs={m.subdirs ?? []} />
                  </div>
                </td>
                <td className="actions-col cell-actions">
                  <div className="row gap-sm end wrap">
                    {!isLocal && (
                      <button className="btn btn-sm" onClick={() => onTransfer(r)}>
                        Transfer to base
                      </button>
                    )}
                    <button className="btn btn-sm" onClick={() => onVerify(r)}>
                      Verify
                    </button>
                    {isLocal && (
                      <>
                        <button className="btn btn-sm" onClick={() => onIngest(r)}>
                          Load into Neo4j
                        </button>
                        <button className="btn btn-sm btn-primary" onClick={() => onOpen(r)} disabled={opening === m.name}>
                          {opening === m.name && <Spinner />} Open in viewer
                        </button>
                      </>
                    )}
                  </div>
                </td>
              </tr>
            );
          })}
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
  const hasStore = map.has_image_store ?? (map.subdirs ?? []).includes("image_store");
  // default for a store: keyframes only, fast codec (JPEG q75, half-res depth + zstd)
  const [images, setImages] = useState<"keyframes" | "all" | "none">(hasStore ? "keyframes" : "all");
  const [compression, setCompression] = useState<"none" | "lossless" | "fast">(hasStore ? "fast" : "none");
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
              onStart({
                src: machine,
                experiment: map.name,
                as_name: asName && asName !== map.name ? asName : undefined,
                include_bags: bags,
                bwlimit_kbps: limit ? kbps : undefined,
                images,
                compression: images === "none" ? "none" : compression,
              })
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
      {hasStore ? (
        <div className="row gap wrap">
          <Field label="Image store frames">
            <select className="input" value={images} onChange={e => setImages(e.target.value as typeof images)} aria-label="Image store frames">
              <option value="keyframes">Keyframes only (what SAM3 searches)</option>
              <option value="all">All frames</option>
              <option value="none">None (skip the image store)</option>
            </select>
          </Field>
          <Field label="Compression">
            <select
              className="input"
              value={images === "none" ? "none" : compression}
              disabled={images === "none"}
              onChange={e => setCompression(e.target.value as typeof compression)}
              aria-label="Image store compression"
            >
              <option value="fast">Fast (RGB q75, half-res depth + zstd)</option>
              <option value="lossless">Lossless (stored RGB, depth delta + zstd)</option>
              <option value="none">None (copy as stored)</option>
            </select>
          </Field>
        </div>
      ) : (
        <p className="muted small">This run has no image store; the run directory is copied as is.</p>
      )}
      {hasStore && images === "all" && compression === "none" && <p className="muted small">Plain rsync of the whole run directory.</p>}
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
