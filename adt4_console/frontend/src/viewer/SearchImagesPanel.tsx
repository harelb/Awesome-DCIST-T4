// Map viewer "Images" tab: open-vocabulary search over the robots' stored
// keyframes, separate from chat. SigLIP2 ranks the keyframe pool; SAM3 then
// detects + reprojects (dry run: nothing is written until "Add to Neo4j").
// "SigLIP2 only" just browses the ranked frames ("where might X be").
import {useCallback, useEffect, useRef, useState} from "react";
import {Badge, Dot, Modal, Segmented, Spinner, Toggle} from "../components/ui";
import {errorText} from "../lib/api";
import {useViewer} from "../stores/viewer";
import {
  DEFAULT_PARAMS,
  MODE_LABEL,
  cacheChip,
  commitCandidate,
  discardPending,
  fetchFrame,
  fetchHealth,
  fmtSecs,
  fmtVec,
  pushHistory,
  rleDecode,
  runQuery,
  runRank,
  seenIn,
  sourceLabel,
  sourceOptions,
  timingRows,
  viewImageUrl,
  visibleCandidates,
  visibleFrames,
  type CandState,
  type HistoryEntry,
  type RankedFrame,
  type Sam3CacheStats,
  type Sam3Candidate,
  type Sam3Health,
  type Sam3Mode,
  type Sam3View,
  type SearchKind,
  type SearchParams,
} from "./sam3Search";
import "./searchImages.css";

const HEALTH_POLL_MS = 10_000;

/** One frame shown full size: the raw keyframe with the candidate's mask/box on top. */
interface Shown {
  title: string;
  frames: {key: string | null | undefined; fallback: string | null; view?: Sam3View; caption: string}[];
  i: number;
}

let nextId = 1;

export function SearchImagesPanel() {
  const [health, setHealth] = useState<Sam3Health | null>(null);
  const [params, setParams] = useState<SearchParams>(DEFAULT_PARAMS);
  const [history, setHistory] = useState<HistoryEntry[]>([]);
  const [currentId, setCurrentId] = useState<number | null>(null);
  const [busy, setBusy] = useState<{kind: SearchKind; since: number} | null>(null);
  const [shown, setShown] = useState<Shown | null>(null);
  const fly = useViewer(s => s.fly);

  const loadHealth = useCallback(async () => {
    try {
      setHealth(await fetchHealth());
    } catch (e) {
      setHealth({ok: false, error: errorText(e)});
    }
  }, []);
  useEffect(() => {
    void loadHealth();
    const id = window.setInterval(() => void loadHealth(), HEALTH_POLL_MS);
    return () => window.clearInterval(id);
  }, [loadHealth]);

  const current = history.find(h => h.id === currentId) ?? null;
  const set = (patch: Partial<SearchParams>) => setParams(p => ({...p, ...patch}));
  const update = (id: number, f: (e: HistoryEntry) => HistoryEntry) => setHistory(list => list.map(e => (e.id === id ? f(e) : e)));

  const search = async (p: SearchParams = params) => {
    if (!p.prompt.trim() || busy) return;
    const entry: HistoryEntry = {id: nextId++, at: Date.now(), params: {...p, prompt: p.prompt.trim()}, status: {}};
    setBusy({kind: p.kind, since: Date.now()});
    setCurrentId(entry.id);
    setHistory(list => pushHistory(list, entry));
    try {
      if (p.kind === "rank") entry.rank = await runRank(p);
      else entry.result = await runQuery(p);
    } catch (e) {
      entry.error = errorText(e);
      void loadHealth();
    } finally {
      setBusy(null);
      setHistory(list => pushHistory(list, {...entry}));
    }
  };

  const setStatus = (id: number, idx: number, st: CandState) => update(id, e => ({...e, status: {...e.status, [idx]: st}}));

  const add = async (e: HistoryEntry, c: Sam3Candidate) => {
    const pid = e.result?.pending_id;
    if (!pid) return;
    setStatus(e.id, c.index, {state: "busy", text: "adding ..."});
    try {
      const r = await commitCandidate(pid, c.index);
      const how = r.merged ? `merged into ${r.symbol ?? "an existing object"}` : `added as ${r.symbol ?? "a new object"}`;
      setStatus(e.id, c.index, {state: "added", text: how});
    } catch (err) {
      setStatus(e.id, c.index, {state: "error", text: errorText(err)});
    }
  };

  /** Dismiss one candidate locally; once nothing is left to add, drop the server's pending result. */
  const dismiss = async (e: HistoryEntry, c: Sam3Candidate | null) => {
    const pid = e.result?.pending_id;
    const all = e.result?.candidates ?? [];
    const status = {...e.status};
    for (const x of c ? [c] : all) if (status[x.index]?.state !== "added") status[x.index] = {state: "discarded", text: "discarded"};
    update(e.id, x => ({...x, status}));
    const open = all.filter(x => !status[x.index] || status[x.index].state === "error" || status[x.index].state === "busy");
    if (pid && open.length === 0) {
      try {
        await discardPending(pid);
      } catch {
        /* expired already: nothing to drop */
      }
    }
  };

  const showViews = (c: Sam3Candidate, pendingId: string | null | undefined, i = 0) =>
    setShown({
      title: `${c.label} #${c.index}`,
      i,
      frames: c.views.map((v, k) => ({key: v.frame_key, fallback: v.image_uri ?? (pendingId ? viewImageUrl(pendingId, c.index, v.view ?? k, 1280) : null), view: v, caption: `view ${k + 1}/${c.views.length}${v.score != null ? ` · SAM3 ${v.score.toFixed(2)}` : ""}${v.source ? ` · ${sourceLabel(v.source)}` : ""}`})),
    });
  const showRanked = (frames: RankedFrame[], i: number) =>
    setShown({
      title: `SigLIP2 ranking`,
      i,
      frames: frames.map(f => ({key: f.frame_key, fallback: f.thumb_uri, caption: `#${f.rank + 1} · similarity ${f.score.toFixed(4)} · ${sourceLabel(f.source)}`})),
    });

  const down = health && !health.ok;
  const srcOpts = sourceOptions(health?.sources);
  const knownSource = srcOpts.some(o => o.value === params.source);

  return (
    <div className="simg">
      <HealthBar health={health} onRetry={() => void loadHealth()} />
      <form
        className="simg-form"
        onSubmit={ev => {
          ev.preventDefault();
          void search();
        }}
      >
        <div className="simg-query">
          <input className="input" value={params.prompt} onChange={e => set({prompt: e.target.value})} placeholder="What to look for, e.g. red fire extinguisher" aria-label="Image search text" maxLength={200} />
          <button className="btn btn-primary" type="submit" disabled={!params.prompt.trim() || Boolean(busy)}>
            {params.kind === "rank" ? "Rank" : "Search"}
          </button>
        </div>
        <div className="simg-row">
          <Segmented<SearchKind>
            ariaLabel="Search kind"
            value={params.kind}
            onChange={kind => set({kind})}
            options={[
              {value: "sam3", label: "SAM3 detect"},
              {value: "rank", label: "SigLIP2 only"},
            ]}
          />
          <select className="input input-sm simg-source" value={knownSource ? params.source : "all"} onChange={e => set({source: e.target.value})} aria-label="Keyframe pool" title="Which keyframes to search">
            {srcOpts.map(o => (
              <option key={o.value} value={o.value}>
                {o.label}
                {o.n !== undefined ? ` · ${o.n.toLocaleString()}` : ""}
              </option>
            ))}
          </select>
        </div>
        <details className="simg-knobs">
          <summary>
            Knobs <span className="muted small mono">{knobSummary(params, health)}</span>
          </summary>
          <div className="simg-knob-grid">
            {params.kind === "sam3" && (
              <label>
                <span>Mode</span>
                <select className="input input-sm" value={params.mode} onChange={e => set({mode: e.target.value as Sam3Mode})}>
                  {(Object.keys(MODE_LABEL) as Sam3Mode[]).map(m => (
                    <option key={m} value={m}>
                      {MODE_LABEL[m]}
                    </option>
                  ))}
                </select>
              </label>
            )}
            <NumKnob label={params.kind === "rank" ? "Frames shown" : "Top-K (SigLIP2)"} title={params.kind === "rank" ? "How many ranked keyframes to show" : "Frames SigLIP2 hands to SAM3 (batched / top-K modes)"} value={params.topK} min={1} max={params.kind === "rank" ? 200 : 500} onChange={topK => set({topK})} disabled={params.kind === "sam3" && params.mode === "frustum"} />
            {params.kind === "sam3" && (
              <>
                <NumKnob label="Max frames" title="Frustum mode: most frames SAM3 runs on" value={params.maxFrames} min={1} max={500} onChange={maxFrames => set({maxFrames})} disabled={params.mode !== "frustum"} />
                <NumKnob label="Patience" title="Frustum mode: stop after this many frames add nothing new" value={params.patience} min={1} max={500} onChange={patience => set({patience})} disabled={params.mode !== "frustum"} />
                <label title="SAM3 detection confidence threshold for this search">
                  <span>SAM3 conf</span>
                  <span className="simg-conf">
                    <input className="input input-sm" type="number" step={0.05} min={0.05} max={0.95} value={params.conf ?? ""} placeholder={health?.conf_threshold !== undefined ? String(health.conf_threshold) : "server"} onChange={e => set({conf: e.target.value === "" ? null : clampNum(Number(e.target.value), 0.01, 0.99)})} />
                  </span>
                </label>
              </>
            )}
            <label title={params.kind === "rank" ? "Hide frames below this SigLIP2 similarity" : "Hide candidates below this SAM3 score"}>
              <span>Min score</span>
              <input className="input input-sm" type="number" step={params.kind === "rank" ? 0.01 : 0.05} min={0} max={1} value={params.minScore} onChange={e => set({minScore: clampNum(Number(e.target.value) || 0, 0, 1)})} />
            </label>
            <button type="button" className="btn btn-xs" onClick={() => setParams(p => ({...DEFAULT_PARAMS, prompt: p.prompt, kind: p.kind, source: p.source}))}>
              Defaults
            </button>
          </div>
        </details>
      </form>

      <div className="simg-results">
        {busy && <BusyLine kind={busy.kind} since={busy.since} />}
        {!busy && !current && (
          <div className="muted small simg-empty">
            {down ? "Start the SAM3 query server to search the robots' images." : "Search the stored keyframes by text. SigLIP2 only shows where the thing might be (fast, no SAM3); SAM3 detect finds, masks and places it in 3D. Nothing is added to the map until you click Add to Neo4j."}
          </div>
        )}
        {current && !(busy && current.id === currentId && !current.result && !current.rank && !current.error) && (
          <EntryView
            e={current}
            onAdd={c => void add(current, c)}
            onDismiss={c => void dismiss(current, c)}
            onFly={p => fly(p as [number, number, number])}
            onShowViews={(c, i) => showViews(c, current.result?.pending_id, i)}
            onShowRanked={showRanked}
            onRetry={() => void search(current.params)}
          />
        )}
      </div>

      {history.length > 0 && (
        <details className="simg-history" open={history.length > 1}>
          <summary>Recent searches ({history.length})</summary>
          <ul>
            {history.map(h => (
              <li key={h.id}>
                <button className={`simg-hist-item ${h.id === currentId ? "on" : ""}`} onClick={() => setCurrentId(h.id)}>
                  <span className="simg-hist-prompt">{h.params.prompt}</span>
                  <span className="muted small mono">
                    {h.params.kind === "rank" ? "SigLIP2" : h.params.mode} · {histOutcome(h)} · {new Date(h.at).toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"})}
                  </span>
                </button>
                <button
                  className="btn btn-xs"
                  title="Load these settings into the form"
                  onClick={() => setParams(h.params)}
                  aria-label={`Reuse settings of ${h.params.prompt}`}
                >
                  ↺
                </button>
              </li>
            ))}
          </ul>
        </details>
      )}
      {shown && <FrameViewer shown={shown} onClose={() => setShown(null)} onStep={i => setShown(s => (s ? {...s, i} : s))} />}
    </div>
  );
}

function clampNum(v: number, lo: number, hi: number) {
  return Number.isFinite(v) ? Math.min(Math.max(v, lo), hi) : lo;
}

function knobSummary(p: SearchParams, h: Sam3Health | null): string {
  if (p.kind === "rank") return `top ${p.topK}${p.minScore ? ` · ≥${p.minScore}` : ""}`;
  const conf = p.conf ?? h?.conf_threshold;
  const budget = p.mode === "frustum" ? `${p.maxFrames} frames · patience ${p.patience}` : `top ${p.topK}`;
  return `${p.mode} · ${budget}${conf !== undefined ? ` · conf ${conf}` : ""}${p.minScore ? ` · ≥${p.minScore}` : ""}`;
}

function histOutcome(h: HistoryEntry): string {
  if (h.error) return "error";
  if (h.rank) return `${h.rank.frames.length} frames`;
  if (h.result) return `${h.result.candidates.length} found`;
  return "running";
}

function NumKnob({label, title, value, min, max, onChange, disabled}: {label: string; title: string; value: number; min: number; max: number; onChange: (v: number) => void; disabled?: boolean}) {
  return (
    <label title={title} className={disabled ? "dim" : undefined}>
      <span>{label}</span>
      <input className="input input-sm" type="number" min={min} max={max} step={1} value={value} disabled={disabled} onChange={e => onChange(Math.round(clampNum(Number(e.target.value), min, max)))} />
    </label>
  );
}

function HealthBar({health, onRetry}: {health: Sam3Health | null; onRetry: () => void}) {
  if (!health) return <div className="simg-health muted small">Checking SAM3 query server ...</div>;
  if (!health.ok)
    return (
      <div className="simg-health simg-down" role="alert">
        <Dot tone="err" />
        <span className="grow">
          <strong>{health.error ?? "SAM3 query server unavailable"}</strong>
          {health.detail && <span className="muted small"> · {health.detail}</span>}
        </span>
        <button className="btn btn-xs" onClick={onRetry}>
          Retry
        </button>
      </div>
    );
  return (
    <div className="simg-health small" title={health.url}>
      <Dot tone={health.busy ? "warn" : "ok"} />
      <span className="grow">
        SAM3 {health.busy ? "busy" : "ready"} on <span className="mono">{health.server}</span>
        <span className="muted">
          {" "}
          · {(health.keyframes ?? 0).toLocaleString()} keyframes · {health.pending ?? 0} pending
          {health.conf_threshold !== undefined && ` · conf ${health.conf_threshold}`}
          {health.embed_model && ` · ${health.embed_model.split("/").pop()}`}
          {featureCacheText(health)}
        </span>
      </span>
    </div>
  );
}

function featureCacheText(h: Sam3Health): string {
  const fc = h.feature_cache;
  if (!fc || !fc.pool) return "";
  const n = fc.pool_cached ?? 0;
  return n >= fc.pool ? " · SAM3 features cached" : ` · SAM3 features ${n.toLocaleString()}/${fc.pool.toLocaleString()}${fc.building ? " (building)" : ""}`;
}

function BusyLine({kind, since}: {kind: SearchKind; since: number}) {
  const [, tick] = useState(0);
  useEffect(() => {
    const id = window.setInterval(() => tick(t => t + 1), 250);
    return () => window.clearInterval(id);
  }, []);
  return (
    <div className="simg-busy">
      <Spinner label={kind === "rank" ? "ranking keyframes" : "SigLIP2 ranking + SAM3 detection"} />
      <span className="mono small muted">{((Date.now() - since) / 1000).toFixed(1)} s</span>
    </div>
  );
}

function Timings({t, total, cache}: {t: Record<string, number> | undefined; total?: number; cache?: Sam3CacheStats}) {
  const rows = timingRows(t);
  const chip = cacheChip(cache);
  if (!rows.length && !chip) return null;
  return (
    <div className="simg-timings mono small" aria-label="Timing breakdown">
      {rows.map(([k, v]) => (
        <span key={k} className="simg-chip">
          {k} {fmtSecs(v)}
        </span>
      ))}
      {chip && (
        <span className={`simg-chip ${cache && cache.hits < cache.frames ? "simg-chip-miss" : "simg-chip-hit"}`} title={chip.title}>
          {chip.text}
        </span>
      )}
      {total !== undefined && <span className="simg-chip simg-chip-total">total {fmtSecs(total)}</span>}
    </div>
  );
}

function EntryView({
  e,
  onAdd,
  onDismiss,
  onFly,
  onShowViews,
  onShowRanked,
  onRetry,
}: {
  e: HistoryEntry;
  onAdd: (c: Sam3Candidate) => void;
  onDismiss: (c: Sam3Candidate | null) => void;
  onFly: (p: number[]) => void;
  onShowViews: (c: Sam3Candidate, i?: number) => void;
  onShowRanked: (frames: RankedFrame[], i: number) => void;
  onRetry: () => void;
}) {
  const p = e.params;
  if (e.error)
    return (
      <div className="simg-error" role="alert">
        <div>
          <strong>“{p.prompt}”</strong> failed: {e.error}
        </div>
        <button className="btn btn-xs" onClick={onRetry}>
          Retry
        </button>
      </div>
    );
  if (e.rank) {
    const frames = visibleFrames(e.rank, p.minScore);
    return (
      <>
        <div className="simg-summary small">
          <span>
            <strong>“{p.prompt}”</strong> · top {frames.length} of {e.rank.pool.toLocaleString()} keyframes ({sourceLabel(e.rank.source)})
          </span>
        </div>
        <Timings t={e.rank.timings} total={e.rank.elapsed_s} />
        {frames.length === 0 ? (
          <div className="muted small simg-empty">{e.rank.note ?? "No frames above the minimum score."}</div>
        ) : (
          <div className="simg-rank-grid">
            {frames.map((f, i) => (
              <figure key={f.frame_key} className="simg-rank">
                <button className="simg-thumb" onClick={() => onShowRanked(frames, i)} aria-label={`Show frame ranked ${f.rank + 1}`}>
                  {f.thumb_uri ? <img src={f.thumb_uri} alt={`keyframe ${f.frame_key}`} loading="lazy" /> : <span className="muted small">no image</span>}
                  <span className="simg-rank-n mono">#{f.rank + 1}</span>
                </button>
                <figcaption className="small mono">
                  <span title="SigLIP2 cosine similarity">{f.score.toFixed(4)}</span>
                  {f.camera_position && (
                    <button className="btn btn-xs" onClick={() => onFly(f.camera_position as number[])} title={`Fly to the camera at ${fmtVec(f.camera_position, 1)}`}>
                      Fly to
                    </button>
                  )}
                </figcaption>
              </figure>
            ))}
          </div>
        )}
      </>
    );
  }
  const r = e.result;
  if (!r) return null;
  const cands = visibleCandidates(r, p.minScore);
  const hidden = r.candidates.length - cands.length;
  const open = r.candidates.some(c => !e.status[c.index] || e.status[c.index].state === "error");
  return (
    <>
      <div className="simg-summary small">
        <span>
          <strong>“{p.prompt}”</strong> · {r.candidates.length} candidate{r.candidates.length === 1 ? "" : "s"}
          {r.detections !== undefined && r.detections > r.candidates.length && (
            <span className="muted" title="Detections of the same object in several frames are grouped into one candidate">
              {" "}
              (from {r.detections} detections)
            </span>
          )}
          {hidden > 0 && <span className="muted"> ({hidden} below min score)</span>}
          <span className="muted">
            {r.pool !== undefined && ` · pool ${r.pool.toLocaleString()}`} · {sourceLabel(r.source ?? p.source)} · {p.mode}
            {r.conf !== undefined && ` · conf ${r.conf}`}
          </span>
        </span>
        {r.pending_id && open && (
          <button className="btn btn-xs" onClick={() => onDismiss(null)} title="Discard every candidate of this search not yet added">
            Discard all
          </button>
        )}
      </div>
      <Timings t={r.timings} total={r.elapsed_s} cache={r.cache} />
      {r.candidates.length === 0 && <div className="muted small simg-empty">{r.note ?? "SAM3 found nothing in these keyframes. Try SigLIP2 only to see what ranks highest, lower SAM3 conf, or the batched mode."}</div>}
      <div className="simg-grid">
        {cands.map(c => (
          <CandidateCard key={c.index} c={c} pendingId={r.pending_id} st={e.status[c.index]} expiresIn={r.expires_in_s} onAdd={() => onAdd(c)} onDismiss={() => onDismiss(c)} onFly={() => onFly(c.centroid)} onShow={i => onShowViews(c, i)} />
        ))}
      </div>
    </>
  );
}

function CandidateCard({c, pendingId, st, onAdd, onDismiss, onFly, onShow}: {c: Sam3Candidate; pendingId?: string | null; st?: CandState; expiresIn?: number; onAdd: () => void; onDismiss: () => void; onFly: () => void; onShow: (i: number) => void}) {
  const [vi, setVi] = useState(0);
  const views = c.views;
  const v = views[vi];
  // rendered by the server only on request: otherwise lazy-load this view's image
  const img = v?.image_uri ?? (v && pendingId ? viewImageUrl(pendingId, c.index, v.view ?? vi) : c.image_uri);
  const done = st && (st.state === "added" || st.state === "discarded");
  const size = c.bbox_min && c.bbox_max ? c.bbox_max.map((x, k) => Math.abs(x - c.bbox_min[k])) : null;
  return (
    <article className={`simg-card ${st?.state === "discarded" ? "is-discarded" : ""} ${st?.state === "added" ? "is-added" : ""}`}>
      <div className="simg-card-img">
        {img ? (
          <button className="simg-img-btn" onClick={() => onShow(vi)} aria-label="Show full frame">
            <img src={img} alt={`SAM3 mask of ${c.label}, view ${vi + 1}`} loading="lazy" decoding="async" />
          </button>
        ) : (
          <div className="simg-noimg muted small">no image</div>
        )}
        {views.length > 1 && (
          <div className="simg-carousel">
            <button className="icon-btn" onClick={() => setVi((vi - 1 + views.length) % views.length)} aria-label="Previous view">
              ‹
            </button>
            <span className="mono small">
              {vi + 1}/{views.length}
              {c.num_views > views.length ? ` (of ${c.num_views})` : ""}
            </span>
            <button className="icon-btn" onClick={() => setVi((vi + 1) % views.length)} aria-label="Next view">
              ›
            </button>
          </div>
        )}
      </div>
      <div className="simg-card-body">
        <div className="row gap between">
          <span className="simg-label">
            <strong>{c.label}</strong> <span className="muted small mono">#{c.index}</span>
          </span>
          <span className="mono small" title="SAM3 score (fused over views)">
            {c.score.toFixed(2)}
          </span>
        </div>
        <div className="small mono simg-pos">
          {fmtVec(c.centroid)}
          {size && <span className="muted"> · {fmtVec(size, 1)} m</span>}
        </div>
        <div className="small muted">
          <span className={(c.num_frames ?? 1) > 1 ? "simg-seen-multi" : undefined} title={c.frame_keys?.length ? `keyframes: ${c.frame_keys.join(", ")}` : undefined}>
            {seenIn(c)}
          </span>
          {c.num_views > (c.num_frames ?? c.num_views) && ` (${c.num_views} detections)`} · {c.num_points.toLocaleString()} pts
          {v?.score != null && ` · view ${v.score.toFixed(2)}`}
        </div>
        {c.would_merge_into ? (
          <Badge tone="info" title="Adding merges this detection into the existing object">
            would merge into {c.would_merge_into.symbol} ({c.would_merge_into.distance_m.toFixed(2)} m)
          </Badge>
        ) : (
          <Badge tone="warn">new object</Badge>
        )}
        <div className="simg-actions">
          <button className="btn btn-xs btn-primary" disabled={Boolean(done) || st?.state === "busy"} onClick={onAdd}>
            Add to Neo4j
          </button>
          <button className="btn btn-xs" disabled={Boolean(done) || st?.state === "busy"} onClick={onDismiss}>
            Discard
          </button>
          <button className="btn btn-xs" onClick={onFly} title="Centre the map on this object">
            Fly to
          </button>
          <button className="btn btn-xs" onClick={() => onShow(vi)} disabled={!views.length}>
            Show frame
          </button>
        </div>
        {st && (
          <div className={`small ${st.state === "added" ? "tone-ok" : st.state === "error" ? "tone-err" : "muted"}`} role="status">
            {st.text}
          </div>
        )}
      </div>
    </article>
  );
}

/** Full keyframe from the server, with the view's mask (decoded from RLE) and box drawn on top. */
function FrameViewer({shown, onClose, onStep}: {shown: Shown; onClose: () => void; onStep: (i: number) => void}) {
  const f = shown.frames[shown.i];
  const [maskOn, setMaskOn] = useState(true);
  const [frame, setFrame] = useState<{key: string; uri: string} | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const v = f?.view;
  const shape = v?.mask_shape;
  const raw = f && frame && frame.key === f.key ? frame.uri : null;
  const overlay = Boolean(raw && v && shape);

  useEffect(() => {
    setErr(null);
    if (!f?.key) return;
    let alive = true;
    fetchFrame(f.key)
      .then(r => alive && setFrame({key: f.key as string, uri: r.uri}))
      .catch(e => alive && setErr(errorText(e)));
    return () => {
      alive = false;
    };
  }, [f?.key]);

  useEffect(() => {
    const cv = canvas.current;
    if (!cv || !v || !shape || !overlay) return;
    const mask = rleDecode(v.mask_rle, shape);
    const [h, w] = shape;
    cv.width = w;
    cv.height = h;
    const ctx = cv.getContext("2d");
    if (!ctx || !mask) return;
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < mask.length; i++) {
      if (!mask[i]) continue;
      img.data[i * 4] = 255;
      img.data[i * 4 + 1] = 0;
      img.data[i * 4 + 2] = 255;
      img.data[i * 4 + 3] = 115;
    }
    ctx.putImageData(img, 0, 0);
  }, [v, shape, overlay]);

  if (!f) return null;
  // without the raw frame (yet) show the server's annotated image, which has the mask baked in
  const src = raw ?? f.fallback;
  const box = v?.bbox_2d;
  return (
    <Modal title={`${shown.title}: ${f.caption}`} onClose={onClose} wide>
      <div className="simg-frame">
        {src ? <img src={src} alt={f.caption} /> : <div className="muted">no image</div>}
        {overlay && shape && (
          <>
            <canvas ref={canvas} className="simg-mask" style={{opacity: maskOn ? 1 : 0}} aria-hidden="true" />
            {maskOn && box && box.length === 4 && (
              <svg className="simg-box" viewBox={`0 0 ${shape[1]} ${shape[0]}`} preserveAspectRatio="none" aria-hidden="true">
                <rect x={box[0]} y={box[1]} width={box[2] - box[0]} height={box[3] - box[1]} />
              </svg>
            )}
          </>
        )}
      </div>
      {err && <div className="small tone-err">Full frame unavailable ({err}); showing the search's image.</div>}
      <div className="row gap center">
        <button className="btn btn-sm" disabled={shown.i === 0} onClick={() => onStep(shown.i - 1)}>
          Previous
        </button>
        {v && <Toggle checked={maskOn} onChange={setMaskOn} label="Mask" />}
        <span className="muted small mono">{f.key}</span>
        <button className="btn btn-sm" disabled={shown.i >= shown.frames.length - 1} onClick={() => onStep(shown.i + 1)}>
          Next
        </button>
      </div>
    </Modal>
  );
}
