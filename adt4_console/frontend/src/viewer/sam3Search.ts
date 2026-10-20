// "Search images" tool: types, API calls and pure helpers for the SAM3 query
// server (via the console's /api/sam3/* routes). The UI is SearchImagesPanel.tsx.
import {getJSON, postJSON} from "../lib/api";
import {withToken} from "../lib/auth";

export type Sam3Mode = "frustum" | "batched" | "topk";
/** "sam3": SigLIP2 ranking + SAM3 detection (dry run); "rank": SigLIP2 only */
export type SearchKind = "sam3" | "rank";

export interface Sam3Source {
  id: string;
  keyframes: number;
}

export interface Sam3Health {
  ok: boolean;
  url?: string;
  server?: string;
  error?: string;
  detail?: string;
  status?: string;
  keyframes?: number;
  pending?: number;
  busy?: boolean;
  conf_threshold?: number;
  embed_model?: string;
  admission_policy?: string | null;
  live_store_root?: string | null;
  sources?: Sam3Source[];
  /** SAM3 image-feature cache (frames cached / being built) */
  feature_cache?: {pool?: number; pool_cached?: number; building?: boolean; queued?: number; gpu?: number; disk?: number} | null;
  compile_warmup_s?: number;
}

export interface Sam3View {
  /** rank of the view within its candidate (0 = best); the lazy image's `view` */
  view?: number;
  frame_key?: string | null;
  source?: string | null;
  frame_id: number | null;
  timestamp_ns: number;
  bbox_2d: number[];
  score: number | null;
  mask_rle: string | number[] | null;
  mask_shape: number[] | null;
  image_uri: string | null;
}

export interface Sam3Candidate {
  index: number;
  label: string;
  score: number;
  centroid: number[];
  bbox_min: number[];
  bbox_max: number[];
  num_points: number;
  num_views: number;
  /** distinct keyframes the object was detected in (same-object clustering) */
  num_frames?: number;
  /** every member view's frame key, best first */
  frame_keys?: string[];
  would_merge_into: {symbol: string; distance_m: number} | null;
  image_uri: string | null;
  views: Sam3View[];
}

export interface Sam3QueryResult {
  prompt: string;
  dry_run?: boolean;
  pending_id: string | null;
  expires_in_s?: number;
  candidates: Sam3Candidate[];
  elapsed_s: number;
  timings: Record<string, number>;
  pool?: number;
  source?: string;
  conf?: number;
  note?: string;
  /** raw per-view detections before same-object clustering */
  detections?: number;
  association?: string;
  /** SAM3 image-feature cache use of this query */
  cache?: Sam3CacheStats;
}

export interface Sam3CacheStats {
  frames: number;
  hits: number;
  hit_rate: number | null;
  gpu?: number;
  cpu?: number;
  disk?: number;
  miss?: number;
  feat_s?: number;
}

export interface RankedFrame {
  rank: number;
  frame_key: string;
  frame_id: number | null;
  timestamp_ns: number;
  source: string;
  score: number;
  camera_position: number[] | null;
  thumb_uri: string | null;
}

export interface RankResult {
  prompt: string;
  source: string;
  pool: number;
  frames: RankedFrame[];
  elapsed_s: number;
  timings: Record<string, number>;
  note?: string;
}

export interface FrameImage {
  frame_key: string;
  uri: string;
  width: number;
  height: number;
  timestamp_ns?: number;
  source?: string;
}

export interface SearchParams {
  kind: SearchKind;
  prompt: string;
  source: string; // "all" | "prior" | "live" | "live/<robot>/<epoch>"
  mode: Sam3Mode;
  topK: number;
  maxFrames: number;
  patience: number;
  /** SAM3 confidence threshold; null = the server's */
  conf: number | null;
  /** client-side cut on the candidate (SAM3) / frame (SigLIP2) score */
  minScore: number;
}

/** Serving defaults (same budgets as the chat tool, sam3_client.py). */
export const DEFAULT_PARAMS: SearchParams = {
  kind: "sam3",
  prompt: "",
  source: "all",
  mode: "frustum",
  topK: 60,
  maxFrames: 30,
  patience: 3,
  conf: null,
  minScore: 0,
};

export const MODE_LABEL: Record<Sam3Mode, string> = {
  frustum: "Frustum (fast)",
  batched: "Batched (high recall)",
  topk: "Top-K sequential",
};

export type CandState = {state: "busy" | "added" | "discarded" | "error"; text: string};

export interface HistoryEntry {
  id: number;
  at: number;
  params: SearchParams;
  /** SAM3 dry-run result (kind "sam3") */
  result?: Sam3QueryResult;
  /** SigLIP2 ranking (kind "rank") */
  rank?: RankResult;
  error?: string;
  /** per-candidate action state, by candidate index */
  status: Record<number, CandState>;
}

export const HISTORY_MAX = 20;

/** Newest first, bounded; an entry with the same id replaces the old one in place. */
export function pushHistory(list: HistoryEntry[], e: HistoryEntry, max = HISTORY_MAX): HistoryEntry[] {
  if (list.some(x => x.id === e.id)) return list.map(x => (x.id === e.id ? e : x));
  return [e, ...list].slice(0, max);
}

export function sourceLabel(id: string): string {
  if (id === "all") return "All keyframes";
  if (id === "prior") return "Prior map (Neo4j)";
  if (id === "live") return "All live mirrors";
  const m = /^live\/([^/]+)\/(.+)$/.exec(id);
  if (m) {
    const t = Number(m[2]);
    const when = Number.isFinite(t) && t > 1e9 ? new Date(t * 1000).toLocaleString([], {month: "short", day: "numeric", hour: "2-digit", minute: "2-digit"}) : m[2];
    return `Live: ${m[1]} (${when})`;
  }
  return id;
}

/** Source picker options from /health (always offers "all"; "live" when there is more than one mirror). */
export function sourceOptions(sources: Sam3Source[] | undefined): {value: string; label: string; n?: number}[] {
  const list = sources ?? [];
  const total = list.reduce((n, s) => n + s.keyframes, 0);
  const live = list.filter(s => s.id.startsWith("live/"));
  const out: {value: string; label: string; n?: number}[] = [{value: "all", label: sourceLabel("all"), n: list.length ? total : undefined}];
  for (const s of list.filter(s => !s.id.startsWith("live/"))) out.push({value: s.id, label: sourceLabel(s.id), n: s.keyframes});
  if (live.length > 1) out.push({value: "live", label: sourceLabel("live"), n: live.reduce((n, s) => n + s.keyframes, 0)});
  for (const s of live) out.push({value: s.id, label: sourceLabel(s.id), n: s.keyframes});
  return out;
}

/** Timing rows to show (seconds), in pipeline order, then any extra keys. */
export function timingRows(t: Record<string, number> | undefined): [string, number][] {
  if (!t) return [];
  const order = ["connect", "embed_rank", "detect", "evidence", "render"];
  const notSeconds = ["cache_hit"];
  const keys = [...order.filter(k => k in t), ...Object.keys(t).filter(k => !order.includes(k) && !notSeconds.includes(k))];
  return keys.map(k => [k, t[k]] as [string, number]);
}

/** "cache 95% (19/20 frames)" chip text + a tooltip with the tier split; null without stats. */
export function cacheChip(c: Sam3CacheStats | undefined): {text: string; title: string} | null {
  if (!c || !c.frames) return null;
  const pct = Math.round(100 * (c.hit_rate ?? c.hits / c.frames));
  const tiers = (["gpu", "cpu", "disk", "miss"] as const).filter(k => (c[k] ?? 0) > 0).map(k => `${k} ${c[k]}`);
  return {
    text: `cache ${pct}% (${c.hits}/${c.frames})`,
    title: `SAM3 image features of the ${c.frames} frames searched: ${tiers.join(" · ")}${c.miss ? " (miss = computed now, cached for next time)" : ""}`,
  };
}

/** "seen in N frames" of a candidate (falls back to its view count). */
export function seenIn(c: Pick<Sam3Candidate, "num_frames" | "num_views">): string {
  const n = c.num_frames ?? c.num_views;
  return `seen in ${n} frame${n === 1 ? "" : "s"}`;
}

/** Lazy image of a dry-run candidate view (mask + box drawn by the server), for <img loading="lazy">. */
export function viewImageUrl(pendingId: string, index: number, view: number, maxDim = 480): string {
  const q = new URLSearchParams({pending_id: pendingId, index: String(index), view: String(view), max_dim: String(maxDim)});
  return withToken(`api/sam3/frame.jpg?${q.toString()}`);
}

export function visibleCandidates(r: Sam3QueryResult | undefined, minScore: number): Sam3Candidate[] {
  return (r?.candidates ?? []).filter(c => (c.score ?? 0) >= minScore);
}

export function visibleFrames(r: RankResult | undefined, minScore: number): RankedFrame[] {
  return (r?.frames ?? []).filter(f => (f.score ?? 0) >= minScore);
}

/** Decode a hydra_image_store mask RLE (row-major runs, background first). */
export function rleDecode(rle: string | number[] | null | undefined, shape: number[] | null | undefined): Uint8Array | null {
  if (!rle || !shape || shape.length < 2) return null;
  let counts: number[];
  try {
    counts = typeof rle === "string" ? (JSON.parse(rle) as number[]) : rle;
  } catch {
    return null;
  }
  if (!Array.isArray(counts)) return null;
  const n = shape[0] * shape[1];
  const out = new Uint8Array(n);
  let p = 0;
  for (let i = 0; i < counts.length; i++) {
    const c = Number(counts[i]);
    if (!Number.isFinite(c) || c < 0 || p + c > n) return null;
    if (i % 2 === 1) out.fill(1, p, p + c);
    p += c;
  }
  return p === n ? out : null;
}

export const fmtVec = (v: number[] | null | undefined, nd = 2) => (v && v.length ? `(${v.map(x => x.toFixed(nd)).join(", ")})` : "—");

export const fmtSecs = (s: number | undefined) => (s === undefined ? "—" : s < 1 ? `${Math.round(s * 1000)} ms` : `${s.toFixed(2)} s`);

// --- API ------------------------------------------------------------------------

export const fetchHealth = () => getJSON<Sam3Health>("api/sam3/health");

/** views per candidate returned by a search (best first) */
export const MAX_VIEWS = 3;

export function runQuery(p: SearchParams): Promise<Sam3QueryResult> {
  return postJSON<Sam3QueryResult>("api/sam3/query", {
    prompt: p.prompt.trim(),
    mode: p.mode,
    top_k: p.topK,
    max_frames: p.maxFrames,
    patience: p.patience,
    source: p.source,
    conf: p.conf ?? undefined,
    max_views: MAX_VIEWS,
    // images are fetched lazily per shown view (viewImageUrl)
    render: "none",
  });
}

export function runRank(p: SearchParams): Promise<RankResult> {
  return postJSON<RankResult>("api/sam3/rank", {prompt: p.prompt.trim(), top_k: p.topK, source: p.source});
}

export const fetchFrame = (key: string, maxDim = 1280) => getJSON<FrameImage>(`api/sam3/frame?key=${encodeURIComponent(key)}&max_dim=${maxDim}`);

export const commitCandidate = (pendingId: string, index: number, src?: string) =>
  postJSON<{symbol?: string; created?: boolean; merged?: boolean}>("api/sam3/commit", {pending_id: pendingId, index, src});

export const discardPending = (pendingId: string) => postJSON<{discarded: boolean}>("api/sam3/discard", {pending_id: pendingId});
