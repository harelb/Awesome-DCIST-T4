// Chat conversation state that is not React: the transcript the server needs
// for the next turn, and the operator's decisions (approved writes, added
// objects) phrased as notes the model reads.

/** One transcript item, as the server's ``transcript`` event sends them. */
export type TranscriptItem =
  | {role: "user"; text: string}
  | {role: "assistant"; text: string; calls?: {id: string; name: string; args: unknown}[]}
  | {role: "tool"; id: string; name: string; content: string};

/** Keep the most recent items, starting at a user message (what the server accepts). */
export function trimTranscript(items: TranscriptItem[], max = 60): TranscriptItem[] {
  let out = items.slice(-max);
  while (out.length && out[0].role !== "user") out = out.slice(1);
  return out;
}

export interface WriteOutcome {
  ok: boolean;
  rejected?: boolean;
  query?: string;
  counters?: Record<string, number>;
  rows?: unknown[];
  error?: string;
}

/** The operator's answer to a proposed write, for the model. */
export function writeNote(summary: string, r: WriteOutcome): string {
  if (r.rejected) return `[operator] Rejected the proposed write (${summary || "no summary"}); it did not run.`;
  if (!r.ok) return `[operator] Approved the write (${summary || "no summary"}) but it failed: ${r.error ?? "unknown error"}`;
  const c = Object.entries(r.counters ?? {})
    .map(([k, v]) => `${k.replace(/_/g, " ")} ${v}`)
    .join(", ");
  return `[operator] Approved and ran the write (${summary || "no summary"}): ${c || "no changes"}.`;
}

export interface CommitOutcome {
  symbol?: string;
  merged?: boolean | string;
}

/** An approved SAM3 candidate, for the model: which symbol it now has. */
export function candidateNote(label: string, r: CommitOutcome): string {
  const sym = r.symbol ?? "an object";
  return r.merged
    ? `[operator] Approved the "${label}" candidate; it was merged into the mapped object ${sym}.`
    : `[operator] Approved the "${label}" candidate; it was added to the map as ${sym}.`;
}

/** Seconds left as m:ss ("expired" at 0). */
export function fmtLeft(s: number): string {
  if (s <= 0) return "expired";
  const m = Math.floor(s / 60);
  return `${m}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

/** A Cypher result cell as short text (points / lists / objects compact). */
export function cellText(v: unknown): string {
  if (v === null || v === undefined) return "";
  if (Array.isArray(v) && v.length <= 4 && v.every(x => typeof x === "number")) return `[${v.map(x => (Number.isInteger(x) ? x : x.toFixed(2))).join(", ")}]`;
  if (typeof v === "object") {
    const s = JSON.stringify(v);
    return s.length > 120 ? s.slice(0, 117) + "..." : s;
  }
  return String(v);
}

/** The SAM3 query server's merge radius: a committed candidate merges into a
 *  mapped object with the same label within this distance
 *  (``sam3_frontend/config.py`` ``dedup_centroid_dist_m``). */
export const SAM3_MERGE_RADIUS_M = 0.75;

interface CandLike {
  label: string;
  centroid: number[] | null;
  would_merge_into?: {symbol: string; distance_m: number | null} | null;
  status?: {state: string} | undefined;
}

/** After one candidate was committed as ``symbol`` at ``at``, what each other
 *  undecided candidate with the same label would now merge into (or its old
 *  value). Same indices as ``cands``. */
export function mergeTargetsAfterCommit<T extends CandLike>(cands: T[], label: string, at: number[] | null, symbol: string, radius = SAM3_MERGE_RADIUS_M): T["would_merge_into"][] {
  return cands.map(c => {
    if (c.status || !at || !c.centroid || c.label !== label) return c.would_merge_into ?? null;
    const d = Math.hypot(...c.centroid.map((v, k) => v - (at[k] ?? 0)));
    if (d > radius) return c.would_merge_into ?? null;
    const cur = c.would_merge_into;
    return cur && cur.distance_m !== null && cur.distance_m <= d ? cur : {symbol, distance_m: Math.round(d * 100) / 100};
  });
}

interface SearchCard {
  type: string;
  pending_id?: string;
}

/** "k of n" for each SAM3 card among its search's cards (same pending id); null
 *  for other items. Same indices as ``items``. */
export function candidateOrdinals(items: SearchCard[]): ({k: number; n: number} | null)[] {
  const n = new Map<string, number>();
  for (const it of items) if (it.type === "object_candidate" && it.pending_id) n.set(it.pending_id, (n.get(it.pending_id) ?? 0) + 1);
  const seen = new Map<string, number>();
  return items.map(it => {
    if (it.type !== "object_candidate" || !it.pending_id) return null;
    const k = (seen.get(it.pending_id) ?? 0) + 1;
    seen.set(it.pending_id, k);
    return {k, n: n.get(it.pending_id) ?? k};
  });
}

/** Where the chat log scrolls to: the first card of the newest SAM3 search
 *  that arrived at or after ``from`` (the current turn), so every candidate is
 *  below it instead of only the last one being in view; null: the bottom. */
export function scrollAnchor(items: SearchCard[], from: number): number | null {
  let i = items.length - 1;
  while (i >= from && items[i].type !== "object_candidate") i--;
  if (i < from) return null;
  const pid = items[i].pending_id;
  while (i - 1 >= from && items[i - 1].type === "object_candidate" && items[i - 1].pending_id === pid) i--;
  return i;
}
