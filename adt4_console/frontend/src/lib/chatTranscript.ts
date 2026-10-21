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
