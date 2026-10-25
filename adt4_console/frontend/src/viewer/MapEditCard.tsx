// Chat card for propose_map_edit (API.md §17.5): the diff the dry run produced,
// approve / reject. Approval applies exactly those ops (or stages them while
// robots are live); the outcome goes back to the assistant as an [operator] note.
import {Badge} from "../components/ui";
import {postJSON} from "../lib/api";
import {uiSym} from "../lib/mapEdit";

export interface MapEditItem {
  type: "map_edit";
  id: string;
  src: string | null;
  summary: string;
  diff: string[];
  affected: string[];
  staged: boolean;
  status?: {state: "busy" | "done" | "rejected" | "error"; text: string};
}

export interface MapEditOutcome {
  ok: boolean;
  rejected?: boolean;
  staged?: boolean;
  summary?: string[];
  mutation_id?: string | null;
  error?: string;
}

export function toMapEditItem(o: Record<string, unknown>): MapEditItem {
  return {
    type: "map_edit",
    id: String(o.id),
    src: typeof o.src === "string" ? o.src : null,
    summary: String(o.summary ?? ""),
    diff: Array.isArray(o.diff) ? o.diff.map(String) : [],
    affected: Array.isArray(o.affected) ? o.affected.map(String) : [],
    staged: Boolean(o.staged),
  };
}

export const decideMapEdit = (id: string, approve: boolean) => postJSON<MapEditOutcome>("api/chat/map_edit", {id, approve});

/** What the assistant is told about the operator's decision. */
export function mapEditNote(it: MapEditItem, r: MapEditOutcome): string {
  const what = it.summary || it.diff.join("; ");
  if (r.rejected) return `[operator] Rejected the map edit "${what}"; nothing changed.`;
  if (!r.ok) return `[operator] The map edit "${what}" failed: ${r.error ?? "unknown error"}.`;
  if (r.staged) return `[operator] Approved the map edit "${what}"; it is staged and reaches the robots when the operator publishes the staged edits.`;
  return `[operator] Approved and applied the map edit "${what}": ${(r.summary ?? it.diff).join("; ")}.`;
}

export function statusOf(r: MapEditOutcome): NonNullable<MapEditItem["status"]> {
  if (r.rejected) return {state: "rejected", text: "rejected; nothing changed"};
  if (!r.ok) return {state: "error", text: r.error ?? "failed"};
  return {state: "done", text: r.staged ? "staged (publish it in Map edit)" : "applied · undo it in Map edit › History"};
}

export function MapEditCard({it, onDecide}: {it: MapEditItem; onDecide: (approve: boolean) => void}) {
  const st = it.status;
  const decided = st && st.state !== "error" && st.state !== "busy";
  return (
    <div className="msg msg-write map-edit-card" data-testid="map-edit-card">
      <div className="row gap between">
        <span>
          <Badge tone="warn">map edit</Badge> <strong>{it.summary || "Map edit"}</strong>
        </span>
        {it.staged && <Badge tone="info">staged until published</Badge>}
      </div>
      <ul className="map-edit-diff mono small">
        {it.diff.map((d, i) => (
          <li key={i}>{d}</li>
        ))}
      </ul>
      {it.affected.length > 0 && (
        <div className="small muted">
          touches {it.affected.slice(0, 8).map(uiSym).join(", ")}
          {it.affected.length > 8 ? ` +${it.affected.length - 8}` : ""}
        </div>
      )}
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(true)}>
          {it.staged ? "Approve & stage" : "Approve & apply"}
        </button>
        <button className="btn btn-sm" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(false)}>
          Reject
        </button>
        {st && (
          <span className={`small ${st.state === "done" ? "tone-ok" : st.state === "error" ? "tone-err" : "muted"}`} role="status">
            {st.text}
          </span>
        )}
      </div>
    </div>
  );
}
