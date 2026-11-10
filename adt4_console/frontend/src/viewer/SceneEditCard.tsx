// Chat card for plan repair's scene edits (heracles_agents' SGET tools, e.g.
// remove_scene_place): what the call will do, approve / reject. Approval runs
// exactly that call; the result (e.g. "Deleted place t5571 and its 7 edges
// (map_version 2)") is shown and goes back to the assistant as an [operator]
// note. An applied edit can be undone from the card (undo_scene_edit: the scene
// change writer rebuilds the map without the latest chat edit).
import {Badge} from "../components/ui";
import {postJSON} from "../lib/api";

export interface SceneEditItem {
  type: "scene_edit";
  id: string;
  tool: string;
  args: Record<string, unknown>;
  summary: string;
  text: string;
  src: string | null;
  status?: {state: "busy" | "done" | "rejected" | "error" | "undone"; text: string};
  /** the applied edit can still be undone from this card */
  undoable?: boolean;
}

export interface SceneEditOutcome {
  ok: boolean;
  rejected?: boolean;
  tool?: string;
  result?: string;
  text?: string;
  map_version?: number | null;
  undoable?: boolean;
  error?: string;
}

export function toSceneEditItem(o: Record<string, unknown>): SceneEditItem {
  return {
    type: "scene_edit",
    id: String(o.id),
    tool: String(o.tool ?? ""),
    args: o.args && typeof o.args === "object" ? (o.args as Record<string, unknown>) : {},
    summary: String(o.summary ?? ""),
    text: String(o.text ?? o.tool ?? ""),
    src: typeof o.src === "string" ? o.src : null,
  };
}

export const decideSceneEdit = (id: string, approve: boolean) => postJSON<SceneEditOutcome>("api/chat/scene_edit", {id, approve});
export const undoSceneEdit = (src: string | null) => postJSON<SceneEditOutcome>("api/chat/scene_edit/undo", {src});

/** What the assistant is told about the operator's decision. */
export function sceneEditNote(it: SceneEditItem, r: SceneEditOutcome): string {
  const what = it.summary || it.text;
  if (r.rejected) return `[operator] Rejected the scene edit "${what}"; nothing changed.`;
  if (!r.ok) return `[operator] The scene edit "${what}" was not applied: ${r.result ?? r.error ?? "unknown error"}`;
  return `[operator] Approved the scene edit "${what}": ${r.result ?? "done"}`;
}

export function sceneEditStatus(r: SceneEditOutcome): NonNullable<SceneEditItem["status"]> {
  if (r.rejected) return {state: "rejected", text: "rejected; nothing changed"};
  if (!r.ok) return {state: "error", text: r.result ?? r.error ?? "failed"};
  return {state: "done", text: r.result ?? "applied"};
}

export function SceneEditCard({it, onDecide, onUndo}: {it: SceneEditItem; onDecide: (approve: boolean) => void; onUndo: () => void}) {
  const st = it.status;
  const decided = st && st.state !== "error" && st.state !== "busy";
  return (
    <div className="msg msg-write scene-edit-card" data-testid="scene-edit-card">
      <div className="row gap between">
        <span>
          <Badge tone="warn">plan repair edit</Badge> <strong>{it.summary || it.text}</strong>
        </span>
      </div>
      <div className="small">
        {it.text} <span className="muted mono">({it.tool})</span>
      </div>
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(true)}>
          Approve &amp; apply
        </button>
        <button className="btn btn-sm" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(false)}>
          Reject
        </button>
        {st?.state === "done" && it.undoable && (
          <button className="btn btn-sm" onClick={onUndo} title="Undo the latest chat map edit (the map is rebuilt without it)">
            Undo
          </button>
        )}
      </div>
      {st && (
        <div className={`small ${st.state === "done" || st.state === "undone" ? "tone-ok" : st.state === "error" ? "tone-err" : "muted"}`} role="status">
          {st.text}
        </div>
      )}
    </div>
  );
}
