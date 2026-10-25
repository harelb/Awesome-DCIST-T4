// The one banner over the map for whatever the next click does: a mission tool
// (start, draw area, mock objects, new target), a map-edit tool (lasso, add
// object), edit mode's selection, or goto picking. While a drawing tool is armed
// the keys work too: Enter finishes, Backspace drops the last corner, Esc cancels.
import {useEffect} from "react";
import {useMission} from "../stores/mission";
import {setEditMode, useMapEdit} from "../stores/mapEdit";
import {armEditTool, cancelTool, dropLastCorner, finishDrawing, useActiveTool, type ActiveTool} from "../stores/tools";
import {useViewer} from "../stores/viewer";

const typing = (t: EventTarget | null) => {
  const el = t as HTMLElement | null;
  return !!el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName));
};

/** Enter / Backspace / Esc for the armed tool (not while typing in a field). */
export function useToolKeys(t: ActiveTool) {
  const armed = t.owner !== "none";
  useEffect(() => {
    if (!armed) return;
    const onKey = (e: KeyboardEvent) => {
      if (typing(e.target) || e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.key === "Enter" && finishDrawing()) e.preventDefault();
      else if (e.key === "Backspace" && dropLastCorner()) e.preventDefault();
      else if (e.key === "Escape") {
        cancelTool();
        e.preventDefault();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [armed]);
}

const KEYS = "Enter or double-click finishes · Backspace drops the last corner · Esc cancels";

export function ToolBanner({phone = false}: {phone?: boolean}) {
  const t = useActiveTool();
  const pickMode = useViewer(s => s.pickMode);
  const editOn = useMapEdit(s => s.on);
  const draft = useMission(s => s.draft.length);
  const lasso = useMapEdit(s => s.lasso.length);
  const task = useMission(s => (s.tool.kind === "retarget" ? s.current?.tasks.find(x => x.id === (s.tool as {task: string}).task) : null));
  useToolKeys(t);
  const tap = phone ? "Tap" : "Click";
  let text: string;
  let corners: number | null = null;
  let finish: (() => void) | null = null;
  let finishLabel = "Finish";
  let done: () => void = cancelTool;
  let doneLabel = "Cancel";
  if (t.owner === "mission") {
    const tool = t.tool;
    switch (tool.kind) {
      case "start":
        text = `${tap} the map: start of ${tool.agent}`;
        break;
      case "area":
        text = `Draw area: ${tap.toLowerCase()} the corners${phone ? "" : ` · ${KEYS}`}`;
        corners = draft;
        finish = () => void finishDrawing();
        break;
      case "plant":
        text = `${tap} to plant mock “${tool.label}” objects`;
        doneLabel = "Done";
        break;
      case "retarget":
        text = `${tap} the new target of ${tool.task}${task ? ` (${task.agent} ${task.type.replace("_", " ")})` : ""}: an object, a place or a map point`;
        break;
    }
  } else if (t.owner === "edit") {
    if (t.tool === "lasso") {
      text = `Lasso: ${tap.toLowerCase()} the corners${phone ? "" : " · Enter selects inside · Backspace drops the last corner · Esc cancels"}`;
      corners = lasso;
      finish = () => void finishDrawing();
      finishLabel = "Select inside";
    } else {
      text = "Add object: click where it is";
      done = () => armEditTool("select");
      doneLabel = "Done";
    }
  } else if (pickMode === "goto") {
    text = phone ? "Tap places, objects or the mesh." : "Goto picking: click places, objects or the mesh.";
    done = () => useViewer.getState().set({pickMode: "inspect"});
    doneLabel = "Done";
  } else if (editOn && pickMode === "edit") {
    text = `Map edit: ${tap.toLowerCase()} to select, Shift-${tap.toLowerCase()} adds`;
    done = () => setEditMode(false);
    doneLabel = "Done";
  } else return null;
  return (
    <div className={`pick-banner ${t.owner === "edit" || (t.owner === "none" && editOn) ? "edit-banner" : ""}`} data-testid="tool-banner" role="status">
      <span className="pick-text">{text!}</span>
      {corners !== null && <span className="mono small">{corners} corner{corners === 1 ? "" : "s"}</span>}
      {finish && (
        <button className="btn btn-xs btn-primary" disabled={(corners ?? 0) < 3} onClick={finish} title={(corners ?? 0) < 3 ? "Needs at least 3 corners" : undefined}>
          {finishLabel}
        </button>
      )}
      <button className="btn btn-xs" onClick={done}>
        {doneLabel}
      </button>
    </div>
  );
}
