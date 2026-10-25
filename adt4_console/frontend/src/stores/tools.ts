// One map tool at a time: the mission tools (set a start, draw an area, plant a
// mock object, pick a new target) and the map-edit tools (lasso, add object)
// share the map clicks, so arming one disarms the other, and the viewer shows
// one banner for whichever is armed. Also the drawing keys (Enter / double-click
// finish, Backspace drops the last corner, Esc cancels) and saving a finished
// drawing as an area (the area popover: name, purpose, scope), and Ctrl/Cmd+Z:
// undo the newest of the last start move and this browser's last map edit.
import {useMission, finishDraft, addMissionArea, takenAreaNames, undoStart, type AreaPurpose, type AreaScope, type MapTool, type StartMove} from "./mission";
import {finishLasso, idlePickMode, loadAreas, makeRegion, undoEdit, useMapEdit, type EditTool, type EditUndo} from "./mapEdit";
import {useViewer} from "./viewer";

export type ActiveTool = {owner: "none"} | {owner: "mission"; tool: Exclude<MapTool, {kind: "none"}>} | {owner: "edit"; tool: "lasso" | "add"};

/** Which tool the next map click goes to (pure, for tests and the banner). */
export function activeToolOf(missionTool: MapTool, editOn: boolean, editTool: EditTool): ActiveTool {
  if (missionTool.kind !== "none") return {owner: "mission", tool: missionTool};
  if (editOn && (editTool === "lasso" || editTool === "add")) return {owner: "edit", tool: editTool};
  return {owner: "none"};
}

export function activeTool(): ActiveTool {
  const e = useMapEdit.getState();
  return activeToolOf(useMission.getState().tool, e.on, e.tool);
}

export function useActiveTool(): ActiveTool {
  const missionTool = useMission(s => s.tool);
  const on = useMapEdit(s => s.on);
  const editTool = useMapEdit(s => s.tool);
  return activeToolOf(missionTool, on, editTool);
}

/** Arm a mission tool (or {kind: "none"} to disarm it); a map-edit lasso / add is dropped. */
export function armMissionTool(tool: MapTool) {
  const e = useMapEdit.getState();
  if (e.tool !== "select" || e.lasso.length) e.set({tool: "select", lasso: []});
  useMission.getState().set({tool, draft: []});
  useViewer.getState().set({pickMode: tool.kind === "none" ? idlePickMode() : "mission"});
}

/** Pick a map-edit tool; an armed mission tool (and its unfinished drawing) is dropped. */
export function armEditTool(tool: EditTool) {
  const m = useMission.getState();
  if (m.tool.kind !== "none" || m.draft.length) m.set({tool: {kind: "none"}, draft: []});
  useMapEdit.getState().set({tool, lasso: []});
  if (useMapEdit.getState().on) useViewer.getState().set({pickMode: "edit"});
}

/** Esc: drop whatever is armed (and its unfinished drawing). */
export function cancelTool() {
  const t = activeTool();
  if (t.owner === "mission") armMissionTool({kind: "none"});
  else if (t.owner === "edit") armEditTool("select");
}

/** Enter / double-click: finish the drawing (an area goes to the popover, a lasso selects). */
export function finishDrawing(): boolean {
  const t = activeTool();
  if (t.owner === "mission" && t.tool.kind === "area") {
    finishDraft();
    useViewer.getState().set({pickMode: idlePickMode()});
    return true;
  }
  if (t.owner === "edit" && t.tool === "lasso") {
    finishLasso();
    return true;
  }
  return false;
}

/** Backspace: drop the last corner of the drawing. */
export function dropLastCorner(): boolean {
  const t = activeTool();
  if (t.owner === "mission" && t.tool.kind === "area") {
    const d = useMission.getState().draft;
    useMission.getState().set({draft: d.slice(0, -1)});
    return true;
  }
  if (t.owner === "edit" && t.tool === "lasso") {
    const l = useMapEdit.getState().lasso;
    useMapEdit.getState().set({lasso: l.slice(0, -1)});
    return true;
  }
  return false;
}

/** The drawing's corner count (the banner), or null when nothing is being drawn. */
export function cornerCount(t: ActiveTool): number | null {
  if (t.owner === "mission" && t.tool.kind === "area") return useMission.getState().draft.length;
  if (t.owner === "edit" && t.tool === "lasso") return useMapEdit.getState().lasso.length;
  return null;
}

/** Names of the map's areas (for uniqueness and default names). */
export function mapAreaNames(): string[] {
  return useMapEdit.getState().regions.map(r => r.name ?? "").filter(Boolean);
}

/** Why a name cannot be used, or null. */
export function areaNameProblem(name: string): string | null {
  const n = name.trim();
  if (!n) return "Give the area a name";
  if (takenAreaNames(mapAreaNames()).has(n.toLowerCase())) return `There is already an area called “${n}”`;
  return null;
}

/** Save the drawing waiting in the popover. Mission scope stays in the browser
 *  (and goes into the mission); map scope makes a Room through the edit API
 *  (no-go zones are mission scope only). Returns an error text, or null. */
export async function saveArea(name: string, purpose: AreaPurpose, scope: AreaScope): Promise<string | null> {
  const poly = useMission.getState().pendingArea;
  if (!poly) return "Nothing drawn";
  const bad = areaNameProblem(name);
  if (bad) return bad;
  if (scope === "map" && purpose === "no_go") return "No-go zones belong to the mission only";
  if (scope === "mission") {
    addMissionArea(name.trim(), poly, purpose);
    return null;
  }
  const r = await saveToMap(name.trim(), poly);
  if (!r) return useMapEdit.getState().error ?? "Could not save the area to the map";
  useMission.getState().set({pendingArea: null});
  return null;
}

/** A Room node with this boundary (the places and objects inside become its members). */
export async function saveToMap(name: string, polygon: number[][]) {
  const e = useMapEdit.getState();
  if (!e.src) await loadAreas();
  const r = await makeRegion(name, polygon);
  if (r) void loadAreas();
  return r;
}

/** "Save to map" on a mission-only search area: once it is in the map (not just
 *  staged), the mission copy goes (one area, one label). */
export async function promoteArea(name: string): Promise<boolean> {
  const a = useMission.getState().areas.find(x => x.name === name);
  if (!a) return false;
  const r = await saveToMap(a.name, a.polygon);
  if (!r) return false;
  if (!r.staged) useMission.getState().set({areas: useMission.getState().areas.filter(x => x.name !== name)});
  return true;
}

/** What Ctrl/Cmd+Z undoes: the newer of the last agent-start move and the last
 *  map edit this browser committed (null: nothing). A start move without a time
 *  (older state) counts as older than any edit. */
export function undoTarget(starts: readonly StartMove[], edits: readonly EditUndo[]): "start" | "edit" | null {
  const s = starts[starts.length - 1];
  const e = edits[edits.length - 1];
  if (!s && !e) return null;
  if (!e) return "start";
  if (!s) return "edit";
  return (s.at ?? 0) > e.at ? "start" : "edit";
}

/** Ctrl/Cmd+Z: undo the newest start move or map edit; returns what it undid. */
export async function undoLatest(): Promise<"start" | "edit" | null> {
  const edits = useMapEdit.getState().editUndo;
  const t = undoTarget(useMission.getState().startUndo, edits);
  if (t === "start") await undoStart();
  else if (t === "edit") await undoEdit(edits[edits.length - 1].mutationId);
  return t;
}
