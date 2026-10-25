// Map edit mode (API.md §17), the Map tab: the edited Neo4j source, the
// selection (click, Shift-click, search, lasso), the armed tool, staged edits /
// history / snapshots, the map's areas (Room nodes; listed with the mission's
// own areas in the Mission tab), the undo toast and the subtree focus. Map
// clicks arrive from ViewerPage while the viewer's pickMode is "edit".
import {create} from "zustand";
import {errorText} from "../lib/api";
import {editApi, nodesInPolygon, uiSummary, uiSym, type EditLayer, type EditOp, type EditResult, type EditState, type HistoryItem, type Region, type SnapshotInfo} from "../lib/mapEdit";
import {useViewer} from "./viewer";
import {useJobs} from "./jobs";
import {openMission, useMission} from "./mission";
import {ws} from "../ws";

export type EditTool = "select" | "lasso" | "add";

/** A map edit this browser made, undoable (Ctrl+Z takes the newest of these and the start moves). */
export interface EditUndo {
  mutationId: string;
  summary: string;
  at: number;
}

export interface UndoToast {
  text: string;
  mutationId: string | null;
  staged: boolean;
  t: number;
}

interface MapEditStore {
  on: boolean;
  src: string | null;
  selection: string[];
  tool: EditTool;
  lasso: [number, number][];
  lassoLayers: EditLayer[];
  addClass: string;
  state: EditState | null;
  history: HistoryItem[];
  regions: Region[];
  snapshots: SnapshotInfo[];
  stagedSummary: string[];
  historyOpen: boolean;
  focusRoot: string | null;
  toast: UndoToast | null;
  /** this browser's committed edits, newest last */
  editUndo: EditUndo[];
  busy: boolean;
  error: string | null;
  set: (p: Partial<MapEditStore>) => void;
}

export const useMapEdit = create<MapEditStore>(set => ({
  on: false,
  src: null,
  selection: [],
  tool: "select",
  lasso: [],
  lassoLayers: ["Object"],
  addClass: "",
  state: null,
  history: [],
  regions: [],
  snapshots: [],
  stagedSummary: [],
  historyOpen: false,
  focusRoot: null,
  toast: null,
  editUndo: [],
  busy: false,
  error: null,
  set: p => set(p),
}));

// e2e scripts (scripts/capture-map-edit.mjs) read the edit state from here
if (typeof window !== "undefined") (window as unknown as {__adt4MapEdit?: unknown}).__adt4MapEdit = useMapEdit;

// Shift-click adds to the selection: the canvas picker reports only the node, so
// remember the modifier of the last pointer-up (capture phase, before the picker).
let lastShift = false;
if (typeof window !== "undefined") window.addEventListener("pointerup", e => (lastShift = e.shiftKey || e.metaKey || e.ctrlKey), true);
export const shiftHeld = () => lastShift;

/** The Neo4j source edits go to: the planner's map (first Neo4j source). */
export function editSource(): string | null {
  const v = useViewer.getState();
  const enabled = v.order.find(id => v.sources[id]?.kind === "neo4j" && v.sources[id]?.enabled);
  return enabled ?? v.order.find(id => v.sources[id]?.kind === "neo4j") ?? null;
}

/** The pick mode the viewer returns to after a mission tool: "edit" while editing. */
export const idlePickMode = (): "edit" | "inspect" => (useMapEdit.getState().on ? "edit" : "inspect");

export function editModel() {
  const src = useMapEdit.getState().src;
  return src ? (useViewer.getState().sources[src]?.model ?? null) : null;
}

let unsub: (() => void) | null = null;

/** Turn edit mode on/off (map clicks select instead of inspecting). Editing
 *  disarms a mission tool: one map tool at a time (stores/tools.ts). */
export function setEditMode(on: boolean) {
  const s = useMapEdit.getState();
  const src = on ? (s.src ?? editSource()) : s.src;
  if (on && useMission.getState().tool.kind !== "none") useMission.getState().set({tool: {kind: "none"}, draft: []});
  s.set({on, src, tool: "select", lasso: [], error: on && !src ? "No Neo4j map to edit: enable a Neo4j source (file maps are read-only)." : null});
  const v = useViewer.getState();
  if (on && src) {
    if (!v.sources[src]?.enabled) v.enableSource(src, true);
    v.set({pickMode: "edit"});
    void refreshEdit();
    unsub?.();
    unsub = ws.on(`edit/${src}`, () => void refreshEdit());
  } else {
    if (v.pickMode === "edit") v.set({pickMode: "inspect"});
    unsub?.();
    unsub = null;
    s.set({focusRoot: null});
  }
}

/** Reload mode/staged/history/regions/snapshots. */
export async function refreshEdit() {
  const src = useMapEdit.getState().src;
  if (!src) return;
  try {
    const [state, hist, regions, snaps] = await Promise.all([editApi.state(src), editApi.history(src, 40), editApi.regions(src), editApi.snapshots(src)]);
    let stagedSummary: string[] = [];
    if (state.staged.length) stagedSummary = (await editApi.staged(src)).summary;
    useMapEdit.getState().set({state, history: hist.history, regions: regions.regions, snapshots: snaps.snapshots, stagedSummary, error: null});
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
  }
}

/** The map's areas (Room nodes) and the edit mode (immediate / staged), also
 *  outside edit mode: the Mission tab lists them with the mission's own areas. */
export async function loadAreas() {
  const s = useMapEdit.getState();
  const src = s.src ?? editSource();
  if (!src) return;
  if (!s.src) s.set({src});
  try {
    const [state, regions] = await Promise.all([editApi.state(src), editApi.regions(src)]);
    if (useMapEdit.getState().src === src) useMapEdit.getState().set({state, regions: regions.regions});
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
  }
}

/** Open a node in the Map tab: edit mode on, the node selected. */
export function editInMapTab(src: string, sym: string) {
  const s = useMapEdit.getState();
  if (s.src !== src) s.set({src});
  if (!useMapEdit.getState().on) setEditMode(true);
  useMapEdit.getState().set({selection: [sym]});
  if (typeof window !== "undefined") window.dispatchEvent(new CustomEvent("adt4-show-tab", {detail: "map"}));
}

/** A search hit while editing: it goes into the selection (Shift adds), the tab stays. */
export function pickIntoSelection(src: string, sym: string): boolean {
  const s = useMapEdit.getState();
  if (!s.on || src !== s.src) return false;
  selectNode(sym, shiftHeld());
  return true;
}

function toastFor(r: EditResult, verb?: string): UndoToast {
  const lines = (r.summary ?? []).map(uiSummary);
  const text = r.staged ? `Staged (${r.count ?? "?"} pending): ${lines.join("; ") || verb || "edit"}` : lines.join("; ") || verb || "Done";
  return {text, mutationId: r.staged ? null : r.mutation_id, staged: r.staged, t: Date.now()};
}

/** Run ops (one unit of work, or staged while robots are live). */
export async function applyOps(ops: EditOp[], label?: string): Promise<EditResult | null> {
  const s = useMapEdit.getState();
  if (!s.src || !ops.length) return null;
  s.set({busy: true, error: null});
  try {
    const r = await editApi.batch(s.src, ops, label);
    const toast = toastFor(r, label);
    const undo = r.staged || !r.mutation_id ? [] : [{mutationId: r.mutation_id, summary: toast.text, at: Date.now()}];
    useMapEdit.getState().set({toast, editUndo: [...useMapEdit.getState().editUndo, ...undo].slice(-20)});
    await afterEdit();
    return r;
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
    return null;
  } finally {
    useMapEdit.getState().set({busy: false});
  }
}

async function afterEdit() {
  const src = useMapEdit.getState().src;
  await refreshEdit();
  // the open mission's preview says whether the map changed since it was planned
  const cur = useMission.getState().current;
  if (cur) void openMission(cur.id);
  // the viewer reloads on the graph/<src>/version push; nudge it in case the push lags
  if (src) void useViewer.getState().reloadSource(src);
}

export async function undoEdit(mutationId?: string | null) {
  const s = useMapEdit.getState();
  if (!s.src) return;
  s.set({busy: true, error: null});
  try {
    const r = await editApi.undo(s.src, mutationId);
    const stack = useMapEdit.getState().editUndo;
    // the server undoes `mutationId`, or its last edit (ours when it is the newest)
    const gone = mutationId ?? lastUndoable(useMapEdit.getState().history)?.mutation_id ?? null;
    useMapEdit.getState().set({
      toast: {text: `Undone: ${r.summary.map(uiSummary).join("; ")}`, mutationId: null, staged: false, t: Date.now()},
      editUndo: gone ? stack.filter(u => u.mutationId !== gone) : stack.slice(0, -1),
    });
    await afterEdit();
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
  } finally {
    useMapEdit.getState().set({busy: false});
  }
}

export async function publishStaged(confirm: string): Promise<boolean> {
  const s = useMapEdit.getState();
  if (!s.src) return false;
  s.set({busy: true, error: null});
  try {
    const r = await editApi.publish(s.src, confirm);
    const cur = useMapEdit.getState().state;
    useMapEdit.getState().set({
      toast: {text: `Published ${r.published} staged edit(s) to the robots' map`, mutationId: r.mutation_id, staged: false, t: Date.now()},
      stagedSummary: [],
      state: cur ? {...cur, staged: []} : cur,
    });
    await afterEdit();
    return true;
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
    return false;
  } finally {
    useMapEdit.getState().set({busy: false});
  }
}

export async function discardStaged(index?: number) {
  const s = useMapEdit.getState();
  if (!s.src) return;
  try {
    await editApi.discard(s.src, index);
    await refreshEdit();
  } catch (e) {
    s.set({error: errorText(e)});
  }
}

/** Retire the selection (undoable). */
export async function deleteSelection() {
  const s = useMapEdit.getState();
  const ops = s.selection.map(symbol => ({op: "retire_node", symbol, reason: "removed by the operator"}));
  const r = await applyOps(ops, `remove ${s.selection.length} node(s)`);
  if (r && !r.staged) useMapEdit.getState().set({selection: []});
}

/** A map area (a Room node) over a polygon: the places and objects inside become its members. */
export async function makeRegion(name: string, polygon: number[][]): Promise<EditResult | null> {
  return applyOps([{op: "group", parent_layer: "Room", class: "region", name, polygon}], `area '${name}'`);
}

export async function takeSnapshot(name: string) {
  const s = useMapEdit.getState();
  if (!s.src) return;
  s.set({busy: true});
  try {
    await editApi.snapshot(s.src, name);
    await refreshEdit();
  } catch (e) {
    useMapEdit.getState().set({error: errorText(e)});
  } finally {
    useMapEdit.getState().set({busy: false});
  }
}

/** Restore runs as a job with a typed confirm ("restore"): open the job drawer. */
export async function restoreSnapshot(id: string) {
  const s = useMapEdit.getState();
  if (!s.src) return;
  try {
    const {job_id} = await editApi.restore(s.src, id);
    useJobs.getState().open(job_id);
  } catch (e) {
    s.set({error: errorText(e)});
  }
}

// ------------------------------------------------------------ map clicks
export function selectNode(sym: string, additive: boolean) {
  const s = useMapEdit.getState();
  if (!additive) return s.set({selection: [sym]});
  s.set({selection: s.selection.includes(sym) ? s.selection.filter(x => x !== sym) : [...s.selection, sym]});
}

/** A click on the map in edit mode: `sym` (when a node of the edited source was
 *  hit) and the map-frame point. Returns true when the edit mode consumed it. */
export function onEditClick(src: string | null, sym: string | null, p: [number, number, number] | null): boolean {
  const s = useMapEdit.getState();
  if (!s.on) return false;
  if (s.tool === "lasso") {
    if (p) s.set({lasso: [...s.lasso, [p[0], p[1]]]});
    return true;
  }
  if (s.tool === "add") {
    if (p) void addObjectAt(p);
    return true;
  }
  if (sym && src && src === s.src) {
    selectNode(sym, shiftHeld());
    useViewer.getState().select({src, sym});
  } else if (!shiftHeld()) s.set({selection: []});
  return true;
}

export async function addObjectAt(p: [number, number, number]) {
  const s = useMapEdit.getState();
  const cls = s.addClass.trim().toLowerCase();
  if (!cls) {
    s.set({error: "Type the new object's class first"});
    return;
  }
  const r = await applyOps([{op: "create_node", layer: "Object", pos: p, class: cls, name: cls}], `add ${cls}`);
  const sym = r?.changes.find(c => c.kind === "create")?.symbol;
  if (sym) useMapEdit.getState().set({selection: [uiSym(String(sym))]});
}

/** Close the lasso: select the nodes inside it. */
export function finishLasso() {
  const s = useMapEdit.getState();
  const model = editModel();
  if (!model || s.lasso.length < 3) return s.set({lasso: [], tool: "select"});
  const syms = nodesInPolygon(model, s.lasso, s.lassoLayers);
  const sel = shiftHeld() ? [...new Set([...s.selection, ...syms])] : syms;
  s.set({selection: sel, tool: "select", lasso: []});
}

/** Short "what it undoes" for an Undo button: "Undo: remove O(18)". */
export function undoLabel(summary: string | null | undefined, max = 36): string {
  // the server's "(retired, can be undone)" goes without saying on an Undo button
  const t = (summary ?? "").replace(/^(Done|Staged[^:]*):\s*/, "").replace(/\s*\([^)]*can be undone\)/g, "").trim();
  if (!t) return "Undo";
  return `Undo: ${t.length > max ? `${t.slice(0, max - 1)}…` : t}`;
}

/** The newest undoable edit in the map's history (what "Undo last" undoes). */
export function lastUndoable(history: HistoryItem[]): HistoryItem | null {
  return history.find(h => h.undoable && !h.undo_of && !h.undone_by) ?? null;
}
