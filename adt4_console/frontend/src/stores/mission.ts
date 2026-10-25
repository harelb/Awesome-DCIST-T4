// Mission planner state (API.md §15): the agent roster, the operator's
// mission-only areas (search areas and no-go zones; map areas are Room nodes,
// see stores/mapEdit.ts) and planted mock objects, the selected mission with
// its preview, the run (simulation or live) streamed on `mission/<id>`, and the
// map tool the next click goes to (stores/tools.ts arms one tool at a time).
// Satellite display lives here too (§14).
import {create} from "zustand";
import {deleteJSON, errorText, getJSON, postJSON, putJSON} from "../lib/api";
import type {GeoInfo, TileLayer} from "../lib/geo";
import {roundXY, startLockReason, type StartDrag} from "../lib/agentStart";
import {ws} from "../ws";
import {useViewer} from "./viewer";
import {OVERLAY_SELECTOR, overlayInsets} from "../lib/fitView";

export type AgentKind = "spot" | "husky" | "quad";
export const MAX_AGENTS = 5;

export interface Agent {
  id: string;
  kind: AgentKind;
  start?: number[] | null;
  speed?: number | null;
  binding?: string | null;
  color?: string | null;
  altitude?: number;
}

export interface Task {
  id: string;
  agent: string;
  type: "goto" | "inspect" | "explore_area" | "request_image" | "wait_for" | "return_home" | "hold";
  target?: string | number[] | null;
  area?: number[][] | null;
  depends_on?: string[];
  params?: Record<string, unknown>;
  note?: string;
}

export interface Polygon {
  id: string;
  polygon: number[][];
  kind?: "no_go";
  note?: string;
}

export interface Planted {
  label: string;
  pos: number[];
  size_m?: number;
}

export interface Mission {
  id: string;
  name: string;
  text: string;
  src: string | null;
  planner: string;
  agents: Agent[];
  tasks: Task[];
  geofences: Polygon[];
  planted: Planted[];
  created_at: number;
  revision: number;
  saved?: boolean;
  chat?: {items: unknown[]; transcript: unknown[]} | null;
  decisions?: unknown[];
  map_revision?: number | null;
  map_version?: number | null;
  map_snapshot?: string | null;
}

export interface Issue {
  level: "error" | "warning";
  msg: string;
  /** "missing_target" (the map has no such symbol), "map_stale", "planted_unmatched",
   *  "start_offgraph" (a ground start away from the places graph: ``snap`` is the nearest place) */
  code?: string;
  snap?: number[];
  offgraph_m?: number;
  target?: string | number[] | null;
  task?: string;
  agent?: string;
  agents?: string[];
  t?: number;
  pos?: number[];
}

export interface TaskPreview {
  agent: string;
  type: string;
  path: number[][];
  length_m: number;
  move_s: number;
  dwell_s: number;
  tentative: boolean;
  symbol: string | null;
  target: number[] | null;
  offgraph_m: number;
  deps: string[];
  event: string | null;
  t_start: number;
  t_end: number;
}

export interface Preview {
  tasks: Record<string, TaskPreview>;
  agents: Record<string, {start: number[] | null; end_s: number; distance_m: number}>;
  issues: Issue[];
  makespan: number | null;
  geo: unknown;
  /** the map it was planned on vs now (API.md §17.7); null: not an editable Neo4j map */
  map?: MapStatus | null;
}

export interface MapStatus {
  planned_revision: number | null;
  revision: number;
  planned_map_version: number | null;
  map_version: number;
  edits_since: number;
  stale: boolean;
  /** planned before map versions were recorded: possibly stale */
  unknown?: boolean;
  message: string | null;
  snapshot: string | null;
}

export interface Candidate {
  id: string;
  label: string;
  query: string;
  pos: number[];
  image: string;
  task: string;
  agent: string;
  bbox: number[];
  status: "pending" | "approved" | "rejected";
  sym: string | null;
  t: number;
  manual?: boolean;
}

export interface RunState {
  mission: string;
  kind: "sim" | "live";
  revision: number;
  t: number;
  finished: boolean;
  playing: boolean;
  speed: number;
  agents: Record<string, {pos: number[]; yaw: number; task: string | null; status: string}>;
  tasks: Record<string, {state: string; times: [number, number | null] | null; note: string | null}>;
  candidates: Candidate[];
  events: {t: number; kind: string; [k: string]: unknown}[];
  images: number;
}

export interface MissionSummary {
  id: string;
  name: string;
  text: string;
  agents: number;
  tasks: number;
  created_at: number;
  revision: number;
  planner: string;
  run: string | null;
  saved?: boolean;
  decisions?: number;
  has_chat?: boolean;
  /** quick check: missing targets + agents without a start */
  errors?: number;
}

/** What the next map click does ("retarget": a node or point replaces a task's target). */
export type MapTool = {kind: "none"} | {kind: "start"; agent: string} | {kind: "area"} | {kind: "plant"; label: string} | {kind: "retarget"; task: string};

/** A mission-only search area (browser + the chat context; "Save to map" makes it a Room). */
export interface Area {
  name: string;
  polygon: number[][];
}

export type AreaPurpose = "search" | "no_go";
export type AreaScope = "mission" | "map";

const ROSTER_KEY = "adt4.mission.roster.v1";

function loadLocal(): {roster: Agent[]; areas: Area[]; planted: Planted[]; fences: Polygon[]} {
  try {
    const v = JSON.parse(window.localStorage.getItem(ROSTER_KEY) || "null");
    if (v && Array.isArray(v.roster)) return {roster: v.roster, areas: v.areas ?? [], planted: v.planted ?? [], fences: v.fences ?? []};
  } catch {
    /* private window / junk */
  }
  return {roster: [], areas: [], planted: [], fences: []};
}

interface MissionStore {
  // map + satellite
  geo: GeoInfo | null;
  geoSrc: string | null;
  layers: TileLayer[];
  satellite: boolean;
  satOpacity: number;
  world: {src: string | null; bounds: number[] | null; start_hint: number[] | null} | null;
  // operator inputs
  roster: Agent[];
  areas: Area[];
  fences: Polygon[];
  planted: Planted[];
  tool: MapTool;
  draft: number[][];
  /** a finished drawing waiting for its name / purpose / scope (the area popover) */
  pendingArea: number[][] | null;
  /** send the mission context with chat turns (propose_mission) */
  chatOn: boolean;
  // missions
  list: MissionSummary[];
  current: Mission | null;
  preview: Preview | null;
  run: RunState | null;
  error: string | null;
  busy: string | null;
  /** the image open in the detection viewer */
  openImage: {mission: string; image: string; candidate?: string} | null;
  /** an agent glyph being dragged on the map (its new start) */
  drag: StartDrag | null;
  /** a dragged start snaps to the nearest place node (Shift inverts it while dragging) */
  snapStarts: boolean;
  /** start moves, newest last (Undo puts the last one back) */
  startUndo: StartMove[];
  set: (patch: Partial<MissionStore>) => void;
}

export const useMission = create<MissionStore>(set => ({
  geo: null,
  geoSrc: null,
  layers: [],
  satellite: true,
  satOpacity: 1,
  world: null,
  ...(() => {
    const l = loadLocal();
    return {roster: l.roster, areas: l.areas, fences: l.fences, planted: l.planted};
  })(),
  tool: {kind: "none"},
  draft: [],
  pendingArea: null,
  chatOn: true,
  list: [],
  current: null,
  preview: null,
  run: null,
  error: null,
  busy: null,
  openImage: null,
  drag: null,
  snapStarts: false,
  startUndo: [],
  set: patch => set(patch as Partial<MissionStore>),
}));

// e2e scripts read the mission state from here (like window.__adt4MapEdit)
if (typeof window !== "undefined") (window as unknown as {__adt4Mission?: unknown}).__adt4Mission = useMission;

useMission.subscribe((s, p) => {
  if (s.roster === p.roster && s.areas === p.areas && s.planted === p.planted && s.fences === p.fences) return;
  try {
    window.localStorage.setItem(ROSTER_KEY, JSON.stringify({roster: s.roster, areas: s.areas, planted: s.planted, fences: s.fences}));
  } catch {
    /* quota / private window */
  }
});

const st = () => useMission.getState();
const enc = encodeURIComponent;

// ------------------------------------------------------------------ geo
export async function loadGeo(src: string | null): Promise<void> {
  try {
    const layers = await getJSON<{layers: TileLayer[]}>("api/geo/layers");
    st().set({layers: layers.layers});
    const world = await getJSON<{src: string | null; bounds: number[] | null; start_hint: number[] | null}>(`api/missions/world${src ? `?src=${enc(src)}` : ""}`).catch(() => null);
    st().set({world});
    const s = src ?? world?.src ?? null;
    if (!s) return;
    const b = world?.bounds;
    const cx = b ? (b[0] + b[2]) / 2 : 0;
    const cy = b ? (b[1] + b[3]) / 2 : 0;
    const r = await getJSON<{anchor: GeoInfo["anchor"] | null; affine?: GeoInfo["affine"]}>(`api/geo/anchor/${enc(s)}?cx=${cx}&cy=${cy}`);
    st().set({geo: r.anchor && r.affine ? {anchor: r.anchor, affine: r.affine} : null, geoSrc: s});
  } catch (e) {
    st().set({error: `geo: ${errorText(e)}`});
  }
}

// ------------------------------------------------------------- missions
/** The chat's mission context: mission-only areas and no-go zones (the server
 *  adds the map's areas to the same list, API.md §15.4). */
export function missionContext() {
  const s = st();
  return {
    roster: s.roster.map(a => ({id: a.id, kind: a.kind, start: a.start ?? null, binding: a.binding ?? null})),
    areas: s.areas.map(a => ({name: a.name, polygon: a.polygon, scope: "mission" as const})),
    no_go: s.fences.map(f => ({name: f.id, polygon: f.polygon})),
    planted: s.planted,
    mission: s.current,
  };
}

export async function refreshList(): Promise<void> {
  try {
    const r = await getJSON<{missions: MissionSummary[]}>("api/missions");
    st().set({list: r.missions});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

let unsub: (() => void) | null = null;

function follow(id: string | null) {
  unsub?.();
  unsub = null;
  if (!id) return;
  unsub = ws.on(`mission/${id}`, raw => {
    const data = raw as RunState;
    if (st().current?.id === data.mission) st().set({run: data});
  });
}

/** Open a mission in the Mission tab (``fit``: frame it on the map, as when the
 *  operator picks one; a re-preview after a map edit leaves the camera alone). */
export async function openMission(id: string, opts: {fit?: boolean} = {}): Promise<void> {
  try {
    const r = await getJSON<{mission: Mission; preview: Preview; run: RunState | null}>(`api/missions/${enc(id)}`);
    const again = st().current?.id === id;
    st().set({current: r.mission, preview: r.preview, run: r.run, error: null, roster: rosterWithMission(st().roster, r.mission), ...(again ? {} : {startUndo: []})});
    follow(id);
    if (opts.fit ?? !again) fitMission();
    if (!st().list.some(m => m.id === id)) void refreshList();
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

/** Accept the current map for the open mission (after re-previewing it; API.md §17.7). */
export async function rebaseMission(id: string): Promise<void> {
  try {
    const r = await postJSON<{mission: Mission; preview: Preview}>(`api/missions/${enc(id)}/rebase`);
    st().set({current: r.mission, preview: r.preview, error: null});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

export function closeMission() {
  follow(null);
  st().set({current: null, preview: null, run: null});
}

/** Store a mission (new, or a revision of the current one) and open it. */
export async function saveMission(m: Partial<Mission>, revise = false): Promise<Mission | null> {
  st().set({busy: "save", error: null});
  try {
    const r = revise && m.id ? await putJSON<{mission: Mission; preview: Preview}>(`api/missions/${enc(m.id)}`, m) : await postJSON<{mission: Mission; preview: Preview}>("api/missions", m);
    const fresh = st().current?.id !== r.mission.id;
    st().set({current: r.mission, preview: r.preview, run: null});
    follow(r.mission.id);
    if (fresh) fitMission();
    void refreshList();
    return r.mission;
  } catch (e) {
    st().set({error: errorText(e)});
    return null;
  } finally {
    st().set({busy: null});
  }
}

/** Keep the current mission: a name, the chat that made it and the approvals
 *  of its current simulated run (for an exact replay later). */
export async function saveCurrent(name: string, chat: {items: unknown[]; transcript: unknown[]} | null): Promise<boolean> {
  const m = st().current;
  if (!m) return false;
  try {
    const r = await postJSON<{mission: Mission; decisions: number}>(`api/missions/${enc(m.id)}/save`, {name, chat});
    st().set({current: r.mission});
    void refreshList();
    return true;
  } catch (e) {
    st().set({error: errorText(e)});
    return false;
  }
}

export async function deleteMission(id: string) {
  await deleteJSON(`api/missions/${enc(id)}`).catch(e => st().set({error: errorText(e)}));
  if (st().current?.id === id) closeMission();
  void refreshList();
}

/** The roster's operator inputs folded into the current mission (agents keep
 *  the mission's ids; starts/bindings from the roster win). */
export function withOperatorInputs(m: Mission): Mission {
  return foldInputs(m, st());
}

type Inputs = Pick<MissionStore, "roster" | "planted" | "fences">;

function foldInputs(m: Mission, s: Inputs): Mission {
  const byId = new Map(s.roster.map(a => [a.id, a]));
  return {
    ...m,
    agents: m.agents.map(a => {
      const r = byId.get(a.id);
      if (!r) return a;
      // a bound roster agent without a start means "start where the robot is"
      const start = r.binding && r.start == null ? null : (r.start ?? a.start);
      return {...a, start, binding: r.binding ?? a.binding, kind: r.kind ?? a.kind};
    }),
    planted: s.planted,
    geofences: s.fences.length ? s.fences : m.geofences,
  };
}

const same = (a: unknown, b: unknown) => JSON.stringify(a ?? null) === JSON.stringify(b ?? null);
const plantKey = (p: Planted[]) => p.map(x => [x.label, x.pos[0], x.pos[1], x.size_m ?? 1.5]);
const fenceKey = (f: Polygon[]) => f.map(x => [x.id, x.polygon]);

/** What "Update mission from roster" would change in `m`, one short phrase
 *  each ("spot1 robot", "mock objects"); [] when the mission already matches. */
export function rosterDiff(m: Mission, s: Inputs = st()): string[] {
  const next = foldInputs(m, s);
  const out: string[] = [];
  m.agents.forEach((a, i) => {
    const b = next.agents[i];
    const what = [!same(a.start, b.start) && "start", !same(a.binding, b.binding) && "robot", a.kind !== b.kind && "kind"].filter(Boolean);
    if (what.length) out.push(`${a.id} ${what.join("/")}`);
  });
  if (!same(plantKey(m.planted ?? []), plantKey(next.planted))) out.push("mock objects");
  if (!same(fenceKey(m.geofences ?? []), fenceKey(next.geofences))) out.push("no-go zones");
  return out;
}

export async function simCmd(cmd: string, value?: number | boolean, autoApprove?: boolean): Promise<void> {
  const m = st().current;
  if (!m) return;
  try {
    const r = await postJSON<RunState>(`api/missions/${enc(m.id)}/sim`, {cmd, value: value ?? null, auto_approve: autoApprove ?? null});
    st().set({run: r, error: null});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

export async function decideCandidate(cid: string, approve: boolean, commit = false): Promise<void> {
  const m = st().current;
  if (!m) return;
  st().set({busy: `cand:${cid}`});
  try {
    await postJSON(`api/missions/${enc(m.id)}/candidates/${enc(cid)}`, {approve, commit});
  } catch (e) {
    st().set({error: errorText(e)});
  } finally {
    st().set({busy: null});
  }
}

export async function annotate(image: string, bbox: number[], label: string): Promise<void> {
  const m = st().current;
  if (!m) return;
  try {
    await postJSON(`api/missions/${enc(m.id)}/annotate`, {image, bbox, label});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

export async function execute(): Promise<void> {
  const m = st().current;
  if (!m) return;
  try {
    const r = await postJSON<RunState>(`api/missions/${enc(m.id)}/execute`, {});
    st().set({run: r, error: null});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

export async function abort(): Promise<void> {
  const m = st().current;
  if (!m) return;
  try {
    const r = await postJSON<RunState>(`api/missions/${enc(m.id)}/abort`, {});
    st().set({run: r});
  } catch (e) {
    st().set({error: errorText(e)});
  }
}

export async function planWithOmniplanner(text: string): Promise<void> {
  const s = st();
  st().set({busy: "omni", error: null});
  try {
    const r = await postJSON<{mission: Mission; preview: Preview}>("api/missions/plan/omniplanner", {text, agents: s.roster, src: s.world?.src ?? null});
    st().set({current: r.mission, preview: r.preview, run: null});
    follow(r.mission.id);
    fitMission();
    void refreshList();
  } catch (e) {
    st().set({error: `omniplanner: ${errorText(e)}`});
  } finally {
    st().set({busy: null});
  }
}

// --------------------------------------------------------------- map tools
const AGENT_PREFIX: Record<AgentKind, string> = {spot: "spot", husky: "husky", quad: "quad"};

export function addAgent(kind: AgentKind, binding: string | null = null, start: number[] | null = null) {
  const s = st();
  if (s.roster.length >= MAX_AGENTS) return;
  let id = binding ?? "";
  if (!id) {
    let n = 1;
    while (s.roster.some(a => a.id === `${AGENT_PREFIX[kind]}${n}`)) n++;
    id = `${AGENT_PREFIX[kind]}${n}`;
  }
  const hint = s.world?.start_hint;
  const st0 = start ?? (hint ? freeStart(hint, s.roster) : null);
  s.set({roster: [...s.roster, {id, kind, binding, start: st0}]});
}

/** New agents start this far apart (deconfliction warns under 2 m). */
export const START_SPACING_M = 5;
export const MIN_START_GAP_M = 4;

/** The first spot in a row east of `hint` (START_SPACING_M steps) at least
 *  MIN_START_GAP_M from every other agent's start. */
export function freeStart(hint: number[], roster: Agent[]): number[] {
  const taken = roster.map(a => a.start).filter((p): p is number[] => !!p && p.length >= 2);
  for (let k = 0; ; k++) {
    const p = [hint[0] + k * START_SPACING_M, hint[1]];
    if (taken.every(q => Math.hypot(p[0] - q[0], p[1] - q[1]) >= MIN_START_GAP_M)) return p;
  }
}

export function updateAgent(id: string, patch: Partial<Agent>) {
  st().set({roster: st().roster.map(a => (a.id === id ? {...a, ...patch} : a))});
}

export function removeAgent(id: string) {
  st().set({roster: st().roster.filter(a => a.id !== id)});
}

/** A map click while a mission tool is armed (global/map frame); `sym`: the
 *  planner-map node that was hit, if any (a retarget takes the symbol). */
export function onMapClick(p: number[], sym: string | null = null): boolean {
  const s = st();
  const t = s.tool;
  const xy = [Math.round(p[0] * 100) / 100, Math.round(p[1] * 100) / 100];
  switch (t.kind) {
    case "start":
      s.set({tool: {kind: "none"}});
      void moveStart(t.agent, xy);
      return true;
    case "plant":
      s.set({planted: [...s.planted, {label: t.label, pos: xy, size_m: 1.5}]});
      return true;
    case "area":
      s.set({draft: [...s.draft, xy]});
      return true;
    case "retarget":
      s.set({tool: {kind: "none"}});
      void retargetTask(t.task, sym ?? xy);
      return true;
    default:
      return false;
  }
}

// ------------------------------------------------------------ agent starts
/** One start move, for Undo: what the roster and the open mission had before. */
export interface StartMove {
  agent: string;
  /** when (ms): Ctrl+Z undoes the newer of this and the last map edit */
  at?: number;
  mission: string | null;
  roster?: number[] | null;
  missionStart?: number[] | null;
}

/** The roster after opening a mission: its agents (ids, kinds, starts, bindings)
 *  replace the roster's ones of the same id or are added, so the roster shows the
 *  mission being looked at; other roster agents stay while there is room. */
export function rosterWithMission(roster: Agent[], m: Mission): Agent[] {
  const fromMission = m.agents.map(a => {
    const r = roster.find(x => x.id === a.id);
    return {...(r ?? {}), id: a.id, kind: a.kind, start: a.start ?? null, binding: a.binding ?? null, ...(a.speed != null ? {speed: a.speed} : {}), ...(a.kind === "quad" && a.altitude != null ? {altitude: a.altitude} : {})} as Agent;
  });
  const ids = new Set(fromMission.map(a => a.id));
  const rest = roster.filter(a => !ids.has(a.id) && !fromMission.some(x => x.binding && x.binding === a.binding));
  return [...fromMission, ...rest].slice(0, MAX_AGENTS);
}

/** The start an agent has now: the open mission's (what is previewed / simulated),
 *  else the roster's. ``undefined``: no such agent. */
export function startOf(id: string): number[] | null | undefined {
  const s = st();
  const ma = s.current?.agents.find(a => a.id === id);
  if (ma) return ma.start ?? null;
  const ra = s.roster.find(a => a.id === id);
  return ra ? (ra.start ?? null) : undefined;
}

/** The open mission with one agent's start replaced (null: the bound robot's live pose). */
export function withStart(m: Mission, id: string, start: number[] | null): Mission {
  return {...m, agents: m.agents.map(a => (a.id === id ? {...a, start} : a))};
}

/** Move an agent's start (map frame; null = its bound robot's live pose). The
 *  roster takes it, and when the agent is in the open mission the mission does
 *  too, as a new revision: the server re-previews it (and drops a paused or
 *  finished simulation). Refused while a run is in progress (startLockReason).
 *  Returns false when refused or the revision failed. */
export async function moveStart(id: string, start: number[] | null, opts: {undoable?: boolean} = {}): Promise<boolean> {
  const s = st();
  const m = s.current;
  const inMission = !!m?.agents.some(a => a.id === id);
  if (inMission) {
    const why = startLockReason(s.run);
    if (why) {
      s.set({error: why});
      return false;
    }
  }
  const xy = start ? roundXY(start) : null;
  const ra = s.roster.find(a => a.id === id);
  const undo: StartMove = {agent: id, mission: inMission ? m!.id : null, at: Date.now()};
  if (ra) undo.roster = ra.start ?? null;
  if (inMission) undo.missionStart = m!.agents.find(a => a.id === id)!.start ?? null;
  const patch: Partial<MissionStore> = {};
  if (ra) patch.roster = s.roster.map(a => (a.id === id ? {...a, start: xy} : a));
  if (opts.undoable !== false) patch.startUndo = [...s.startUndo, undo].slice(-20);
  // shown at once (the overlay draws the mission's start); the revision follows
  if (inMission) patch.current = withStart(m!, id, xy);
  s.set(patch);
  if (!inMission) return true;
  const saved = await saveMission(withStart(m!, id, xy), true);
  if (!saved) {
    // put the drawn start back (the error is shown by saveMission)
    const c = st().current;
    if (c?.id === m!.id) st().set({current: withStart(c, id, undo.missionStart ?? null)});
    return false;
  }
  return true;
}

/** Put the last start move back. */
export async function undoStart(): Promise<boolean> {
  const s = st();
  const u = s.startUndo[s.startUndo.length - 1];
  if (!u) return false;
  s.set({startUndo: s.startUndo.slice(0, -1)});
  const m = s.current;
  if (u.mission && m?.id === u.mission && m.agents.some(a => a.id === u.agent)) return moveStart(u.agent, u.missionStart ?? null, {undoable: false});
  if (u.roster !== undefined) st().set({roster: st().roster.map(a => (a.id === u.agent ? {...a, start: u.roster ?? null} : a))});
  return true;
}

/** Corners without the repeats a double-click leaves (two clicks on one spot). */
export function dedupeCorners(pts: number[][], eps = 0.3): number[][] {
  const out: number[][] = [];
  for (const p of pts) {
    const q = out[out.length - 1];
    if (!q || Math.hypot(p[0] - q[0], p[1] - q[1]) > eps) out.push(p);
  }
  while (out.length > 1 && Math.hypot(out[0][0] - out[out.length - 1][0], out[0][1] - out[out.length - 1][1]) <= eps) out.pop();
  return out;
}

/** Close the drawing: with 3+ corners it waits in the area popover. */
export function finishDraft(): boolean {
  const s = st();
  const pts = dedupeCorners(s.draft);
  s.set({draft: [], tool: {kind: "none"}, pendingArea: pts.length >= 3 ? pts : null});
  return pts.length >= 3;
}

/** Area names in use (case-insensitive), mission and map scope. */
export function takenAreaNames(mapNames: readonly (string | null | undefined)[] = []): Set<string> {
  const s = st();
  return new Set([...s.areas.map(a => a.name), ...s.fences.map(f => f.id), ...mapNames].filter((n): n is string => !!n).map(n => n.trim().toLowerCase()));
}

/** The first free default name: A1, A2, ... ("No-go 1", ... for no-go zones). */
export function nextAreaName(purpose: AreaPurpose, taken: Set<string>): string {
  const prefix = purpose === "no_go" ? "No-go " : "A";
  for (let n = 1; ; n++) if (!taken.has(`${prefix}${n}`.toLowerCase())) return `${prefix}${n}`;
}

/** Keep a finished drawing as a mission-only area (map scope: stores/tools.ts saveArea). */
export function addMissionArea(name: string, polygon: number[][], purpose: AreaPurpose) {
  const s = st();
  if (purpose === "no_go") s.set({fences: [...s.fences, {id: name, polygon, kind: "no_go"}], pendingArea: null});
  else s.set({areas: [...s.areas, {name, polygon}], pendingArea: null});
}

export function removeMissionArea(name: string, purpose: AreaPurpose) {
  const s = st();
  if (purpose === "no_go") s.set({fences: s.fences.filter(f => f.id !== name)});
  else s.set({areas: s.areas.filter(a => a.name !== name)});
}

// ------------------------------------------------------ fixing a mission
/** A revision of the open mission with one task's target replaced. */
export function withTarget(m: Mission, taskId: string, target: string | number[]): Mission {
  return {...m, tasks: m.tasks.map(t => (t.id === taskId ? {...t, target} : t))};
}

/** A revision of the open mission without one task (nothing depends on it any more). */
export function withoutTask(m: Mission, taskId: string): Mission {
  return {...m, tasks: m.tasks.filter(t => t.id !== taskId).map(t => (t.depends_on?.includes(taskId) ? {...t, depends_on: t.depends_on.filter(d => d !== taskId)} : t))};
}

export async function retargetTask(taskId: string, target: string | number[]) {
  const m = st().current;
  if (m) await saveMission(withTarget(m, taskId, target), true);
}

export async function dropTask(taskId: string) {
  const m = st().current;
  if (m) await saveMission(withoutTask(m, taskId), true);
}

/** What "Re-plan in chat" puts in the chat box: the mission's command plus the problems. */
export function replanText(m: Mission, problems: string[]): string {
  const what = m.text.trim() || m.name || m.id;
  return `${what}\n\nRe-plan this mission: ${problems.join("; ")}`;
}

// ------------------------------------------------------------ fit mission
/** Map points of the open mission: agent starts and poses, planned paths and
 *  targets, search areas and no-go zones. */
export function missionPoints(m: Mission | null, pv: Preview | null, run: RunState | null): number[][] {
  if (!m) return [];
  const out: number[][] = [];
  const add = (p: number[] | null | undefined) => {
    if (p && p.length >= 2 && Number.isFinite(p[0]) && Number.isFinite(p[1])) out.push([p[0], p[1]]);
  };
  for (const a of m.agents) add(pv?.agents[a.id]?.start ?? a.start ?? null);
  for (const tp of Object.values(pv?.tasks ?? {})) {
    tp.path.forEach(add);
    add(tp.target);
  }
  for (const t of m.tasks) (t.area ?? []).forEach(add);
  for (const g of m.geofences ?? []) g.polygon.forEach(add);
  for (const a of Object.values(run?.agents ?? {})) add(a.pos);
  return out;
}

/** Frame the open mission on the map (the "⌖ Fit mission" button, F, opening a
 *  mission, Simulate), clear of the overlays covering the map's edges. */
export function fitMission(): boolean {
  const s = st();
  const pts = missionPoints(s.current, s.preview, s.run);
  if (!pts.length) return false;
  const canvas = typeof document !== "undefined" ? document.querySelector(".vcanvas") : null;
  const sheets = typeof document !== "undefined" ? [...document.querySelectorAll(".vsheet")] : [];
  const insets = canvas ? overlayInsets(canvas, OVERLAY_SELECTOR, 16, sheets) : {top: 16, right: 16, bottom: 16, left: 16};
  useViewer.getState().fit(pts, insets);
  return true;
}
