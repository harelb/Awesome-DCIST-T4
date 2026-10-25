// Mission planner P1 (UX review): the live-edit cue and labelled undos, chat
// card ↔ Mission tab, Fit mission, "Update mission from roster", start spacing,
// chat lookups folded per turn, detections to review, the /mission tab strip,
// decluttered labels + mission layer defaults, shortcuts, mission names.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {addAgent, freeStart, missionPoints, rosterDiff, useMission, type Agent, type Mission, type Preview, type RunState, type StartMove} from "../src/stores/mission";
import {lastUndoable, undoLabel, useMapEdit, type EditUndo} from "../src/stores/mapEdit";
import {undoTarget} from "../src/stores/tools";
import {useViewer} from "../src/stores/viewer";
import {parseStoredStyle, useViewStyle} from "../src/stores/viewStyle";
import {enterMissionView, leaveMissionView, MISSION_HIDDEN_DEFAULT, setLayerHidden} from "../src/stores/missionView";
import {fitTopView, overlayInsets} from "../src/lib/fitView";
import {declutter, labelPrio} from "../src/lib/labels";
import {layerHidden} from "../src/lib/graphModel";
import {cypherErrorShort, lookupSummary, toolGroups} from "../src/lib/chatTranscript";
import {cardRunState, MissionCard, type MissionCardItem} from "../src/viewer/ChatPanel";
import {DetectionToast, fmtWhen, inMap} from "../src/viewer/MissionPanel";
import {LiveEditStrip} from "../src/viewer/MapEditPanel";
import {splitTabs} from "../src/viewer/ViewerPage";
import {runShortcut, shortcutOf} from "../src/viewer/shortcuts";
import type {HistoryItem} from "../src/lib/mapEdit";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

let root: Root | null = null;
let host: HTMLDivElement | null = null;
function render(el: React.ReactElement) {
  host = document.createElement("div");
  document.body.appendChild(host);
  root = createRoot(host);
  act(() => root!.render(el));
  return host;
}
afterEach(() => {
  act(() => root?.unmount());
  host?.remove();
  root = null;
  host = null;
  vi.restoreAllMocks();
});
const button = (h: HTMLElement, text: string) => [...h.querySelectorAll("button")].find(b => b.textContent?.includes(text));

const mission = (over: Partial<Mission> = {}): Mission => ({
  id: "m1",
  name: "crate",
  text: "find the crate",
  src: "neo4j:penn",
  planner: "llm",
  agents: [
    {id: "spot1", kind: "spot", start: [0, 0], binding: null},
    {id: "quad1", kind: "quad", start: [5, 0], binding: null},
  ],
  tasks: [{id: "t1", agent: "quad1", type: "explore_area", area: [[10, 10], [20, 10], [20, 20]]}],
  geofences: [],
  planted: [],
  created_at: 0,
  revision: 1,
  ...over,
});

const run = (over: Partial<RunState> = {}): RunState => ({
  mission: "m1",
  kind: "sim",
  revision: 1,
  t: 3,
  finished: false,
  playing: false,
  speed: 1,
  agents: {},
  tasks: {},
  candidates: [],
  events: [],
  images: 0,
  ...over,
});

beforeEach(() => {
  useMission.setState({roster: [], areas: [], fences: [], planted: [], current: null, preview: null, run: null, list: [], startUndo: [], world: {src: "neo4j:x", bounds: [0, 0, 10, 10], start_hint: [5, 5]}});
  useMapEdit.setState({on: false, state: null, selection: [], editUndo: [], history: [], busy: false, toast: null});
});

describe("1 · live map editing is unmistakable", () => {
  it("the strip says where an edit goes", () => {
    useMapEdit.setState({on: true, state: {mode: "immediate", staged: []} as never});
    let h = render(<LiveEditStrip />);
    expect(h.querySelector("[data-testid=live-edit-strip]")?.className).toContain("live");
    expect(h.textContent).toContain("Live map editing — changes write to Neo4j");
    act(() => root?.unmount());
    useMapEdit.setState({state: {mode: "staged", staged: [{}, {}], live: false} as never});
    h = render(<LiveEditStrip />);
    expect(h.textContent).toContain("Staged — publish to send (2 waiting)");
    act(() => useMapEdit.setState({on: false}));
    expect(h.querySelector("[data-testid=live-edit-strip]")).toBeNull();
  });
  it("undo buttons say what they undo", () => {
    expect(undoLabel("remove O(18)")).toBe("Undo: remove O(18)");
    expect(undoLabel("remove O5 (chair) (retired, can be undone)")).toBe("Undo: remove O5 (chair)");
    expect(undoLabel("Staged (2 pending): set class")).toBe("Undo: set class");
    expect(undoLabel("x".repeat(80), 20)).toHaveLength(26);
    expect(undoLabel("")).toBe("Undo");
    const h = (id: string, more: Partial<HistoryItem>): HistoryItem => ({mutation_id: id, revision: 1, kind: "map_edit", committed_at: null, summary: id, provenance: {}, undo_of: null, undone_by: null, undoable: true, n_ops: 1, affected: [], map_version: null, ...more});
    expect(lastUndoable([h("u", {undo_of: "a"}), h("a", {undone_by: "u"}), h("b", {})])?.mutation_id).toBe("b");
  });
});

describe("2 · chat card ↔ Mission tab", () => {
  const card: MissionCardItem = {type: "mission", id: "m1", revision: 1, name: "crate", agents: [], tasks: [], issues: [], assumptions: [], makespan: 30};
  it("a superseded card offers the latest revision instead of Simulate", () => {
    useMission.setState({list: [{id: "m1", name: "crate", text: "", agents: 1, tasks: 1, created_at: 0, revision: 3, planner: "llm", run: null}]});
    const h = render(<MissionCard it={card} />);
    expect(button(h, "Superseded by r3 — open latest")).toBeTruthy();
    expect(button(h, "Simulate")).toBeUndefined();
  });
  it("Simulate is off (and says why) while a run plays", () => {
    useMission.setState({current: mission(), run: run({playing: true})});
    const h = render(<MissionCard it={card} />);
    const sim = button(h, "Simulating");
    expect(sim?.disabled).toBe(true);
    expect(button(h, "Show in Mission tab")).toBeTruthy();
    expect(cardRunState(run({kind: "live"}))).toBe("Running live");
    expect(cardRunState(run({playing: true, finished: true}))).toBeNull();
    expect(cardRunState(null)).toBeNull();
  });
});

describe("3 · fit mission", () => {
  const ins = {top: 0, right: 0, bottom: 0, left: 0};
  it("frames the points, centred in the free part of the canvas", () => {
    const pts = [[0, 0], [100, 0], [100, 50], [0, 50]];
    const f = fitTopView(pts, [1, 0], [0, 1], 1000, 500, Math.PI / 2, ins)!;
    expect(f.target[0]).toBeCloseTo(50);
    expect(f.target[1]).toBeCloseTo(25);
    // a 124 m wide box (12 % margins) over 1000 px; height = mpp * h / (2 tan 45°)
    expect(f.mpp).toBeCloseTo(0.124);
    expect(f.height).toBeCloseTo((0.124 * 500) / 2);
    // a toolbar over the top 100 px: the box drops into the rest (target moves up / north)
    const g = fitTopView(pts, [1, 0], [0, 1], 1000, 500, Math.PI / 2, {...ins, top: 100})!;
    expect(g.target[1]).toBeGreaterThan(25);
    expect(g.mpp).toBeGreaterThanOrEqual(f.mpp);
  });
  it("respects a rotated top view", () => {
    // screen right = map north: the long (x) side runs up the screen
    const f = fitTopView([[0, 0], [100, 0]], [0, 1], [-1, 0], 1000, 1000, Math.PI / 2, {...ins, left: 200})!;
    expect(f.target[0]).toBeCloseTo(50);
    // a 200 px panel on the left: the view centre moves 100 px to the left of
    // the box centre (screen left = map south here) so the box sits right of it
    expect(f.target[1]).toBeCloseTo(-100 * f.mpp);
    expect(fitTopView([], [1, 0], [0, 1], 10, 10, 1, ins)).toBeNull();
  });
  it("fits a phone's thin strip between the toolbars and a half-height sheet", () => {
    // 400×800 px canvas: toolbars + a notice cover the top 300 px, the sheet the bottom 400
    const pts = [[0, 0], [100, 0], [100, 50], [0, 50]];
    const f = fitTopView(pts, [1, 0], [0, 1], 400, 800, Math.PI / 2, {top: 300, right: 16, bottom: 400, left: 16})!;
    // 62 m tall box (12 % margins) in 100 free px
    expect(f.mpp).toBeCloseTo(0.62);
    // the box centre (y = 25) sits at screen y 350, 50 px above the canvas centre
    expect(f.target[1]).toBeCloseTo(25 - 50 * 0.62);
  });
  it("collects agents, paths, areas and run poses; overlays become insets", () => {
    const pv: Preview = {tasks: {t1: {path: [[1, 1], [2, 2]], target: [3, 3]} as never}, agents: {spot1: {start: [9, 9], end_s: 0, distance_m: 0}}, issues: [], makespan: 1, geo: null};
    const pts = missionPoints(mission(), pv, run({agents: {quad1: {pos: [7, 7], yaw: 0, task: null, status: "idle"}}}));
    expect(pts).toEqual(expect.arrayContaining([[9, 9], [5, 0], [1, 1], [2, 2], [3, 3], [10, 10], [20, 20], [7, 7]]));
    expect(missionPoints(null, null, null)).toEqual([]);
    const canvas = document.createElement("div");
    const bar = document.createElement("div");
    bar.className = "vtoolbar";
    canvas.appendChild(bar);
    vi.spyOn(canvas, "getBoundingClientRect").mockReturnValue({top: 0, bottom: 600, left: 0, right: 800, width: 800, height: 600} as DOMRect);
    vi.spyOn(bar, "getBoundingClientRect").mockReturnValue({top: 10, bottom: 46, left: 10, right: 400, width: 390, height: 36} as DOMRect);
    expect(overlayInsets(canvas, ".vtoolbar")).toEqual({top: 62, right: 16, bottom: 16, left: 16});
    // a phone sheet over the lower 60 %: the status line just above it (upper
    // half of the canvas, lower half of the visible map) is a bottom overlay
    const status = document.createElement("div");
    status.className = "vstatus";
    canvas.appendChild(status);
    const sheet = document.createElement("div");
    vi.spyOn(status, "getBoundingClientRect").mockReturnValue({top: 210, bottom: 232, left: 10, right: 300, width: 290, height: 22} as DOMRect);
    vi.spyOn(sheet, "getBoundingClientRect").mockReturnValue({top: 240, bottom: 600, left: 0, right: 800, width: 800, height: 360} as DOMRect);
    expect(overlayInsets(canvas, ".vtoolbar, .vstatus", 16, [sheet])).toEqual({top: 62, right: 16, bottom: 406, left: 16});
  });
});

describe("4 · update mission from roster", () => {
  it("lists what differs, nothing when it matches", () => {
    const m = mission({planted: [{label: "crate", pos: [1, 1], size_m: 1.5}]});
    const roster: Agent[] = [{id: "spot1", kind: "spot", start: [0, 0], binding: null}, {id: "quad1", kind: "quad", start: [5, 0], binding: null}];
    expect(rosterDiff(m, {roster, planted: m.planted, fences: []})).toEqual([]);
    const moved = [{...roster[0], binding: "euclid"}, {...roster[1], start: [6, 0]}];
    expect(rosterDiff(m, {roster: moved, planted: [], fences: [{id: "No-go 1", polygon: [[0, 0], [1, 0], [1, 1]]}]})).toEqual(["spot1 robot", "quad1 start", "mock objects", "no-go zones"]);
  });
});

describe("5 · new agents start apart", () => {
  it("at least 4 m from every other start, also after a removal", () => {
    expect(freeStart([0, 0], [])).toEqual([0, 0]);
    expect(freeStart([0, 0], [{id: "a", kind: "spot", start: [0, 0]}, {id: "b", kind: "spot", start: [8, 1]}])).toEqual([15, 0]);
    addAgent("spot");
    addAgent("spot");
    addAgent("spot");
    useMission.setState({roster: useMission.getState().roster.filter(a => a.id !== "spot2")});
    addAgent("spot");
    const starts = useMission.getState().roster.map(a => a.start!);
    for (let i = 0; i < starts.length; i++)
      for (let j = i + 1; j < starts.length; j++) expect(Math.hypot(starts[i][0] - starts[j][0], starts[i][1] - starts[j][1])).toBeGreaterThanOrEqual(4);
  });
});

describe("6 · chat lookups fold into one disclosure per turn", () => {
  const tool = (name: string, error?: string) => ({type: "tool", name, result: error ? {error} : {rows: []}});
  it("groups a turn's lookups at the first one", () => {
    const items = [{type: "user"}, tool("run_cypher", "bad"), {type: "text"}, tool("run_cypher"), tool("search_robot_images"), {type: "user"}, tool("find_objects")];
    const g = toolGroups(items);
    expect(g.get(1)).toEqual([1, 3]);
    expect(g.get(3)).toBeNull();
    expect(g.has(4)).toBe(false); // a SAM3 search stays visible
    expect(g.get(6)).toEqual([6]);
  });
  it("summarises queries, retries and failures", () => {
    expect(lookupSummary([tool("run_cypher", "x"), tool("run_cypher", "y"), tool("run_cypher")]).text).toBe("3 queries, 2 retried");
    expect(lookupSummary([tool("run_cypher"), tool("run_cypher", "x")]).text).toBe("2 queries, 1 failed");
    expect(lookupSummary([tool("find_objects")]).text).toBe("1 search");
  });
  it("shortens a Neo4j error wall to one line", () => {
    const wall = "{code: Neo.ClientError.Statement.SyntaxError} {message: Invalid input 'RETRUN': expected 'RETURN' (line 1, column 12 (offset: 11))\n\"MATCH (n) RETRUN n\"\n           ^}";
    expect(cypherErrorShort(wall)).toBe("Syntax error: Invalid input 'RETRUN': expected 'RETURN' (line 1, column 12 (offset: 11))");
    expect(cypherErrorShort("x".repeat(300))).toHaveLength(140);
  });
});

describe("7 · detections to review", () => {
  const cand = (id: string, status: "pending" | "approved" | "rejected", sym: string | null = null) => ({id, label: "crate", query: "crate", pos: [0, 0], image: "i", task: "t1", agent: "quad1", bbox: [0, 0, 1, 1], status, sym, t: 1});
  it("a sticky notice until reviewed; a new detection brings it back", () => {
    useMission.setState({current: mission(), run: run({candidates: [cand("c1", "pending")]})});
    const review = vi.fn();
    const h = render(<DetectionToast onReview={review} />);
    expect(h.textContent).toContain("quad1 found “crate” — review");
    act(() => button(h, "Review")!.click());
    expect(review).toHaveBeenCalled();
    expect(h.querySelector("[data-testid=detect-toast]")).toBeNull();
    act(() => useMission.setState({run: run({candidates: [cand("c1", "pending"), cand("c2", "pending")]})}));
    expect(h.textContent).toContain("found “crate”");
  });
  it("an accepted detection can still be added to the map", () => {
    expect(inMap(cand("c", "approved", "sim:crate1"))).toBe(false);
    expect(inMap(cand("c", "approved", "O(12)"))).toBe(true);
    expect(inMap(cand("c", "rejected", "O(12)"))).toBe(false);
  });
});

describe("8 · /mission side-panel tabs", () => {
  const strip = ["inspect", "mission", "map", "plan", "teleop", "images", "live", "camera"] as const;
  it("Mission | Map | Inspect, the rest under More (desktop only)", () => {
    expect(splitTabs([...strip], true, false)).toEqual({shown: ["mission", "map", "inspect"], more: ["plan", "teleop", "images", "live", "camera"]});
    expect(splitTabs([...strip], true, true)).toEqual({shown: [...strip], more: []});
    expect(splitTabs([...strip], false, false).more).toEqual([]);
  });
});

describe("9 · labels and mission layers", () => {
  it("keeps the important labels that do not overlap", () => {
    const boxes = [
      {x: 100, y: 100, w: 60, h: 14, prio: labelPrio("objects", 50, false)},
      {x: 110, y: 104, w: 60, h: 14, prio: labelPrio("objects", 10, false)},
      {x: 300, y: 100, w: 60, h: 14, prio: labelPrio("places", 0, false)},
      {x: 105, y: 100, w: 60, h: 14, prio: labelPrio("places", 500, true)},
    ];
    expect(declutter(boxes, 10)).toEqual([3, 2]);
    expect(declutter(boxes, 1)).toEqual([3]);
  });
  it("places layers are off on /mission; the operator's choice is kept and the viewer's set comes back", () => {
    expect(layerHidden({"kind:places": true}, 3, 1)).toBe(true);
    expect(layerHidden({"kind:places": true, "3/1": false}, 3, 1)).toBe(false);
    expect(layerHidden({"kind:places": true}, 2, 79)).toBe(false);
    useViewStyle.setState({missionHidden: {}});
    useViewer.setState({hiddenLayers: {"2/115": true, "4/0": true}});
    enterMissionView();
    expect(useViewer.getState().hiddenLayers).toEqual(MISSION_HIDDEN_DEFAULT);
    setLayerHidden("3/1", false);
    expect(useViewStyle.getState().missionHidden).toEqual({"3/1": false});
    leaveMissionView();
    expect(useViewer.getState().hiddenLayers).toEqual({"2/115": true, "4/0": true});
    setLayerHidden("4/0", false); // outside /mission: not remembered for it
    expect(useViewStyle.getState().missionHidden).toEqual({"3/1": false});
    enterMissionView();
    expect(layerHidden(useViewer.getState().hiddenLayers, 3, 1)).toBe(false);
    leaveMissionView();
    expect(parseStoredStyle(JSON.stringify({missionHidden: {"3/1": false, bad: 3}})).missionHidden).toEqual({"3/1": false});
  });
});

describe("10 · shortcuts", () => {
  const key = (k: string, over: Partial<KeyboardEvent> = {}) => ({key: k, ctrlKey: false, metaKey: false, altKey: false, shiftKey: false, target: document.body, ...over}) as KeyboardEvent;
  it("maps keys, never while typing", () => {
    expect(shortcutOf(key("z", {ctrlKey: true}))).toBe("undo");
    expect(shortcutOf(key("Z", {metaKey: true}))).toBe("undo");
    expect(shortcutOf(key("z", {ctrlKey: true, shiftKey: true}))).toBeNull();
    expect(shortcutOf(key("Delete"))).toBe("delete");
    expect(shortcutOf(key("f"))).toBe("fit");
    expect(shortcutOf(key(" "))).toBe("play");
    expect(shortcutOf(key(" ", {target: document.createElement("button")}))).toBeNull();
    expect(shortcutOf(key("f", {target: document.createElement("input")}))).toBeNull();
    expect(shortcutOf(key("z", {ctrlKey: true, target: document.createElement("textarea")}))).toBeNull();
  });
  it("Ctrl+Z takes the newer of the last start move and the last map edit", () => {
    const sm = (at?: number): StartMove => ({agent: "spot1", mission: null, at});
    const ed = (at: number): EditUndo => ({mutationId: "u", summary: "x", at});
    expect(undoTarget([], [])).toBeNull();
    expect(undoTarget([sm(5)], [])).toBe("start");
    expect(undoTarget([], [ed(1)])).toBe("edit");
    expect(undoTarget([sm(5)], [ed(9)])).toBe("edit");
    expect(undoTarget([sm(10)], [ed(9)])).toBe("start");
    expect(undoTarget([sm()], [ed(1)])).toBe("edit");
  });
  it("Delete needs an edit selection; F needs a mission; Space no live run", () => {
    expect(runShortcut("delete")).toBe(false);
    expect(runShortcut("fit")).toBe(false);
    expect(runShortcut("play")).toBe(false);
    expect(runShortcut("undo")).toBe(false);
    const fit = vi.spyOn(useViewer.getState(), "fit");
    useMission.setState({current: mission()});
    expect(runShortcut("fit")).toBe(true);
    expect(fit).toHaveBeenCalled();
    useMission.setState({run: run({kind: "live"})});
    expect(runShortcut("play")).toBe(false);
  });
});

describe("11 · mission names", () => {
  it("the picker shows when", () => {
    const now = new Date(2026, 9, 24, 16, 0);
    expect(fmtWhen(new Date(2026, 9, 24, 14, 5).getTime() / 1000, now)).toMatch(/14:05|2:05/);
    expect(fmtWhen(new Date(2026, 9, 23, 14, 5).getTime() / 1000, now)).toMatch(/Oct 23|23 Oct/);
  });
});
