// Relocating a mission agent's start: typed x/y and lat/lon (round trip through
// the GPS anchor), validation, snapping, the drag gesture (commit / tap / cancel),
// run locks, and a moved start reaching the open mission (a revision whose
// preview comes back) plus undo.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {lonLatToMap, mapToLonLat, type GeoInfo} from "../src/lib/geo";
import {dragTo, grab, inputOf, moveResetsRun, nearestPlace, parseCoord, release, startFromInput, startLockReason, startWarning} from "../src/lib/agentStart";
import {moveStart, onMapClick, rosterWithMission, startOf, undoStart, useMission, withOperatorInputs, type Mission, type Preview, type RunState} from "../src/stores/mission";
import {draggableAgents, shownPos} from "../src/viewer/MissionOverlay";
import {StartEditor} from "../src/viewer/StartEditor";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

// same Pennovation-like affine as mission.test.ts
const YAW = -0.5;
const c = Math.cos(YAW);
const s = Math.sin(YAW);
const fwd = [
  [(c * 1) / 85300, (-s * 1) / 85300, -75.1997],
  [(s * 1) / 111000, (c * 1) / 111000, 39.9409],
];
const det = fwd[0][0] * fwd[1][1] - fwd[0][1] * fwd[1][0];
const i2 = [
  [fwd[1][1] / det, -fwd[0][1] / det],
  [-fwd[1][0] / det, fwd[0][0] / det],
];
const inv = [
  [i2[0][0], i2[0][1], -(i2[0][0] * fwd[0][2] + i2[0][1] * fwd[1][2])],
  [i2[1][0], i2[1][1], -(i2[1][0] * fwd[0][2] + i2[1][1] * fwd[1][2])],
];
const GEO: GeoInfo = {anchor: {epsg: 32618, yaw: YAW, t: [482979.48, 4421319.72], z0: 0}, affine: {center: [0, 0], map_to_lonlat: fwd, lonlat_to_map: inv, max_err_m: 0}};

const PREVIEW: Preview = {tasks: {}, agents: {}, issues: [], makespan: 0, geo: null};
const mission = (): Mission =>
  ({
    id: "m1",
    name: "",
    text: "t",
    src: null,
    planner: "llm",
    agents: [
      {id: "spot1", kind: "spot", start: [1, 2]},
      {id: "spot2", kind: "spot", start: null, binding: "euclid"},
    ],
    tasks: [{id: "t1", agent: "spot1", type: "goto", target: [10, 10]}],
    geofences: [],
    planted: [],
    created_at: 0,
    revision: 3,
  }) as Mission;
const run = (patch: Partial<RunState>): RunState => ({mission: "m1", kind: "sim", revision: 3, t: 5, finished: false, playing: false, speed: 1, agents: {}, tasks: {}, candidates: [], events: [], images: 0, ...patch});

/** fetch answering PUT /api/missions/<id> with the body as the next revision (and its preview) */
function mockPut(previewOf: (m: Mission) => Preview = () => PREVIEW) {
  return vi.spyOn(globalThis, "fetch").mockImplementation(async (_url, init) => {
    if ((init as RequestInit | undefined)?.method !== "PUT") return new Response(JSON.stringify({missions: []}), {status: 200, headers: {"Content-Type": "application/json"}});
    const body = JSON.parse(String((init as RequestInit).body)) as Mission;
    const m = {...body, revision: body.revision + 1};
    return new Response(JSON.stringify({mission: m, preview: previewOf(m)}), {status: 200, headers: {"Content-Type": "application/json"}});
  });
}

/** the PUT bodies sent (the mission revisions) */
const puts = (f: ReturnType<typeof mockPut>) => f.mock.calls.filter(c => (c[1] as RequestInit | undefined)?.method === "PUT").map(c => JSON.parse(String((c[1] as RequestInit).body)) as Mission);

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
beforeEach(() => {
  useMission.setState({roster: [], current: null, preview: null, run: null, error: null, busy: null, tool: {kind: "none"}, startUndo: [], drag: null, geo: null, world: {src: null, bounds: [0, 0, 50, 30], start_hint: [5, 5]}});
});
const flush = () => act(async () => void (await new Promise(r => setTimeout(r, 0))));

describe("typed starts", () => {
  it("parses only finite numbers", () => {
    expect(parseCoord(" 12.5 ")).toBe(12.5);
    expect(parseCoord("-3")).toBe(-3);
    expect(parseCoord("4,25")).toBe(4.25); // decimal comma
    expect(parseCoord(".5")).toBe(0.5);
    for (const bad of ["", "abc", "1e999", "NaN", "Infinity", "1.2.3", "5m"]) expect(parseCoord(bad)).toBeNull();
  });
  it("map x/y: 2 decimals; bad fields say which", () => {
    expect(startFromInput({mode: "xy", x: "1.234", y: "-5.678"}, null)).toEqual({start: [1.23, -5.68]});
    expect(startFromInput({mode: "xy", x: "x", y: "1"}, null)).toEqual({error: "x must be a number (metres)"});
    expect(startFromInput({mode: "xy", x: "1", y: ""}, null)).toEqual({error: "y must be a number (metres)"});
  });
  it("lat/lon round trip through the anchor (1 cm)", () => {
    for (const p of [[0, 0], [12.34, -56.78], [-120.5, 33.21]]) {
      const inp = inputOf(p, "lonlat", GEO);
      expect(inp.mode).toBe("lonlat");
      const back = startFromInput(inp, GEO);
      expect("start" in back).toBe(true);
      const q = (back as {start: number[]}).start;
      expect(Math.hypot(q[0] - p[0], q[1] - p[1])).toBeLessThan(0.015);
    }
    const [lon, lat] = mapToLonLat(GEO, 7, 8);
    expect(lonLatToMap(GEO, lon, lat)[0]).toBeCloseTo(7, 6);
  });
  it("lat/lon needs an anchor and sane degrees", () => {
    expect(startFromInput({mode: "lonlat", lat: "39.9", lon: "-75.2"}, null)).toEqual({error: "this map has no GPS anchor: enter map x / y"});
    expect("error" in startFromInput({mode: "lonlat", lat: "91", lon: "-75.2"}, GEO)).toBe(true);
    expect("error" in startFromInput({mode: "lonlat", lat: "39.9", lon: "east"}, GEO)).toBe(true);
    expect(inputOf(null, "xy", null)).toEqual({mode: "xy", x: "", y: ""});
    expect(inputOf([1, 2], "lonlat", null)).toEqual({mode: "xy", x: "1.00", y: "2.00"}); // no anchor: x/y
  });
  it("outside the map is a warning, not a block", () => {
    expect(startWarning([10, 10], [0, 0, 50, 30])).toBeNull();
    expect(startWarning([53, 10], [0, 0, 50, 30])).toBeNull(); // within the margin
    expect(startWarning([80, 10], [0, 0, 50, 30])).toMatch(/outside the map/);
    expect(startWarning([80, 10], null)).toBeNull();
  });
});

describe("dragging a start", () => {
  const places = [
    {sym: "P(1)", x: 10, y: 10},
    {sym: "P(2)", x: 13, y: 10},
  ];
  it("snaps to the nearest place within reach", () => {
    expect(nearestPlace([11, 10.5], places)?.sym).toBe("P(1)");
    expect(nearestPlace([12, 10], places)?.sym).toBe("P(2)");
    expect(nearestPlace([30, 30], places)).toBeNull();
  });
  it("release commits the new start; a tap or a no-op does not", () => {
    let d = grab("spot1", 7, [1.001, 2.004], 100, 100);
    expect(d.from).toEqual([1, 2]);
    d = dragTo(d, [1.5, 2], 103, 100, 5); // within the tap slop
    expect(d.moved).toBe(false);
    expect(release(d)).toBeNull(); // a tap on the glyph moves nothing
    d = dragTo(d, [8.456, 9.123], 160, 140, 5);
    expect(d.moved).toBe(true);
    expect(release(d)).toEqual([8.46, 9.12]);
    d = dragTo(d, [11, 10.5], 170, 150, 5, nearestPlace([11, 10.5], places));
    expect(d.snap).toBe("P(1)");
    expect(release(d)).toEqual([10, 10]);
    d = dragTo(d, [1, 2], 100, 101, 5); // dragged back where it was
    expect(d.moved).toBe(true);
    expect(release(d)).toBeNull();
  });
  it("which agents a press may grab (none while a run locks the starts)", () => {
    const m = mission();
    const pv: Preview = {...PREVIEW, agents: {spot2: {start: [20, 20], end_s: 0, distance_m: 0}}};
    const base = {current: m, roster: [], run: null, preview: pv};
    expect(draggableAgents(base, false).map(a => [a.agent.id, a.pos])).toEqual([
      ["spot1", [1, 2]],
      ["spot2", [20, 20]], // a bound robot is drawn (and grabbed) at its live pose
    ]);
    expect(draggableAgents(base, true)).toEqual([]); // a map tool is armed
    expect(draggableAgents({...base, run: run({playing: true})}, false)).toEqual([]);
    expect(draggableAgents({...base, run: run({kind: "live"})}, false)).toEqual([]);
    expect(draggableAgents({...base, run: run({playing: false})}, false)).toHaveLength(2);
    // the mission's own start wins over the (older) preview's
    expect(shownPos({id: "spot1", kind: "spot", start: [3, 3]}, null, {...PREVIEW, agents: {spot1: {start: [1, 2], end_s: 0, distance_m: 0}}})).toEqual([3, 3]);
  });
  it("run locks: live until it ends, a playing simulation; a paused one is reset by a move", () => {
    expect(startLockReason(null)).toBeNull();
    expect(startLockReason(run({kind: "live"}))).toMatch(/live run/);
    expect(startLockReason(run({kind: "live", finished: true}))).toBeNull();
    expect(startLockReason(run({playing: true}))).toMatch(/pause/);
    expect(startLockReason(run({playing: false}))).toBeNull();
    expect(moveResetsRun(run({playing: false}))).toBe(true);
    expect(moveResetsRun(null)).toBe(false);
  });
});

describe("a moved start reaches the mission", () => {
  it("roster-only agent: the roster takes it, no request", async () => {
    const f = mockPut();
    useMission.setState({roster: [{id: "spot9", kind: "spot", start: [0, 0]}]});
    expect(await moveStart("spot9", [4.567, 1])).toBe(true);
    expect(useMission.getState().roster[0].start).toEqual([4.57, 1]);
    expect(f).not.toHaveBeenCalled();
  });
  it("mission agent: a new revision with the start, its preview, the run dropped; undo puts it back", async () => {
    const f = mockPut(m => ({...PREVIEW, agents: {spot1: {start: m.agents[0].start!, end_s: 0, distance_m: 0}}, tasks: {t1: {path: [m.agents[0].start!, [10, 10]]} as never}}));
    const m = mission();
    useMission.setState({current: m, preview: PREVIEW, run: run({finished: true}), roster: rosterWithMission([], m)});
    expect(await moveStart("spot1", [6, 7])).toBe(true);
    expect(puts(f)).toHaveLength(1);
    expect(String(f.mock.calls[0][0])).toContain("api/missions/m1");
    expect(puts(f)[0].agents[0].start).toEqual([6, 7]);
    const st = useMission.getState();
    expect(st.current!.revision).toBe(4);
    expect(st.current!.agents[0].start).toEqual([6, 7]);
    expect(st.preview!.tasks.t1.path[0]).toEqual([6, 7]); // the re-preview from the moved start
    expect(st.run).toBeNull(); // the finished simulation is gone (a new revision)
    expect(st.roster.find(a => a.id === "spot1")!.start).toEqual([6, 7]);
    expect(startOf("spot1")).toEqual([6, 7]);
    expect(await undoStart()).toBe(true);
    expect(puts(f)[1].agents[0].start).toEqual([1, 2]);
    expect(useMission.getState().current!.agents[0].start).toEqual([1, 2]);
    expect(useMission.getState().roster.find(a => a.id === "spot1")!.start).toEqual([1, 2]);
    expect(useMission.getState().startUndo).toEqual([]);
  });
  it("refused while the simulation plays (nothing changes)", async () => {
    const f = mockPut();
    const m = mission();
    useMission.setState({current: m, preview: PREVIEW, run: run({playing: true}), roster: rosterWithMission([], m)});
    expect(await moveStart("spot1", [6, 7])).toBe(false);
    expect(f).not.toHaveBeenCalled();
    expect(useMission.getState().error).toMatch(/pause/);
    expect(useMission.getState().current!.agents[0].start).toEqual([1, 2]);
    expect(useMission.getState().startUndo).toEqual([]);
  });
  it("a failed revision puts the drawn start back", async () => {
    vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(JSON.stringify({detail: "nope"}), {status: 400, headers: {"Content-Type": "application/json"}}));
    const m = mission();
    useMission.setState({current: m, preview: PREVIEW, roster: rosterWithMission([], m)});
    expect(await moveStart("spot1", [6, 7])).toBe(false);
    expect(useMission.getState().current!.agents[0].start).toEqual([1, 2]);
  });
  it("the map's start tool goes through the same path", async () => {
    const f = mockPut();
    const m = mission();
    useMission.setState({current: m, preview: PREVIEW, roster: rosterWithMission([], m), tool: {kind: "start", agent: "spot1"}});
    expect(onMapClick([3.333, 4.444, 0])).toBe(true);
    await flush();
    expect(puts(f)[0].agents[0].start).toEqual([3.33, 4.44]);
  });
  it("opening a mission puts its agents in the roster (others stay while there is room)", () => {
    const r = rosterWithMission(
      [
        {id: "spot1", kind: "husky", start: [9, 9], color: "#f00"},
        {id: "quad1", kind: "quad", start: [0, 0]},
        {id: "x", kind: "spot", start: [0, 0], binding: "euclid"}, // the mission's spot2 is bound to euclid
      ],
      mission(),
    );
    expect(r.map(a => a.id)).toEqual(["spot1", "spot2", "quad1"]);
    expect(r[0]).toMatchObject({kind: "spot", start: [1, 2], color: "#f00"});
    expect(r[1]).toMatchObject({start: null, binding: "euclid"});
  });
  it("Update mission from roster: a bound roster agent without a start starts at its live pose", () => {
    useMission.setState({roster: [{id: "spot1", kind: "spot", start: null, binding: "euclid"}, {id: "spot2", kind: "spot", start: null}]});
    const m = {...mission(), agents: [{id: "spot1", kind: "spot", start: [1, 2]}, {id: "spot2", kind: "spot", start: [5, 5]}]} as Mission;
    const out = withOperatorInputs(m);
    expect(out.agents[0].start).toBeNull();
    expect(out.agents[1].start).toEqual([5, 5]); // unbound: keeps the mission's
  });
});

describe("StartEditor", () => {
  const typeInto = (el: HTMLInputElement, v: string) => {
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
    act(() => {
      set.call(el, v);
      el.dispatchEvent(new Event("input", {bubbles: true}));
    });
  };
  const key = (el: HTMLElement, k: string) => act(() => void el.dispatchEvent(new KeyboardEvent("keydown", {key: k, bubbles: true})));
  it("typed x / y set the mission start on Enter; a bad number is shown, not sent", async () => {
    const f = mockPut();
    const m = mission();
    useMission.setState({current: m, preview: PREVIEW, roster: rosterWithMission([], m)});
    const h = render(<StartEditor agent={useMission.getState().roster[0]} mode="xy" />);
    const x = h.querySelector<HTMLInputElement>("input[aria-label='spot1 start map x (m)']")!;
    const y = h.querySelector<HTMLInputElement>("input[aria-label='spot1 start map y (m)']")!;
    expect(x.value).toBe("1.00");
    expect(y.value).toBe("2.00");
    typeInto(x, "abc");
    key(x, "Enter");
    expect(h.querySelector("[role=alert]")?.textContent).toMatch(/x must be a number/);
    expect(x.getAttribute("aria-invalid")).toBe("true");
    expect(puts(f)).toHaveLength(0);
    key(x, "Escape"); // back to the start
    expect(x.value).toBe("1.00");
    typeInto(x, "12.345");
    typeInto(y, "-4");
    key(y, "Enter");
    await flush();
    expect(puts(f)[0].agents[0].start).toEqual([12.35, -4]);
    expect(x.value).toBe("12.35"); // follows the new start
  });
  it("lat / lon entry lands on the same map point", async () => {
    const f = mockPut();
    const m = mission();
    const [lon, lat] = mapToLonLat(GEO, 20, -10);
    useMission.setState({current: m, preview: PREVIEW, roster: rosterWithMission([], m), geo: GEO});
    const h = render(<StartEditor agent={useMission.getState().roster[0]} mode="lonlat" />);
    const la = h.querySelector<HTMLInputElement>("input[aria-label='spot1 start latitude (°)']")!;
    const lo = h.querySelector<HTMLInputElement>("input[aria-label='spot1 start longitude (°)']")!;
    typeInto(la, lat.toFixed(7));
    typeInto(lo, lon.toFixed(7));
    key(lo, "Enter");
    await flush();
    const start = puts(f)[0].agents[0].start;
    expect(Math.hypot(start[0] - 20, start[1] + 10)).toBeLessThan(0.02);
  });
  it("off the graph: the preview's warning with a snap; live pose for a bound robot; locked while playing", async () => {
    const m = mission();
    const pv: Preview = {...PREVIEW, agents: {spot2: {start: [20, 21], end_s: 0, distance_m: 0}}, issues: [{level: "warning", code: "start_offgraph", agent: "spot1", msg: "spot1 starts 12 m from the mapped places", snap: [0, 5], offgraph_m: 12}]};
    const f = mockPut(() => pv);
    useMission.setState({current: m, preview: pv, roster: rosterWithMission([], m)});
    const h = render(
      <>
        <StartEditor agent={useMission.getState().roster[0]} mode="xy" />
        <StartEditor agent={useMission.getState().roster[1]} mode="xy" />
      </>,
    );
    const warn = h.querySelector("[data-testid=start-offgraph-spot1]")!;
    expect(warn.textContent).toMatch(/12 m off the places graph/);
    act(() => [...warn.querySelectorAll("button")].find(b => b.textContent?.includes("Snap"))!.click());
    await flush();
    expect(puts(f)[0].agents[0].start).toEqual([0, 5]);
    // spot2 is bound with no start: "live pose" is on, the fields show its live pose greyed
    const live = h.querySelector<HTMLInputElement>("[data-testid=start-edit-spot2] input[type=checkbox]")!;
    expect(live.checked).toBe(true);
    const x2 = h.querySelector<HTMLInputElement>("input[aria-label='spot2 start map x (m)']")!;
    expect(x2.disabled).toBe(true);
    expect(x2.placeholder).toBe("20.0");
    act(() => live.click()); // off: a fixed start where it is now
    await flush();
    expect(puts(f)[1].agents[1].start).toEqual([20, 21]);
    act(() => useMission.setState({run: run({playing: true})}));
    expect(h.querySelector<HTMLInputElement>("input[aria-label='spot1 start map x (m)']")!.disabled).toBe(true);
    expect(h.textContent).toMatch(/pause it to move a start/);
  });
});
