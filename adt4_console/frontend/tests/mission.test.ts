import {beforeEach, describe, expect, it} from "vitest";
import {fmtLonLat, lonLatToMap, lonLatToTile, mapToLonLat, mapToUtm, pickZoom, tileToLonLat, tilesForBox, utmZone, type GeoInfo} from "../src/lib/geo";
import {addAgent, addMissionArea, dedupeCorners, finishDraft, MAX_AGENTS, missionContext, nextAreaName, onMapClick, takenAreaNames, useMission, withOperatorInputs, type Mission} from "../src/stores/mission";
import {toChatItem} from "../src/viewer/ChatPanel";
import {agentColors} from "../src/viewer/MissionOverlay";

// Pennovation-like affine: ~1.17e-5 deg lon / m, ~0.9e-5 deg lat / m, rotated by the anchor yaw
const YAW = -0.5;
const KLON = 1 / 85300;
const KLAT = 1 / 111000;
const c = Math.cos(YAW);
const s = Math.sin(YAW);
const fwd = [
  [c * KLON, -s * KLON, -75.1997],
  [s * KLAT, c * KLAT, 39.9409],
];
// exact inverse of the 2×2 part
const det = fwd[0][0] * fwd[1][1] - fwd[0][1] * fwd[1][0];
const inv2 = [
  [fwd[1][1] / det, -fwd[0][1] / det],
  [-fwd[1][0] / det, fwd[0][0] / det],
];
const inv = [
  [inv2[0][0], inv2[0][1], -(inv2[0][0] * fwd[0][2] + inv2[0][1] * fwd[1][2])],
  [inv2[1][0], inv2[1][1], -(inv2[1][0] * fwd[0][2] + inv2[1][1] * fwd[1][2])],
];
const GEO: GeoInfo = {
  anchor: {epsg: 32618, yaw: YAW, t: [482979.48, 4421319.72], z0: -11.5},
  affine: {center: [0, 0], map_to_lonlat: fwd, lonlat_to_map: inv, max_err_m: 0},
};

describe("geo", () => {
  it("map <-> lon/lat round trips through the affine", () => {
    const [lon, lat] = mapToLonLat(GEO, 30, -80);
    const [x, y] = lonLatToMap(GEO, lon, lat);
    expect(x).toBeCloseTo(30, 6);
    expect(y).toBeCloseTo(-80, 6);
    expect(fmtLonLat(lon, lat)).toMatch(/°N .*°W$/);
  });
  it("map -> UTM is the rigid anchor", () => {
    const [e, n] = mapToUtm(GEO.anchor, 10, 0);
    expect(Math.hypot(e - 482979.48, n - 4421319.72)).toBeCloseTo(10, 9);
    expect(utmZone(32618)).toBe("18N");
    expect(utmZone(32718)).toBe("18S");
  });
  it("tile math is self-inverse", () => {
    const [x, y] = lonLatToTile(-75.1997, 39.9409, 19);
    const [lon, lat] = tileToLonLat(x, y, 19);
    expect(lon).toBeCloseTo(-75.1997, 9);
    expect(lat).toBeCloseTo(39.9409, 9);
  });
  it("tiles cover the box, with map-frame corners ~tile-sized", () => {
    const box = {min: [-150, -160], max: [60, 20]};
    const z = pickZoom(GEO, box, 19, 12, 160);
    const tiles = tilesForBox(GEO, box, z)!;
    expect(tiles.length).toBeGreaterThan(0);
    expect(tiles.length).toBeLessThanOrEqual(160);
    const t = tiles[0];
    const side = Math.hypot(t.corners[1][0] - t.corners[0][0], t.corners[1][1] - t.corners[0][1]);
    // a z tile is 40075 km * cos(lat) / 2^z wide
    expect(side).toBeCloseTo((40075016 * Math.cos((39.94 * Math.PI) / 180)) / 2 ** z, -1);
    expect(tilesForBox(GEO, box, 22, 10)).toBeNull(); // over the cap
  });
});

describe("mission store", () => {
  beforeEach(() => {
    useMission.setState({roster: [], areas: [], fences: [], planted: [], draft: [], pendingArea: null, tool: {kind: "none"}, current: null, world: {src: "neo4j:x", bounds: [0, 0, 10, 10], start_hint: [5, 5]}});
  });
  it("adds up to five agents with unique ids and spread starts", () => {
    for (let i = 0; i < 7; i++) addAgent(i === 0 ? "quad" : "spot");
    const r = useMission.getState().roster;
    expect(r).toHaveLength(MAX_AGENTS);
    expect(r.map(a => a.id)).toEqual(["quad1", "spot1", "spot2", "spot3", "spot4"]);
    expect(r[0].start).toEqual([5, 5]);
    expect(r[1].start).toEqual([10, 5]);
  });
  it("map tools: start, areas, mock objects", () => {
    addAgent("spot");
    useMission.setState({tool: {kind: "start", agent: "spot1"}});
    expect(onMapClick([1.234, 2.345, 0])).toBe(true);
    expect(useMission.getState().roster[0].start).toEqual([1.23, 2.35]);
    expect(useMission.getState().tool).toEqual({kind: "none"});
    expect(onMapClick([0, 0, 0])).toBe(false); // no tool armed
    useMission.setState({tool: {kind: "area"}});
    // a double-click on the last corner leaves it twice: one corner
    for (const p of [[0, 0], [10, 0], [10, 10], [10, 10.1]]) onMapClick(p);
    expect(finishDraft()).toBe(true);
    expect(useMission.getState().pendingArea).toEqual([[0, 0], [10, 0], [10, 10]]);
    expect(useMission.getState().tool).toEqual({kind: "none"});
    addMissionArea("courtyard", useMission.getState().pendingArea!, "search");
    expect(useMission.getState().areas).toEqual([{name: "courtyard", polygon: [[0, 0], [10, 0], [10, 10]]}]);
    expect(useMission.getState().pendingArea).toBeNull();
    useMission.setState({tool: {kind: "area"}});
    onMapClick([1, 1]);
    expect(finishDraft()).toBe(false); // < 3 corners: dropped
    expect(useMission.getState().pendingArea).toBeNull();
    useMission.setState({pendingArea: [[0, 0], [1, 0], [1, 1]]});
    addMissionArea("No-go 1", useMission.getState().pendingArea!, "no_go");
    expect(useMission.getState().fences).toEqual([{id: "No-go 1", polygon: [[0, 0], [1, 0], [1, 1]], kind: "no_go"}]);
    useMission.setState({tool: {kind: "plant", label: "crate"}});
    onMapClick([3, 4]);
    expect(useMission.getState().planted).toEqual([{label: "crate", pos: [3, 4], size_m: 1.5}]);
    const ctx = missionContext();
    expect(ctx.roster[0]).toEqual({id: "spot1", kind: "spot", start: [1.23, 2.35], binding: null});
    expect(ctx.areas[0]).toMatchObject({name: "courtyard", scope: "mission"});
    expect(ctx.no_go).toEqual([{name: "No-go 1", polygon: [[0, 0], [1, 0], [1, 1]]}]);
  });
  it("default area names skip every name in use, in both scopes", () => {
    useMission.setState({areas: [{name: "A1", polygon: []}], fences: [{id: "No-go 1", polygon: []}]});
    const taken = takenAreaNames(["a2", null, "Courtyard"]);
    expect(nextAreaName("search", taken)).toBe("A3");
    expect(nextAreaName("no_go", taken)).toBe("No-go 2");
    expect(taken.has("courtyard")).toBe(true);
    expect(dedupeCorners([[0, 0], [5, 0], [5, 5], [0.1, 0]])).toEqual([[0, 0], [5, 0], [5, 5]]); // closing click on the first corner
  });
  it("operator edits fold into a mission", () => {
    addAgent("spot");
    useMission.setState({roster: [{id: "spot1", kind: "spot", start: [9, 9], binding: "euclid"}], planted: [{label: "car", pos: [1, 1]}]});
    const m = {id: "m1", agents: [{id: "spot1", kind: "spot", start: [0, 0]}, {id: "x", kind: "quad"}], geofences: [], tasks: []} as unknown as Mission;
    const out = withOperatorInputs(m);
    expect(out.agents[0]).toMatchObject({start: [9, 9], binding: "euclid"});
    expect(out.agents[1]).toEqual({id: "x", kind: "quad"});
    expect(out.planted).toEqual([{label: "car", pos: [1, 1]}]);
  });
});

describe("mission chat card + colours", () => {
  it("a propose_mission event becomes a mission card", () => {
    const it = toChatItem({type: "mission", id: "m1", revision: 2, name: "find crate", agents: [{id: "quad1", kind: "quad"}], tasks: [], issues: [{level: "warning", msg: "w"}], makespan: 12.5});
    expect(it).toMatchObject({type: "mission", id: "m1", revision: 2, makespan: 12.5});
  });
  it("bound agents take their robot's colour, others the palette", () => {
    const cols = agentColors([{id: "a", kind: "spot", binding: "euclid"}, {id: "b", kind: "quad"}, {id: "c", kind: "spot", color: "#123456"}], r => (r === "euclid" ? "#ff0000" : "#000"));
    expect(cols.a).toBe("#ff0000");
    expect(cols.b).toMatch(/^rgb\(/);
    expect(cols.c).toBe("#123456");
  });
});
