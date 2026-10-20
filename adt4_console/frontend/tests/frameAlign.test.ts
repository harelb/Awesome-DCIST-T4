import {describe, expect, it} from "vitest";
import * as THREE from "three";
import {
  alignMatrix,
  alignStatus,
  applyPoint,
  invertPoint,
  invertRay,
  layerKey,
  layerMatrix,
  nearestGotoPlace,
  poseInGlobal,
  robotOfSource,
  snapToPlannerMap,
  type Vec3,
} from "../src/lib/frameAlign";
import {GraphModel} from "../src/lib/graphModel";
import type {NodeRecord} from "../src/codec";
import type {FrameAlign} from "../src/types";

// map -> hamilton/odom: 90 deg yaw, then (10, 5, 0.7)
const S = Math.SQRT1_2;
const FA: FrameAlign = {parent: "map", child: "hamilton/odom", T: [10, 5, 0.7, S, 0, 0, S], stamp: 100, ok: true, reason: "", age_s: 0.5};
const NOT: FrameAlign = {parent: "map", child: "hamilton/odom", T: null, stamp: null, ok: false, reason: "not relocalized: LookupException: frame map does not exist"};

const close = (a: number[], b: number[]) => a.forEach((v, i) => expect(v).toBeCloseTo(b[i], 6));

describe("frame alignment matrix", () => {
  it("maps odom points into map", () => {
    const m = alignMatrix(FA)!;
    close(applyPoint(m, [1, 0, 0]), [10, 6, 0.7]); // +x odom = +y map after 90 deg
    close(invertPoint(m, applyPoint(m, [3, -2, 1])), [3, -2, 1]);
  });
  it("is null when not relocalized or malformed", () => {
    expect(alignMatrix(NOT)).toBeNull();
    expect(alignMatrix(undefined)).toBeNull();
    expect(alignMatrix({...FA, T: [1, 2, 3]})).toBeNull();
    expect(alignMatrix({...FA, T: [0, 0, 0, 0, 0, 0, 0]})).toBeNull();
    expect(applyPoint(null, [1, 2, 3])).toEqual([1, 2, 3]);
  });
  it("applies to child-frame layers only", () => {
    expect(layerMatrix(FA, "hamilton/odom")).not.toBeNull();
    expect(layerMatrix(FA, "")).not.toBeNull(); // live graph / tiles: the DSG frame
    expect(layerMatrix(FA, undefined)).not.toBeNull();
    expect(layerMatrix(FA, "map")).toBeNull(); // already global (pose fallback)
    expect(layerMatrix(FA, "hamilton/vision_other")).toBeNull(); // unknown: as is
    expect(layerMatrix(NOT, "hamilton/odom")).toBeNull();
    expect(layerKey(FA, "map")).toBe("");
    // the key only changes when the transform does (no re-glide at 4 Hz)
    expect(layerKey({...FA, stamp: 200, age_s: 0.1}, "")).toBe(layerKey(FA, ""));
    expect(layerKey({...FA, T: [10.2, 5, 0.7, S, 0, 0, S]}, "")).not.toBe(layerKey(FA, ""));
  });
  it("transforms a ray into the layer frame", () => {
    const m = alignMatrix(FA)!;
    const r = invertRay(m, [10, 6, 10.7], [0, 0, -1]);
    close(r.o, [1, 0, 10]);
    close(r.d, [0, 0, -1]);
  });
  it("names live sources' robots", () => {
    expect(robotOfSource("live:hamilton")).toBe("hamilton");
    expect(robotOfSource("neo4j:prior")).toBeNull();
    expect(robotOfSource("file:/x")).toBeNull();
  });
});

describe("robot pose in the global frame", () => {
  it("composes the alignment with an odom-frame pose", () => {
    const p = poseInGlobal({x: 1, y: 0, z: 0, qw: 1, qx: 0, qy: 0, qz: 0, frame: "hamilton/odom"}, FA);
    close([p.x, p.y, p.z], [10, 6, 0.7]);
    const yaw = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(p.qx, p.qy, p.qz, p.qw), "ZYX").z;
    expect(yaw).toBeCloseTo(Math.PI / 2, 6);
    expect(p.frame).toBe("map");
  });
  it("leaves map-frame and unaligned poses alone", () => {
    const pose = {x: 1, y: 2, z: 0, qw: 1, qx: 0, qy: 0, qz: 0, frame: "map"};
    expect(poseInGlobal(pose, FA)).toBe(pose);
    const odom = {...pose, frame: "hamilton/odom"};
    expect(poseInGlobal(odom, NOT)).toBe(odom);
  });
});

describe("alignment status", () => {
  it("reports relocalized with the transform age", () => {
    const s = alignStatus(FA, 2);
    expect(s.tone).toBe("ok");
    expect(s.text).toBe("aligned to map (relocalized) · 2.5 s");
    expect(s.title).toMatch(/yaw 90\.0°/);
  });
  it("warns when not relocalized, with the reason", () => {
    const s = alignStatus(NOT);
    expect(s.tone).toBe("warn");
    expect(s.text).toBe("hamilton/odom frame — not relocalized");
    expect(s.title).toMatch(/LookupException/);
  });
  it("handles static / identity / missing", () => {
    expect(alignStatus({...FA, stamp: 0, age_s: null}).text).toBe("aligned to map (relocalized) · static");
    expect(alignStatus({...FA, child: "map"}).text).toBe("in map frame");
    expect(alignStatus(undefined).tone).toBe("none");
  });
});

const rec = (sym: string, layer: number, pos: Vec3): NodeRecord => ({sym, layer, part: 0, pos, label: -1, bbox: null, color: [0, 0, 0], name: "", active: true});

function model(nodes: NodeRecord[]) {
  const m = new GraphModel();
  m.loadSnapshot({nodes, edges: [], labelspaces: {}} as never);
  return m;
}

describe("goto picking across frames", () => {
  // prior map (global frame) and the live robot graph (odom frame)
  const prior = model([rec("P(1)", 3, [0, 0, 0]), rec("P(2)", 3, [20, 20, 0])]);
  // live P(7) sits at odom (1, 0, 0) = map (10, 6, 0.7)
  const live = model([rec("P(7)", 3, [1, 0, 0]), rec("O(1)", 2, [0, 0, 0])]);
  const models = [
    {id: "neo4j:prior", model: prior},
    {id: "live:hamilton", model: live},
  ];
  const aligned = (id: string) => (robotOfSource(id) ? alignMatrix(FA) : null);

  it("snaps a map-frame click to an aligned live place", () => {
    const best = nearestGotoPlace(models, [10.2, 6.1, 0.7], aligned)!;
    expect(best.src).toBe("live:hamilton");
    expect(best.sym).toBe("P(7)");
    close(best.pos, [10, 6, 0.7]); // drawn where the live place is shown
  });
  it("clicks on the prior map stay in the map frame", () => {
    const best = nearestGotoPlace(models, [0.5, 0.2, 0], aligned)!;
    expect(best.src).toBe("neo4j:prior");
    expect(best.sym).toBe("P(1)");
    close(best.pos, [0, 0, 0]);
  });
  it("without the conversion the raw odom position would win wrongly", () => {
    // unaligned: P(7) is at (1, 0, 0) and beats P(1) for a click at (0.9, 0, 0)
    expect(nearestGotoPlace(models, [0.9, 0, 0], () => null)!.sym).toBe("P(7)");
    // aligned: P(7) is ~11 m away, P(1) wins
    expect(nearestGotoPlace(models, [0.9, 0, 0], aligned)!.sym).toBe("P(1)");
  });
});

describe("goto picks snap to the planner's map", () => {
  // planner (Neo4j) map: places + one object, global frame
  const planner = {id: "neo4j:live", model: model([rec("P(1)", 3, [0, 0, 0]), rec("P(2)", 3, [20, 20, 0]), rec("O(3)", 2, [4, 0, 0]), rec("a(4)", 2, [1, 1, 0])])};
  planner.model.nodes.get("a(4)")!.part = 97; // agent keyframe partition
  const none = () => null;

  it("a point near the live robot's place maps to the planner place nearby", () => {
    // live P(7) is drawn at map (10, 6, 0.7); the planner only has P(1) / P(2) / O(3)
    const best = snapToPlannerMap(planner, [10, 6, 0.7], none)!;
    expect(best.src).toBe("neo4j:live");
    expect(best.sym).toBe("P(1)"); // 11.7 m away (P(2) is 17 m)
  });
  it("never returns agent poses or objects for a plain pick", () => {
    expect(snapToPlannerMap(planner, [1, 1, 0], none)!.sym).toBe("P(1)");
    expect(snapToPlannerMap(planner, [4, 0, 0], none)!.sym).toBe("P(1)");
  });
  it("an object pick keeps a planner object within 1 m, else falls back to a place", () => {
    expect(snapToPlannerMap(planner, [4.5, 0, 0], none, {object: true})!.sym).toBe("O(3)");
    expect(snapToPlannerMap(planner, [6, 0, 0], none, {object: true})!.sym).toBe("P(1)");
  });
  it("agent poses in a robot-prefix partition are not objects", () => {
    const m = model([rec("P(1)", 3, [0, 0, 0]), rec("h(9)", 2, [4, 0, 0])]);
    m.nodes.get("h(9)")!.part = 104; // 'h'
    expect(snapToPlannerMap({id: "neo4j:live", model: m}, [4, 0, 0], none, {object: true})!.sym).toBe("P(1)");
  });
  it("nothing within range is null", () => {
    expect(snapToPlannerMap(planner, [100, 100, 0], none)).toBeNull();
  });
  it("searches in the planner source's own frame", () => {
    // planner drawn through the 90 deg alignment: P(1) at local (0,0,0) shows at map (10, 5, 0.7)
    const best = snapToPlannerMap(planner, [10.2, 5.1, 0.7], () => alignMatrix(FA))!;
    expect(best.sym).toBe("P(1)");
    close(best.pos, [10, 5, 0.7]);
  });
});

describe("robots store alignment lookup", () => {
  it("live sources take their robot's alignment from the C1 state, else the link", async () => {
    const {useRobots, sourceMatrix, frameAlignOf, robotLayerMatrix} = await import("../src/stores/robots");
    useRobots.setState({state: {}, link: {hamilton: {connected: true, frame_align: NOT}}});
    expect(sourceMatrix("live:hamilton")).toBeNull();
    useRobots.setState({state: {hamilton: {frame_align: FA}}});
    expect(frameAlignOf("hamilton")).toBe(FA);
    close(applyPoint(sourceMatrix("live:hamilton"), [1, 0, 0]), [10, 6, 0.7]);
    expect(sourceMatrix("neo4j:prior")).toBeNull();
    expect(robotLayerMatrix("hamilton", "map")).toBeNull();
    expect(robotLayerMatrix("hamilton", "hamilton/odom")).not.toBeNull();
  });
});
