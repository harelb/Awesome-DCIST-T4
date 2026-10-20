import {describe, expect, it} from "vitest";
import {visitGoal, pddlSymbol} from "../src/viewer/PlanPanel";
import {chatHttpError, parseNdjson, toChatItem} from "../src/viewer/ChatPanel";
import {ApiError, errorText} from "../src/lib/api";
import {bpsToSlider, linkClasses, sliderToBps} from "../src/viewer/LivePanel";
import {currentActionIndex} from "../src/viewer/planUtil";
import {fuzzyFilter} from "../src/lib/fuzzy";
import {labelColor} from "../src/lib/palette";

describe("plan helpers", () => {
  it("builds omniplanner visit goals", () => {
    expect(pddlSymbol("O(12)")).toBe("o12");
    expect(visitGoal(["O(12)"])).toBe("(visited-object o12)");
    expect(visitGoal(["O(1)", "P(850)"])).toBe("(and (visited-object o1) (visited-place p850))");
  });
  it("tracks the current action from executor results", () => {
    const plan = {goals: [], llm: [], plan: {plan_id: "p", robot_name: "r", t: 0, actions: [{type: "FOLLOW"}, {type: "FOLLOW"}]}, results: []};
    expect(currentActionIndex(plan)).toBe(0);
    expect(currentActionIndex({...plan, results: [{plan_id: "p", action_index: 0, status: "SUCCESS"}]})).toBe(1);
    expect(currentActionIndex({...plan, results: [{plan_id: "p", action_index: 1, status: "RUNNING"}]})).toBe(1);
    expect(currentActionIndex({...plan, results: [{plan_id: "old", action_index: 1, status: "SUCCESS"}]})).toBe(0);
  });
});

describe("chat NDJSON", () => {
  it("splits complete lines and keeps the partial tail", () => {
    const [objs, rest] = parseNdjson('{"type":"text","text":"a"}\n{"type":"done"}\n{"type":"te');
    expect(objs).toEqual([{type: "text", text: "a"}, {type: "done"}]);
    expect(rest).toBe('{"type":"te');
  });
  it("turns SAM3 object_candidate lines into candidate cards", () => {
    const line = JSON.stringify({
      type: "object_candidate", pending_id: "p-1", index: 1, label: "fire extinguisher", score: 0.912,
      image_uri: "data:image/jpeg;base64,/9j/AA", centroid: [1, 2, 0.5], bbox_min: [0.9, 1.9, 0.1], bbox_max: [1.1, 2.1, 0.9],
      num_views: 2, src: "neo4j:live",
    });
    const [objs, rest] = parseNdjson(line + "\n" + '{"type":"done"}\n');
    expect(rest).toBe("");
    const items = (objs as Record<string, unknown>[]).map(toChatItem);
    expect(items[1]).toBeNull(); // done is not rendered
    expect(items[0]).toEqual({
      type: "object_candidate", pending_id: "p-1", index: 1, label: "fire extinguisher", score: 0.912,
      image_uri: "data:image/jpeg;base64,/9j/AA", centroid: [1, 2, 0.5], bbox_min: [0.9, 1.9, 0.1], bbox_max: [1.1, 2.1, 0.9],
      num_views: 2, src: "neo4j:live",
    });
  });
  it("rejects non-image URIs and missing pending ids", () => {
    const it = toChatItem({type: "object_candidate", pending_id: "p", image_uri: "javascript:alert(1)", centroid: "x"});
    expect(it).toMatchObject({image_uri: null, centroid: null, index: 0});
    expect(toChatItem({type: "object_candidate"})).toMatchObject({type: "error"});
    expect(toChatItem({type: "mystery"})).toBeNull();
  });
  it("503s say why instead of always blaming ROS", () => {
    expect(chatHttpError(503, "SAM3 query server unreachable")).toContain("SAM3");
    expect(errorText(new ApiError(503, "neo4j bolt://x: refused"))).toBe("Unavailable: neo4j bolt://x: refused");
    expect(errorText(new ApiError(503, "ROS unavailable: disabled"))).toBe("ROS not connected");
  });
});

describe("live link classes", () => {
  it("lists C6 and unknown classes only when the link reports them", () => {
    expect(linkClasses({connected: true, budgets: {C1: 1}}).map(c => c.cls)).toEqual(["C1", "C2", "C3", "C4", "C5"]);
    const l = {connected: true, budgets: {C1: 1, C6: 150000, C10: 5}, rates: {C7: 2}};
    expect(linkClasses(l).map(c => c.cls)).toEqual(["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C10"]);
    expect(linkClasses(l).find(c => c.cls === "C6")!.name).toBe("image store");
  });
});

describe("misc", () => {
  it("budget slider is a log scale that round-trips", () => {
    for (const b of [1000, 5000, 50000, 100000]) expect(Math.abs(sliderToBps(bpsToSlider(b)) - b) / b).toBeLessThan(0.01);
  });
  it("fuzzy filter ranks substring matches first", () => {
    expect(fuzzyFilter(["spot_relocalize", "spot_mapping", "phoenix_mapping"], "map", s => s)).toEqual(["spot_mapping", "phoenix_mapping"]);
    expect(fuzzyFilter(["spot_mapping", "base_station"], "sptmp", s => s)).toEqual(["spot_mapping"]);
  });
  it("label palette is deterministic", () => {
    expect(labelColor(5)).toEqual(labelColor(5));
    expect(labelColor(-1)).toEqual([200, 200, 200]);
  });
});
