import {describe, expect, it} from "vitest";
import {visitGoal, pddlSymbol} from "../src/viewer/PlanPanel";
import {parseNdjson} from "../src/viewer/ChatPanel";
import {bpsToSlider, sliderToBps} from "../src/viewer/LivePanel";
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
