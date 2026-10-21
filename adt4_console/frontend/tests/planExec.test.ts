import {describe, expect, it} from "vitest";
import {planExecStatus, refusalText} from "../src/lib/planExec";
import type {RobotPlan, TeleopState} from "../src/types";

const NOW = 10_000;
const plan = (over: Partial<RobotPlan> = {}, n = 3, t = NOW - 30): RobotPlan => ({
  goals: [],
  plan: {plan_id: "P", robot_name: "euclid", t, actions: Array.from({length: n}, () => ({type: "FOLLOW"}))},
  results: [],
  llm: [],
  ...over,
});
const ok = (i: number, t: number) => ({plan_id: "P", action_index: i, action_type: "FOLLOW", status: "SUCCESS", detail: "", t});
const teleop = (over: Partial<TeleopState> = {}) => ({plan_executing: true, plan_paused: false, ...over}) as TeleopState;

describe("plan execution status", () => {
  it("nothing without a plan", () => {
    expect(planExecStatus("euclid", undefined, {now: NOW}).tone).toBe("none");
  });
  it("executing the next step, with time on it", () => {
    const s = planExecStatus("euclid", plan({results: [ok(0, NOW - 20)]}), {now: NOW, connected: true, teleop: teleop()});
    expect(s).toMatchObject({tone: "ok", state: "executing", text: "executing step 2/3 · 20 s"});
  });
  it("still trying when a step takes too long (the executor retries silently)", () => {
    const s = planExecStatus("euclid", plan({}, 3, NOW - 200), {now: NOW, connected: true, teleop: teleop()});
    expect(s).toMatchObject({tone: "warn", state: "stuck", text: "still trying step 1/3 · 3 min"});
  });
  it("done when every step succeeded", () => {
    const s = planExecStatus("euclid", plan({results: [ok(0, NOW - 50), ok(1, NOW - 40), ok(2, NOW - 30)]}), {now: NOW});
    expect(s).toMatchObject({tone: "ok", state: "done", text: "plan done · 3 steps · 30 s ago"});
  });
  it("failed with the executor's reason", () => {
    const r = {plan_id: "P", action_index: 0, action_type: "FOLLOW", status: "FAILED", detail: "gave up after 3 attempts", t: NOW - 5};
    const s = planExecStatus("euclid", plan({results: [r]}), {now: NOW});
    expect(s).toMatchObject({tone: "err", state: "failed", text: "plan failed at step 1/3 — gave up after 3 attempts"});
  });
  it("ignores results of an older plan", () => {
    const old = {plan_id: "OLD", action_index: 0, status: "FAILED", detail: "x", t: NOW - 100};
    expect(planExecStatus("euclid", plan({results: [old]}), {now: NOW, teleop: teleop()}).state).toBe("executing");
  });
  it("refused: the newest goal never reached the robot", () => {
    const status = [{plugin: "p", robot: "", ok: true, error: "", t: NOW - 2, refusals: {hamilton: "lease_not_owned(held by bosdyn.android.spotapp 964e)"}}];
    const s = planExecStatus("hamilton", plan({status}), {now: NOW});
    expect(s).toMatchObject({tone: "err", state: "refused", text: "plan not sent — the tablet holds the lease"});
    // an older refusal, then a plan that did go out: not refused
    expect(planExecStatus("hamilton", plan({status}, 3, NOW - 1), {now: NOW, teleop: teleop()}).state).toBe("executing");
    expect(refusalText("battery 9%")).toBe("battery 9%");
  });
  it("offline, paused, lease lost and idle while steps remain", () => {
    const p = plan({results: [ok(0, NOW - 20)]});
    expect(planExecStatus("euclid", p, {now: NOW, connected: false}).text).toBe("robot offline · was on step 2/3");
    expect(planExecStatus("euclid", p, {now: NOW, teleop: teleop({plan_paused: true})}).text).toBe("paused · step 2/3");
    expect(planExecStatus("euclid", p, {now: NOW, lease: {known: true, owned: false, owner: "bosdyn.android.spotapp 8f80"}}).text).toBe(
      "stopped · the tablet holds the lease · step 2/3",
    );
    expect(planExecStatus("euclid", p, {now: NOW, teleop: teleop({plan_executing: false})}).state).toBe("idle");
    // just between two steps: not idle yet
    expect(planExecStatus("euclid", plan({results: [ok(0, NOW - 2)]}), {now: NOW, teleop: teleop({plan_executing: false})}).state).toBe("executing");
  });
});
