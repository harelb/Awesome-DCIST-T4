import {afterEach, describe, expect, it, vi} from "vitest";
import {act, type ReactNode} from "react";
import {createRoot, type Root} from "react-dom/client";
import {ConstraintsBox, factText, goalSendMessage, parseConstraintInput, RepairAlerts} from "../src/viewer/PlanRepair";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

let root: Root | null = null;
let host: HTMLDivElement | null = null;
function render(ui: ReactNode) {
  host = document.createElement("div");
  document.body.appendChild(host);
  root = createRoot(host);
  act(() => root!.render(ui));
  return host;
}
afterEach(() => {
  act(() => root?.unmount());
  host?.remove();
  root = null;
  vi.unstubAllGlobals();
});
const flush = () => act(async () => {
  await new Promise(r => setTimeout(r, 0));
});
const byLabel = (l: string) => host!.querySelector(`[aria-label="${l}"]`) as HTMLElement;
const byText = (t: string) => [...host!.querySelectorAll("button")].find(b => b.textContent === t) as HTMLButtonElement;
function type(el: HTMLElement, value: string) {
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
  act(() => {
    setter.call(el, value);
    el.dispatchEvent(new Event("input", {bubbles: true}));
  });
}
const click = (el: HTMLElement) => act(() => el.click());

describe("goalSendMessage", () => {
  it("says when a goal went through plan repair", () => {
    const m = goalSendMessage("PDDL", {ok: true, topic: "/willow/commanded_goal", via: "goal_manager", subscribers: 1, constraints: [{predicate: "forbidden-poi", symbols: ["o14"]}]});
    expect(m).toEqual({tone: "ok", text: "Sent through plan repair (goal_manager, 1 constraint)"});
  });
  it("warns prominently when goal_manager is not running", () => {
    const m = goalSendMessage("PDDL", {
      ok: true,
      topic: "/willow/omniplanner_node/visit_objects_pddl/pddl_goal",
      via: "planner",
      subscribers: 1,
      warning: "plan repair goal_manager is not running; sent straight to omniplanner — no constraints / replanning",
    });
    expect(m.tone).toBe("warn");
    expect(m.text).toContain("Plan repair goal_manager is not running");
  });
  it("still flags a planner that is not listening at all", () => {
    expect(goalSendMessage("Goto", {ok: true, topic: "/x", subscribers: 0}).tone).toBe("err");
    expect(goalSendMessage("Goto", {ok: true, topic: "/x/goto", subscribers: 1})).toEqual({tone: "ok", text: "Sent on /x/goto"});
  });
});

describe("constraints", () => {
  it("parses operator input", () => {
    expect(parseConstraintInput("forbidden-poi O(14)")).toEqual({predicate: "forbidden-poi", symbols: ["o14"]});
    expect(parseConstraintInput("Forbidden-Edge p1, P2")).toEqual({predicate: "forbidden-edge", symbols: ["p1", "p2"]});
    expect(parseConstraintInput("forbidden-poi")).toBeNull();
    expect(parseConstraintInput("avoid o1")).toBeNull();
    expect(factText({predicate: "forbidden-edge", symbols: ["p1", "p2"]})).toBe("(forbidden-edge p1 p2)");
  });

  it("adds, removes and clears through the API", async () => {
    const calls: [string, string, unknown][] = [];
    vi.stubGlobal("fetch", vi.fn(async (url: string, init?: RequestInit) => {
      calls.push([String(init?.method ?? "GET"), String(url), init?.body ? JSON.parse(String(init.body)) : undefined]);
      return new Response(JSON.stringify({robot: "hamilton", constraints: []}), {status: 200, headers: {"Content-Type": "application/json"}});
    }));
    render(<ConstraintsBox robot="hamilton" constraints={[{predicate: "forbidden-poi", symbols: ["o14"]}]} />);
    type(byLabel("New constraint"), "forbidden-edge p1 p2");
    click(byText("Add"));
    await flush();
    expect(calls.length).toBe(1);
    expect(calls[0][0]).toBe("POST");
    expect(calls[0][1]).toContain("api/robots/hamilton/constraints");
    expect(calls[0][2]).toEqual({constraints: [{predicate: "forbidden-edge", symbols: ["p1", "p2"]}]});
    click(byLabel("Remove (forbidden-poi o14)"));
    await flush();
    expect(calls[1][0]).toBe("DELETE");
    expect(calls[1][2]).toEqual({constraints: [{predicate: "forbidden-poi", symbols: ["o14"]}]});
    click(byText("Clear all"));
    await flush();
    expect(calls[2][0]).toBe("DELETE");
    // malformed input is refused locally
    type(byLabel("New constraint"), "forbidden-poi");
    click(byText("Add"));
    await flush();
    expect(host!.textContent).toContain("Use “forbidden-poi o14”");
    expect(calls.length).toBe(3);
  });
});

describe("RepairAlerts", () => {
  it("shows newest first with planner failures marked", () => {
    const el = render(
      <RepairAlerts
        alerts={[
          {t: 1, kind: "plan_failed", text: "multi_robot_pddl_constrained: no plan", seq: 1},
          {t: 2, kind: "alert", text: "Could not plan '(visited-object o7)'. Continuing the current plan.", seq: 2},
        ]}
      />,
    );
    const items = [...el.querySelectorAll("li")];
    expect(items[0].textContent).toContain("Could not plan");
    expect(items[1].className).toContain("tone-err");
    expect(items[1].textContent).toContain("planner failed:");
  });
  it("renders nothing without alerts", () => {
    expect(render(<RepairAlerts alerts={[]} />).innerHTML).toBe("");
  });
});
