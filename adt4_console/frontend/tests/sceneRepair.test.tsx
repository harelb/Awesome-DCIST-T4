import {afterEach, describe, expect, it, vi} from "vitest";
import {act, type ReactNode} from "react";
import {createRoot, type Root} from "react-dom/client";
import type {SceneRecommendation, SceneRepairState} from "../src/types";

const handlers = new Map<string, (d: unknown) => void>();
vi.mock("../src/ws", () => ({
  ws: {
    on: (prefix: string, h: (d: unknown) => void) => {
      handlers.set(prefix, h);
      return () => handlers.delete(prefix);
    },
  },
}));

import {changeText, SCENE_REPAIR_TOPIC, SceneRepairPanel, sourceKind} from "../src/viewer/PlanRepair";

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
  handlers.clear();
  vi.unstubAllGlobals();
});
const flush = () => act(async () => {
  await new Promise(r => setTimeout(r, 0));
});
const byText = (t: string) => [...host!.querySelectorAll("button")].find(b => b.textContent === t) as HTMLButtonElement;
const click = (el: HTMLElement) => act(() => el.click());

const REC: SceneRecommendation = {
  id: "sr1",
  robot: "hamilton",
  t: 1_700_000_000,
  source: "awcd/hamilton",
  origin: "perception",
  map_version: 12,
  reason: "new cone O43 is 3.1 m away, closer than goal O20 at 7.4 m",
  reasons: ["new cone O43 is 3.1 m away, closer than goal O20 at 7.4 m"],
  priority: "high",
  changes: [{kind: "added", symbol: "O43", label: "cone", old: null, new: [3.1, 0, 0], source: "awcd/hamilton", map_version: 12}],
  goal: {pddl: "(visited-object o20)", constraints: [], engine: null},
  proposed_goal: "(visited-object o43)",
  rewrites: [{from: "o20", to: "o43"}],
  status: "pending",
  result: null,
};

function state(over: Partial<SceneRepairState> = {}): SceneRepairState {
  return {
    map_version: 12,
    changes: [
      {id: 1, source: "awcd/hamilton", origin: "perception", map_version: 12, t: 1_700_000_000, changes: REC.changes},
      {id: 2, source: "console/harel", origin: "edit", map_version: 13, t: 1_700_000_010, summary: "edge P1 - P2 removed", changes: []},
    ],
    events: [],
    recommendations: [REC],
    pending: {hamilton: "sr1"},
    ...over,
  };
}

function stubFetch(initial: SceneRepairState) {
  const calls: [string, string, unknown][] = [];
  vi.stubGlobal("fetch", vi.fn(async (url: string, init?: RequestInit) => {
    calls.push([String(init?.method ?? "GET"), String(url), init?.body ? JSON.parse(String(init.body)) : undefined]);
    const body = String(url).includes("/confirm") || String(url).includes("/decline") ? {...REC, status: "confirmed"} : initial;
    return new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});
  }));
  return calls;
}

describe("scene change helpers", () => {
  it("names sources and changes", () => {
    expect(sourceKind("awcd/hamilton")).toBe("awcd");
    expect(sourceKind("sget/chat")).toBe("sget");
    expect(sourceKind("console/harel")).toBe("console");
    expect(sourceKind("something")).toBe("other");
    expect(changeText({kind: "removed", symbol: "O20", label: "cone", old: null, new: null})).toBe("removed O20 cone");
  });
});

describe("SceneRepairPanel", () => {
  it("shows the pending recommendation with the original and proposed goal and the change feed", async () => {
    stubFetch(state());
    render(<SceneRepairPanel robot="hamilton" />);
    await flush();
    expect(host!.textContent).toContain("Replan recommended · hamilton");
    expect(host!.textContent).toContain("closer than goal O20");
    expect(host!.textContent).toContain("(visited-object o20)");
    expect(host!.textContent).toContain("(visited-object o43)");
    expect(byText("Replan with O43")).toBeTruthy();
    expect(byText("Replan same goal")).toBeTruthy();
    expect(byText("Dismiss")).toBeTruthy();
    const feed = host!.querySelector(".scene-feed")!;
    expect(feed.textContent).toContain("edge P1 - P2 removed");
    expect(feed.querySelector(".src-console")).toBeTruthy();
    expect(feed.querySelector(".src-awcd")).toBeTruthy();
    expect(feed.textContent).toContain("v13");
  });

  it("confirms the proposed goal, the original, or dismisses", async () => {
    const calls = stubFetch(state());
    render(<SceneRepairPanel robot="hamilton" />);
    await flush();
    click(byText("Replan with O43"));
    await flush();
    const posts = () => calls.filter(c => c[0] === "POST");
    expect(posts()[0][1]).toContain("api/plan/scene_repair/sr1/confirm");
    expect(posts()[0][2]).toEqual({variant: "proposed"});
    click(byText("Replan same goal"));
    await flush();
    expect(posts()[1][2]).toEqual({variant: "original"});
    act(() => (host!.querySelector('[aria-label="Resume robot on dismiss"]') as HTMLInputElement).click());
    click(byText("Dismiss"));
    await flush();
    expect(posts()[2][1]).toContain("api/plan/scene_repair/sr1/decline");
    expect(posts()[2][2]).toEqual({resume: true});
  });

  it("hides another robot's recommendation and follows hub pushes", async () => {
    stubFetch(state({recommendations: [], pending: {}, changes: []}));
    render(<SceneRepairPanel robot="hamilton" />);
    await flush();
    expect(host!.textContent).toBe("");
    act(() => handlers.get(SCENE_REPAIR_TOPIC)!(state({recommendations: [{...REC, robot: "willow"}], pending: {willow: "sr1"}})));
    expect(host!.textContent).not.toContain("Replan recommended");
    expect(host!.textContent).toContain("Scene changes");
    // an edit without a rewrite offers only the same goal
    const edit: SceneRecommendation = {...REC, id: "sr2", origin: "edit", source: "sget/chat", rewrites: [], proposed_goal: REC.goal.pddl, reason: "map edited (sget/chat): O5 removed"};
    act(() => handlers.get(SCENE_REPAIR_TOPIC)!(state({recommendations: [edit], pending: {hamilton: "sr2"}})));
    expect(host!.textContent).toContain("map edited (sget/chat)");
    expect([...host!.querySelectorAll("button")].map(b => b.textContent)).toEqual(["Replan same goal", "Dismiss"]);
  });

  it("says staged edits must be published before anything replans", async () => {
    stubFetch(state({recommendations: [], changes: [], pending: {}, staged: {src: "live", count: 2, summary: "delete O20", t: 1}}));
    render(<SceneRepairPanel robot="hamilton" />);
    await flush();
    const note = host!.querySelector(".scene-staged")!;
    expect(note.textContent).toContain("2 staged map edits not published yet");
    expect(note.textContent).toContain("Publish them");
  });
});
