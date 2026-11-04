// GRSTAPS-X, the second omniplanner engine (API.md §15.5): the planner's account
// in failures, the schedule Gantt, the engine picker, and the engine a goal is sent with.
import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {fastDownwardOf} from "../src/lib/fastDownward";
import {FastDownwardDetails} from "../src/viewer/FastDownward";
import {PlannerSchedule} from "../src/viewer/PlannerSchedule";
import {EnginePicker} from "../src/viewer/MissionPanel";
import {refreshEngines, useEngines} from "../src/stores/engines";

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
  useEngines.setState({engines: [], engine: "fast_downward", loaded: false});
});

const json = (body: unknown) => new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});

describe("planner account", () => {
  it("prefers planner_log and falls back to fast_downward", () => {
    expect(fastDownwardOf({planner_log: {planner: "GRSTAPS-X", digest: "no license"}, fast_downward: null})?.planner).toBe("GRSTAPS-X");
    expect(fastDownwardOf({fast_downward: {digest: "unreachable"}})?.digest).toBe("unreachable");
    expect(fastDownwardOf({detail: "x"})).toBeNull();
  });

  it("names the planner that failed", () => {
    const el = render(<FastDownwardDetails fd={{planner: "GRSTAPS-X", digest: "No Gurobi license", log: ["[error] GRBException"], problem_file: "/w/itags_input.json"}} />);
    const d = el.querySelector('[data-testid="fast-downward"]')!;
    expect(d.textContent).toContain("GRSTAPS-X: No Gurobi license");
    expect(d.textContent).toContain("itags_input.json");
  });

  it("still says Fast Downward for its own account", () => {
    const el = render(<FastDownwardDetails fd={{digest: "solved"}} />);
    expect(el.textContent).toContain("Fast Downward: solved");
  });
});

describe("PlannerSchedule", () => {
  const schedule = {
    makespan: 40,
    rows: [
      {robot: "spot1", action: "inspect-object", target: "O(4)", start: 10, finish: 20, coalition: ["spot1", "spot2"]},
      {robot: "spot2", action: "inspect-object", target: "O(4)", start: 10, finish: 20, coalition: ["spot1", "spot2"]},
      {robot: "spot2", action: "visit-location", target: "O(8)", start: 30, finish: 40, coalition: ["spot2"]},
    ],
  };

  it("draws one row per robot, a bar per task, joint tasks marked", () => {
    const el = render(<PlannerSchedule schedule={schedule} />);
    expect(el.querySelectorAll(".tl-row")).toHaveLength(2);
    const bars = [...el.querySelectorAll<HTMLElement>('[data-testid="schedule-bar"]')];
    expect(bars).toHaveLength(3);
    expect(bars.filter(b => b.classList.contains("tl-joint"))).toHaveLength(2);
    expect(bars[0].title).toContain("together with spot2");
    expect(bars[2].style.left).toBe("75%");
    expect(el.textContent).toContain("makespan 40 s");
  });

  it("renders nothing without a schedule", () => {
    const el = render(<PlannerSchedule schedule={null} />);
    expect(el.querySelector('[data-testid="planner-schedule"]')).toBeNull();
  });
});

describe("engine picker", () => {
  it("offers GRSTAPS-X greyed out with the reason when it cannot plan", async () => {
    vi.spyOn(globalThis, "fetch").mockResolvedValue(
      json({
        default: "fast_downward",
        engines: [
          {id: "fast_downward", label: "Fast Downward", available: true, reason: null},
          {id: "grstaps", label: "GRSTAPS-X", available: false, reason: "no Gurobi license at /x"},
        ],
      }),
    );
    await refreshEngines();
    const el = render(<EnginePicker />);
    const radios = [...el.querySelectorAll<HTMLInputElement>('input[type="radio"]')];
    expect(radios.map(r => r.disabled)).toEqual([false, true]);
    expect(el.querySelector("label.muted")!.getAttribute("title")).toContain("no Gurobi license");
  });

  it("switches to GRSTAPS-X and falls back when it goes away", async () => {
    const engines = (ok: boolean) =>
      json({
        default: "fast_downward",
        engines: [
          {id: "fast_downward", label: "Fast Downward", available: true, reason: null},
          {id: "grstaps", label: "GRSTAPS-X", available: ok, reason: ok ? null : "gone"},
        ],
      });
    const f = vi.spyOn(globalThis, "fetch").mockResolvedValueOnce(engines(true));
    await refreshEngines();
    const el = render(<EnginePicker />);
    act(() => el.querySelectorAll<HTMLInputElement>('input[type="radio"]')[1].click());
    expect(useEngines.getState().engine).toBe("grstaps");
    f.mockResolvedValueOnce(engines(false));
    await act(async () => {
      await refreshEngines();
    });
    expect(useEngines.getState().engine).toBe("fast_downward");
  });
});

describe("goal images", () => {
  it("shows each robot's goals and a full view whose mask toggles", async () => {
    const {GoalImages, _resetGoalImageCache} = await import("../src/viewer/GoalImages");
    const {useViewer} = await import("../src/stores/viewer");
    _resetGoalImageCache();
    useViewer.setState({sources: {"neo4j:penn": {kind: "neo4j", enabled: true}} as never, order: ["neo4j:penn"]});
    vi.spyOn(globalThis, "fetch").mockImplementation(async (url: RequestInfo | URL) => {
      const u = String(url);
      if (u.includes("/node/O(3)")) return json({images: ["/api/graph/neo4j:penn/image/O(3)/0", "/api/graph/neo4j:penn/image/O(3)/1"], masks: [true, false], label_name: "chair"});
      return json({images: []});
    });
    const el = render(<GoalImages goals={{spot1: ["O(3)"], spot2: ["t100"]}} />);
    await act(async () => {
      await new Promise(r => setTimeout(r, 0));
    });
    expect([...el.querySelectorAll(".goal-images-robot")].map(e => e.textContent)).toEqual(["spot1", "spot2"]);
    const thumb = el.querySelector<HTMLButtonElement>('[data-testid="goal-thumb"]')!;
    expect(thumb.querySelector("img")!.getAttribute("src")).toContain("mask=1");
    expect(el.textContent).toContain("t100"); // a place: no image, its symbol only
    act(() => thumb.click());
    const full = () => document.querySelector<HTMLImageElement>('[data-testid="goal-image-full"]')!;
    expect(full().src).toContain("mask=1");
    const mask = () => document.querySelector<HTMLInputElement>('.toggle input[type="checkbox"]')!;
    act(() => mask().click());
    expect(full().src).not.toContain("mask=1");
    act(() => mask().click());
    expect(full().src).toContain("mask=1");
    act(() => [...document.querySelectorAll<HTMLButtonElement>("button")].find(b => b.textContent === "Next")!.click());
    expect(full().src).toContain("/O(3)/1");
    expect(full().src).not.toContain("mask=1"); // that view has no mask
  });

  it("collects a mission's goal symbols per agent", async () => {
    const {missionGoals} = await import("../src/viewer/ChatPanel");
    expect(
      missionGoals([
        {agent: "spot1", type: "goto", target: "O(3)"},
        {agent: "spot1", type: "inspect", target: "O(3)"},
        {agent: "spot2", type: "goto", target: [1, 2]},
        {agent: "quad1", type: "goto", target: "$found:crate"},
        {agent: "spot2", type: "place", target: "t598"},
      ]),
    ).toEqual({spot1: ["O(3)"], spot2: ["t598"]});
  });
});

describe("goal images for places and regions", () => {
  it("show the frame nearest to where the plan ends, without a mask toggle", async () => {
    const {GoalImages, _resetGoalImageCache} = await import("../src/viewer/GoalImages");
    const {useViewer} = await import("../src/stores/viewer");
    _resetGoalImageCache();
    useViewer.setState({sources: {"neo4j:penn": {kind: "neo4j", enabled: true}} as never, order: ["neo4j:penn"]});
    const calls: string[] = [];
    vi.spyOn(globalThis, "fetch").mockImplementation(async (url: RequestInfo | URL) => {
      const u = String(url);
      calls.push(u);
      if (u.includes("/near?"))
        return json({
          frames: [
            {image: "/api/graph/neo4j:penn/near/image?x=5&y=6&n=6&i=0", kind: "subkeyframe", dist_m: 0.4, yaw_deg: 12},
            {image: "/api/graph/neo4j:penn/near/image?x=5&y=6&n=6&i=1", kind: "keyframe", dist_m: 1.3, yaw_deg: null},
          ],
        });
      return json({images: []}); // a place: no images of its own
    });
    const el = render(<GoalImages goals={{gauss: [{sym: "t4826", xy: [5, 6], yaw: 1.5}]}} />);
    await act(async () => {
      await new Promise(r => setTimeout(r, 0));
    });
    const near = calls.find(u => u.includes("/near?"))!;
    expect(near).toContain("x=5") && expect(near).toContain("yaw=1.5");
    const thumb = el.querySelector<HTMLButtonElement>('[data-testid="goal-thumb"]')!;
    expect(thumb.textContent).toContain("t4826 ⌖");
    expect(thumb.title).toContain("sub-keyframe 0.4 m from where the plan ends");
    act(() => thumb.click());
    expect(document.querySelector(".toggle")).toBeNull();
    expect(document.body.textContent).toContain("nearest view to where the plan ends");
    act(() => [...document.querySelectorAll<HTMLButtonElement>("button")].find(b => b.textContent === "Next")!.click());
    expect(document.querySelector<HTMLImageElement>('[data-testid="goal-image-full"]')!.src).toContain("i=1");
  });
});

describe("map database reset", () => {
  it("shows what a reset undoes and starts it in one click", async () => {
    const {DatabaseReset} = await import("../src/pages/Maps");
    vi.spyOn(globalThis, "fetch").mockImplementation(async () =>
      json({target: "pennfused", source: "/d/fused_cfused/hydra/backend/dsg_with_mesh.json", run: "/d/fused_cfused", nodes: 18535, added_objects: 2, map_edits: 1}),
    );
    const start = vi.fn(async () => "job1");
    const el = render(<DatabaseReset targets={[{name: "pennfused", uri: "neo4j://127.0.0.1:7699", prior: false}]} onStart={start} />);
    await act(async () => {
      await new Promise(r => setTimeout(r, 0));
    });
    const row = el.querySelector('[data-testid="db-reset-row"]')!;
    expect(row.textContent).toContain("fused_cfused");
    expect(row.textContent).toContain("2 added object(s), 1 map edit(s)");
    await act(async () => {
      el.querySelector<HTMLButtonElement>('[data-testid="db-reset"]')!.click();
    });
    expect(start).toHaveBeenCalledWith("pennfused", "Reset neo4j:pennfused to fused_cfused");
  });
});

describe("mission card goals", () => {
  it("keep the server's goal end points (places need them for the nearest view)", async () => {
    const {toChatItem} = await import("../src/viewer/ChatPanel");
    const it = toChatItem({type: "mission", id: "m1", revision: 1, agents: [], tasks: [], issues: [], goals: {topaz: [{sym: "t(10376)", xy: [48, 41], yaw: 1.2}]}});
    expect((it as {goals: unknown}).goals).toEqual({topaz: [{sym: "t(10376)", xy: [48, 41], yaw: 1.2}]});
  });
});

describe("GRSTAPS-X region goals", () => {
  it("label a region visit in the schedule", () => {
    const el = render(
      <PlannerSchedule
        schedule={{
          makespan: 30,
          rows: [
            {robot: "spot1", action: "visit-location", target: "P(12)", start: 0, finish: 10, region: "r6", label: "explore r6 · 1/3"},
            {robot: "spot1", action: "visit-location", target: "P(40)", start: 10, finish: 20, region: "r2", label: "visit r2"},
          ],
        }}
      />,
    );
    const bars = [...el.querySelectorAll<HTMLElement>('[data-testid="schedule-bar"]')];
    expect(bars[0].title).toContain("explore r6 · 1/3 at P(12)");
    expect(bars[0].textContent).toBe("⬚");
    expect(bars[1].title).toContain("visit r2 at P(40)");
  });

  it("name map regions and drawn areas the way the server does", async () => {
    const {regionGoalSymbol} = await import("../src/viewer/AreaTools");
    const areas = [
      {name: "Dock", polygon: [[0, 0], [1, 0], [1, 1]]},
      {name: "Yard", polygon: [[0, 0], [2, 0], [2, 2]]},
    ];
    const row = {key: "", name: "Yard", purpose: "search" as const, scope: "mission" as const, color: ""};
    expect(regionGoalSymbol(row, areas)).toBe("area2");
    expect(regionGoalSymbol({...row, scope: "map", region: {symbol: "R(6)"} as never}, areas)).toBe("r6");
    expect(regionGoalSymbol({...row, purpose: "no_go"}, areas)).toBeNull();
  });

  it("preview a drawn area with its polygon and show the waypoints", async () => {
    const {AreasSection} = await import("../src/viewer/AreaTools");
    const {useMission} = await import("../src/stores/mission");
    const {usePlanPreview} = await import("../src/stores/planPreview");
    useMission.setState({areas: [{name: "Yard", polygon: [[0, 0], [9, 0], [9, 9]]}] as never, fences: [], roster: [{id: "spot1", kind: "spot", start: [1, 1]}] as never});
    useEngines.setState({loaded: true, engines: [{id: "grstaps", label: "GRSTAPS-X", available: true, reason: null}]});
    const bodies: Record<string, unknown>[] = [];
    vi.spyOn(globalThis, "fetch").mockImplementation(async (url: RequestInfo | URL, init?: RequestInit) => {
      if (String(url).includes("preview_pddl")) {
        bodies.push(JSON.parse(String(init?.body)));
        return json({routes: {}, driven_m: {}, actions: [], robots: ["spot1"], ground_s: 0, solve_s: 0.2, pois: 3, engine: "grstaps",
          schedule: {makespan: 95, rows: []}, regions: {area1: {kind: "explore", places: ["P(1)", "P(2)", "P(3)"], points: [[1, 1], [5, 1], [8, 6]], total: 3, spacing_m: 8}}}); // prettier-ignore
      }
      return json({regions: [], state: {}, history: [], snapshots: []});
    });
    const el = render(<AreasSection />);
    await act(async () => {
      await new Promise(r => setTimeout(r, 0)); // the map's areas load
    });
    const explore = [...el.querySelectorAll<HTMLButtonElement>("button")].find(b => b.textContent === "Explore")!;
    await act(async () => {
      explore.click();
      await new Promise(r => setTimeout(r, 0));
    });
    expect(bodies[0]).toMatchObject({pddl_goal: "(explored-region area1)", engine: "grstaps", areas: [{name: "Yard", polygon: [[0, 0], [9, 0], [9, 9]]}]});
    expect(el.querySelector('[data-testid="region-goal-result"]')!.textContent).toBe("3 waypoint(s) 8 m apart · makespan 1.6 min");
    expect(usePlanPreview.getState().preview?.regions?.area1.points).toHaveLength(3);
    act(() => usePlanPreview.getState().set(null, null));
  });
});
