// Mission planner UX: one map tool at a time, the area popover (purpose / scope),
// fixes for targets the map no longer has, the assistant's assumptions on the
// mission card, disabled reasons and the collapsible nav.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {MemoryRouter, Route, Routes} from "react-router";
import {useMission, withoutTask, withTarget, replanText, onMapClick, type Mission, type Issue} from "../src/stores/mission";
import {useMapEdit, setEditMode} from "../src/stores/mapEdit";
import {useViewer} from "../src/stores/viewer";
import {useChat} from "../src/stores/chat";
import {activeTool, activeToolOf, armEditTool, armMissionTool, cancelTool, dropLastCorner, finishDrawing, saveArea} from "../src/stores/tools";
import {editApi, type Region} from "../src/lib/mapEdit";
import {AreaPopover, areaRows} from "../src/viewer/AreaTools";
import {ToolBanner} from "../src/viewer/ToolBanner";
import {disabledWhy, errorBadge, IssueRow} from "../src/viewer/MissionPanel";
import {MissionCard, toChatItem, type MissionCardItem} from "../src/viewer/ChatPanel";
import {loadNavCollapsed, NAV_KEY, saveNavCollapsed, Shell} from "../src/components/Shell";

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

const region = (symbol: string, name: string): Region => ({symbol, name, class: "region", center: null, polygon: [[0, 0], [1, 0], [1, 1]], source: null, members: {}});

beforeEach(() => {
  useMission.setState({roster: [], areas: [], fences: [], planted: [], draft: [], pendingArea: null, tool: {kind: "none"}, current: null, preview: null, busy: null});
  useMapEdit.setState({on: false, src: "neo4j:penn", selection: [], tool: "select", lasso: [], regions: [], state: null, busy: false, error: null, toast: null});
  useViewer.setState({pickMode: "inspect", sources: {"neo4j:penn": {id: "neo4j:penn", kind: "neo4j", label: "penn", enabled: true, status: "ready", error: null, model: null, rev: 1} as never}, order: ["neo4j:penn"]});
});

describe("one map tool at a time", () => {
  it("arming a mission tool drops the lasso and the other way round", () => {
    useMapEdit.setState({on: true});
    armEditTool("lasso");
    useMapEdit.setState({lasso: [[0, 0]]});
    expect(activeTool()).toEqual({owner: "edit", tool: "lasso"});
    armMissionTool({kind: "area"});
    expect(useMapEdit.getState().tool).toBe("select");
    expect(useMapEdit.getState().lasso).toEqual([]);
    expect(useViewer.getState().pickMode).toBe("mission");
    expect(activeTool()).toEqual({owner: "mission", tool: {kind: "area"}});
    useMission.setState({draft: [[1, 1]]});
    armEditTool("lasso");
    expect(useMission.getState().tool).toEqual({kind: "none"});
    expect(useMission.getState().draft).toEqual([]);
    expect(useViewer.getState().pickMode).toBe("edit");
    expect(activeTool()).toEqual({owner: "edit", tool: "lasso"});
  });
  it("turning edit mode on disarms a mission tool", () => {
    armMissionTool({kind: "start", agent: "spot1"});
    setEditMode(true);
    expect(useMission.getState().tool).toEqual({kind: "none"});
    expect(useViewer.getState().pickMode).toBe("edit");
    setEditMode(false);
  });
  it("select mode and no tool are not an armed tool", () => {
    expect(activeToolOf({kind: "none"}, true, "select")).toEqual({owner: "none"});
    expect(activeToolOf({kind: "none"}, false, "lasso")).toEqual({owner: "none"}); // lasso only counts while editing
  });
  it("keys: Backspace drops a corner, Enter finishes into the popover, Esc cancels", () => {
    armMissionTool({kind: "area"});
    for (const p of [[0, 0], [4, 0], [4, 4], [0, 4]]) onMapClick(p);
    expect(dropLastCorner()).toBe(true);
    expect(useMission.getState().draft).toHaveLength(3);
    expect(finishDrawing()).toBe(true);
    expect(useMission.getState().pendingArea).toHaveLength(3);
    expect(useViewer.getState().pickMode).toBe("inspect");
    armMissionTool({kind: "area"});
    onMapClick([9, 9]);
    cancelTool();
    expect(useMission.getState().tool).toEqual({kind: "none"});
    expect(useMission.getState().draft).toEqual([]);
  });
  it("one banner: the armed tool's, with its keys; the edit banner otherwise", () => {
    useMapEdit.setState({on: true});
    useViewer.setState({pickMode: "edit"});
    const h = render(<ToolBanner />);
    expect(h.querySelectorAll("[data-testid=tool-banner]")).toHaveLength(1);
    expect(h.textContent).toContain("Map edit");
    act(() => armMissionTool({kind: "area"}));
    expect(h.querySelectorAll("[data-testid=tool-banner]")).toHaveLength(1);
    expect(h.textContent).toContain("Draw area");
    expect(h.textContent).not.toContain("Map edit");
    expect((button(h, "Finish") as HTMLButtonElement).disabled).toBe(true);
    act(() => {
      for (const p of [[0, 0], [4, 0], [4, 4]]) onMapClick(p);
    });
    expect(h.textContent).toContain("3 corners");
    act(() => {
      window.dispatchEvent(new KeyboardEvent("keydown", {key: "Enter"}));
    });
    expect(useMission.getState().pendingArea).toHaveLength(3);
    expect(h.textContent).toContain("Map edit"); // back to the edit banner
    act(() => armEditTool("lasso"));
    expect(h.textContent).toContain("Lasso");
    act(() => {
      window.dispatchEvent(new KeyboardEvent("keydown", {key: "Escape"}));
    });
    expect(useMapEdit.getState().tool).toBe("select");
    act(() => setEditMode(false));
  });
});

describe("areas: popover, scope, one list", () => {
  it("names the drawing after the first free name of both scopes", () => {
    useMapEdit.setState({regions: [region("R0", "A1")]});
    useMission.setState({pendingArea: [[0, 0], [5, 0], [5, 5]]});
    const h = render(<AreaPopover />);
    expect((h.querySelector("input[aria-label='Area name']") as HTMLInputElement).value).toBe("A2");
    // mission scope by default: no write warning
    expect(h.querySelector("[data-testid=area-map-warning]")).toBeNull();
    act(() => button(h, "Save to map")!.click()); // the scope option
    expect(h.querySelector("[data-testid=area-map-warning]")?.textContent).toMatch(/Writes the area into the map now/);
    act(() => button(h, "No-go zone")!.click());
    // no-go zones are mission scope only
    expect(h.querySelector("[data-testid=area-map-warning]")).toBeNull();
    expect((h.querySelector("input[aria-label='Area name']") as HTMLInputElement).value).toBe("No-go 1");
    const mapOpt = [...h.querySelectorAll("[aria-label=Scope] button")].find(b => b.textContent === "Save to map") as HTMLButtonElement;
    expect(mapOpt.disabled).toBe(true);
  });
  it("staged mode says the area waits for publish", () => {
    useMapEdit.setState({state: {mode: "staged", live: true} as never});
    useMission.setState({pendingArea: [[0, 0], [5, 0], [5, 5]]});
    const h = render(<AreaPopover />);
    act(() => button(h, "Save to map")!.click());
    expect(h.querySelector("[data-testid=area-map-warning]")?.textContent).toMatch(/staged until you publish/);
  });
  it("rejects a name in use in either scope", async () => {
    useMapEdit.setState({regions: [region("R0", "Courtyard")]});
    useMission.setState({pendingArea: [[0, 0], [5, 0], [5, 5]]});
    expect(await saveArea("courtyard", "search", "mission")).toMatch(/already an area/);
    expect(await saveArea("Lot", "no_go", "map")).toMatch(/mission only/);
    expect(await saveArea("Lot", "search", "mission")).toBeNull();
    expect(useMission.getState().areas.map(a => a.name)).toEqual(["Lot"]);
  });
  it("map scope makes a Room through the edit API", async () => {
    const batch = vi.spyOn(editApi, "batch").mockResolvedValue({mutation_id: "m1", revision: 2, changes: [], summary: ["area Lot"], affected: [], map_version: 3, staged: false});
    vi.spyOn(editApi, "state").mockResolvedValue({mode: "immediate", staged: []} as never);
    vi.spyOn(editApi, "regions").mockResolvedValue({regions: [region("R5", "Lot")]});
    vi.spyOn(editApi, "history").mockResolvedValue({history: []});
    vi.spyOn(editApi, "snapshots").mockResolvedValue({snapshots: []});
    vi.spyOn(useViewer.getState(), "reloadSource").mockResolvedValue(undefined as never);
    useMission.setState({pendingArea: [[0, 0], [5, 0], [5, 5]]});
    expect(await saveArea("Lot", "search", "map")).toBeNull();
    expect(batch).toHaveBeenCalledWith("neo4j:penn", [{op: "group", parent_layer: "Room", class: "region", name: "Lot", polygon: [[0, 0], [5, 0], [5, 5]]}], "area 'Lot'");
    expect(useMission.getState().pendingArea).toBeNull();
    expect(useMission.getState().areas).toEqual([]); // not a mission area too
  });
  it("lists both scopes once: a mission area named like a map area is the map's", () => {
    const rows = areaRows([{name: "A1"}, {name: "Gate"}], [{id: "No-go 1"}], [region("R0", "a1")]);
    expect(rows.map(r => [r.name, r.scope, r.purpose])).toEqual([
      ["a1", "map", "search"],
      ["Gate", "mission", "search"],
      ["No-go 1", "mission", "no_go"],
    ]);
  });
});

describe("missions the map moved under", () => {
  const mission = {
    id: "m1",
    name: "patrol",
    text: "spot1 inspect O(17), then go to O(413)",
    agents: [{id: "spot1", kind: "spot", start: [0, 0]}],
    tasks: [
      {id: "t1", agent: "spot1", type: "inspect", target: "O(17)"},
      {id: "t2", agent: "spot1", type: "goto", target: "O(413)", depends_on: ["t1"]},
    ],
    geofences: [],
    planted: [],
  } as unknown as Mission;
  const issue: Issue = {level: "error", msg: "t1: O(17) is not in the scene graph", code: "missing_target", task: "t1", target: "O(17)"};
  it("retarget / drop / re-plan build the right revision and text", () => {
    expect(withTarget(mission, "t1", "O(5)").tasks[0].target).toBe("O(5)");
    const dropped = withoutTask(mission, "t1");
    expect(dropped.tasks.map(t => t.id)).toEqual(["t2"]);
    expect(dropped.tasks[0].depends_on).toEqual([]);
    expect(replanText(mission, [issue.msg])).toBe("spot1 inspect O(17), then go to O(413)\n\nRe-plan this mission: t1: O(17) is not in the scene graph");
    expect(errorBadge({errors: 1})).toBe("  ⚠ 1 error");
    expect(errorBadge({errors: 0})).toBe("");
  });
  it("a missing target offers pick on map, drop task and re-plan in chat", () => {
    useMission.setState({current: mission});
    const h = render(
      <ul>
        <IssueRow issue={issue} mission={mission} problems={[issue.msg]} />
      </ul>,
    );
    act(() => button(h, "Pick on map")!.click());
    expect(useMission.getState().tool).toEqual({kind: "retarget", task: "t1"});
    expect(useViewer.getState().pickMode).toBe("mission");
    act(() => button(h, "Re-plan in chat")!.click());
    expect(useChat.getState().prefill).toMatch(/^spot1 inspect O\(17\).*Re-plan this mission: t1: O\(17\)/s);
    useChat.setState({prefill: null});
  });
  it("a pick on the map replaces the target (the node's symbol, else the point)", async () => {
    const put = vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(JSON.stringify({mission: {...mission, revision: 2}, preview: {tasks: {}, agents: {}, issues: [], makespan: 0, geo: null}}), {status: 200, headers: {"Content-Type": "application/json"}}));
    useMission.setState({current: mission, tool: {kind: "retarget", task: "t1"}});
    expect(onMapClick([3, 4, 0], "O(5)")).toBe(true);
    await vi.waitFor(() => expect(put).toHaveBeenCalled());
    const body = JSON.parse(String((put.mock.calls[0][1] as RequestInit).body));
    expect(put.mock.calls[0][0]).toBe("api/missions/m1");
    expect(body.tasks[0].target).toBe("O(5)");
    expect(useMission.getState().tool).toEqual({kind: "none"});
  });
  it("other issues offer no fixes", () => {
    const h = render(
      <ul>
        <IssueRow issue={{level: "warning", msg: "mock object 'crate' matches no search", code: "planted_unmatched"}} mission={mission} problems={[]} />
      </ul>,
    );
    expect(h.querySelectorAll("button")).toHaveLength(0);
  });
  it("says why Simulate / Execute are disabled", () => {
    expect(disabledWhy(2, 0, false)).toMatch(/2 errors below are fixed/);
    expect(disabledWhy(0, 0, false)).toMatch(/bind agents/);
    expect(disabledWhy(0, 1, false)).toBeNull();
  });
});

describe("the assistant's assumptions", () => {
  it("come with the card and show in amber", () => {
    const it_ = toChatItem({type: "mission", id: "m9", revision: 1, name: "crate", agents: [], tasks: [], issues: [], assumptions: ["'box' read as the class crate", "A1 used as the courtyard"], makespan: 3}) as MissionCardItem;
    expect(it_.assumptions).toEqual(["'box' read as the class crate", "A1 used as the courtyard"]);
    const h = render(<MissionCard it={it_} />);
    const box = h.querySelector("[data-testid=mission-assumptions]");
    expect(box?.textContent).toContain("A1 used as the courtyard");
    expect(box?.className).toContain("mission-assumptions");
  });
  it("an old card without them shows none", () => {
    const it_ = toChatItem({type: "mission", id: "m9", revision: 1, agents: [], tasks: [], issues: []}) as MissionCardItem;
    expect(it_.assumptions).toEqual([]);
    const h = render(<MissionCard it={it_} />);
    expect(h.querySelector("[data-testid=mission-assumptions]")).toBeNull();
  });
});

describe("collapsible nav", () => {
  beforeEach(() => window.localStorage.removeItem(NAV_KEY));
  it("remembers the choice; without one it collapses on small screens", () => {
    expect(loadNavCollapsed(1280)).toBe(true);
    expect(loadNavCollapsed(1600)).toBe(false);
    saveNavCollapsed(false);
    expect(loadNavCollapsed(1280)).toBe(false);
    saveNavCollapsed(true);
    expect(loadNavCollapsed(1600)).toBe(true);
  });
  it("survives blocked storage", () => {
    vi.spyOn(Storage.prototype, "getItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    vi.spyOn(Storage.prototype, "setItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    expect(loadNavCollapsed(1280)).toBe(true);
    expect(() => saveNavCollapsed(false)).not.toThrow();
  });
  it("the toggle switches the shell to icons only", () => {
    saveNavCollapsed(false);
    const h = render(
      <MemoryRouter initialEntries={["/x"]}>
        <Routes>
          <Route element={<Shell />}>
            <Route path="/x" element={<div />} />
          </Route>
        </Routes>
      </MemoryRouter>,
    );
    const shell = h.querySelector(".shell")!;
    expect(shell.classList.contains("nav-collapsed")).toBe(false);
    act(() => (h.querySelector("[data-testid=nav-toggle]") as HTMLButtonElement).click());
    expect(shell.classList.contains("nav-collapsed")).toBe(true);
    expect(window.localStorage.getItem(NAV_KEY)).toBe("1");
  });
});
