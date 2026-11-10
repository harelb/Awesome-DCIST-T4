import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {DirectRouteButtons, directRouteResultText, directRouteWarning, sendableRoutes} from "../src/viewer/DirectRoute";
import {missionRoutes, type Mission, type Preview} from "../src/stores/mission";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

describe("direct route rules", () => {
  it("only routes with two or more finite points are sendable", () => {
    const r = sendableRoutes({euclid: [[0, 0, 0], [1, 0, 0]], gauss: [[0, 0]], bad: [[0, NaN], [1, 1]]});
    expect(r.map(([k]) => k)).toEqual(["euclid"]);
    expect(sendableRoutes(null)).toEqual([]);
  });

  it("mission routes join each bound ground agent's task paths", () => {
    const m = {
      agents: [
        {id: "spot1", kind: "spot", binding: "euclid"},
        {id: "q", kind: "quad", binding: "penn_quad"},
        {id: "virt", kind: "spot", binding: null},
      ],
      tasks: [
        {id: "t1", agent: "spot1", type: "goto"},
        {id: "t2", agent: "q", type: "goto"},
        {id: "t3", agent: "spot1", type: "goto"},
        {id: "t4", agent: "virt", type: "goto"},
      ],
    } as unknown as Mission;
    const pv = {
      tasks: {
        t1: {path: [[0, 0, 0], [1, 0, 0]]},
        t2: {path: [[5, 5], [6, 6]]},
        t3: {path: [[1, 0, 0], [1, 2, 0]]},
        t4: {path: [[9, 9], [9, 8]]},
      },
    } as unknown as Preview;
    expect(missionRoutes(m, pv)).toEqual({euclid: [[0, 0, 0], [1, 0, 0], [1, 2, 0]]});
    expect(missionRoutes(null, pv)).toEqual({});
  });

  it("result text names waypoints, lease and warnings", () => {
    expect(directRouteResultText("euclid", {ok: true, waypoints: 8, length_m: 7, lease: {euclid: {ok: true}}})).toBe(
      "Sent 8 waypoints (7 m) to euclid's executor; took the Spot lease",
    );
    expect(directRouteResultText("euclid", {waypoints: 3, lease: {euclid: {ok: false, message: "no answer"}}, warning: "w"})).toContain("lease not taken: no answer; w");
  });
});

describe("DirectRouteButtons", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  let calls: {url: string; body: Record<string, unknown> | null}[] = [];

  beforeEach(() => {
    calls = [];
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string, init?: RequestInit) => {
        calls.push({url: String(url), body: init?.body ? JSON.parse(String(init.body)) : null});
        return new Response(JSON.stringify({ok: true, topic: "/euclid/omniplanner_node/compiled_plan_out", waypoints: 4, length_m: 3, lease: {euclid: {ok: true, message: ""}}}), {status: 200});
      }),
    );
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
  });

  const click = async (text: string) => {
    const b = [...host!.querySelectorAll("button")].find(x => x.textContent?.includes(text));
    expect(b, `button ${text}`).toBeTruthy();
    await act(async () => {
      b!.click();
      await new Promise(r => setTimeout(r, 10));
    });
  };

  it("asks first, then posts the exact path and shows the result", async () => {
    const path = [[0, 0, 0], [1, 0, 0], [3, 0, 0]];
    await act(async () => root!.render(<DirectRouteButtons routes={{euclid: path}} yaws={{euclid: 0.5}} />));
    await click("Send route directly → euclid");
    expect(calls).toEqual([]);
    expect(host!.textContent).toContain(directRouteWarning("euclid"));
    expect(directRouteWarning("euclid")).toBe(
      "Sends this exact path to euclid's executor. No plan repair, no constraints; a later omniplanner plan replaces it.",
    );
    await click("Send to euclid");
    expect(calls).toEqual([{url: "api/robots/euclid/path/direct", body: {path, yaw: 0.5}}]);
    expect(host!.querySelector("[role=status]")!.textContent).toBe("Sent 4 waypoints (3 m) to euclid's executor; took the Spot lease");
  });

  it("cancel sends nothing", async () => {
    await act(async () => root!.render(<DirectRouteButtons routes={{euclid: [[0, 0], [2, 0]]}} />));
    await click("Send route directly");
    await click("Cancel");
    expect(calls).toEqual([]);
    expect(host!.textContent).toContain("Send route directly → euclid");
  });

  it("renders nothing without a sendable route", async () => {
    await act(async () => root!.render(<DirectRouteButtons routes={{euclid: [[0, 0]]}} />));
    expect(host!.querySelector("[data-testid=direct-route]")).toBeNull();
  });
});
