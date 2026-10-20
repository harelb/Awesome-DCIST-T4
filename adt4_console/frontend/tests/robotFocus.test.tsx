import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import * as THREE from "three";
import {FOCUS_KEY, STALE_MS, cycleFocus, loadFocus, resolveFocus, robotLiveness, rosterOf, saveFocus} from "../src/viewer/robotFocus";
import {useViewer} from "../src/stores/viewer";
import {useRobots} from "../src/stores/robots";
import {useFleet} from "../src/stores/fleet";
import {DIM, applyDim, focusLook} from "../src/viewer/DimGroup";
import {poolForFocus} from "../src/viewer/sam3Search";
import {CameraPanel} from "../src/viewer/CameraPanel";
import {PlanPanel} from "../src/viewer/PlanPanel";
import {LivePanel} from "../src/viewer/LivePanel";
import type {LinkInfo} from "../src/types";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

function memStore() {
  const m = new Map<string, string>();
  return {getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, v: string) => void m.set(k, v), m};
}
const throwing = {
  getItem: () => {
    throw new Error("SecurityError");
  },
  setItem: () => {
    throw new Error("QuotaExceeded");
  },
};

describe("robot focus rules", () => {
  it("persists and falls back to all on junk or blocked storage", () => {
    const s = memStore();
    expect(loadFocus(s)).toBe("all");
    expect(saveFocus("euclid", s)).toBe(true);
    expect(s.m.get(FOCUS_KEY)).toBe("euclid");
    expect(loadFocus(s)).toBe("euclid");
    s.m.set(FOCUS_KEY, "   ");
    expect(loadFocus(s)).toBe("all");
    expect(loadFocus(throwing)).toBe("all");
    expect(saveFocus("euclid", throwing)).toBe(false);
    expect(loadFocus(null)).toBe("all");
  });

  it("an unknown robot resolves to all, an empty roster keeps the stored robot", () => {
    expect(resolveFocus("euclid", ["euclid", "hamilton"])).toBe("euclid");
    expect(resolveFocus("ghost", ["euclid", "hamilton"])).toBe("all");
    expect(resolveFocus("euclid", [])).toBe("euclid");
    expect(resolveFocus("all", ["euclid"])).toBe("all");
  });

  it("cycles all -> robots -> all, both ways", () => {
    const r = ["euclid", "hamilton"];
    expect(cycleFocus("all", r)).toBe("euclid");
    expect(cycleFocus("euclid", r)).toBe("hamilton");
    expect(cycleFocus("hamilton", r)).toBe("all");
    expect(cycleFocus("all", r, -1)).toBe("hamilton");
    expect(cycleFocus("ghost", r)).toBe("euclid");
    expect(cycleFocus("all", [])).toBe("all");
  });

  it("liveness: online / stale / offline", () => {
    const now = 100_000;
    expect(robotLiveness(now - 1000, true, now)).toBe("online");
    expect(robotLiveness(now - STALE_MS - 1, true, now)).toBe("stale");
    expect(robotLiveness(undefined, true, now)).toBe("stale");
    expect(robotLiveness(now, false, now)).toBe("offline");
    expect(robotLiveness(undefined, undefined, now)).toBe("offline");
  });

  it("roster: topology robots plus anything heard from, sorted and unique", () => {
    expect(rosterOf([{name: "willow", role: "base_station"}, {name: "lewis", role: "robot"}], ["euclid"], ["euclid", "hamilton"])).toEqual(["euclid", "hamilton", "lewis"]);
  });

  it("setFocus: robot becomes the plan robot and takes over an active follow", () => {
    useViewer.setState({planRobot: "hamilton", follow: "hamilton"});
    useViewer.getState().setFocus("euclid");
    expect(useViewer.getState()).toMatchObject({focus: "euclid", planRobot: "euclid", follow: "euclid"});
    useViewer.getState().set({follow: null});
    useViewer.getState().setFocus("hamilton");
    expect(useViewer.getState()).toMatchObject({focus: "hamilton", planRobot: "hamilton", follow: null});
    // all robots keeps the last plan robot (the Plan tab's own pick)
    useViewer.getState().setFocus("all");
    expect(useViewer.getState()).toMatchObject({focus: "all", planRobot: "hamilton"});
  });

  it("3D look: others dim (default), hide or show; nothing dims with all robots", () => {
    expect(focusLook("euclid", null, "dim")).toBe("normal");
    expect(focusLook("euclid", "euclid", "dim")).toBe("normal");
    expect(focusLook("hamilton", "euclid", "dim")).toBe("dim");
    expect(focusLook("hamilton", "euclid", "hide")).toBe("hidden");
    expect(focusLook("hamilton", "euclid", "show")).toBe("normal");
    expect(focusLook(null, "euclid", "dim")).toBe("normal"); // shared maps
  });

  it("applyDim scales the owner's opacity and restores it, re-reading owner changes", () => {
    const m = new THREE.MeshBasicMaterial({transparent: false, opacity: 1});
    applyDim(m, DIM);
    expect(m.opacity).toBeCloseTo(DIM);
    expect(m.transparent).toBe(true);
    applyDim(m, DIM); // idempotent per frame
    expect(m.opacity).toBeCloseTo(DIM);
    m.opacity = 0.5; // the owner (React prop) changed it while dimmed
    applyDim(m, DIM);
    expect(m.opacity).toBeCloseTo(0.5 * DIM);
    applyDim(m, 1);
    expect(m.opacity).toBeCloseTo(0.5);
    expect(m.transparent).toBe(false);
  });

  it("image search pool follows the focus", () => {
    const src = [
      {id: "prior", keyframes: 10},
      {id: "live/euclid/1700000000", keyframes: 3},
      {id: "live/euclid/1800000000", keyframes: 4},
      {id: "live/hamilton/1750000000", keyframes: 5},
    ];
    expect(poolForFocus(src, "euclid", "all")).toBe("live/euclid/1800000000");
    expect(poolForFocus(src, "hamilton", "live/euclid/1800000000")).toBe("live/hamilton/1750000000");
    expect(poolForFocus(src, "lewis", "live/euclid/1800000000")).toBe("all");
    expect(poolForFocus(src, "lewis", "prior")).toBe("prior");
    expect(poolForFocus(src, null, "live/euclid/1800000000")).toBe("all");
    expect(poolForFocus(src, null, "prior")).toBe("prior");
  });
});

const json = (d: unknown) => new Response(JSON.stringify(d), {status: 200, headers: {"Content-Type": "application/json"}});
const link = (r: string): LinkInfo => ({connected: true, staleness_s: 0.3, latency_ms: 40, nodes: 1, tiles: 0, budgets: {C1: 5000}, rates: {C1: 100}, drops: 0, gaps: 0, resyncs: 0, auto: true, epoch: r}) as unknown as LinkInfo;

describe("panels follow the focus", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  const urls: string[] = [];
  beforeEach(() => {
    urls.length = 0;
    const m = (name: string) => ({name, role: "robot", ip: "", user: ""});
    useFleet.setState({machines: {network: "t", machines: [m("euclid"), m("hamilton")]}});
    useRobots.setState({state: {}, plan: {}, teleop: {}, link: {euclid: link("euclid"), hamilton: link("hamilton")}});
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string) => {
        const u = String(url);
        urls.push(u);
        if (u.startsWith("api/plan/robots")) return json([{robot: "euclid", plugins: []}, {robot: "hamilton", plugins: []}]);
        if (u.endsWith("/cameras")) return json({cameras: u.includes("euclid") ? ["frontleft", "hand"] : ["rgb"]});
        if (u.includes("/camera/")) return new Response(new Blob(["x"], {type: "image/jpeg"}), {status: 200});
        if (u.startsWith("api/live")) return json({robots: {euclid: link("euclid"), hamilton: link("hamilton")}});
        return json({goals: [], plan: null, results: [], llm: []});
      }),
    );
    vi.stubGlobal("URL", Object.assign(URL, {createObjectURL: () => "blob:x", revokeObjectURL: () => undefined}));
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
    useViewer.getState().setFocus("all");
  });
  const render = async (el: React.ReactNode) => {
    await act(async () => root!.render(el));
    await act(async () => new Promise(r => setTimeout(r, 20)));
  };
  const text = () => host!.textContent ?? "";

  it("Camera: one main camera per robot with all robots; that robot's cameras when focused", async () => {
    useViewer.getState().setFocus("all");
    await render(<CameraPanel />);
    expect(host!.querySelectorAll(".camera-robots .camera-tile").length).toBe(2);
    expect(urls.some(u => u.includes("api/robots/euclid/camera/frontleft.jpg"))).toBe(true);
    expect(urls.some(u => u.includes("api/robots/hamilton/camera/rgb.jpg"))).toBe(true);
    expect(host!.querySelector('select[aria-label="Robot"]')).toBeNull();
    // clicking a robot's tile focuses it
    await act(async () => (host!.querySelector(".camera-robots .camera-tile-img") as HTMLButtonElement).click());
    await act(async () => new Promise(r => setTimeout(r, 20)));
    expect(useViewer.getState().focus).toBe("euclid");
    expect(host!.querySelector(".focus-chip")?.textContent).toContain("euclid");
    const camSel = host!.querySelector('select[aria-label="Camera stream"]') as HTMLSelectElement;
    expect([...camSel.options].map(o => o.value)).toEqual(["frontleft", "hand"]);
    // the focus moves: the panel follows
    await act(async () => useViewer.getState().setFocus("hamilton"));
    await act(async () => new Promise(r => setTimeout(r, 20)));
    expect(host!.querySelector(".focus-chip")?.textContent).toContain("hamilton");
    expect([...(host!.querySelector('select[aria-label="Camera stream"]') as HTMLSelectElement).options].map(o => o.value)).toEqual(["rgb"]);
  });

  it("Plan: the focus replaces the robot select; all robots keeps it", async () => {
    useViewer.getState().setFocus("hamilton");
    await render(<PlanPanel />);
    expect(host!.querySelector('select[aria-label="Plan robot"]')).toBeNull();
    expect(host!.querySelector(".focus-chip")?.textContent).toContain("hamilton");
    await act(async () => useViewer.getState().setFocus("all"));
    const sel = host!.querySelector('select[aria-label="Plan robot"]') as HTMLSelectElement;
    expect(sel.value).toBe("hamilton");
  });

  it("Live link: only the focused robot's link", async () => {
    useViewer.getState().setFocus("euclid");
    await render(<LivePanel />);
    expect(host!.querySelectorAll(".live-robot").length).toBe(1);
    expect(host!.querySelector(".live-robot strong")?.textContent).toBe("euclid");
    expect(text()).toContain("1 other robot hidden");
    await act(async () => useViewer.getState().setFocus("all"));
    // fleet totals come back with all robots
    expect([...host!.querySelectorAll(".live-robot strong")].map(e => e.textContent)).toEqual(["Fleet total", "euclid", "hamilton"]);
  });
});
