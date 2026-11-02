import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {ChangeDetectionControl, canStart} from "../src/viewer/ChangeDetection";
import {useRobots} from "../src/stores/robots";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const FA = {parent: "map", child: "euclid/odom", ok: true, T: [0, 0, 0, 1, 0, 0, 0], stamp: 1, age_s: 0.2};

function setRobot(reloc: unknown) {
  useRobots.setState(s => ({
    state: {...s.state, euclid: {pose: {x: 0, y: 0, z: 0, qw: 1, qx: 0, qy: 0, qz: 0, frame: "euclid/odom", t: 1}, frame_align: FA, reloc} as never},
    stateAt: {...s.stateAt, euclid: Date.now()},
  }));
}

describe("change detection control", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  const posts: {url: string; body: unknown}[] = [];
  const mount = () => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<ChangeDetectionControl robot="euclid" />));
    return host;
  };
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
    posts.length = 0;
  });
  const stub = () =>
    vi.stubGlobal("fetch", vi.fn(async (url: string, init?: RequestInit) => {
      if (init?.method === "POST") {
        posts.push({url: String(url), body: JSON.parse(String(init.body))});
        return new Response(JSON.stringify({robot: "euclid", state: "running", t: 2, ok: true, results: [{cmd: "start", message: "started"}]}));
      }
      return new Response(JSON.stringify({robot: "euclid", state: "paused", t: 1}));
    }));

  it("only the legend's ok verdict allows Start", () => {
    expect(canStart("ok")).toBe(true);
    expect(canStart("warn")).toBe(false);
    expect(canStart("err")).toBe(false);
  });

  it("Start is disabled until relocalized; 'Start anyway' overrides", async () => {
    stub();
    setRobot({prior_lcs: 0, self_lcs: 0, best_assoc: 0, last_lc_age_s: null, lcs: [], jumps: 0, last_jump: null});
    const el = mount();
    await act(async () => await new Promise(r => setTimeout(r, 0)));
    const start = el.querySelector("[data-testid=cd-start-euclid]") as HTMLButtonElement;
    expect(el.textContent).toContain("paused");
    expect(start.disabled).toBe(true);
    expect(el.querySelector("[data-testid=cd-gate-euclid]")?.textContent).toContain("no prior match yet");
    act(() => (Array.from(el.querySelectorAll("button")).find(b => b.textContent === "Start anyway…") as HTMLButtonElement).click());
    await act(async () => (el.querySelector("[data-testid=cd-override-euclid]") as HTMLButtonElement).click());
    expect(posts).toEqual([{url: "api/robots/euclid/change_detection/reset_start", body: {override: true}}]);
    expect(el.textContent).toContain("running");
  });

  it("Start is enabled once relocalized and sends no override", async () => {
    stub();
    setRobot({prior_lcs: 4, self_lcs: 0, best_assoc: 6, last_lc_age_s: 3, lcs: [], jumps: 1, last_jump: {dxy: 2, dyaw: 3, age_s: 5}});
    const el = mount();
    await act(async () => await new Promise(r => setTimeout(r, 0)));
    const start = el.querySelector("[data-testid=cd-start-euclid]") as HTMLButtonElement;
    expect(start.disabled).toBe(false);
    expect(el.querySelector("[data-testid=cd-gate-euclid]")).toBeNull();
    await act(async () => start.click());
    expect(posts).toEqual([{url: "api/robots/euclid/change_detection/reset_start", body: {override: false}}]);
  });
});
