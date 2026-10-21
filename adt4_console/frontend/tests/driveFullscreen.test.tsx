import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {ARM_CONFIRMED_KEY, CAPS, DEAD_ZONE, StickTracker, captureTouches, stickVector, sticksToTwist} from "../src/lib/teleop";
import {DriveFullscreen, pickView} from "../src/viewer/DriveFullscreen";
import {TeleopPanel} from "../src/viewer/TeleopPanel";
import {useViewer} from "../src/stores/viewer";
import {useRobots} from "../src/stores/robots";
import {useFleet} from "../src/stores/fleet";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

describe("stick math", () => {
  it("stickVector: screen down is stick down, clamped to the unit circle", () => {
    expect(stickVector({x: 100, y: 100}, {x: 100, y: 50}, 50)).toEqual({x: 0, y: 1});
    expect(stickVector({x: 100, y: 100}, {x: 125, y: 100}, 50)).toEqual({x: 0.5, y: 0});
    const v = stickVector({x: 0, y: 0}, {x: 300, y: -400}, 50);
    expect(Math.hypot(v.x, v.y)).toBeCloseTo(1);
    expect(v.x).toBeCloseTo(0.6);
    expect(v.y).toBeCloseTo(0.8);
    // the turn stick ignores vertical motion
    expect(stickVector({x: 0, y: 0}, {x: 25, y: -400}, 50, true)).toEqual({x: 0.5, y: 0});
    expect(stickVector({x: 0, y: 0}, {x: NaN, y: 0}, 50)).toEqual({x: 0, y: 0});
  });

  it("sticksToTwist: move = forward + strafe, turn = yaw, signs as Spot's body frame", () => {
    const full = sticksToTwist({x: 0, y: 1}, null, 1);
    expect(full).toEqual({vx: CAPS.vx, vy: 0, wz: 0});
    const back = sticksToTwist({x: 0, y: -1}, null, 0.5);
    expect(back.vx).toBeCloseTo(-CAPS.vx * 0.5);
    // stick right strafes right (-y) and turns right (-wz)
    expect(sticksToTwist({x: 1, y: 0}, null, 1).vy).toBeCloseTo(-CAPS.vy);
    expect(sticksToTwist(null, {x: 1, y: 0}, 1).wz).toBeCloseTo(-CAPS.wz);
    expect(sticksToTwist(null, {x: -1, y: 0}, 1).wz).toBeCloseTo(CAPS.wz);
    // both at once
    const both = sticksToTwist({x: 0, y: 1}, {x: -1, y: 0}, 1);
    expect(both.vx).toBeCloseTo(CAPS.vx);
    expect(both.wz).toBeCloseTo(CAPS.wz);
  });

  it("sticksToTwist: dead zones per axis, so a crooked forward push does not strafe", () => {
    expect(sticksToTwist({x: DEAD_ZONE * 0.9, y: DEAD_ZONE * 0.9}, {x: DEAD_ZONE * 0.5, y: 0}, 1)).toEqual({vx: 0, vy: 0, wz: 0});
    const t = sticksToTwist({x: 0.1, y: 0.9}, null, 1);
    expect(t.vy).toBe(0);
    expect(t.vx).toBeGreaterThan(0);
    expect(sticksToTwist(null, null, 1)).toEqual({vx: 0, vy: 0, wz: 0});
  });

  it("sticksToTwist never exceeds the caps", () => {
    const t = sticksToTwist({x: 5, y: 5}, {x: -9, y: 0}, 7);
    expect(Math.abs(t.vx)).toBeLessThanOrEqual(CAPS.vx);
    expect(Math.abs(t.vy)).toBeLessThanOrEqual(CAPS.vy);
    expect(Math.abs(t.wz)).toBeLessThanOrEqual(CAPS.wz);
  });
});

describe("StickTracker (multi-touch)", () => {
  it("left half grabs move, right half grabs turn, both at once", () => {
    const tr = new StickTracker(50);
    expect(tr.down(1, 100, 500, 800)).toBe("move");
    expect(tr.down(2, 700, 500, 800)).toBe("turn");
    tr.move(1, 100, 450); // full forward
    tr.move(2, 650, 500); // full left turn
    const t = tr.twist(1);
    expect(t.vx).toBeCloseTo(CAPS.vx);
    expect(t.wz).toBeCloseTo(CAPS.wz);
    expect(tr.get("move")?.origin).toEqual({x: 100, y: 500});
  });

  it("one finger per stick; lifting a finger releases only its stick", () => {
    const tr = new StickTracker(50);
    tr.down(1, 100, 500, 800);
    expect(tr.down(3, 200, 400, 800)).toBeNull(); // move already held
    tr.move(3, 200, 300); // ignored
    tr.down(2, 700, 500, 800);
    tr.move(1, 100, 450);
    tr.move(2, 750, 500);
    expect(tr.up(1)).toBe("move");
    const t = tr.twist(1);
    expect(t.vx).toBe(0);
    expect(t.wz).toBeCloseTo(-CAPS.wz);
    expect(tr.up(2)).toBe("turn");
    expect(tr.twist(1)).toEqual({vx: 0, vy: 0, wz: 0});
    expect(tr.size).toBe(0);
    // after a release the stick can be grabbed again
    expect(tr.down(4, 50, 50, 800)).toBe("move");
  });

  it("an unknown pointer is ignored, clear() drops everything", () => {
    const tr = new StickTracker(50);
    expect(tr.move(9, 1, 1)).toBeNull();
    expect(tr.up(9)).toBeNull();
    tr.down(1, 10, 10, 800);
    tr.move(1, 10, -40);
    tr.clear();
    expect(tr.twist(1)).toEqual({vx: 0, vy: 0, wz: 0});
  });
});

describe("captureTouches", () => {
  it("cancels touch defaults with a non-passive listener and removes it", () => {
    const el = document.createElement("div");
    const add = vi.spyOn(el, "addEventListener");
    const off = captureTouches(el);
    expect(add).toHaveBeenCalledWith("touchmove", expect.any(Function), {passive: false});
    const ev = new Event("touchmove", {cancelable: true});
    el.dispatchEvent(ev);
    expect(ev.defaultPrevented).toBe(true);
    off();
    const ev2 = new Event("touchmove", {cancelable: true});
    el.dispatchEvent(ev2);
    expect(ev2.defaultPrevented).toBe(false);
  });
});

describe("pickView", () => {
  it("restores a remembered camera if the robot still has it", () => {
    expect(pickView("cam:left", ["frontleft", "left"])).toEqual({kind: "cam", cam: "left"});
    expect(pickView("cam:gone", ["frontleft", "left"])).toEqual({kind: "cam", cam: "frontleft"});
    expect(pickView("all", ["a", "b"])).toEqual({kind: "all"});
    expect(pickView("all", ["a"])).toEqual({kind: "cam", cam: "a"});
    expect(pickView("map", ["a"])).toEqual({kind: "map"});
    expect(pickView(null, [])).toEqual({kind: "map"});
  });
});

// --- component: arm, two fingers, safety ---------------------------------
const sockets: FakeWs[] = [];
class FakeWs {
  readyState = 0;
  sent: Record<string, unknown>[] = [];
  onopen: ((ev: Event) => void) | null = null;
  onmessage: ((ev: MessageEvent) => void) | null = null;
  onclose: ((ev: CloseEvent) => void) | null = null;
  onerror: ((ev: Event) => void) | null = null;
  closed = false;
  constructor(public url: string) {
    sockets.push(this);
    queueMicrotask(() => {
      this.readyState = 1;
      this.onopen?.(new Event("open"));
    });
  }
  send(s: string) {
    this.sent.push(JSON.parse(s));
  }
  close() {
    this.closed = true;
    this.readyState = 3;
  }
}
const json = (d: unknown) => new Response(JSON.stringify(d), {status: 200, headers: {"Content-Type": "application/json"}});
const teleopState = {active: false, allowed: true, interlock: "", busy: "", standing: true, commanded: {vx: 0, vy: 0, wz: 0}, caps: {vx: 1.6, vy: 0.4, wz: 0.8}, deadman_s: 0.5, cmd_age_s: null, last_reject: "", last_event: "", commands: 0, rejected: 0, plan_executing: false, plan_paused: false, executor: "Idle"};

function pointer(el: Element, type: string, id: number, x: number, y: number) {
  const ev = new MouseEvent(type, {bubbles: true, cancelable: true, clientX: x, clientY: y, button: 0});
  Object.defineProperty(ev, "pointerId", {value: id});
  Object.defineProperty(ev, "pointerType", {value: "touch"});
  act(() => void el.dispatchEvent(ev));
}

describe("DriveFullscreen", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  let posts: string[] = [];
  let exited = 0;

  beforeEach(async () => {
    sockets.length = 0;
    posts = [];
    exited = 0;
    sessionStorage.removeItem(ARM_CONFIRMED_KEY);
    vi.stubGlobal("WebSocket", FakeWs);
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string, init?: RequestInit) => {
        const u = String(url);
        if (init?.method === "POST") {
          posts.push(u);
          return json({ok: true, message: `${u.split("/").pop()} done`});
        }
        if (u.startsWith("api/plan/robots")) return json([{robot: "hamilton", plugins: []}]);
        if (u.includes("/cameras")) return json({cameras: []}); // -> map view, no video in jsdom
        if (u.includes("/teleop")) return json({robot: "hamilton", state: teleopState, lease: null, link: null, ros: true});
        return json({});
      }),
    );
    useRobots.setState({state: {}, link: {}, plan: {}, teleop: {}});
    useFleet.setState({machines: {network: "test", machines: [{name: "hamilton", role: "robot", ip: "", user: ""}]}} as never);
    act(() => useViewer.getState().setFocus("hamilton"));
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    await act(async () => root!.render(<DriveFullscreen robot="hamilton" onExit={() => exited++} />));
    await act(async () => new Promise(r => setTimeout(r, 20)));
    const z = zone();
    z.getBoundingClientRect = () => ({left: 0, top: 0, width: 800, height: 400, right: 800, bottom: 400, x: 0, y: 0, toJSON: () => ({})});
  });
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    act(() => useViewer.getState().setFocus("all"));
    useViewer.setState({driveFull: false});
    vi.unstubAllGlobals();
  });

  const zone = () => document.querySelector('[data-testid="drive-full-zone"]') as HTMLDivElement;
  const arm = () => document.querySelector('[data-testid="drive-full-arm"]') as HTMLButtonElement;
  const cmds = () => sockets.flatMap(s => s.sent).filter(m => m.type === "cmd");
  const armNow = async () => {
    act(() => arm().click());
    const yes = document.querySelector('[data-testid="drive-full-arm-yes"]') as HTMLButtonElement | null;
    if (yes) await act(async () => yes.click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
  };

  it("sticks do nothing until armed; arming asks once", async () => {
    pointer(zone(), "pointerdown", 1, 100, 300);
    pointer(zone(), "pointermove", 1, 100, 100);
    expect(sockets.length).toBe(0);
    expect(document.body.textContent).toContain("Tap Drive to arm");
    pointer(zone(), "pointerup", 1, 100, 100);
    act(() => arm().click());
    expect(document.body.textContent).toContain("Drive hamilton?");
    await act(async () => (document.querySelector('[data-testid="drive-full-arm-yes"]') as HTMLButtonElement).click());
    expect(arm().textContent).toBe("ARMED");
  });

  it("two fingers drive and turn at once; releasing each zeroes its axis", async () => {
    await armNow();
    pointer(zone(), "pointerdown", 1, 150, 300); // left: move
    pointer(zone(), "pointerdown", 2, 650, 300); // right: turn
    pointer(zone(), "pointermove", 1, 150, 0); // full forward
    pointer(zone(), "pointermove", 2, 450, 300); // full left
    // non-edge changes go out on the next 10 Hz tick
    await act(async () => new Promise(r => setTimeout(r, 150)));
    let c = cmds().at(-1)!;
    expect(c.vx as number).toBeCloseTo(CAPS.vx * 0.25); // 25 % default speed
    expect(c.wz as number).toBeCloseTo(CAPS.wz * 0.25);
    pointer(zone(), "pointerup", 1, 150, 0);
    await act(async () => new Promise(r => setTimeout(r, 150)));
    c = cmds().at(-1)!;
    expect(c.vx).toBe(0);
    expect(c.wz as number).toBeCloseTo(CAPS.wz * 0.25);
    pointer(zone(), "pointercancel", 2, 450, 300);
    expect(cmds().at(-1)).toMatchObject({vx: 0, vy: 0, wz: 0});
  });

  it("hiding the page disarms mid-drive; exit disarms and leaves", async () => {
    await armNow();
    pointer(zone(), "pointerdown", 1, 150, 300);
    pointer(zone(), "pointermove", 1, 150, 0);
    expect(cmds().at(-1)!.vx as number).toBeGreaterThan(0);
    Object.defineProperty(document, "visibilityState", {value: "hidden", configurable: true});
    act(() => void document.dispatchEvent(new Event("visibilitychange")));
    Object.defineProperty(document, "visibilityState", {value: "visible", configurable: true});
    expect(arm().textContent).toBe("Drive");
    expect(sockets[0].sent.at(-1)).toMatchObject({type: "release"});
    expect(sockets[0].closed).toBe(true);
    // a finger still down after the disarm does not drive
    pointer(zone(), "pointermove", 1, 150, 10);
    expect(sockets.length).toBe(1);
    await armNow();
    act(() => (document.querySelector('[aria-label="Exit full-screen drive"]') as HTMLButtonElement).click());
    expect(exited).toBe(1);
    expect(sockets[1].sent.at(-1)).toMatchObject({type: "release"});
  });

  it("STOP disarms and posts stop", async () => {
    await armNow();
    await act(async () => (document.querySelector('[data-testid="drive-full-stop"]') as HTMLButtonElement).click());
    expect(arm().textContent).toBe("Drive");
    expect(posts.some(p => p.endsWith("/teleop/stop"))).toBe(true);
  });

  it("unmounting while armed releases the robot", async () => {
    await armNow();
    pointer(zone(), "pointerdown", 1, 150, 300);
    pointer(zone(), "pointermove", 1, 150, 0);
    act(() => root!.unmount());
    root = null;
    expect(sockets[0].sent.at(-1)).toMatchObject({type: "release"});
    expect(sockets[0].closed).toBe(true);
  });
});

describe("Teleop tab stands down for the full-screen view", () => {
  it("opening full-screen drive disarms the tab and blocks arming it", async () => {
    sockets.length = 0;
    vi.stubGlobal("WebSocket", FakeWs);
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string) => {
        const u = String(url);
        if (u.startsWith("api/plan/robots")) return json([{robot: "hamilton", plugins: []}]);
        if (u.includes("/teleop")) return json({robot: "hamilton", state: teleopState, lease: null, link: null, ros: true});
        return json({});
      }),
    );
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    useRobots.setState({state: {}, link: {}, plan: {}, teleop: {}});
    act(() => useViewer.getState().setFocus("hamilton"));
    const host = document.createElement("div");
    document.body.appendChild(host);
    const root = createRoot(host);
    await act(async () => root.render(<TeleopPanel />));
    await act(async () => new Promise(r => setTimeout(r, 20)));
    const drive = () => document.querySelector('[data-testid="teleop-drive"]') as HTMLButtonElement;
    act(() => drive().click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    expect(document.querySelector(".teleop-armed")).not.toBeNull();
    act(() => useViewer.setState({driveFull: true}));
    expect(document.querySelector(".teleop-armed")).toBeNull();
    expect(sockets[0].sent.at(-1)).toMatchObject({type: "release"});
    act(() => drive().click());
    expect(document.querySelector(".teleop-armed")).toBeNull();
    act(() => root.unmount());
    host.remove();
    useViewer.setState({driveFull: false});
    act(() => useViewer.getState().setFocus("all"));
    vi.unstubAllGlobals();
  });
});
