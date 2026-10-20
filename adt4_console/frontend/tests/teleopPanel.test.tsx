import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {TeleopPanel} from "../src/viewer/TeleopPanel";
import {ARM_CONFIRMED_KEY} from "../src/lib/teleop";
import {useViewer} from "../src/stores/viewer";
import {useRobots} from "../src/stores/robots";
import {useFleet} from "../src/stores/fleet";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

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

describe("TeleopPanel", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  let posts: string[] = [];

  beforeEach(async () => {
    sockets.length = 0;
    posts = [];
    try {
      sessionStorage.removeItem(ARM_CONFIRMED_KEY);
    } catch {
      /* */
    }
    vi.stubGlobal("WebSocket", FakeWs);
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string, init?: RequestInit) => {
        const u = String(url);
        if (init?.method === "POST") {
          posts.push(u);
          return json({ok: true, message: `${u.split("/").pop()} done`});
        }
        if (u.startsWith("api/plan/robots")) return json([{robot: "hamilton", plugins: []}, {robot: "euclid", plugins: []}]);
        if (u.includes("/teleop"))
          return json({robot: "hamilton", state: {active: false, allowed: true, interlock: "", busy: "", standing: true, commanded: {vx: 0, vy: 0, wz: 0}, caps: {vx: 0.6, vy: 0.4, wz: 0.8}, deadman_s: 0.5, cmd_age_s: null, last_reject: "", last_event: "", commands: 0, rejected: 0, plan_executing: false, plan_paused: false, executor: "Idle"}, lease: null, link: null, ros: true});
        if (u.startsWith("api/time")) return json({t: Date.now() / 1000});
        return json({});
      }),
    );
    useRobots.setState({state: {}, link: {}, plan: {}, teleop: {}});
    const m = (name: string) => ({name, role: "robot", ip: "", user: ""});
    useFleet.setState({machines: {network: "test", machines: [m("hamilton"), m("euclid"), m("lewis")]}});
    act(() => useViewer.getState().setFocus("hamilton"));
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    await act(async () => root!.render(<TeleopPanel />));
    await act(async () => new Promise(r => setTimeout(r, 20)));
  });
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    act(() => useViewer.getState().setFocus("all"));
    vi.unstubAllGlobals();
  });

  const panel = () => document.querySelector(".teleop-panel") as HTMLDivElement;
  const drive = () => document.querySelector('[data-testid="teleop-drive"]') as HTMLButtonElement;
  const key = (type: "keydown" | "keyup", code: string, extra: KeyboardEventInit = {}) =>
    act(() => {
      panel().dispatchEvent(new KeyboardEvent(type, {code, key: code.replace("Key", "").toLowerCase(), bubbles: true, ...extra}));
    });
  const cmds = () => sockets.flatMap(s => s.sent).filter(m => m.type === "cmd");

  it("shows the robot state and keys do nothing while disarmed", async () => {
    expect(document.body.textContent).toContain("standing");
    panel().focus();
    key("keydown", "KeyW");
    expect(sockets.length).toBe(0);
    expect(document.querySelector(".teleop-armed")).toBeNull();
  });

  it("arming asks once per session, shows ARMED, keys drive, blur disarms", async () => {
    act(() => drive().click());
    expect(document.body.textContent).toContain("Drive hamilton?");
    await act(async () => (document.querySelector('[data-testid="teleop-arm-yes"]') as HTMLButtonElement).click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    expect(document.querySelector(".teleop-armed")?.textContent).toContain("ARMED");
    expect(document.activeElement).toBe(panel());
    key("keydown", "KeyW");
    expect(cmds().at(-1)).toMatchObject({vx: 0.3}); // 50 % default speed
    key("keydown", "KeyQ");
    key("keyup", "KeyW");
    key("keyup", "KeyQ");
    expect(cmds().at(-1)).toMatchObject({vx: 0, vy: 0, wz: 0});
    // focus leaves the panel: disarmed, release sent, socket closed
    const outside = document.createElement("button");
    document.body.appendChild(outside);
    act(() => outside.focus());
    expect(document.querySelector(".teleop-armed")).toBeNull();
    expect(sockets[0].sent.at(-1)).toMatchObject({type: "release"});
    expect(sockets[0].closed).toBe(true);
    // second arm in the same session: no confirmation
    act(() => drive().click());
    expect(document.body.textContent).not.toContain("Drive hamilton?");
    expect(document.querySelector(".teleop-armed")).not.toBeNull();
    outside.remove();
  });

  it("space stops, hiding the page disarms", async () => {
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    act(() => drive().click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    key("keydown", "KeyD", {shiftKey: true});
    expect(cmds().at(-1)?.vy).toBeCloseTo(-0.4 * 0.5 * 0.3);
    key("keydown", "Space");
    expect(cmds().at(-1)).toMatchObject({vx: 0, vy: 0, wz: 0});
    Object.defineProperty(document, "visibilityState", {value: "hidden", configurable: true});
    act(() => void document.dispatchEvent(new Event("visibilitychange")));
    expect(document.querySelector(".teleop-armed")).toBeNull();
    Object.defineProperty(document, "visibilityState", {value: "visible", configurable: true});
  });

  it("buttons post the teleop routes; Stop disarms first", async () => {
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    act(() => drive().click());
    const btn = (label: string) => [...document.querySelectorAll("button")].find(b => b.textContent?.trim() === label) as HTMLButtonElement;
    await act(async () => btn("Stand").click());
    await act(async () => btn("STOP").click());
    expect(document.querySelector(".teleop-armed")).toBeNull();
    for (const c of ["Sit", "Pause", "Resume"]) await act(async () => btn(c).click());
    expect(posts).toEqual(["stand", "stop", "sit", "pause", "resume"].map(c => `api/robots/hamilton/teleop/${c}`));
  });

  it("drives only the focused robot: all robots shows a picker and opens no socket", async () => {
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    act(() => useViewer.getState().setFocus("all"));
    await act(async () => new Promise(r => setTimeout(r, 0)));
    expect(document.body.textContent).toContain("Pick a robot to drive");
    expect(drive()).toBeNull();
    panel().focus();
    key("keydown", "KeyW");
    expect(sockets.length).toBe(0);
    // a pick button focuses that robot (not arming it)
    act(() => (document.querySelector('[data-testid="teleop-pick-euclid"]') as HTMLButtonElement).click());
    expect(useViewer.getState().focus).toBe("euclid");
    expect(useViewer.getState().planRobot).toBe("euclid");
    expect(document.querySelector(".teleop-armed")).toBeNull();
    expect(drive()).not.toBeNull();
    expect(sockets.length).toBe(0);
  });

  it("changing the focus while armed disarms first: zero + release, socket closed", async () => {
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    act(() => drive().click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    key("keydown", "KeyW");
    expect(cmds().at(-1)).toMatchObject({vx: 0.3});
    expect(sockets[0].url).toContain("/robots/hamilton/teleop/ws");
    act(() => useViewer.getState().setFocus("euclid"));
    const sent = sockets[0].sent;
    expect(sent.at(-2)).toMatchObject({type: "cmd", vx: 0, vy: 0, wz: 0});
    expect(sent.at(-1)).toMatchObject({type: "release"});
    expect(sockets[0].closed).toBe(true);
    expect(document.querySelector(".teleop-armed")).toBeNull();
    expect(document.body.textContent).toContain("Disarmed: robot focus changed");
    // the new robot is not armed and gets nothing until Drive is pressed again
    key("keydown", "KeyW");
    expect(sockets.length).toBe(1);
    act(() => drive().click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    expect(sockets.length).toBe(2);
    expect(sockets[1].url).toContain("/robots/euclid/teleop/ws");
  });

  it("focus to all robots while armed disarms too", async () => {
    sessionStorage.setItem(ARM_CONFIRMED_KEY, "1");
    act(() => drive().click());
    await act(async () => new Promise(r => setTimeout(r, 0)));
    act(() => useViewer.getState().setFocus("all"));
    expect(sockets[0].sent.at(-1)).toMatchObject({type: "release"});
    expect(sockets[0].closed).toBe(true);
    expect(document.body.textContent).toContain("Pick a robot to drive");
  });

  it("a focused robot without an executor here drives nothing", async () => {
    act(() => useViewer.getState().setFocus("lewis"));
    expect(document.body.textContent).toContain("lewis has no executor here");
    expect(drive()).toBeNull();
  });
});
