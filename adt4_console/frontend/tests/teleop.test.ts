import {describe, expect, it} from "vitest";
import {
  ARM_CONFIRMED_KEY,
  CAPS,
  SEND_HZ,
  SLOW,
  TeleopDriver,
  armConfirmed,
  clampTwist,
  joystickToTwist,
  keysToTwist,
  poseRate,
  setArmConfirmed,
  teleopWsUrl,
  yawOf,
  type DriverDeps,
} from "../src/lib/teleop";
import {PANE_TABS, popOut, defaultLayout, sanitizeLayout} from "../src/viewer/paneLayout";
import {COMMON_HELP} from "../src/viewer/controlsHelp";

describe("key mapping", () => {
  it("maps WASD QE to body velocity at the slider speed", () => {
    expect(keysToTwist(["KeyW"], 1)).toEqual({vx: CAPS.vx, vy: 0, wz: 0});
    expect(keysToTwist(["KeyS"], 0.5)).toEqual({vx: -CAPS.vx / 2, vy: 0, wz: 0});
    expect(keysToTwist(["KeyA"], 1).vy).toBe(CAPS.vy); // +y = left
    expect(keysToTwist(["KeyD"], 1).vy).toBe(-CAPS.vy);
    expect(keysToTwist(["KeyQ"], 1).wz).toBe(CAPS.wz); // +wz = turn left
    expect(keysToTwist(["KeyE"], 1).wz).toBe(-CAPS.wz);
  });
  it("opposite keys cancel, combos add axes, other keys are ignored", () => {
    expect(keysToTwist(["KeyW", "KeyS"], 1).vx).toBe(0);
    expect(keysToTwist(["KeyW", "KeyA", "KeyE", "KeyX"], 1)).toEqual({vx: CAPS.vx, vy: CAPS.vy, wz: -CAPS.wz});
  });
  it("shift is slow; speed is clamped to [0, 1] of the caps", () => {
    expect(keysToTwist(["KeyW"], 1, true).vx).toBeCloseTo(CAPS.vx * SLOW);
    expect(keysToTwist(["KeyW"], 5).vx).toBe(CAPS.vx);
    expect(keysToTwist(["KeyW"], -1).vx === 0).toBe(true);
  });
  it("never exceeds the caps", () => {
    expect(clampTwist({vx: 9, vy: -9, wz: NaN})).toEqual({vx: CAPS.vx, vy: -CAPS.vy, wz: 0});
  });
});

describe("joystick mapping", () => {
  it("has a dead zone", () => {
    expect(joystickToTwist(0.05, 0.05, 1)).toEqual({vx: 0, vy: 0, wz: 0});
  });
  it("up drives forward, sideways turns (or strafes)", () => {
    expect(joystickToTwist(0, 1, 1).vx).toBeCloseTo(CAPS.vx);
    const right = joystickToTwist(1, 0, 1);
    expect(right.wz).toBeCloseTo(-CAPS.wz);
    expect(right.vy).toBe(0);
    const strafe = joystickToTwist(-1, 0, 0.5, true);
    expect(strafe.vy).toBeCloseTo(CAPS.vy / 2);
    expect(strafe.wz).toBe(0);
  });
  it("is capped at full deflection", () => {
    const t = joystickToTwist(5, 5, 1);
    expect(Math.abs(t.vx)).toBeLessThanOrEqual(CAPS.vx);
    expect(Math.abs(t.wz)).toBeLessThanOrEqual(CAPS.wz);
  });
});

class FakeSocket {
  sent: Record<string, unknown>[] = [];
  readyState = 0;
  closed = false;
  onopen: ((ev: Event) => void) | null = null;
  onmessage: ((ev: MessageEvent) => void) | null = null;
  onclose: ((ev: CloseEvent) => void) | null = null;
  onerror: ((ev: Event) => void) | null = null;
  constructor(public url: string) {}
  open() {
    this.readyState = 1;
    this.onopen?.(new Event("open"));
  }
  send(s: string) {
    this.sent.push(JSON.parse(s));
  }
  close() {
    this.closed = true;
    this.readyState = 3;
  }
  reply(m: unknown) {
    this.onmessage?.({data: JSON.stringify(m)} as MessageEvent);
  }
}

function harness() {
  const sockets: FakeSocket[] = [];
  const timers = new Map<number, {fn: () => void; ms: number}>();
  let id = 0;
  let now = 1000;
  const deps: DriverDeps = {
    makeSocket: url => {
      const s = new FakeSocket(url);
      sockets.push(s);
      return s as unknown as ReturnType<DriverDeps["makeSocket"]>;
    },
    now: () => now,
    setInterval: (fn, ms) => {
      timers.set(++id, {fn, ms});
      return id;
    },
    clearInterval: h => void timers.delete(h as number),
  };
  const fire = (ms: number) => {
    for (const t of [...timers.values()]) if (t.ms === ms) t.fn();
  };
  return {sockets, timers, deps, fire, advance: (d: number) => (now += d)};
}

describe("TeleopDriver", () => {
  it("sends nothing until armed", () => {
    const h = harness();
    const d = new TeleopDriver("ws://x/api/robots/hamilton/teleop/ws", undefined, h.deps);
    d.command({vx: 0.3, vy: 0, wz: 0});
    expect(h.sockets.length).toBe(0);
    expect(h.timers.size).toBe(0);
  });

  it("streams at SEND_HZ while armed, clamped", () => {
    const h = harness();
    const d = new TeleopDriver("ws://x", undefined, h.deps);
    d.arm();
    const s = h.sockets[0];
    s.open();
    expect([...h.timers.values()].some(t => t.ms === 1000 / SEND_HZ)).toBe(true);
    d.command({vx: 2, vy: 0, wz: 0}); // edge: sent at once
    h.fire(1000 / SEND_HZ);
    h.fire(1000 / SEND_HZ);
    const cmds = s.sent.filter(m => m.type === "cmd");
    expect(cmds.length).toBe(3);
    expect(cmds.every(m => m.vx === CAPS.vx)).toBe(true);
  });

  it("disarm sends zero + release, closes, stops the timers", () => {
    const h = harness();
    const d = new TeleopDriver("ws://x", undefined, h.deps);
    d.arm();
    const s = h.sockets[0];
    s.open();
    d.command({vx: 0.3, vy: 0, wz: 0});
    d.disarm();
    expect(s.sent.at(-2)).toMatchObject({type: "cmd", vx: 0, vy: 0, wz: 0});
    expect(s.sent.at(-1)).toMatchObject({type: "release"});
    expect(s.closed).toBe(true);
    expect(h.timers.size).toBe(0);
    const n = s.sent.length;
    d.command({vx: 0.3, vy: 0, wz: 0});
    expect(s.sent.length).toBe(n);
  });

  it("measures the round trip from echoed t and reports errors", () => {
    const h = harness();
    const seen: (string | null)[] = [];
    const d = new TeleopDriver("ws://x", st => seen.push(st.error), h.deps);
    d.arm();
    const s = h.sockets[0];
    s.open();
    const ping = s.sent.find(m => m.type === "ping")!;
    h.advance(40);
    s.reply({type: "pong", t: ping.t});
    expect(d.status.rttMs).toBeCloseTo(40);
    s.reply({type: "error", error: "hamilton is being driven by another console session"});
    expect(d.status.error).toMatch(/another console/);
  });

  it("reconnects on the next tick when the socket drops while armed", () => {
    const h = harness();
    const d = new TeleopDriver("ws://x", undefined, h.deps);
    d.arm();
    h.sockets[0].open();
    h.sockets[0].onclose?.({} as CloseEvent);
    h.fire(1000 / SEND_HZ);
    expect(h.sockets.length).toBe(2);
  });
});

describe("helpers", () => {
  it("arm confirmation is per session and survives broken storage", () => {
    const m = new Map<string, string>();
    const s = {getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, v: string) => void m.set(k, v)};
    expect(armConfirmed(s)).toBe(false);
    setArmConfirmed(s);
    expect(m.get(ARM_CONFIRMED_KEY)).toBe("1");
    expect(armConfirmed(s)).toBe(true);
    const bad = {getItem: () => { throw new Error("x"); }, setItem: () => { throw new Error("x"); }};
    expect(armConfirmed(bad)).toBe(false);
    expect(() => setArmConfirmed(bad)).not.toThrow();
  });
  it("builds the socket URL next to the page", () => {
    expect(teleopWsUrl("ham ilton", {protocol: "https:", host: "h:1", href: "https://h:1/console/viewer"})).toBe("wss://h:1/console/api/robots/ham%20ilton/teleop/ws");
  });
  it("pose rate", () => {
    expect(poseRate({x: 0, y: 0, yaw: 3.1, t: 0}, {x: 0.3, y: 0.4, yaw: -3.1, t: 1})!.v).toBeCloseTo(0.5);
    expect(poseRate({x: 0, y: 0, yaw: 3.1, t: 0}, {x: 0, y: 0, yaw: -3.1, t: 1})!.wz).toBeCloseTo(2 * Math.PI - 6.2);
    expect(poseRate(null, {x: 0, y: 0, yaw: 0, t: 1})).toBeNull();
    expect(yawOf({qw: Math.cos(0.25), qx: 0, qy: 0, qz: Math.sin(0.25)})).toBeCloseTo(0.5);
  });
});

describe("pane + help wiring", () => {
  it("Teleop is a pane tab that can float", () => {
    expect(PANE_TABS).toContain("teleop");
    const l = popOut(defaultLayout(), "teleop", 1600, 900);
    expect(sanitizeLayout(JSON.parse(JSON.stringify(l))).windows.teleop).toBeTruthy();
  });
  it("the legend documents the drive keys", () => {
    const sec = COMMON_HELP.find(s => /Teleop/.test(s.title))!;
    const keys = sec.entries.map(e => e.keys).join(" | ");
    for (const k of ["W / S", "A / D", "Q / E", "Shift", "Space", "Drive"]) expect(keys).toContain(k);
  });
});
