// Spot teleop (Teleop tab): key / joystick -> body velocity, and the 10 Hz
// velocity stream to the console (/api/robots/<r>/teleop/ws). Pure logic, no
// React, so the safety rules are unit tested:
//  * velocities never exceed CAPS (the console and the robot clamp again);
//  * the stream runs only while armed; disarming sends a release (the console
//    zeros a moving robot) and closes the socket;
//  * the console zeros the robot itself when the stream stops (its deadman).
import {withToken} from "./auth";

/** Robot hard caps (spot_executor.teleop.HARD_MAX); body frame: +x forward,
 *  +y left, +wz counter-clockwise (turn left). */
export const CAPS = {vx: 1.6, vy: 0.4, wz: 0.8};
export const SEND_HZ = 10;
export const PING_MS = 1000;
/** Shift = slow: this fraction of the slider speed. */
export const SLOW = 0.3;
/** Joystick dead zone (fraction of the radius). */
export const DEAD_ZONE = 0.12;
export const ARM_CONFIRMED_KEY = "adt4.teleop.armConfirmed";

export interface Twist {
  vx: number;
  vy: number;
  wz: number;
}
export const ZERO: Twist = {vx: 0, vy: 0, wz: 0};

export const isZero = (t: Twist) => Math.abs(t.vx) < 1e-3 && Math.abs(t.vy) < 1e-3 && Math.abs(t.wz) < 1e-3;
const clamp = (v: number, cap: number) => Math.max(-cap, Math.min(cap, Number.isFinite(v) ? v : 0));

export function clampTwist(t: Twist): Twist {
  return {vx: clamp(t.vx, CAPS.vx), vy: clamp(t.vy, CAPS.vy), wz: clamp(t.wz, CAPS.wz)};
}

/** Drive keys (KeyboardEvent.code) -> axis contributions. */
export const DRIVE_KEYS: Record<string, Partial<Record<keyof Twist, number>>> = {
  KeyW: {vx: 1},
  KeyS: {vx: -1},
  KeyA: {vy: 1},
  KeyD: {vy: -1},
  KeyQ: {wz: 1},
  KeyE: {wz: -1},
};
export const isDriveKey = (code: string) => code in DRIVE_KEYS || code === "Space";

/** Held keys -> velocity. ``speed`` is the slider fraction (0..1] of CAPS;
 *  ``slow`` (Shift) scales it by SLOW. Opposite keys cancel. */
export function keysToTwist(held: Iterable<string>, speed: number, slow = false): Twist {
  const s = Math.max(0, Math.min(1, speed)) * (slow ? SLOW : 1);
  const axis = {vx: 0, vy: 0, wz: 0};
  for (const code of held) {
    const d = DRIVE_KEYS[code];
    if (!d) continue;
    for (const k of Object.keys(d) as (keyof Twist)[]) axis[k] += d[k] ?? 0;
  }
  return clampTwist({
    vx: Math.sign(axis.vx) * CAPS.vx * s,
    vy: Math.sign(axis.vy) * CAPS.vy * s,
    wz: Math.sign(axis.wz) * CAPS.wz * s,
  });
}

/** On-screen joystick: ``x``/``y`` in [-1, 1] (right / up positive). Up drives
 *  forward; left/right turns, or strafes in ``strafe`` mode. Dead zone, then
 *  linear to the slider speed. */
export function joystickToTwist(x: number, y: number, speed: number, strafe = false): Twist {
  const r = Math.hypot(x, y);
  if (!(r > DEAD_ZONE)) return {...ZERO};
  const k = Math.min(1, (r - DEAD_ZONE) / (1 - DEAD_ZONE)) / r; // rescale past the dead zone
  const s = Math.max(0, Math.min(1, speed));
  const fx = x * k;
  const fy = y * k;
  return clampTwist({
    vx: fy * CAPS.vx * s,
    vy: strafe ? -fx * CAPS.vy * s : 0,
    wz: strafe ? 0 : -fx * CAPS.wz * s,
  });
}

/** A stick deflection: ``x``/``y`` in [-1, 1] (right / up positive). */
export interface Stick {
  x: number;
  y: number;
}

/** Deflection of a touch at ``p`` on a stick centred at ``origin`` with
 *  ``radius`` px (screen y grows down). Clamped to the unit circle; an
 *  x-only stick (the turn stick) ignores vertical motion. */
export function stickVector(origin: {x: number; y: number}, p: {x: number; y: number}, radius: number, xOnly = false): Stick {
  let x = (p.x - origin.x) / radius;
  let y = xOnly ? 0 : -(p.y - origin.y) / radius;
  if (!Number.isFinite(x) || !Number.isFinite(y)) return {x: 0, y: 0};
  const n = Math.hypot(x, y);
  if (n > 1) {
    x /= n;
    y /= n;
  }
  return {x: x + 0, y: y + 0}; // no -0
}

/** Axis dead zone, then linear to full scale. */
const deadZone = (v: number) => {
  const a = Math.min(1, Math.abs(v));
  return a <= DEAD_ZONE ? 0 : (Math.sign(v) * (a - DEAD_ZONE)) / (1 - DEAD_ZONE);
};

/** Two sticks like Spot's tablet app: ``move`` drives forward/back (up/down)
 *  and strafes (sideways); ``turn`` yaws (sideways; right turns right). Axis
 *  dead zones, so a slightly crooked push forward does not also strafe. */
export function sticksToTwist(move: Stick | null, turn: Stick | null, speed: number): Twist {
  const s = Math.max(0, Math.min(1, speed));
  const vx = move ? deadZone(move.y) * CAPS.vx * s : 0;
  const vy = move ? -deadZone(move.x) * CAPS.vy * s : 0;
  const wz = turn ? -deadZone(turn.x) * CAPS.wz * s : 0;
  return clampTwist({vx: vx || 0, vy: vy || 0, wz: wz || 0});
}

export type StickId = "move" | "turn";

interface Grab {
  id: StickId;
  origin: {x: number; y: number};
  value: Stick;
}

/** Multi-touch for the two sticks. A touch landing in the left half of the
 *  area grabs ``move``, the right half ``turn``; the stick centres where the
 *  finger landed (nothing small to aim at). One finger per stick: a second
 *  finger on a held stick is ignored, and lifting a finger releases only its
 *  own stick. */
export class StickTracker {
  private grabs = new Map<number, Grab>();

  constructor(public radius: number) {}

  /** ``x``/``y`` relative to the touch area, ``width`` its width (px). */
  down(pointerId: number, x: number, y: number, width: number): StickId | null {
    const held = this.grabs.get(pointerId);
    if (held) return held.id;
    const id: StickId = x < width / 2 ? "move" : "turn";
    for (const g of this.grabs.values()) if (g.id === id) return null;
    this.grabs.set(pointerId, {id, origin: {x, y}, value: {x: 0, y: 0}});
    return id;
  }

  move(pointerId: number, x: number, y: number): StickId | null {
    const g = this.grabs.get(pointerId);
    if (!g) return null;
    g.value = stickVector(g.origin, {x, y}, this.radius, g.id === "turn");
    return g.id;
  }

  up(pointerId: number): StickId | null {
    const g = this.grabs.get(pointerId);
    this.grabs.delete(pointerId);
    return g?.id ?? null;
  }

  clear() {
    this.grabs.clear();
  }

  get size() {
    return this.grabs.size;
  }

  get(id: StickId): {origin: {x: number; y: number}; value: Stick} | null {
    for (const g of this.grabs.values()) if (g.id === id) return {origin: g.origin, value: g.value};
    return null;
  }

  twist(speed: number): Twist {
    return sticksToTwist(this.get("move")?.value ?? null, this.get("turn")?.value ?? null, speed);
  }
}

const TOUCH_EVENTS = ["touchstart", "touchmove", "touchend", "gesturestart", "gesturechange"];

/** Keep the browser from turning touches on ``el`` into scrolling, pinch
 *  zoom, selection or synthesized mouse / focus events. iOS Safari does not
 *  reliably honour ``touch-action: none`` inside a scrolling container, and
 *  React's touch listeners are passive (cannot preventDefault), so this is a
 *  native non-passive listener. Pointer events still fire. Returns the
 *  cleanup. */
export function captureTouches(el: HTMLElement): () => void {
  const stop = (e: Event) => {
    if (e.cancelable) e.preventDefault();
  };
  for (const t of TOUCH_EVENTS) el.addEventListener(t, stop, {passive: false});
  return () => {
    for (const t of TOUCH_EVENTS) el.removeEventListener(t, stop);
  };
}

/** Whether arming was confirmed before in this browser session. */
export function armConfirmed(storage: Pick<Storage, "getItem"> | null | undefined = safeSession()): boolean {
  try {
    return storage?.getItem(ARM_CONFIRMED_KEY) === "1";
  } catch {
    return false;
  }
}

export function setArmConfirmed(storage: Pick<Storage, "setItem"> | null | undefined = safeSession()) {
  try {
    storage?.setItem(ARM_CONFIRMED_KEY, "1");
  } catch {
    /* storage blocked: the confirmation is asked again next time */
  }
}

function safeSession(): Storage | null {
  try {
    return typeof window !== "undefined" ? window.sessionStorage : null;
  } catch {
    return null;
  }
}

export function teleopWsUrl(robot: string, loc: Pick<Location, "protocol" | "host" | "href"> = window.location): string {
  const proto = loc.protocol === "https:" ? "wss:" : "ws:";
  const base = new URL(`api/robots/${encodeURIComponent(robot)}/teleop/ws`, new URL(".", loc.href)).pathname;
  return withToken(`${proto}//${loc.host}${base}`);
}

export interface DriverStatus {
  connected: boolean;
  /** browser <-> console round trip, EWMA (ms) */
  rttMs: number | null;
  /** last command the console acknowledged (clamped) */
  acked: Twist | null;
  /** last error from the console (e.g. another session drives) */
  error: string | null;
  sent: number;
}

type WsLike = Pick<WebSocket, "send" | "close" | "readyState"> & {
  onopen: ((ev: Event) => void) | null;
  onmessage: ((ev: MessageEvent) => void) | null;
  onclose: ((ev: CloseEvent) => void) | null;
  onerror: ((ev: Event) => void) | null;
};

export interface DriverDeps {
  makeSocket: (url: string) => WsLike;
  now: () => number;
  setInterval: (fn: () => void, ms: number) => unknown;
  clearInterval: (h: unknown) => void;
}

const defaultDeps = (): DriverDeps => ({
  makeSocket: url => new WebSocket(url) as unknown as WsLike,
  now: () => performance.now(),
  setInterval: (fn, ms) => window.setInterval(fn, ms),
  clearInterval: h => window.clearInterval(h as number),
});

/** The velocity stream of one armed session. ``arm()`` opens the socket and
 *  sends ``command()`` every 1/SEND_HZ s; ``disarm()`` sends a release and
 *  closes it. Never sends while disarmed. */
export class TeleopDriver {
  private ws: WsLike | null = null;
  private tick: unknown = null;
  private ping: unknown = null;
  private seq = 0;
  private cmd: Twist = {...ZERO};
  private armed = false;
  status: DriverStatus = {connected: false, rttMs: null, acked: null, error: null, sent: 0};

  constructor(
    private url: string,
    private onStatus: (s: DriverStatus) => void = () => undefined,
    private deps: DriverDeps = defaultDeps(),
  ) {}

  get isArmed() {
    return this.armed;
  }

  /** The velocity to stream (clamped); sent on the next tick, or at once when
   *  it changes to or from zero so starts and stops are not delayed. */
  command(t: Twist) {
    const next = clampTwist(t);
    const edge = isZero(next) !== isZero(this.cmd);
    this.cmd = next;
    if (this.armed && edge) this.sendCmd();
  }

  arm() {
    if (this.armed) return;
    this.armed = true;
    this.cmd = {...ZERO};
    this.connect();
    this.tick = this.deps.setInterval(() => this.sendCmd(), 1000 / SEND_HZ);
    this.ping = this.deps.setInterval(() => this.send({type: "ping"}), PING_MS);
  }

  disarm() {
    if (!this.armed) return;
    this.armed = false;
    this.cmd = {...ZERO};
    if (this.tick != null) this.deps.clearInterval(this.tick);
    if (this.ping != null) this.deps.clearInterval(this.ping);
    this.tick = this.ping = null;
    this.send({type: "cmd", ...ZERO});
    this.send({type: "release"});
    const ws = this.ws;
    this.ws = null;
    try {
      ws?.close();
    } catch {
      /* already closed */
    }
    this.update({connected: false, acked: null});
  }

  private connect() {
    let ws: WsLike;
    try {
      ws = this.deps.makeSocket(this.url);
    } catch (e) {
      this.update({error: `teleop socket: ${e instanceof Error ? e.message : String(e)}`});
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.update({connected: true, error: null});
      this.send({type: "ping"});
    };
    ws.onmessage = ev => this.onMessage(ev.data);
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.update({connected: false});
      // still armed: reconnect on the next tick
    };
    ws.onerror = () => undefined;
  }

  private onMessage(data: unknown) {
    let m: {type?: string; t?: number; cmd?: Twist; error?: string};
    try {
      m = JSON.parse(String(data));
    } catch {
      return;
    }
    if ((m.type === "pong" || m.type === "ack") && typeof m.t === "number") {
      const rtt = this.deps.now() - m.t;
      if (rtt >= 0 && rtt < 60_000) {
        const prev = this.status.rttMs;
        this.update({rttMs: prev == null ? rtt : prev * 0.7 + rtt * 0.3});
      }
    }
    if (m.type === "ack") this.update({acked: m.cmd ?? null, error: null});
    else if (m.type === "error") this.update({error: m.error ?? "error"});
  }

  private sendCmd() {
    if (!this.armed) return;
    if (!this.ws) this.connect();
    if (this.send({type: "cmd", ...this.cmd})) this.update({sent: this.status.sent + 1});
  }

  private send(m: Record<string, unknown>): boolean {
    const ws = this.ws;
    if (!ws || ws.readyState !== 1) return false;
    try {
      ws.send(JSON.stringify({...m, seq: ++this.seq, t: this.deps.now()}));
      return true;
    } catch {
      return false;
    }
  }

  private update(p: Partial<DriverStatus>) {
    this.status = {...this.status, ...p};
    this.onStatus(this.status);
  }
}

/** Planar speed (m/s) and yaw rate (rad/s) from two timestamped poses (seconds). */
export function poseRate(
  a: {x: number; y: number; yaw: number; t: number} | null,
  b: {x: number; y: number; yaw: number; t: number} | null,
): {v: number; wz: number} | null {
  if (!a || !b) return null;
  const dt = b.t - a.t;
  if (!(dt > 0.05) || dt > 3) return null;
  let dyaw = b.yaw - a.yaw;
  while (dyaw > Math.PI) dyaw -= 2 * Math.PI;
  while (dyaw < -Math.PI) dyaw += 2 * Math.PI;
  return {v: Math.hypot(b.x - a.x, b.y - a.y) / dt, wz: dyaw / dt};
}

export function yawOf(q: {qw: number; qx: number; qy: number; qz: number}): number {
  return Math.atan2(2 * (q.qw * q.qz + q.qx * q.qy), 1 - 2 * (q.qy * q.qy + q.qz * q.qz));
}
