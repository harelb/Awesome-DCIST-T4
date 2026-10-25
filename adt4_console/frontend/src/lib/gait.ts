// Procedural walking for the viewer's robot models: Spot's leg kinematics (the
// joint origins scripts/bake_robot_models.py stands the model with), a trot whose
// feet stay on the ground (2-link IK per leg), the Husky's wheel spin, and a pose
// track that smooths ~5 Hz pose updates into per-frame positions and a ground speed.
//
// The GLB parts are baked in the standing pose (body frame), so a part is moved by
// ``M(angles) * M(stand)^-1``: its geometry and outline hull move rigidly with it.
import * as THREE from "three";

export type LegName = "front_left" | "front_right" | "rear_left" | "rear_right";
export const SPOT_LEGS: LegName[] = ["front_left", "front_right", "rear_left", "rear_right"];

export interface LegAngles {
  /** abduction (about x) */
  hipX: number;
  /** hip pitch (about y; + swings the leg back) */
  hipY: number;
  /** knee (about y; Spot's knee bends back, so more negative folds the leg) */
  knee: number;
}

// spot_macro.xacro joint origins (as in bake_robot_models.py)
const SPOT_HIP: Record<LegName, [number, number]> = {
  front_left: [0.29785, 0.055],
  front_right: [0.29785, -0.055],
  rear_left: [-0.29785, 0.055],
  rear_right: [-0.29785, -0.055],
};
const SPOT_HIP_Y_OFFSET = 0.110945;
const SPOT_KNEE: [number, number] = [0.025, -0.3205]; // (x, z) in the upper leg
const SPOT_FOOT: [number, number] = [0, -0.3365]; // (x, z) in the lower leg
export const SPOT_STAND: LegAngles = {hipX: 0, hipY: 0.8, knee: -1.6};

/** World transforms (body frame) of a leg's hip, upper and lower leg. */
export function spotLegMatrices(leg: LegName, a: LegAngles): {hip: THREE.Matrix4; upper: THREE.Matrix4; lower: THREE.Matrix4} {
  const [hx, hy] = SPOT_HIP[leg];
  const side = leg.endsWith("left") ? 1 : -1;
  const hip = new THREE.Matrix4().makeTranslation(hx, hy, 0).multiply(new THREE.Matrix4().makeRotationX(a.hipX));
  const upper = hip
    .clone()
    .multiply(new THREE.Matrix4().makeTranslation(0, side * SPOT_HIP_Y_OFFSET, 0))
    .multiply(new THREE.Matrix4().makeRotationY(a.hipY));
  const lower = upper
    .clone()
    .multiply(new THREE.Matrix4().makeTranslation(SPOT_KNEE[0], 0, SPOT_KNEE[1]))
    .multiply(new THREE.Matrix4().makeRotationY(a.knee));
  return {hip, upper, lower};
}

// three's rotation about y maps (x, z) to (x c + z s, -x s + z c): the (x, z)
// polar angle goes down by the rotation angle.
const rotY = ([x, z]: [number, number], t: number): [number, number] => [x * Math.cos(t) + z * Math.sin(t), -x * Math.sin(t) + z * Math.cos(t)];
const polar = ([x, z]: [number, number]) => Math.atan2(z, x);

/** Foot (x, z) relative to the hip-pitch joint for (hipY, knee). */
export function spotFootFK(hipY: number, knee: number): [number, number] {
  const k = rotY(SPOT_KNEE, hipY);
  const f = rotY(SPOT_FOOT, hipY + knee);
  return [k[0] + f[0], k[1] + f[1]];
}

const A_LEN = Math.hypot(...SPOT_KNEE);
const B_LEN = Math.hypot(...SPOT_FOOT);
const A_ANG = polar(SPOT_KNEE);
const B_ANG = polar(SPOT_FOOT);

/** (hipY, knee) putting the foot at ``p`` (x, z from the hip-pitch joint), the
 *  knee-back branch; out-of-reach targets are pulled onto the reachable shell. */
export function spotLegIK(p: [number, number]): {hipY: number; knee: number} {
  let d = Math.hypot(p[0], p[1]);
  const lo = Math.abs(A_LEN - B_LEN) + 1e-4;
  const hi = A_LEN + B_LEN - 1e-4;
  const dir = d > 1e-9 ? [p[0] / d, p[1] / d] : [0, -1];
  d = Math.min(hi, Math.max(lo, d));
  // |a + R(knee) b| = d  with  a.R(knee)b = |a||b| cos(A_ANG - B_ANG + knee)
  const c = (d * d - A_LEN * A_LEN - B_LEN * B_LEN) / (2 * A_LEN * B_LEN);
  const knee = -Math.acos(Math.max(-1, Math.min(1, c))) - A_ANG + B_ANG;
  const kb = rotY(SPOT_FOOT, knee);
  const v: [number, number] = [SPOT_KNEE[0] + kb[0], SPOT_KNEE[1] + kb[1]];
  let hipY = polar(v) - Math.atan2(dir[1], dir[0]);
  hipY = Math.atan2(Math.sin(hipY), Math.cos(hipY));
  return {hipY, knee};
}

// ------------------------------------------------------------------ the trot

export const GAIT = {
  /** step frequency (Hz) at 1 m/s; scales with sqrt(speed) */
  freqAt1: 1.8,
  minFreq: 1.1,
  maxFreq: 3.0,
  /** ground speed (m/s) below which the robot stands */
  standSpeed: 0.04,
  /** longest stride (m): faster (sped-up sim) feet slide rather than over-reach */
  maxStride: 0.36,
  /** foot lift (m) at full gait */
  lift: 0.075,
  /** body bob (m) at full gait */
  bob: 0.012,
  /** a turn in place reads as this much ground speed per rad/s */
  turnRadius: 0.35,
  /** how fast the gait fades in / out (1/s) */
  blendRate: 5,
  /** low-pass on the pose-derived speed (1/s): poses come in at ~4-5 Hz */
  speedRate: 6,
};

/** Diagonal pairs move together: FL+RR, then FR+RL half a cycle later. */
export const TROT_OFFSET: Record<LegName, number> = {front_left: 0, rear_right: 0, front_right: 0.5, rear_left: 0.5};

/** Step frequency (Hz) for a ground speed (m/s); 0 when standing. */
export function gaitFrequency(speed: number): number {
  const v = Math.abs(speed);
  if (!(v > GAIT.standSpeed)) return 0;
  return Math.min(GAIT.maxFreq, Math.max(GAIT.minFreq, GAIT.freqAt1 * Math.sqrt(v)));
}

export interface GaitPose {
  legs: Record<LegName, LegAngles>;
  /** body z offset (m) */
  bob: number;
}

const STAND_FOOT = spotFootFK(SPOT_STAND.hipY, SPOT_STAND.knee);

/** Foot offset (dx forward, dz up) of a leg at its own phase ``p`` in [0, 1):
 *  stance (first half) drags the foot back along the ground, swing lifts it forward. */
export function footOffset(p: number, stride: number, lift: number): [number, number] {
  p -= Math.floor(p);
  if (p < 0.5) return [stride * (0.5 - p / 0.5), 0];
  const s = (p - 0.5) / 0.5;
  const e = (1 - Math.cos(Math.PI * s)) / 2;
  return [stride * (-0.5 + e), lift * Math.sin(Math.PI * s)];
}

/** The trot at cycle ``phase`` with ``stride`` (m, signed: < 0 walks backwards)
 *  and ``weight`` (0 = standing, 1 = full gait: the lift and bob scale with it). */
export function trotPose(phase: number, stride: number, weight: number): GaitPose {
  const w = Math.max(0, Math.min(1, weight));
  const legs = {} as Record<LegName, LegAngles>;
  for (const leg of SPOT_LEGS) {
    const [dx, dz] = footOffset(phase + TROT_OFFSET[leg], stride, GAIT.lift * w);
    const {hipY, knee} = spotLegIK([STAND_FOOT[0] + dx, STAND_FOOT[1] + dz]);
    legs[leg] = {hipX: 0, hipY, knee};
  }
  // lowest as each diagonal pair lands (twice per cycle)
  const bob = GAIT.bob * w * 0.5 * (1 - Math.cos(4 * Math.PI * phase));
  return {legs, bob};
}

/** A robot's gait over time: ``step`` with the frame's dt and the ground speed. */
export class GaitState {
  /** smoothed forward speed and yaw rate */
  speed = 0;
  yawRate = 0;
  phase = 0;
  weight = 0;
  stride = 0;
  /** wheel angle (rad) for the Husky */
  wheel = 0;

  /** ``speed``: signed forward speed (m/s); ``yawRate``: rad/s. Returns whether
   *  anything still moves (the caller keeps rendering frames while it does). */
  step(dt: number, speed: number, yawRate = 0): boolean {
    dt = Math.max(0, Math.min(dt, 0.25));
    const ks = 1 - Math.exp(-GAIT.speedRate * dt);
    this.speed += (speed - this.speed) * ks;
    this.yawRate += (yawRate - this.yawRate) * ks;
    if (Math.abs(this.speed) < 1e-3 && speed === 0) this.speed = 0;
    if (Math.abs(this.yawRate) < 1e-3 && yawRate === 0) this.yawRate = 0;
    speed = this.speed;
    const eff = Math.hypot(speed, this.yawRate * GAIT.turnRadius);
    const f = gaitFrequency(eff);
    const target = f > 0 ? 1 : 0;
    const k = 1 - Math.exp(-GAIT.blendRate * dt);
    this.weight += (target - this.weight) * k;
    if (this.weight < 1e-3 && target === 0) this.weight = 0;
    // keep cycling while fading out so the feet come down rather than freeze mid-air
    const freq = f > 0 ? f : this.weight > 0 ? GAIT.minFreq : 0;
    this.phase = (this.phase + freq * dt) % 1;
    const strideTarget = f > 0 ? Math.sign(speed) * Math.min(GAIT.maxStride, Math.abs(speed) / (2 * f)) : 0;
    this.stride += (strideTarget - this.stride) * k;
    return this.weight > 0 || this.speed !== 0 || this.yawRate !== 0;
  }

  pose(): GaitPose {
    return trotPose(this.phase, this.stride, this.weight);
  }

  /** Spin the wheels (radius ``r``) for ``speed`` over ``dt``. */
  roll(dt: number, speed: number, r: number): boolean {
    dt = Math.max(0, Math.min(dt, 0.25));
    this.speed += (speed - this.speed) * (1 - Math.exp(-GAIT.speedRate * dt));
    if (Math.abs(this.speed) < 1e-3 && speed === 0) this.speed = 0;
    this.wheel = (this.wheel + (this.speed * dt) / r) % (2 * Math.PI);
    return this.speed !== 0;
  }
}

// ------------------------------------------------------------- the model rigs

/** How a model's parts move: Spot's legs, the Husky's wheels (by GLB part name). */
export type RigKind = "legs" | "wheels";

export function rigKind(parts: string[]): RigKind | null {
  if (SPOT_LEGS.every(l => parts.includes(`${l}_lower_leg`))) return "legs";
  if (parts.some(p => p.endsWith("_wheel"))) return "wheels";
  return null;
}

const STAND_INV: Record<string, THREE.Matrix4> = {};
for (const leg of SPOT_LEGS) {
  const m = spotLegMatrices(leg, SPOT_STAND);
  STAND_INV[`${leg}_hip`] = m.hip.clone().invert();
  STAND_INV[`${leg}_upper_leg`] = m.upper.clone().invert();
  STAND_INV[`${leg}_lower_leg`] = m.lower.clone().invert();
}

/** Each moving Spot part's transform from its baked (standing) placement. */
export function spotPartDeltas(pose: GaitPose): Record<string, THREE.Matrix4> {
  const out: Record<string, THREE.Matrix4> = {};
  for (const leg of SPOT_LEGS) {
    const m = spotLegMatrices(leg, pose.legs[leg]);
    out[`${leg}_hip`] = m.hip.multiply(STAND_INV[`${leg}_hip`]);
    out[`${leg}_upper_leg`] = m.upper.multiply(STAND_INV[`${leg}_upper_leg`]);
    out[`${leg}_lower_leg`] = m.lower.multiply(STAND_INV[`${leg}_lower_leg`]);
  }
  return out;
}

// Husky A200 (bake_robot_models.py): wheel base 0.512, track 0.555, axle z 0.03282
export const HUSKY_WHEEL_R = 0.1778;
const HUSKY_AXLE: Record<string, [number, number, number]> = {
  front_left_wheel: [0.256, 0.2775, 0.03282],
  front_right_wheel: [0.256, -0.2775, 0.03282],
  rear_left_wheel: [-0.256, 0.2775, 0.03282],
  rear_right_wheel: [-0.256, -0.2775, 0.03282],
};

/** Each wheel turned by ``angle`` about its axle (forward roll is + about y). */
export function wheelPartDeltas(angle: number): Record<string, THREE.Matrix4> {
  const out: Record<string, THREE.Matrix4> = {};
  const r = new THREE.Matrix4().makeRotationY(angle);
  for (const [name, [x, y, z]] of Object.entries(HUSKY_AXLE))
    out[name] = new THREE.Matrix4().makeTranslation(x, y, z).multiply(r).multiply(new THREE.Matrix4().makeTranslation(-x, -y, -z));
  return out;
}

// ------------------------------------------------------------- the pose track

export interface TrackSample {
  x: number;
  y: number;
  yaw: number;
  /** signed forward speed (m/s, wall time) */
  speed: number;
  /** rad/s */
  yawRate: number;
  /** still interpolating toward the newest pose */
  moving: boolean;
}

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** Poses arriving every ~200 ms, replayed one update interval late so the drawn
 *  pose is interpolated between real ones. A jump of more than ``snapM`` (a sim
 *  reset or seek) snaps instead of sliding. Times in ms. */
export class PoseTrack {
  private pts: {t: number; x: number; y: number; yaw: number}[] = [];
  /** estimated update interval (ms) */
  interval = 200;

  constructor(
    private snapM = 30,
    private maxInterval = 1000,
  ) {}

  push(t: number, x: number, y: number, yaw: number): void {
    const last = this.pts[this.pts.length - 1];
    if (last && t <= last.t) return;
    if (!last || Math.hypot(x - last.x, y - last.y) > this.snapM || t - last.t > 5 * this.maxInterval) {
      this.pts = [{t, x, y, yaw}];
      return;
    }
    const gap = Math.min(this.maxInterval, t - last.t);
    this.interval += (gap - this.interval) * (this.pts.length < 2 ? 1 : 0.3);
    this.interval = Math.max(30, this.interval);
    this.pts.push({t, x, y, yaw});
    if (this.pts.length > 6) this.pts.shift();
  }

  /** Forget the history: the next pose is drawn as is. */
  reset(): void {
    this.pts = [];
  }

  get empty(): boolean {
    return this.pts.length === 0;
  }

  sample(now: number): TrackSample | null {
    const pts = this.pts;
    if (!pts.length) return null;
    const tr = now - this.interval;
    const last = pts[pts.length - 1];
    if (pts.length === 1 || tr >= last.t) {
      // past the newest pose: hold it (the robot stopped, or an update is late)
      return {x: last.x, y: last.y, yaw: last.yaw, speed: 0, yawRate: 0, moving: false};
    }
    let i = pts.length - 2;
    while (i > 0 && pts[i].t > tr) i--;
    const a = pts[i];
    const b = pts[i + 1];
    const dt = (b.t - a.t) / 1000;
    const u = Math.max(0, Math.min(1, (tr - a.t) / (b.t - a.t)));
    const dyaw = wrap(b.yaw - a.yaw);
    const yaw = wrap(a.yaw + dyaw * u);
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const speed = (dx * Math.cos(yaw) + dy * Math.sin(yaw)) / dt;
    // a held pose re-sent while paused is not motion (no frames needed)
    const moving = pts.slice(i + 1).some(p => p.x !== a.x || p.y !== a.y || p.yaw !== a.yaw);
    return {x: a.x + dx * u, y: a.y + dy * u, yaw, speed, yawRate: dyaw / dt, moving};
  }
}
