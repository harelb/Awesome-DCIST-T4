import {describe, expect, it} from "vitest";
import * as THREE from "three";
import {
  GAIT,
  GaitState,
  HUSKY_WHEEL_R,
  PoseTrack,
  rigKind,
  spotPartDeltas,
  wheelPartDeltas,
  SPOT_LEGS,
  SPOT_STAND,
  footOffset,
  gaitFrequency,
  spotFootFK,
  spotLegIK,
  spotLegMatrices,
  trotPose,
} from "../src/lib/gait";

const STAND_FOOT = spotFootFK(SPOT_STAND.hipY, SPOT_STAND.knee);

/** Foot position (body frame) of a leg for its angles, from the link matrices. */
function footOf(leg: (typeof SPOT_LEGS)[number], hipY: number, knee: number) {
  const {lower} = spotLegMatrices(leg, {hipX: 0, hipY, knee});
  return new THREE.Vector3(0, 0, -0.3365).applyMatrix4(lower);
}

describe("spot leg kinematics", () => {
  it("stands with the feet under the hips, ~0.48 m below the body (as baked)", () => {
    for (const leg of SPOT_LEGS) {
      const f = footOf(leg, SPOT_STAND.hipY, SPOT_STAND.knee);
      const hipX = leg.startsWith("front") ? 0.29785 : -0.29785;
      expect(f.x - hipX).toBeGreaterThan(0); // a few cm ahead of the hip
      expect(f.x - hipX).toBeLessThan(0.05);
      expect(f.z).toBeCloseTo(-0.476, 2);
    }
  });

  it("IK inverts FK, and gives the standing angles back for the standing foot", () => {
    const s = spotLegIK(STAND_FOOT);
    expect(s.hipY).toBeCloseTo(SPOT_STAND.hipY, 5);
    expect(s.knee).toBeCloseTo(SPOT_STAND.knee, 5);
    for (const [dx, dz] of [
      [0.15, 0],
      [-0.15, 0],
      [0.05, 0.08],
      [-0.1, 0.05],
    ]) {
      const p: [number, number] = [STAND_FOOT[0] + dx, STAND_FOOT[1] + dz];
      const a = spotLegIK(p);
      const q = spotFootFK(a.hipY, a.knee);
      expect(q[0]).toBeCloseTo(p[0], 5);
      expect(q[1]).toBeCloseTo(p[1], 5);
      expect(a.knee).toBeLessThan(0); // the knee keeps bending back
    }
  });
});

describe("trot", () => {
  it("stands still at zero speed", () => {
    const g = new GaitState();
    for (let i = 0; i < 10; i++) expect(g.step(1 / 60, 0)).toBe(false);
    const p = g.pose();
    expect(p.bob).toBe(0);
    for (const leg of SPOT_LEGS) {
      expect(p.legs[leg].hipY).toBeCloseTo(SPOT_STAND.hipY, 5);
      expect(p.legs[leg].knee).toBeCloseTo(SPOT_STAND.knee, 5);
    }
  });

  it("moves diagonal pairs together, half a cycle apart", () => {
    for (const phase of [0.1, 0.3, 0.6, 0.85]) {
      const p = trotPose(phase, 0.25, 1);
      expect(p.legs.front_left.hipY).toBeCloseTo(p.legs.rear_right.hipY, 6);
      expect(p.legs.front_right.knee).toBeCloseTo(p.legs.rear_left.knee, 6);
      const q = trotPose(phase + 0.5, 0.25, 1);
      expect(q.legs.front_right.hipY).toBeCloseTo(p.legs.front_left.hipY, 6);
    }
  });

  it("keeps stance feet on the ground and lifts swing feet", () => {
    for (let p = 0; p < 1; p += 0.05) {
      const [, dz] = footOffset(p, 0.3, 0.07);
      if (p < 0.5) expect(dz).toBe(0);
      else expect(dz).toBeGreaterThanOrEqual(0);
    }
    expect(footOffset(0.75, 0.3, 0.07)[1]).toBeCloseTo(0.07, 6);
    // stance drags the foot back (front to back), swing brings it forward
    expect(footOffset(0, 0.3, 0)[0]).toBeCloseTo(0.15, 6);
    expect(footOffset(0.49, 0.3, 0)[0]).toBeLessThan(-0.14);
    // stance feet sit at the standing foot height
    const pose = trotPose(0.2, 0.3, 1);
    const f = spotFootFK(pose.legs.front_left.hipY, pose.legs.front_left.knee);
    expect(f[1]).toBeCloseTo(STAND_FOOT[1], 5);
  });

  it("scales the step frequency with speed, capped", () => {
    expect(gaitFrequency(0)).toBe(0);
    expect(gaitFrequency(0.01)).toBe(0);
    expect(gaitFrequency(1)).toBeGreaterThanOrEqual(1.6);
    expect(gaitFrequency(1)).toBeLessThanOrEqual(2);
    expect(gaitFrequency(0.5)).toBeLessThan(gaitFrequency(1));
    expect(gaitFrequency(1.5)).toBeGreaterThan(gaitFrequency(1));
    expect(gaitFrequency(50)).toBe(GAIT.maxFreq);
    expect(gaitFrequency(-1)).toBe(gaitFrequency(1));
  });

  it("advances the phase at the step frequency and eases back to standing", () => {
    const g = new GaitState();
    for (let i = 0; i < 120; i++) g.step(1 / 60, 1);
    expect(g.weight).toBeGreaterThan(0.99);
    const p0 = g.phase;
    g.step(0.1, 1);
    expect((g.phase - p0 + 1) % 1).toBeCloseTo(gaitFrequency(1) * 0.1, 5);
    // stride ~ v / (2 f), capped when the sim runs fast
    expect(g.stride).toBeCloseTo(1 / (2 * gaitFrequency(1)), 2);
    for (let i = 0; i < 300; i++) g.step(1 / 60, 20);
    expect(g.stride).toBeCloseTo(GAIT.maxStride, 3);
    let moving = true;
    for (let i = 0; i < 300 && moving; i++) moving = g.step(1 / 60, 0);
    expect(moving).toBe(false);
    expect(g.weight).toBe(0);
  });

  it("steps when turning in place", () => {
    const g = new GaitState();
    for (let i = 0; i < 60; i++) g.step(1 / 60, 0, 1.0);
    expect(g.weight).toBeGreaterThan(0.9);
    expect(Math.abs(g.stride)).toBeLessThan(1e-6);
  });

  it("rolls wheels by distance / radius (once the speed has settled)", () => {
    const g = new GaitState();
    for (let i = 0; i < 120; i++) g.roll(1 / 60, 1, 0.2);
    const w0 = g.wheel;
    g.roll(0.1, 1, 0.2);
    expect(g.wheel - w0).toBeCloseTo(0.5, 3);
    let moving = true;
    for (let i = 0; i < 600 && moving; i++) moving = g.roll(1 / 60, 0, 0.2);
    expect(moving).toBe(false);
  });
});

describe("rigs", () => {
  it("knows Spot by its legs and the Husky by its wheels", () => {
    const spot = ["body", ...SPOT_LEGS.flatMap(l => [`${l}_hip`, `${l}_upper_leg`, `${l}_lower_leg`])];
    expect(rigKind(spot)).toBe("legs");
    expect(rigKind(["base_link", "top_chassis", "front_left_wheel"])).toBe("wheels");
    expect(rigKind(["body"])).toBeNull();
  });

  it("leaves the standing model as baked, and moves each lower leg with its foot", () => {
    const stand = spotPartDeltas(trotPose(0.3, 0, 0));
    const eye = new THREE.Matrix4().toArray();
    for (const m of Object.values(stand)) expect(Math.max(...m.toArray().map((v, i) => Math.abs(v - eye[i])))).toBeLessThan(1e-9);
    // the baked lower leg's foot point lands where FK puts the walking foot
    const pose = trotPose(0.75, 0.3, 1);
    const d = spotPartDeltas(pose).front_left_lower_leg;
    const bakedFoot = footOf("front_left", SPOT_STAND.hipY, SPOT_STAND.knee);
    const moved = bakedFoot.clone().applyMatrix4(d);
    const want = footOf("front_left", pose.legs.front_left.hipY, pose.legs.front_left.knee);
    expect(moved.distanceTo(want)).toBeLessThan(1e-9);
    expect(moved.z - bakedFoot.z).toBeCloseTo(GAIT.lift, 5); // mid-swing: lifted
  });

  it("turns a wheel about its own axle", () => {
    const d = wheelPartDeltas(1.0).front_left_wheel;
    const axle = new THREE.Vector3(0.256, 0.2775, 0.03282);
    expect(axle.clone().applyMatrix4(d).distanceTo(axle)).toBeLessThan(1e-9);
    const rim = new THREE.Vector3(0.256, 0.2775, 0.03282 - HUSKY_WHEEL_R).applyMatrix4(d);
    expect(rim.distanceTo(axle)).toBeCloseTo(HUSKY_WHEEL_R, 9);
  });
});

describe("pose track", () => {
  it("interpolates one update interval late and estimates the forward speed", () => {
    const t = new PoseTrack();
    for (let i = 0; i <= 5; i++) t.push(1000 + i * 200, i * 0.2, 0, 0); // 1 m/s along x
    expect(t.interval).toBeCloseTo(200, 6);
    const s = t.sample(2000 + 100)!; // replays t = 1900
    expect(s.x).toBeCloseTo(0.9, 6);
    expect(s.speed).toBeCloseTo(1, 6);
    expect(s.moving).toBe(true);
    // past the newest pose: hold, standing
    const h = t.sample(5000)!;
    expect([h.x, h.speed, h.moving]).toEqual([1, 0, false]);
  });

  it("reads backwards motion as negative speed and interpolates yaw the short way", () => {
    const t = new PoseTrack();
    t.push(0, 0, 0, Math.PI - 0.1);
    t.push(200, -0.2, 0, -Math.PI + 0.1); // heading ~ -x, moving -x: forward
    const s = t.sample(300)!;
    expect(Math.abs(s.yaw)).toBeGreaterThan(Math.PI - 0.11);
    expect(s.speed).toBeCloseTo(1, 1);
    const b = new PoseTrack();
    b.push(0, 0, 0, 0);
    b.push(200, -0.1, 0, 0);
    expect(b.sample(300)!.speed).toBeCloseTo(-0.5, 6);
  });

  it("snaps on a jump (sim reset / seek) and when a held pose is re-sent", () => {
    const t = new PoseTrack(30);
    t.push(0, 0, 0, 0);
    t.push(200, 100, 0, 0);
    const s = t.sample(250)!;
    expect([s.x, s.speed]).toEqual([100, 0]);
    const p = new PoseTrack();
    for (let i = 0; i < 4; i++) p.push(i * 200, 5, 5, 1);
    const q = p.sample(700)!;
    expect(q.moving).toBe(false);
    expect(q.speed).toBe(0);
  });
});
