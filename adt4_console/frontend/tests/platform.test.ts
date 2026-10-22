import {describe, expect, it} from "vitest";
import {isSpot, phoenixChips, phoenixGuards, platformOf, progressMatches, resolvePlatform} from "../src/lib/platform";
import {CAPS, PHOENIX_CAPS, capsFor, joystickToTwist, keysToTwist, sticksToTwist, clampTwist} from "../src/lib/teleop";
import {decodeLidar, heightColor, lidarLayerFrame} from "../src/lib/lidar";

describe("platform", () => {
  it("missing = spot, husky = phoenix", () => {
    expect(platformOf(undefined)).toBe("spot");
    expect(platformOf("")).toBe("spot");
    expect(platformOf({platform_type: undefined})).toBe("spot");
    expect(platformOf("husky")).toBe("phoenix");
    expect(platformOf({platform_type: "Husky"})).toBe("phoenix");
    expect(platformOf({platform: "phoenix", platform_type: "spot"})).toBe("phoenix");
    expect(platformOf("jackal")).toBe("jackal");
    expect(isSpot(null)).toBe(true);
    expect(isSpot("phoenix")).toBe(false);
    expect(resolvePlatform(undefined, "", "husky")).toBe("phoenix");
    expect(resolvePlatform("spot", "phoenix")).toBe("spot");
    expect(resolvePlatform()).toBe("spot");
  });
  it("phoenix guards -> chips", () => {
    expect(phoenixChips(phoenixGuards({source: "phoenix", battery_percent: 46.8, estop_pressed: false, safety_stop: false, manual_mode: false})).map(c => c.text)).toEqual(["AUTO", "47 %"]);
    const all = phoenixChips(phoenixGuards({battery_percent: 9, estop_pressed: true, safety_stop: true, manual_mode: true}));
    expect(all.map(c => [c.text, c.tone])).toEqual([["MANUAL", "warn"], ["9 %", "err"], ["E-STOP", "err"], ["SAFETY-STOP", "err"]]);
    expect(all[0].title).toMatch(/will not execute plans/);
    // unknown (null) keys and Spot guards: no chips
    expect(phoenixChips(phoenixGuards({battery_percent: null, estop_pressed: null, manual_mode: null}))).toEqual([]);
    expect(phoenixChips(phoenixGuards({stop: false, lease_known: true}))).toEqual([]);
    expect(phoenixChips(phoenixGuards(undefined))).toEqual([]);
  });
  it("progress belongs to a plan by id or equal stamp", () => {
    expect(progressMatches({plan_id: "12.000000500", dist_to_goal_m: 1, done: false}, "12.000000500")).toBe(true);
    expect(progressMatches({plan_id: "12.5", dist_to_goal_m: 1, done: false}, "12.500000000")).toBe(true);
    expect(progressMatches({plan_id: "12.5", dist_to_goal_m: 1, done: false}, "13.5")).toBe(false);
    expect(progressMatches(null, "1")).toBe(false);
    expect(progressMatches({plan_id: "a", dist_to_goal_m: 1, done: false}, null)).toBe(false);
  });
});

describe("teleop caps per platform", () => {
  it("spot keeps its caps; phoenix uses the Husky's or the backend's", () => {
    expect(capsFor("spot", {vx: 9, vy: 9, wz: 9})).toBe(CAPS);
    expect(capsFor("phoenix")).toEqual(PHOENIX_CAPS);
    expect(capsFor("phoenix", {vx: 0.8, vy: 0, wz: 1})).toEqual({vx: 0.8, vy: 0, wz: 1});
    expect(capsFor("phoenix", {vx: "fast"})).toEqual(PHOENIX_CAPS);
  });
  it("a zero vy cap never strafes and gives no -0", () => {
    expect(keysToTwist(["KeyA", "KeyW"], 1, false, PHOENIX_CAPS)).toEqual({vx: 1, vy: 0, wz: 0});
    expect(Object.is(keysToTwist(["KeyD"], 1, false, PHOENIX_CAPS).vy, 0)).toBe(true);
    expect(joystickToTwist(1, 0, 1, false, PHOENIX_CAPS).wz).toBeCloseTo(-1.2);
    expect(sticksToTwist({x: -1, y: 1}, {x: 0, y: 0}, 0.5, PHOENIX_CAPS)).toEqual({vx: 0.5, vy: 0, wz: 0});
    expect(clampTwist({vx: 5, vy: 5, wz: -5}, PHOENIX_CAPS)).toEqual({vx: 1, vy: 0, wz: -1.2});
    // default stays Spot
    expect(clampTwist({vx: 5, vy: 5, wz: -5})).toEqual({vx: 1.6, vy: 0.4, wz: -0.8});
  });
});

function lidarMsg(pts: [number, number, number][], origin = [1, 2, 0.5, 0, 0, 0, 1], scale = 0.01) {
  const buf = new ArrayBuffer(pts.length * 6 + 1);
  // odd offset: the decoder must not need an aligned Int16Array
  const dv = new DataView(buf, 1);
  pts.forEach((p, i) => p.forEach((v, k) => dv.setInt16((i * 3 + k) * 2, Math.round(v / scale), true)));
  return {frame: "gauss/odom", origin, scale, n: pts.length, xyz: new Uint8Array(buf, 1), intensity: null, stamp: 5, seq: 7};
}

describe("lidar frames", () => {
  it("decodes int16 LE xyz at the scale", () => {
    const f = decodeLidar(lidarMsg([[1, -2, 0.5], [-0.01, 0, 3.2]]));
    expect(f.n).toBe(2);
    expect([...f.positions].map(v => +v.toFixed(3))).toEqual([1, -2, 0.5, -0.01, 0, 3.2]);
    expect(f.origin).toEqual([1, 2, 0.5, 0, 0, 0, 1]);
    expect(f.seq).toBe(7);
    expect(f.colors.length).toBe(6);
    // higher point is redder, lower one bluer
    expect(f.colors[3]).toBeGreaterThan(f.colors[0]);
    expect(f.colors[2]).toBeGreaterThan(f.colors[5]);
  });
  it("colours by height after the sensor rotation", () => {
    // sensor rolled 180 deg about x: sensor +z points down
    const f = decodeLidar(lidarMsg([[0, 0, 1], [0, 0, -1]], [0, 0, 0, 1, 0, 0, 0]));
    expect(f.colors[0]).toBeLessThan(f.colors[3]); // the first point is the low one: less red
  });
  it("rejects malformed frames", () => {
    expect(() => decodeLidar(null)).toThrow();
    expect(() => decodeLidar({origin: [0, 0, 0], xyz: new Uint8Array(6)})).toThrow();
    expect(() => decodeLidar({origin: [0, 0, 0, 0, 0, 0, 1]})).toThrow();
    // n larger than the payload: clipped
    expect(decodeLidar({...lidarMsg([[1, 1, 1]]), n: 99}).n).toBe(1);
  });
  it("height ramp and layer frame", () => {
    expect(heightColor(0)).toEqual([0, 0, 0.5]);
    expect(heightColor(1)).toEqual([0.5, 0, 0]);
    const fa = {parent: "map", child: "gauss/odom", T: [0, 0, 0, 1, 0, 0, 0], ok: true};
    expect(lidarLayerFrame("gauss/odom", "map", fa)).toBe("gauss/odom");
    expect(lidarLayerFrame("gauss/os_sensor", "gauss/odom", fa)).toBe("gauss/odom");
    expect(lidarLayerFrame("", "gauss/odom", null)).toBe("gauss/odom");
    expect(lidarLayerFrame("gauss/odom", undefined, null)).toBe("gauss/odom");
  });
});
