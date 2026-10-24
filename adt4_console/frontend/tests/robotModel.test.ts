import {describe, expect, it} from "vitest";
import * as THREE from "three";
import {robotModelFor} from "../src/lib/robotModel";
import {DEFAULT_VIEW_STYLE, parseStoredStyle} from "../src/stores/viewStyle";
import {hullShell} from "../src/viewer/RobotModel";

describe("robot models", () => {
  it("picks the model by platform; unknown platforms draw the arrow", () => {
    expect(robotModelFor("spot")?.url).toBe("models/spot.glb");
    expect(robotModelFor("phoenix")?.url).toBe("models/husky.glb");
    expect(robotModelFor("drone")).toBeNull();
    expect(robotModelFor(null)).toBeNull();
    // Spot's pose frame (body) is ~0.5 m above its feet; the Husky's base_link is low
    expect(robotModelFor("spot")!.groundZ).toBeLessThan(robotModelFor("phoenix")!.groundZ);
  });

  it("stores the robot look, defaulting to model + outline", () => {
    expect(DEFAULT_VIEW_STYLE.robotModel).toBe("model");
    expect(DEFAULT_VIEW_STYLE.robotHighlight).toBe("outline");
    const s = parseStoredStyle(JSON.stringify({robotModel: "arrow", robotHighlight: "tint"}));
    expect([s.robotModel, s.robotHighlight]).toEqual(["arrow", "tint"]);
    const bad = parseStoredStyle(JSON.stringify({robotModel: "blimp", robotHighlight: 3}));
    expect([bad.robotModel, bad.robotHighlight]).toEqual(["model", "outline"]);
  });

  it("the outline shell is the part's hull grown by d on every side", () => {
    // a thin leg-like box: 0.1 x 0.1 x 0.6
    const part = new THREE.BoxGeometry(0.1, 0.1, 0.6);
    const shell = hullShell(part, 0.03);
    shell.computeBoundingBox();
    const size = shell.boundingBox!.getSize(new THREE.Vector3());
    expect(size.x).toBeCloseTo(0.16, 5);
    expect(size.y).toBeCloseTo(0.16, 5);
    expect(size.z).toBeCloseTo(0.66, 5);
    const c = shell.boundingBox!.getCenter(new THREE.Vector3());
    expect(c.length()).toBeCloseTo(0, 5);
  });
});
