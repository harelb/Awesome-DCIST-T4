import {describe, expect, it} from "vitest";
import * as THREE from "three";
import {agentModelFor, isRobotHighlight, ROBOT_HIGHLIGHTS, robotModelFor} from "../src/lib/robotModel";
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

  it("animates the gait by default; the setting is stored", () => {
    expect(DEFAULT_VIEW_STYLE.robotGait).toBe(true);
    expect(parseStoredStyle(JSON.stringify({robotGait: false})).robotGait).toBe(false);
    expect(parseStoredStyle(JSON.stringify({robotGait: "no"})).robotGait).toBe(true);
  });

  it("mission agents use their platform's model; the quad keeps its glyph", () => {
    expect(agentModelFor("spot")).toBe(robotModelFor("spot"));
    expect(agentModelFor("husky")).toBe(robotModelFor("phoenix"));
    expect(agentModelFor("husky")?.url).toBe("models/husky.glb");
    expect(agentModelFor("quad")).toBeNull();
    expect(agentModelFor(null)).toBeNull();
  });

  it("offers recolouring the model, and loads settings stored before it", () => {
    expect(ROBOT_HIGHLIGHTS.map(o => o.value)).toEqual(["outline", "tint", "paint", "off"]);
    expect(ROBOT_HIGHLIGHTS.find(o => o.value === "paint")?.label).toBe("Recolour");
    expect(isRobotHighlight("paint")).toBe(true);
    expect(isRobotHighlight("glow")).toBe(false);
    expect(parseStoredStyle(JSON.stringify({robotHighlight: "paint"})).robotHighlight).toBe("paint");
    for (const old of ["off", "outline", "tint"]) expect(parseStoredStyle(JSON.stringify({robotHighlight: old})).robotHighlight).toBe(old);
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
