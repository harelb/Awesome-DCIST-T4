import {describe, expect, it} from "vitest";
import {boxEdges, changeName, parseChanges} from "../src/lib/overlays";
import {layerMatrix} from "../src/lib/frameAlign";
import type {FrameAlign} from "../src/types";

const payload = {
  frame: "map",
  objects: [
    {id: 5, kind: "added", box: true, center: [1, 2, 0.5], dims: [0.5, 0.6, 1], quat: [0, 0, 0, 1], label: 3, name: "chair", cc: 0.91, n: 25, conf: 0.7},
    {id: 77, kind: "removed", box: true, center: [5, 0, 1], dims: [1, 0.1, 2], quat: [0, 0, 0, 1], label: 12, name: "door", cc: 0.85, n: 7},
    {id: 99, kind: "removed", box: false, cc: 0.8, n: 6},
    {id: 1, kind: "moved", box: true, center: [0, 0, 0], dims: [1, 1, 1]},
    null,
  ],
  added: 1,
  removed: 2,
  prior: {ready: true, boxes: 8, error: null},
  t: 10.5,
  hash: "abc",
};

describe("change detection payload", () => {
  it("keeps added and removed objects, drops unknown kinds", () => {
    const c = parseChanges(payload)!;
    expect(c.objects.map(o => `${o.kind}:${o.id}`)).toEqual(["added:5", "removed:77", "removed:99"]);
    expect([c.added, c.removed]).toEqual([1, 2]);
    expect(c.prior).toEqual({ready: true, boxes: 8, error: null});
    expect(c.objects[2].box).toBe(false);
    expect(c.objects[0].conf).toBe(0.7);
    expect(c.objects[1].conf).toBeNull();
  });

  it("rejects non-objects", () => {
    expect(parseChanges(null)).toBeNull();
    expect(parseChanges("x")).toBeNull();
    expect(parseChanges({})!.objects).toEqual([]);
  });

  it("names an object by name, else label", () => {
    const [chair, , unboxed] = parseChanges(payload)!.objects;
    expect(changeName(chair)).toBe("chair");
    expect(changeName(unboxed)).toBe("object");
    expect(changeName({...unboxed, label: 4})).toBe("label 4");
  });
});

describe("boxEdges", () => {
  it("gives the 12 edges of the box, each along one axis", () => {
    const e = boxEdges([2, 4, 6]);
    expect(e).toHaveLength(24);
    for (let i = 0; i < 24; i += 2) {
      const d = e[i].map((v, k) => Math.abs(v - e[i + 1][k]));
      expect(d.filter(v => v > 1e-9)).toHaveLength(1);
    }
    expect(Math.max(...e.map(p => p[2]))).toBe(3);
  });
});

describe("frame of the change layer", () => {
  const fa: FrameAlign = {parent: "map", child: "hamilton/odom", T: [10, 5, 0, 1, 0, 0, 0], stamp: 1, ok: true, reason: ""};
  it("draws map-frame reports as is (they sit on the prior map)", () => {
    expect(layerMatrix(fa, "map")).toBeNull();
    expect(layerMatrix(fa, "hamilton/odom")).not.toBeNull();
  });
});
