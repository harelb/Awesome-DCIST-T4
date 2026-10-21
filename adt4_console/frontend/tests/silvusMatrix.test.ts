import {describe, expect, it} from "vitest";
import {linkColor, matrixMode, rssiColor} from "../src/pages/Network";

describe("Silvus matrix", () => {
  it("shows RSSI when the firmware reports it, else the network_status link value", () => {
    expect(matrixMode({a: {b: {rssi_db: -60, link_db: 30}}})).toBe("rssi");
    expect(matrixMode({a: {b: {rssi_db: null, link_db: 36}}, b: "timed out"})).toBe("link");
    expect(matrixMode({a: "timed out"})).toBe("rssi"); // nothing to show either way
  });
  it("colours the link value red (5 dB) .. green (35 dB)", () => {
    expect(linkColor(35)).toBe(rssiColor(-45));
    expect(linkColor(5)).toBe(rssiColor(-90));
    expect(linkColor(100)).toBe(linkColor(35));
    expect(linkColor(null)).toBe("transparent");
  });
});

import {linkLabel, meshLayout, meshSize, radioTitle} from "../src/pages/SilvusMesh";

describe("Silvus mesh view", () => {
  it("names a radio after the computer behind it", () => {
    const r = {name: "radio_1", node_id: 1, battery: 90, via: null, error: null, local: false};
    expect(radioTitle({...r, hosts: [{ip: "192.168.100.2", machine: "hamilton"}]})).toBe("hamilton");
    expect(radioTitle({...r, hosts: [{ip: "10.0.0.9", machine: null}]})).toBe("radio_1");
  });
  it("labels both directions", () => {
    const l = {a: "x", b: "y", ab: 40.4, ba: null, unit: "dB" as const, quality: "good" as const};
    expect(linkLabel(l)).toBe("40 / – dB");
    expect(linkLabel({...l, ab: null, quality: "linked"})).toBe("routed");
  });
  it("keeps every node and its labels inside the view", () => {
    for (const phone of [false, true])
      for (const n of [1, 2, 3, 4, 6]) {
        const [w, h] = meshSize(n, phone);
        for (const [x, y] of meshLayout(n, w, h, phone)) {
          expect(x - 40).toBeGreaterThanOrEqual(0);
          expect(x + 40).toBeLessThanOrEqual(w);
          expect(y - 40).toBeGreaterThanOrEqual(0);
          expect(y + 75).toBeLessThanOrEqual(h);
        }
      }
  });
});
