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
