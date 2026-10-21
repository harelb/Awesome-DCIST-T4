import {describe, expect, it} from "vitest";
import {saturated, totalKbps, type TrafficLink} from "../src/pages/FleetlinkTraffic";

describe("fleetlink traffic table", () => {
  const l: TrafficLink = {connected: true, rates: {C1: 1000, C3: 99_000}, robot_rates: {C3: 99_000}, budgets: {C1: 5000, C3: 100_000}};
  it("sums received classes as kbit/s", () => {
    expect(totalKbps(l)).toBeCloseTo(800);
  });
  it("flags a class at its budget", () => {
    expect(saturated(l, "C3")).toBe(true);
    expect(saturated(l, "C1")).toBe(false);
  });
});
