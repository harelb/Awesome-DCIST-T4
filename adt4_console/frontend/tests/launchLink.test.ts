import {describe, expect, it} from "vitest";
import {launchHref, parseLaunchPreset} from "../src/lib/launchLink";

describe("launch links", () => {
  it("round-trips a prior map with spaces and slashes", () => {
    const href = launchHref("hamilton", "/home/swarm/adt4_output/map 2026");
    const preset = parseLaunchPreset(href.split("?")[1]);
    expect(preset).toEqual({prior: "/home/swarm/adt4_output/map 2026", source: "hamilton", mode: "relocalize"});
  });

  it("keeps mapping mode only when asked for", () => {
    expect(parseLaunchPreset("?prior=/a&source=willow&mode=mapping")?.mode).toBe("mapping");
    expect(parseLaunchPreset("?prior=/a&source=willow&mode=bogus")?.mode).toBe("relocalize");
  });

  it("ignores links without a prior or a source", () => {
    expect(parseLaunchPreset("")).toBeNull();
    expect(parseLaunchPreset("?prior=/a")).toBeNull();
    expect(parseLaunchPreset("?source=willow")).toBeNull();
  });
});
