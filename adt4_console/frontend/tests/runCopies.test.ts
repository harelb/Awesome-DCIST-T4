import {describe, expect, it, vi} from "vitest";
import {copyBadge, copyEntry, copyOfText, pickDefaultSources, preferBaseCopy, priorAvailability, verifyNames} from "../src/lib/runCopies";
import type {MapEntry, MapsResponse} from "../src/types";

const robotRun = (state: "in_sync" | "outdated" | "not_copied"): MapEntry => ({
  name: "map_20261019_2308",
  size: "1.2G",
  path: "~/adt4_output/map_20261019_2308",
  abs_path: "/home/swarm/adt4_output/map_20261019_2308",
  copy:
    state === "not_copied"
      ? {state, machine: "willow"}
      : {state, machine: "willow", name: "map_20261019_2308__hamilton", path: "/home/rrg/adt4_output/map_20261019_2308__hamilton", matched_by: "run_id"},
});

const copyRow: MapEntry = {
  name: "map_20261019_2308__hamilton",
  size: "900M",
  path: "/home/rrg/adt4_output/map_20261019_2308__hamilton",
  copy_of: {machine: "hamilton", name: "map_20261019_2308", path: "/home/swarm/adt4_output/map_20261019_2308", state: "in_sync"},
};

const maps = (state: "in_sync" | "outdated" | "not_copied"): MapsResponse => ({
  base: "willow",
  machines: {
    willow: [{name: "map_20261019_2308", size: "1G", path: "/home/rrg/adt4_output/map_20261019_2308"}, copyRow],
    hamilton: [robotRun(state)],
  },
});

describe("copy badges", () => {
  it("labels in sync / outdated / not copied", () => {
    expect(copyBadge(robotRun("in_sync"), "willow")).toMatchObject({tone: "ok", text: "on willow ✓ in sync"});
    expect(copyBadge(robotRun("outdated"), "willow")).toMatchObject({tone: "warn", text: "on willow – outdated"});
    expect(copyBadge(robotRun("outdated"), "willow")!.title).toContain("robot changed since the copy");
    expect(copyBadge(robotRun("not_copied"), "willow")).toMatchObject({tone: "off", text: "not copied"});
    expect(copyBadge({name: "x", size: 1}, "willow")).toBeNull();
  });

  it("describes a base copy by its source", () => {
    expect(copyOfText(copyRow)).toBe("copy of hamilton:/home/swarm/adt4_output/map_20261019_2308");
    expect(copyOfText({name: "x", size: 1})).toBeNull();
  });

  it("finds the copy entry to open", () => {
    expect(copyEntry(maps("in_sync"), robotRun("in_sync"), "willow")).toBe(copyRow);
    expect(copyEntry(maps("not_copied"), robotRun("not_copied"), "willow")).toBeNull();
  });
});

describe("verify pairing", () => {
  it("pairs a robot run with its copy and a copy with its source", () => {
    expect(verifyNames("hamilton", robotRun("outdated"), "willow")).toEqual({hamilton: "map_20261019_2308", willow: "map_20261019_2308__hamilton"});
    expect(verifyNames("willow", copyRow, "willow")).toEqual({hamilton: "map_20261019_2308", willow: "map_20261019_2308__hamilton"});
    expect(verifyNames("hamilton", robotRun("not_copied"), "willow")).toBeNull();
  });
});

describe("launch with map prefers the base copy", () => {
  const preset = {prior: "~/adt4_output/map_20261019_2308", source: "hamilton", mode: "relocalize" as const};
  it("switches to the in-sync copy", () => {
    const {preset: p, note} = preferBaseCopy(preset, maps("in_sync"), "willow");
    expect(p).toEqual({prior: "/home/rrg/adt4_output/map_20261019_2308__hamilton", source: "willow", mode: "relocalize"});
    expect(note).toContain("in-sync copy");
  });
  it("keeps the robot run when the copy is outdated or missing", () => {
    expect(preferBaseCopy(preset, maps("outdated"), "willow").preset).toBe(preset);
    expect(preferBaseCopy(preset, maps("outdated"), "willow").note).toContain("outdated");
    expect(preferBaseCopy(preset, maps("not_copied"), "willow")).toEqual({preset, note: null});
    expect(preferBaseCopy({...preset, source: "willow"}, maps("in_sync"), "willow").note).toBeNull();
  });
  it("tells whether the viewer can show the prior", () => {
    expect(priorAvailability(maps("not_copied"), "hamilton", preset.prior, "willow").local).toBe(false);
    expect(priorAvailability(maps("in_sync"), "hamilton", preset.prior, "willow")).toMatchObject({local: true, copy: "map_20261019_2308__hamilton"});
    expect(priorAvailability(maps("in_sync"), "willow", "/x", "willow").local).toBe(true);
  });
});

describe("viewer default sources for the active launch", () => {
  it("enables what exists and waits for the rest", () => {
    const r = pickDefaultSources(["neo4j:prior", "file:prior:map_x__hamilton", "live:euclid"], ["file:prior:map_x__hamilton", "live:euclid", "live:hamilton"]);
    expect(r).toEqual({enable: ["file:prior:map_x__hamilton", "live:euclid"], pending: ["live:hamilton"]});
  });

  it("the store enables a pending default once it appears in the catalog", async () => {
    vi.resetModules();
    const catalogs = [
      [{id: "file:prior:map_x__hamilton", kind: "file", label: "prior: map_x (hamilton)", version: 1, nodes: 3}],
      [
        {id: "file:prior:map_x__hamilton", kind: "file", label: "prior: map_x (hamilton)", version: 1, nodes: 3},
        {id: "live:hamilton", kind: "live", label: "hamilton (live)", version: 1, nodes: 0},
      ],
    ];
    let call = 0;
    vi.doMock("../src/lib/api", () => ({
      getJSON: vi.fn(async (path: string) => (path === "api/graph/sources" ? catalogs[Math.min(call++, 1)] : null)),
      getMsgpack: vi.fn(() => new Promise(() => {})), // never resolves: loading stays pending
      errorText: (e: unknown) => String(e),
    }));
    vi.doMock("../src/ws", () => ({ws: {on: () => () => {}}}));
    const {useViewer} = await import("../src/stores/viewer");
    await useViewer.getState().loadCatalog();
    useViewer.getState().set({pendingEnable: ["live:hamilton"]});
    useViewer.getState().enableSource("file:prior:map_x__hamilton", true);
    expect(useViewer.getState().sources["file:prior:map_x__hamilton"].enabled).toBe(true);
    await useViewer.getState().loadCatalog();
    expect(useViewer.getState().sources["live:hamilton"].enabled).toBe(true);
    expect(useViewer.getState().pendingEnable).toEqual([]);
    vi.doUnmock("../src/lib/api");
    vi.doUnmock("../src/ws");
  });
});
