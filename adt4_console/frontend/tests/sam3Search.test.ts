import {describe, expect, it} from "vitest";
import {DEFAULT_PARAMS, cacheChip, pushHistory, seenIn, viewImageUrl, rleDecode, sourceLabel, sourceOptions, timingRows, visibleCandidates, visibleFrames, type HistoryEntry, type Sam3QueryResult} from "../src/viewer/sam3Search";

const entry = (id: number): HistoryEntry => ({id, at: id, params: {...DEFAULT_PARAMS, prompt: `q${id}`}, status: {}});

describe("sam3Search helpers", () => {
  it("decodes hydra_image_store RLE (row-major, background first)", () => {
    // 2x3 mask: row 0 = 0 1 1, row 1 = 1 0 0
    expect(Array.from(rleDecode([1, 3, 2], [2, 3]) ?? [])).toEqual([0, 1, 1, 1, 0, 0]);
    expect(Array.from(rleDecode("[0,2,4]", [2, 3]) ?? [])).toEqual([1, 1, 0, 0, 0, 0]);
    expect(rleDecode([1, 2], [2, 3])).toBeNull(); // does not cover the mask
    expect(rleDecode([9], [2, 3])).toBeNull();
    expect(rleDecode("not json", [2, 3])).toBeNull();
    expect(rleDecode(null, [2, 3])).toBeNull();
  });

  it("keeps history newest-first, bounded, replacing by id", () => {
    let h: HistoryEntry[] = [];
    for (let i = 1; i <= 5; i++) h = pushHistory(h, entry(i), 3);
    expect(h.map(e => e.id)).toEqual([5, 4, 3]);
    h = pushHistory(h, {...entry(4), error: "x"}, 3);
    expect(h.map(e => e.id)).toEqual([5, 4, 3]);
    expect(h[1].error).toBe("x");
  });

  it("builds source options from /health", () => {
    expect(sourceOptions(undefined)).toEqual([{value: "all", label: "All keyframes", n: undefined}]);
    const one = sourceOptions([
      {id: "prior", keyframes: 10},
      {id: "live/hamilton/1760000000", keyframes: 4},
    ]);
    expect(one.map(o => o.value)).toEqual(["all", "prior", "live/hamilton/1760000000"]);
    expect(one[0].n).toBe(14);
    const two = sourceOptions([
      {id: "live/a/1", keyframes: 1},
      {id: "prior", keyframes: 10},
      {id: "live/b/2", keyframes: 2},
    ]);
    expect(two.map(o => o.value)).toEqual(["all", "prior", "live", "live/a/1", "live/b/2"]);
    expect(two[2].n).toBe(3);
    expect(sourceLabel("prior")).toBe("Prior map (Neo4j)");
    expect(sourceLabel("live/hamilton/x")).toBe("Live: hamilton (x)");
  });

  it("orders timings by pipeline stage", () => {
    expect(timingRows({render: 0.1, detect: 1, embed_rank: 0.02, extra: 3}).map(r => r[0])).toEqual(["embed_rank", "detect", "render", "extra"]);
    expect(timingRows({detect: 1, cache_hit: 0.5}).map(r => r[0])).toEqual(["detect"]); // not seconds
    expect(timingRows(undefined)).toEqual([]);
  });

  it("formats cache hit rate, seen-in and lazy view URLs", () => {
    expect(cacheChip({frames: 20, hits: 19, hit_rate: 0.95, gpu: 18, disk: 1, miss: 1})).toEqual({
      text: "cache 95% (19/20)",
      title: "SAM3 image features of the 20 frames searched: gpu 18 · disk 1 · miss 1 (miss = computed now, cached for next time)",
    });
    expect(cacheChip({frames: 0, hits: 0, hit_rate: null})).toBeNull();
    expect(cacheChip(undefined)).toBeNull();
    expect(seenIn({num_frames: 1, num_views: 2})).toBe("seen in 1 frame");
    expect(seenIn({num_views: 3})).toBe("seen in 3 frames");
    expect(viewImageUrl("p 1", 2, 1, 320)).toBe("api/sam3/frame.jpg?pending_id=p+1&index=2&view=1&max_dim=320");
    expect(DEFAULT_PARAMS.patience).toBe(3);
    expect(DEFAULT_PARAMS.maxFrames).toBe(30);
  });

  it("filters by min score", () => {
    const r = {candidates: [{score: 0.9}, {score: 0.3}]} as unknown as Sam3QueryResult;
    expect(visibleCandidates(r, 0.5)).toHaveLength(1);
    expect(visibleCandidates(undefined, 0)).toEqual([]);
    expect(visibleFrames({frames: [{score: 0.1}, {score: 0.05}]} as never, 0.08)).toHaveLength(1);
  });
});
