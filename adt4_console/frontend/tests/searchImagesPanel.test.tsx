import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {SearchImagesPanel} from "../src/viewer/SearchImagesPanel";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const json = (o: unknown, status = 200) => new Response(JSON.stringify(o), {status, headers: {"Content-Type": "application/json"}});
const flush = () => act(async () => void (await new Promise(r => setTimeout(r, 0))));

const HEALTH = {ok: true, server: "willow:8077", keyframes: 12, pending: 0, conf_threshold: 0.5, sources: [{id: "prior", keyframes: 12}]};
const RESULT = {
  prompt: "mug",
  dry_run: true,
  pending_id: "p1",
  elapsed_s: 1.2,
  pool: 12,
  conf: 0.5,
  timings: {embed_rank: 0.02, detect: 1.0, render: 0.1, cache_hit: 0.9},
  detections: 5,
  cache: {frames: 10, hits: 9, hit_rate: 0.9, gpu: 8, disk: 1, miss: 1},
  candidates: [
    {
      index: 0, label: "mug", score: 0.8, centroid: [1, 2, 0.5], bbox_min: [0.9, 1.9, 0.4], bbox_max: [1.1, 2.1, 0.6], num_points: 400, num_views: 3, num_frames: 2, frame_keys: ["f1", "f2"],
      would_merge_into: {symbol: "O7", distance_m: 0.2}, image_uri: null,
      views: [{view: 0, frame_key: "f1", frame_id: 1, timestamp_ns: 1, bbox_2d: [0, 0, 2, 2], score: 0.8, mask_rle: "[0,4]", mask_shape: [2, 2], image_uri: null}],
    },
    {index: 1, label: "mug", score: 0.3, centroid: [5, 2, 0.5], bbox_min: [4.9, 1.9, 0.4], bbox_max: [5.1, 2.1, 0.6], num_points: 90, num_views: 1, would_merge_into: null, image_uri: null, views: []},
  ],
};

function setInput(el: HTMLInputElement, v: string) {
  Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!.call(el, v);
  el.dispatchEvent(new Event("input", {bubbles: true}));
}

describe("SearchImagesPanel", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
  });
  const mount = () => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<SearchImagesPanel />));
  };

  it("says clearly when the SAM3 server is down", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => json({ok: false, error: "SAM3 query server not running on willow:8077"})));
    mount();
    await flush();
    expect(document.body.textContent).toContain("SAM3 query server not running on willow:8077");
  });

  it("searches, shows candidates with merge hints, and commits one", async () => {
    const fetchMock = vi.fn(async (url: string, init?: RequestInit) => {
      if (String(url).startsWith("api/sam3/health")) return json(HEALTH);
      if (String(url) === "api/sam3/query") return json(RESULT);
      if (String(url) === "api/sam3/commit") return json({symbol: "O7", merged: true});
      return json({error: `unexpected ${url} ${init?.method}`}, 404);
    });
    vi.stubGlobal("fetch", fetchMock);
    mount();
    await flush();
    expect(document.body.textContent).toContain("willow:8077");
    act(() => setInput(document.querySelector('input[aria-label="Image search text"]') as HTMLInputElement, "mug"));
    await act(async () => (document.querySelector("form") as HTMLFormElement).requestSubmit());
    await flush();
    const q = fetchMock.mock.calls.find(c => c[0] === "api/sam3/query")!;
    expect(JSON.parse(String(q[1]?.body))).toMatchObject({prompt: "mug", mode: "frustum", top_k: 60, max_frames: 30, patience: 3, source: "all", max_views: 3, render: "none"});
    const text = document.body.textContent ?? "";
    expect(text).toContain("2 candidates");
    expect(text).toContain("(from 5 detections)");
    expect(text).toContain("seen in 2 frames (3 detections)");
    expect(text).toContain("seen in 1 frame");
    expect(text).toContain("cache 90% (9/10)");
    expect(text).not.toContain("cache_hit");
    // the server rendered nothing: the best view's image is lazy-loaded from the console
    const img = document.querySelector("img[loading=lazy]") as HTMLImageElement;
    expect(img.getAttribute("src")).toBe("api/sam3/frame.jpg?pending_id=p1&index=0&view=0&max_dim=480");
    expect(text).toContain("would merge into O7 (0.20 m)");
    expect(text).toContain("new object");
    expect(text).toContain("detect 1.00 s");
    const add = [...document.querySelectorAll("button")].find(b => b.textContent === "Add to Neo4j") as HTMLButtonElement;
    await act(async () => add.click());
    await flush();
    const c = fetchMock.mock.calls.find(c => c[0] === "api/sam3/commit")!;
    expect(JSON.parse(String(c[1]?.body))).toMatchObject({pending_id: "p1", index: 0});
    expect(document.body.textContent).toContain("merged into O7");
    expect(document.body.textContent).toContain("Recent searches (1)");
  });
});
