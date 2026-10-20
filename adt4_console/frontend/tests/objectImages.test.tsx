import {afterEach, describe, expect, it} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import type {NodeRecord} from "../src/codec";
import {GraphModel} from "../src/lib/graphModel";
import {pickBox, rayBox, type BoxSet} from "../src/lib/bboxPick";
import {layoutGraph} from "../src/viewer/layout";
import {imageUrl, nodeImageBase, ObjectImages, SearchThumb} from "../src/viewer/objectImages";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

describe("bounding-box picking", () => {
  const set = (boxes: number[][], syms: string[]): BoxSet => ({syms, boxes: new Float32Array(boxes.flat())});
  const down = [0, 0, -1];

  it("intersects a ray with an AABB", () => {
    expect(rayBox([0.5, 0.5, 10], down, [0, 0, 0, 1, 1, 1])).toEqual([9, 10]);
    expect(rayBox([2, 2, 10], down, [0, 0, 0, 1, 1, 1])).toBeNull();
    expect(rayBox([0.5, 0.5, 0.5], down, [0, 0, 0, 1, 1, 1])).toEqual([0, 0.5]); // starts inside
    expect(rayBox([0.5, 0.5, -1], down, [0, 0, 0, 1, 1, 1])).toBeNull(); // box behind the ray
    expect(rayBox([0.5, 0.5, 10], down, [NaN, 0, 0, 1, 1, 1])).toBeNull();
  });

  it("prefers the smallest enclosing box (a mug on a table)", () => {
    const s = set(
      [
        [-2, -1, 0, 2, 1, 0.8], // table
        [0, 0, 0.8, 0.1, 0.1, 0.95], // mug on it
        [NaN, NaN, NaN, NaN, NaN, NaN], // no box
      ],
      ["O(1)", "O(2)", "O(3)"],
    );
    expect(pickBox([0.05, 0.05, 5], down, [["src", s]])?.sym).toBe("O(2)");
    expect(pickBox([1, 0.5, 5], down, [["src", s]])?.sym).toBe("O(1)");
    expect(pickBox([5, 5, 5], down, [["src", s]])).toBeNull();
  });

  it("breaks volume ties by the nearest box and reports the source", () => {
    const a = set([[0, 0, 0, 1, 1, 1]], ["O(1)"]);
    const b = set([[0, 0, 3, 1, 1, 4]], ["O(9)"]);
    const hit = pickBox([0.5, 0.5, 10], down, [["a", a], ["b", b]]);
    expect(hit?.key).toBe("b");
    expect(hit?.sym).toBe("O(9)");
  });

  it("layout records displayed object boxes (with the layer z offset)", () => {
    const rec = (sym: string, layer: number, pos: [number, number, number], bbox: NodeRecord["bbox"]): NodeRecord => ({
      sym, layer, part: 0, pos, label: 1, bbox, color: [0, 0, 0], name: "", active: false,
    });
    const m = GraphModel.fromSnapshot({
      nodes: [rec("O(1)", 2, [0, 0, 0], [-1, -1, -1, 1, 1, 1]), rec("O(2)", 2, [5, 5, 0], null), rec("P(1)", 3, [0, 0, 0], null)],
      edges: [],
      labelspaces: {},
    });
    const l = layoutGraph(m, {hiddenLayers: {}, edgeKinds: {}, flat: true, stackHeight: 0});
    const objs = l.layers.find(x => x.info.kind === "objects")!;
    const places = l.layers.find(x => x.info.kind === "places")!;
    expect(places.boxes).toBeUndefined();
    const i = objs.syms.indexOf("O(1)");
    expect(Array.from(objs.boxes!.slice(i * 6, i * 6 + 6))).toEqual([-1, -1, -1, 1, 1, 1]);
    expect(Number.isNaN(objs.boxes![objs.syms.indexOf("O(2)") * 6])).toBe(true);
  });
});

describe("object image URLs", () => {
  it("builds variant query strings", () => {
    const base = nodeImageBase("file:run", "O(3)", 0);
    expect(base).toBe("api/graph/file%3Arun/image/O(3)/0");
    expect(imageUrl(base)).toBe(base);
    expect(imageUrl(base, {w: 96, mask: true, crop: true})).toBe(`${base}?w=96&mask=1&crop=1`);
    expect(imageUrl("/x?token=t", {mask: true})).toBe("/x?token=t&mask=1");
  });
});

describe("object image components", () => {
  let root: Root | null = null;
  let el: HTMLDivElement | null = null;
  const mount = (node: React.ReactNode) => {
    el = document.createElement("div");
    document.body.appendChild(el);
    root = createRoot(el);
    act(() => root!.render(node));
    return el;
  };
  afterEach(() => {
    act(() => root?.unmount());
    el?.remove();
    root = null;
  });

  it("search thumbnail: image when the node has one, placeholder otherwise", () => {
    const c = mount(
      <ul>
        <SearchThumb src="file:a" sym="O(1)" has={true} color="red" />
        <SearchThumb src="file:a" sym="O(2)" has={false} color="red" />
        <SearchThumb src="file:a" sym="O(3)" has={null} color="red" />
      </ul>,
    );
    const imgs = c.querySelectorAll("img");
    expect(imgs.length).toBe(1);
    expect(imgs[0].getAttribute("src")).toContain("image/O(1)/0?w=96&mask=1&crop=1");
    expect(c.querySelectorAll(".sr-thumb-empty").length).toBe(2);
  });

  it("inspect: large best view for objects with the mask toggle", () => {
    const images = ["/api/graph/f/image/O(1)/0", "/api/graph/f/image/O(1)/1"];
    const c = mount(<ObjectImages images={images} masks={[true, false]} title="chair" isObject />);
    const hero = c.querySelector(".obj-hero img")!;
    expect(hero.getAttribute("src")).toBe(`${images[0]}?w=720&mask=1`);
    expect(c.querySelectorAll(".image-strip img").length).toBe(1);
    const toggle = c.querySelector<HTMLInputElement>(".toggle input")!;
    act(() => toggle.click());
    expect(c.querySelector(".obj-hero img")!.getAttribute("src")).toBe(`${images[0]}?w=720`);
    act(() => toggle.click());
  });

  it("inspect: agents get a plain strip, no hero, no mask toggle", () => {
    const c = mount(<ObjectImages images={["/a/0", "/a/1"]} masks={[false, false]} title="a(0)" isObject={false} />);
    expect(c.querySelector(".obj-hero")).toBeNull();
    expect(c.querySelectorAll(".image-strip img").length).toBe(2);
    expect(c.querySelector(".toggle")).toBeNull();
  });
});
