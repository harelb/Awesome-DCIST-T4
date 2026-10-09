import {describe, expect, it} from "vitest";
import {deflateSync} from "node:zlib";
import {encode, decode} from "@msgpack/msgpack";
import {decodeGrid, gridToRgba, inflateZlib, OCCUPIED, parsePaths, UNKNOWN} from "../src/lib/overlays";
import {clampHz, frameAge} from "../src/viewer/CameraPanel";

describe("occupancy grid", () => {
  it("maps cells to RGBA: free transparent, occupied red, unknown grey", () => {
    const rgba = gridToRgba(new Int8Array([0, 100, -1, 49, 50]));
    expect([...rgba.slice(0, 4)]).toEqual([0, 0, 0, 0]);
    expect([...rgba.slice(4, 8)]).toEqual(OCCUPIED);
    expect([...rgba.slice(8, 12)]).toEqual(UNKNOWN);
    expect(rgba[15]).toBe(0); // 49: below the occupied threshold
    expect([...rgba.slice(16, 20)]).toEqual(OCCUPIED);
  });

  it("inflates zlib payloads", async () => {
    const src = new Uint8Array([1, 2, 3, 250, 0, 7]);
    expect([...(await inflateZlib(new Uint8Array(deflateSync(src))))]).toEqual([...src]);
  });

  it("decodes the msgpack grid served by the console (row 0 = lowest y)", async () => {
    // 3 wide x 2 high, row-major with x fastest: row 0 = [-1, 0, 100]
    const cells = new Int8Array([-1, 0, 100, 0, 100, 0]);
    const body = encode({
      frame: "euclid/map",
      resolution: 0.1,
      origin: [1, 2, 0.5, 0.3],
      width: 3,
      height: 2,
      data: new Uint8Array(deflateSync(new Uint8Array(cells.buffer))),
      hash: "h1",
      t: 4,
    });
    const g = await decodeGrid(decode(body));
    expect(g.width).toBe(3);
    expect(g.height).toBe(2);
    expect(g.origin).toEqual([1, 2, 0.5, 0.3]);
    expect(g.hash).toBe("h1");
    expect(g.rgba.length).toBe(24);
    expect([...g.rgba.slice(0, 4)]).toEqual(UNKNOWN); // (x0, y0)
    expect([...g.rgba.slice(8, 12)]).toEqual(OCCUPIED); // (x2, y0)
    expect([...g.rgba.slice(16, 20)]).toEqual(OCCUPIED); // (x1, y1)
  });

  it("rejects a grid whose size does not match", async () => {
    const body = decode(encode({width: 4, height: 4, resolution: 0.1, origin: [0, 0, 0, 0], data: new Uint8Array(deflateSync(new Uint8Array(3)))}));
    await expect(decodeGrid(body)).rejects.toThrow(/cells/);
  });
});

describe("executor paths", () => {
  it("parses paths and drops malformed points", () => {
    const p = parsePaths({smooth: [[0, 0, 0], [1, 2, 3], [1, 2], "x"], mlp: [], target: [4, 5, 6], frame: "map", t: 3, hash: "a"});
    expect(p).toEqual({smooth: [[0, 0, 0], [1, 2, 3]], mlp: [], target: [4, 5, 6], frame: "map", t: 3, hash: "a"});
    expect(parsePaths({smooth: null, target: null})?.target).toBeNull();
    expect(parsePaths(null)).toBeNull();
  });
});

describe("camera helpers", () => {
  it("clamps the refresh rate and ages frames", () => {
    expect(clampHz(0.01)).toBe(0.2);
    expect(clampHz(9)).toBe(5);
    expect(frameAge({age: 0.5, at: 1000}, 3000)).toBeCloseTo(2.5);
    expect(frameAge({age: null, at: 1000}, 3000)).toBeNull();
  });
});
