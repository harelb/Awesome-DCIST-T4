import {describe, expect, it} from "vitest";
import {readFileSync} from "node:fs";
import {join} from "node:path";
import {decodeDelta, decodeGraph, decodeTile, unpack, parseSym} from "../src/codec";
import {GraphModel} from "../src/lib/graphModel";

const FIX = join(__dirname, "fixtures");
const bin = (n: string) => new Uint8Array(readFileSync(join(FIX, n)));
const js = (n: string) => JSON.parse(readFileSync(join(FIX, n), "utf8"));

function stateOf(m: GraphModel) {
  const nodes: Record<string, unknown> = {};
  for (const sym of [...m.nodes.keys()].sort()) nodes[sym] = m.nodes.get(sym);
  return {nodes, edges: [...m.edges.values()].map(e => [...e]).sort(), labelspaces: m.labelspaces};
}

describe("codec", () => {
  it("decodes a graph snapshot exactly like fleetlink.codec.decode_records", () => {
    const g = decodeGraph(unpack(bin("graph_small.msgpack")));
    const exp = js("graph_small.json");
    expect(g.nodes.length).toBe(Object.keys(exp.nodes).length);
    expect(stateOf(GraphModel.fromSnapshot(g))).toEqual(exp);
    const o1 = g.nodes.find(n => n.sym === "O(1)")!;
    expect(o1.bbox).toEqual([0.5, 1.5, 0, 1.5, 2.5, 1]);
    expect(g.nodes.find(n => n.sym === "O(2)")!.bbox).toBeNull();
    expect(g.nodes.find(n => n.sym === "a(0)")!.part).toBe(97);
  });

  it("handles unaligned msgpack bin views", () => {
    // msgpack hands out subarray views at arbitrary offsets: decoding must copy
    const raw = unpack(bin("graph_small.msgpack")) as Record<string, Uint8Array>;
    const shifted = new Uint8Array(raw.pos.length + 1);
    shifted.set(raw.pos, 1);
    const g = decodeGraph({...raw, pos: shifted.subarray(1)});
    expect(g.nodes[0].pos).toEqual(decodeGraph(raw).nodes[0].pos);
  });

  it("decodes a graph delta and applies it like GraphState.apply", () => {
    const m = GraphModel.fromSnapshot(decodeGraph(unpack(bin("graph_small.msgpack"))));
    const d = decodeDelta(unpack(bin("delta_small.msgpack")));
    expect(d.seq).toBe(7);
    expect(d.deletes).toEqual(["O(2)"]);
    m.applyDelta(d);
    expect(stateOf(m)).toEqual(js("delta_small.json"));
    expect(m.lastSeq).toBe(7);
  });

  it("decodes a mesh tile (quantised int16 positions)", () => {
    const t = decodeTile(unpack(bin("tile_small.msgpack")));
    const exp = js("tile_small.json");
    expect(t.key).toBe(exp.key);
    expect(t.nv).toBe(exp.nv);
    expect(t.nf).toBe(exp.nf);
    expect(Array.from(t.colors)).toEqual(exp.col);
    expect(Array.from(t.indices)).toEqual(exp.idx);
    expect(t.positions.length).toBe(exp.pos.length);
    exp.pos.forEach((v: number, i: number) => expect(t.positions[i]).toBeCloseTo(v, 5));
  });

  it("parses node symbols", () => {
    expect(parseSym("O(12)")).toEqual({prefix: "O", index: 12});
    expect(parseSym("a(3)")).toEqual({prefix: "a", index: 3});
    expect(parseSym("weird").index).toBe(-1);
  });
});
