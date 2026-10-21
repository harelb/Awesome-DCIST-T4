import {describe, expect, it} from "vitest";
import {toChatItem} from "../src/viewer/ChatPanel";
import {candidateNote, cellText, fmtLeft, trimTranscript, writeNote, type TranscriptItem} from "../src/lib/chatTranscript";

describe("Cypher chat events", () => {
  it("write proposals and image sets become cards", () => {
    expect(toChatItem({type: "cypher_write", id: "w1-1", query: "MERGE (n:X)", summary: "add X", src: "neo4j:live"})).toEqual({
      type: "cypher_write", id: "w1-1", query: "MERGE (n:X)", summary: "add X", src: "neo4j:live",
    });
    const im = toChatItem({type: "images", src: "neo4j:live", items: [{sym: "O(7)", label: "tree", images: ["api/graph/neo4j:live/image/O(7)/0?w=480&crop=1"]}]});
    expect(im).toMatchObject({type: "images", items: [{sym: "O(7)", label: "tree"}]});
    expect(toChatItem({type: "transcript", items: []})).toBeNull(); // kept as state, not drawn
  });
  it("goal proposals keep the robot the server named", () => {
    expect(toChatItem({type: "goal", kind: "pddl", body: {goal: "(visited-object o7)"}, robot: "hamilton"})).toMatchObject({robot: "hamilton"});
  });
});

describe("transcript", () => {
  it("is trimmed to start at a user message", () => {
    const items: TranscriptItem[] = [
      {role: "user", text: "a"},
      {role: "assistant", text: "", calls: [{id: "1", name: "run_cypher", args: {}}]},
      {role: "tool", id: "1", name: "run_cypher", content: "[]"},
      {role: "assistant", text: "done"},
      {role: "user", text: "b"},
    ];
    expect(trimTranscript(items, 3).map(i => i.role)).toEqual(["user"]);
    expect(trimTranscript(items, 5)).toHaveLength(5);
  });
  it("tells the assistant what the operator decided", () => {
    expect(writeNote("add X", {ok: true, counters: {nodes_created: 1}})).toBe("[operator] Approved and ran the write (add X): nodes created 1.");
    expect(writeNote("add X", {ok: true, rejected: true})).toContain("Rejected");
    expect(writeNote("add X", {ok: false, error: "syntax"})).toContain("failed: syntax");
    expect(candidateNote("mug", {symbol: "O31"})).toContain("added to the map as O31");
    expect(candidateNote("mug", {symbol: "O12", merged: true})).toContain("merged into the mapped object O12");
  });
  it("formats cells and countdowns", () => {
    expect(cellText([1.234, 5, -2.5])).toBe("[1.23, 5, -2.50]");
    expect(cellText(null)).toBe("");
    expect(cellText({a: 1})).toBe('{"a":1}');
    expect(fmtLeft(125)).toBe("2:05");
    expect(fmtLeft(0)).toBe("expired");
  });
});

import {looksTabular} from "../src/viewer/ChatPanel";
describe("tabular replies", () => {
  it("bar charts and pipe tables get a fixed-width block", () => {
    expect(looksTabular("tree | ████ (220)\nvehicle | ██ (137)")).toBe(true);
    expect(looksTabular("a | 1\nb | 2\nc | 3")).toBe(true);
    expect(looksTabular("O(7) is a vehicle | parked.")).toBe(false);
  });
});
