import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {GraphModel} from "../src/lib/graphModel";
import {batchUpdateOps, common, descendants, editLayerOf, nodesInPolygon, pointInPolygon, selectionSummary, uiSym} from "../src/lib/mapEdit";
import {finishLasso, onEditClick, useMapEdit} from "../src/stores/mapEdit";
import {useViewer} from "../src/stores/viewer";
import {MapEditCard, mapEditNote, statusOf, toMapEditItem} from "../src/viewer/MapEditCard";
import {toChatItem} from "../src/viewer/ChatPanel";
import type {NodeRecord} from "../src/codec";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const node = (sym: string, layer: number, part: number, pos: [number, number, number], label = 1, name = ""): NodeRecord => ({sym, layer, part, pos, label, bbox: null, color: [0, 0, 0], name, active: false});

function model(): GraphModel {
  return GraphModel.fromSnapshot({
    nodes: [
      node("O(1)", 2, 0, [1, 1, 0]),
      node("O(2)", 2, 0, [3, 1, 0], 2),
      node("O(3)", 2, 0, [20, 20, 0]),
      node("P(1)", 3, 1, [1, 1, 0]),
      node("P(2)", 3, 1, [3, 1, 0]),
      node("R(0)", 4, 0, [2, 1, 0], 1, "courtyard"),
      node("a(0)", 2, 97, [1, 1, 0]),
    ],
    edges: [
      ["R(0)", "P(1)"],
      ["R(0)", "P(2)"],
      ["P(1)", "O(1)"],
      ["P(2)", "O(2)"],
      ["P(1)", "P(2)"],
    ],
    labelspaces: {"2/0": {"1": "tree", "2": "vehicle"}},
  } as never);
}

describe("map edit helpers", () => {
  it("names the heracles layer of a viewer node", () => {
    expect(editLayerOf({layer: 2, part: 0})).toBe("Object");
    expect(editLayerOf({layer: 3, part: 1})).toBe("MeshPlace");
    expect(editLayerOf({layer: 4, part: 0})).toBe("Room");
    expect(editLayerOf({layer: 2, part: 97})).toBeNull(); // agents are not editable
  });
  it("summarises a selection like SGET's status line", () => {
    const m = model();
    expect(selectionSummary(["O(1)", "O(2)", "O(3)", "R(0)"], m)).toBe("3 Objects, 1 Area");
    expect(selectionSummary(["P(1)"], m)).toBe("1 Place");
    expect(selectionSummary([], m)).toBe("nothing selected");
  });
  it("lasso membership by layer", () => {
    const m = model();
    const sq = [[0, 0], [5, 0], [5, 5], [0, 5]];
    expect(pointInPolygon(1, 1, sq)).toBe(true);
    expect(nodesInPolygon(m, sq, ["Object"]).sort()).toEqual(["O(1)", "O(2)"]);
    expect(nodesInPolygon(m, sq).sort()).toEqual(["O(1)", "O(2)", "P(1)", "P(2)", "R(0)"]);
  });
  it("a region's subtree follows parent -> child edges only", () => {
    expect([...descendants(model(), "R(0)")].sort()).toEqual(["O(1)", "O(2)", "P(1)", "P(2)"]);
    expect([...descendants(model(), "P(1)")]).toEqual(["O(1)"]); // the sibling edge is not a child
  });
  it("symbols, common values and batch ops", () => {
    expect(uiSym("O578")).toBe("O(578)");
    expect(uiSym("O(5)")).toBe("O(5)");
    expect(common([1, 1], String)).toBe("1");
    expect(common([1, 2], String)).toBeNull();
    expect(batchUpdateOps(["O(1)", "O(2)"], {class: "crate", name: ""})).toEqual([
      {op: "update_node", symbol: "O(1)", set: {class: "crate"}},
      {op: "update_node", symbol: "O(2)", set: {class: "crate"}},
    ]);
    expect(batchUpdateOps(["O(1)"], {})).toEqual([]);
  });
});

describe("edit mode clicks", () => {
  beforeEach(() => {
    const m = model();
    useViewer.setState({sources: {"neo4j:penn": {id: "neo4j:penn", kind: "neo4j", label: "penn", enabled: true, status: "ready", error: null, model: m, rev: 1} as never}, order: ["neo4j:penn"]});
    useMapEdit.setState({on: true, src: "neo4j:penn", selection: [], tool: "select", lasso: [], lassoLayers: ["Object"]});
  });
  it("click selects, shift-click adds and toggles, a miss clears", () => {
    onEditClick("neo4j:penn", "O(1)", [1, 1, 0]);
    expect(useMapEdit.getState().selection).toEqual(["O(1)"]);
    window.dispatchEvent(new MouseEvent("pointerup", {shiftKey: true}));
    onEditClick("neo4j:penn", "O(2)", [3, 1, 0]);
    expect(useMapEdit.getState().selection).toEqual(["O(1)", "O(2)"]);
    onEditClick("neo4j:penn", "O(1)", [1, 1, 0]);
    expect(useMapEdit.getState().selection).toEqual(["O(2)"]);
    window.dispatchEvent(new MouseEvent("pointerup", {shiftKey: false}));
    onEditClick(null, null, [50, 50, 0]);
    expect(useMapEdit.getState().selection).toEqual([]);
    // nodes of another source are not selectable
    onEditClick("file:prior", "O(1)", [1, 1, 0]);
    expect(useMapEdit.getState().selection).toEqual([]);
  });
  it("the lasso collects corners and selects inside", () => {
    useMapEdit.setState({tool: "lasso"});
    for (const p of [[0, 0], [5, 0], [5, 5], [0, 5]]) onEditClick(null, null, [p[0], p[1], 0]);
    expect(useMapEdit.getState().lasso).toHaveLength(4);
    finishLasso();
    expect(useMapEdit.getState().selection.sort()).toEqual(["O(1)", "O(2)"]);
    expect(useMapEdit.getState().tool).toBe("select");
  });
});

describe("chat map edit card", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
  });
  const card = toMapEditItem({type: "map_edit", id: "e1-1", src: "neo4j:penn", summary: "it is a crate", diff: ["O41 class vehicle -> crate"], affected: ["O41"], staged: false});
  it("streams into a card item", () => {
    expect(toChatItem({type: "map_edit", id: "e1-1", diff: ["x"], ops: []})).toMatchObject({type: "map_edit", id: "e1-1", diff: ["x"], staged: false});
  });
  it("tells the assistant what the operator decided", () => {
    expect(mapEditNote(card, {ok: true, rejected: true})).toMatch(/Rejected .*nothing changed/);
    expect(mapEditNote(card, {ok: true, staged: true})).toMatch(/staged/);
    expect(mapEditNote(card, {ok: true, summary: ["O41 class vehicle -> crate"]})).toMatch(/applied .*O41 class vehicle -> crate/);
    expect(statusOf({ok: false, error: "boom"})).toEqual({state: "error", text: "boom"});
  });
  it("renders the diff and approves", () => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    const onDecide = vi.fn();
    act(() => root!.render(<MapEditCard it={card} onDecide={onDecide} />));
    expect(host.textContent).toContain("O41 class vehicle -> crate");
    expect(host.textContent).toContain("O(41)");
    const approve = [...host.querySelectorAll("button")].find(b => b.textContent?.includes("Approve"))!;
    act(() => approve.click());
    expect(onDecide).toHaveBeenCalledWith(true);
  });
});
