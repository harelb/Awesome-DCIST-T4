// @vitest-environment jsdom
import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {SceneEditCard, sceneEditNote, sceneEditStatus, toSceneEditItem, type SceneEditItem} from "../src/viewer/SceneEditCard";
import {toChatItem} from "../src/viewer/ChatPanel";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const raw = {type: "scene_edit", id: "s1-1", tool: "remove_scene_place", args: {symbol: "t5571"}, summary: "blocked passage",
  text: "Delete place t5571 and its connections", src: "neo4j:live"};

describe("scene edit card", () => {
  let root: Root | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    root = null;
    document.body.innerHTML = "";
  });

  const mount = (it: SceneEditItem, onDecide = vi.fn(), onUndo = vi.fn()) => {
    const host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<SceneEditCard it={it} onDecide={onDecide} onUndo={onUndo} />));
    return {host, onDecide, onUndo};
  };

  it("parses from the chat stream", () => {
    const it = toChatItem(raw) as SceneEditItem;
    expect(it).toEqual(toSceneEditItem(raw));
    expect(it.type).toBe("scene_edit");
    expect(it.args).toEqual({symbol: "t5571"});
  });

  it("asks before it runs: approve or reject, no undo yet", () => {
    const {host, onDecide} = mount(toSceneEditItem(raw));
    expect(host.textContent).toContain("Delete place t5571 and its connections");
    expect(host.textContent).toContain("remove_scene_place");
    const [approve, reject] = Array.from(host.querySelectorAll("button"));
    expect(host.querySelectorAll("button").length).toBe(2);
    act(() => approve.click());
    expect(onDecide).toHaveBeenCalledWith(true);
    act(() => reject.click());
    expect(onDecide).toHaveBeenCalledWith(false);
  });

  it("shows the tool's result and offers Undo for an applied edit", () => {
    const r = {ok: true, result: "Deleted place t5571 and its 7 edges (map_version 2). It can be undone with undo_scene_edit.", undoable: true};
    const it = {...toSceneEditItem(raw), status: sceneEditStatus(r), undoable: true};
    const {host, onUndo} = mount(it);
    expect(host.textContent).toContain("Deleted place t5571 and its 7 edges (map_version 2)");
    const undo = Array.from(host.querySelectorAll("button")).find(b => b.textContent === "Undo")!;
    act(() => undo.click());
    expect(onUndo).toHaveBeenCalled();
    // approved once: approve / reject are disabled
    const approve = host.querySelectorAll("button")[0] as HTMLButtonElement;
    expect(approve.disabled).toBe(true);
  });

  it("a refused edit is an error, not undoable", () => {
    const st = sceneEditStatus({ok: false, result: "Refused: there is no place t9."});
    expect(st).toEqual({state: "error", text: "Refused: there is no place t9."});
    const {host} = mount({...toSceneEditItem(raw), status: st});
    expect(Array.from(host.querySelectorAll("button")).some(b => b.textContent === "Undo")).toBe(false);
  });

  it("tells the assistant the outcome", () => {
    const it = toSceneEditItem(raw);
    expect(sceneEditNote(it, {ok: true, rejected: true})).toContain("Rejected the scene edit");
    expect(sceneEditNote(it, {ok: true, result: "Deleted place t5571"})).toBe('[operator] Approved the scene edit "blocked passage": Deleted place t5571');
    expect(sceneEditNote(it, {ok: false, result: "Refused: x"})).toContain("was not applied: Refused: x");
  });
});
