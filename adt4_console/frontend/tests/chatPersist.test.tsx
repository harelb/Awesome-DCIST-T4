import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {ChatPanel} from "../src/viewer/ChatPanel";
import {CHAT_STORAGE_KEY, clearChat, storable, useChat} from "../src/stores/chat";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

describe("chat conversation outlives the panel", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
    vi.useRealTimers();
    clearChat();
  });

  const mount = async () => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    await act(async () => root!.render(<ChatPanel />));
  };

  it("shows the earlier turns again after switching pages (unmount + mount)", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => new Response("{}", {status: 200})));
    useChat.setState({
      items: [
        {type: "user", text: "how many trash cans are there?"},
        {type: "text", text: "The map has 1 trash can."},
      ],
      transcript: [{role: "user", text: "how many trash cans are there?"}],
    });
    await mount();
    expect(host!.textContent).toContain("The map has 1 trash can.");
    act(() => root!.unmount());
    host!.remove();
    await mount();
    expect(host!.textContent).toContain("how many trash cans are there?");
    expect(host!.textContent).toContain("The map has 1 trash can.");
    expect(useChat.getState().transcript).toHaveLength(1); // the model keeps its context too
  });

  it("saves to localStorage without embedded candidate images", async () => {
    vi.useFakeTimers();
    useChat.setState({
      items: [
        {type: "user", text: "find the green bin"},
        {type: "object_candidate", pending_id: "p", index: 0, label: "bin", score: 0.9, image_uri: "data:image/jpeg;base64,AAAA", centroid: [1, 2, 0], bbox_min: null, bbox_max: null},
      ],
    });
    vi.advanceTimersByTime(400);
    const saved = JSON.parse(window.localStorage.getItem(CHAT_STORAGE_KEY)!);
    expect(saved.items).toHaveLength(2);
    expect(saved.items[1].image_uri).toBeNull();
    expect(useChat.getState().items[1]).toMatchObject({image_uri: "data:image/jpeg;base64,AAAA"}); // shown while open
  });

  it("caps what is stored", () => {
    const many = Array.from({length: 400}, (_, i) => ({type: "user" as const, text: `m${i}`}));
    const kept = storable(many);
    expect(kept).toHaveLength(300);
    expect(kept[0]).toMatchObject({text: "m100"});
  });

  it("New chat clears it everywhere", () => {
    useChat.setState({items: [{type: "user", text: "x"}], transcript: [{role: "user", text: "x"}], busy: true});
    clearChat();
    expect(useChat.getState()).toMatchObject({items: [], transcript: [], busy: false});
  });
});
