import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {candidateOrdinals, scrollAnchor} from "../src/lib/chatTranscript";
import {ChatPanel} from "../src/viewer/ChatPanel";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const IMG = "data:image/jpeg;base64,/9j/AAAA";
const cand = (pid: string, index: number) => ({type: "object_candidate", pending_id: pid, index, label: "green trash can", score: 0.9 - index / 20, image_uri: IMG, centroid: [index, 0, 0], expires_in_s: 600});

describe("SAM3 candidate cards", () => {
  it("number each card within its search", () => {
    const items = [{type: "user"}, cand("a", 0), cand("a", 1), {type: "text"}, cand("b", 0)];
    expect(candidateOrdinals(items)).toEqual([null, {k: 1, n: 2}, {k: 2, n: 2}, null, {k: 1, n: 1}]);
  });

  it("anchor on the first card of this turn's newest search", () => {
    const items = [{type: "user"}, cand("a", 0), cand("a", 1), {type: "text"}, {type: "user"}, {type: "tool"}, cand("b", 0), cand("b", 1), cand("b", 2), {type: "text"}];
    expect(scrollAnchor(items, 4)).toBe(6);
    expect(scrollAnchor(items, 0)).toBe(6);
    expect(scrollAnchor(items.slice(0, 6), 4)).toBeNull(); // no search this turn: the bottom
    expect(scrollAnchor(items.slice(0, 4), 0)).toBe(1);
  });
});

describe("ChatPanel with a 9-candidate search", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
  });

  it("shows every card and scrolls to the first one, not just the last", async () => {
    // field run: 9 candidates for "green trash can"; the log pinned to the bottom showed
    // only the last card in the side pane, and that one (index 8) was the only approval
    const events = [
      {type: "tool", name: "search_robot_images", args: {query: "green trash can"}, result: {num_candidates: 9}},
      ...Array.from({length: 9}, (_, i) => cand("p-9", i)),
      {type: "text", text: "I've searched the robot's images and found 9 candidates."},
      {type: "transcript", items: []},
      {type: "done"},
    ];
    const body = events.map(e => JSON.stringify(e) + "\n").join("");
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string) =>
        String(url).endsWith("api/chat") ? new Response(body, {status: 200, headers: {"Content-Type": "application/x-ndjson"}}) : new Response("{}", {status: 200}),
      ),
    );
    // layout: each chat item 300 px tall, stacked in order (jsdom has none)
    Object.defineProperty(HTMLElement.prototype, "scrollHeight", {configurable: true, get: () => 12 * 300});
    HTMLElement.prototype.scrollTo = function (this: HTMLElement, o?: ScrollToOptions | number) {
      this.scrollTop = typeof o === "object" ? (o.top ?? 0) : (o ?? 0);
    } as HTMLElement["scrollTo"];
    vi.spyOn(HTMLElement.prototype, "getBoundingClientRect").mockImplementation(function (this: HTMLElement) {
      const log = this.closest(".chat-log");
      const k = log && this !== log ? [...log.children].indexOf(this) : 0;
      const top = this === log ? 0 : k * 300 - (log?.scrollTop ?? 0);
      return {top, bottom: top + 300, left: 0, right: 300, width: 300, height: 300, x: 0, y: top, toJSON: () => ({})} as DOMRect;
    });

    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    await act(async () => root!.render(<ChatPanel />));
    const box = host.querySelector<HTMLTextAreaElement>("textarea[aria-label='Chat message']")!;
    await act(async () => {
      Object.getOwnPropertyDescriptor(HTMLTextAreaElement.prototype, "value")!.set!.call(box, "go to the green trash can");
      box.dispatchEvent(new Event("input", {bubbles: true}));
    });
    await act(async () => {
      host!.querySelector<HTMLButtonElement>(".chat-input button")!.click();
      await new Promise(r => setTimeout(r, 50));
    });

    const cards = [...host.querySelectorAll(".msg-candidate")];
    expect(cards).toHaveLength(9);
    expect(cards.map(c => c.textContent?.includes(`${cards.indexOf(c) + 1} of 9`))).toEqual(Array(9).fill(true));
    // every card can be approved
    expect(cards.map(c => (c.querySelector("button.btn-primary") as HTMLButtonElement).disabled)).toEqual(Array(9).fill(false));
    // the first card is at the top of the log (user message + tool card above it)
    const log = host.querySelector<HTMLDivElement>(".chat-log")!;
    const first = [...log.children].indexOf(cards[0]);
    expect(log.scrollTop).toBe(first * 300);
  });
});
