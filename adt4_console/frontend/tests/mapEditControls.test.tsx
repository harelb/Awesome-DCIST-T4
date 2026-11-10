// @vitest-environment jsdom
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";

const api = vi.hoisted(() => ({getJSON: vi.fn(), postJSON: vi.fn()}));
vi.mock("../src/lib/api", () => ({
  getJSON: api.getJSON,
  postJSON: api.postJSON,
  errorText: (e: unknown) => String((e as Error)?.message ?? e),
}));

import {MapEditControls} from "../src/viewer/MapEditControls";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const flush = () => act(async () => {
  await Promise.resolve();
  await Promise.resolve();
});

describe("map edit controls (plan repair undo / clear / reset)", () => {
  let root: Root | null = null;
  let host: HTMLDivElement;
  beforeEach(() => {
    api.getJSON.mockReset();
    api.postJSON.mockReset();
  });
  afterEach(() => {
    act(() => root?.unmount());
    root = null;
    document.body.innerHTML = "";
  });
  const mount = async () => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<MapEditControls src="neo4j:live" pollMs={0} />));
    await flush();
  };
  const btn = (id: string) => host.querySelector(`[data-testid="${id}"]`) as HTMLButtonElement;
  const click = async (id: string) => {
    act(() => btn(id).click());
    await flush();
  };

  it("is disabled with the reason while the writer is not running", async () => {
    api.getJSON.mockResolvedValue({writer: "/willow/scene_change_writer", running: false, reason: "plan repair's scene change writer is not running"});
    await mount();
    expect(btn("map-undo").disabled).toBe(true);
    expect(btn("map-reset").disabled).toBe(true);
    expect(host.querySelector('[data-testid="map-edit-why"]')!.textContent).toContain("not running");
  });

  it("undo asks first, then calls the route and shows the writer's message", async () => {
    api.getJSON.mockResolvedValue({writer: "/willow/scene_change_writer", running: true, reason: null});
    api.postJSON.mockResolvedValue({ok: true, action: "undo", message: "Removed the latest edit; map rebuilt"});
    await mount();
    await click("map-undo");
    expect(api.postJSON).not.toHaveBeenCalled();
    expect(host.querySelector('[data-testid="map-edit-confirm"]')!.textContent).toContain("Undo the latest map edit");
    await click("map-edit-go");
    expect(api.postJSON).toHaveBeenCalledWith("api/maps/scene/undo", {src: "neo4j:live", confirm: undefined});
    expect(host.querySelector('[data-testid="map-edit-result"]')!.textContent).toBe("Removed the latest edit; map rebuilt");
  });

  it("reset needs the typed confirm", async () => {
    api.getJSON.mockResolvedValue({writer: "/willow/scene_change_writer", running: true, reason: null});
    api.postJSON.mockResolvedValue({ok: true, action: "reset", message: "Map reset to the prior"});
    await mount();
    await click("map-reset");
    expect(btn("map-edit-go").disabled).toBe(true);
    const input = host.querySelector('input[aria-label="Type reset to confirm"]') as HTMLInputElement;
    const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
    act(() => {
      setter.call(input, "reset");
      input.dispatchEvent(new Event("input", {bubbles: true}));
    });
    expect(btn("map-edit-go").disabled).toBe(false);
    await click("map-edit-go");
    expect(api.postJSON).toHaveBeenCalledWith("api/maps/scene/reset", {src: "neo4j:live", confirm: "reset"});
  });

  it("shows a refusal from the writer as an error", async () => {
    api.getJSON.mockResolvedValue({writer: "/willow/scene_change_writer", running: true, reason: null});
    api.postJSON.mockResolvedValue({ok: false, action: "clear", message: "No edits to clear"});
    await mount();
    await click("map-clear");
    await click("map-edit-go");
    const r = host.querySelector('[data-testid="map-edit-result"]')!;
    expect(r.textContent).toBe("No edits to clear");
    expect(r.className).toContain("inline-err");
  });
});
