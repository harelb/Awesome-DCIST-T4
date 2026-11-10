import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {imagePixel, ManipulationApprovalDialog, ManipulationApprovalHost, validSelection, type ManipulationRequest} from "../src/viewer/ManipulationApproval";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

function fixture(id = "grasp-1"): ManipulationRequest {
  return {id, robot: "euclid", received_at: Date.now() / 1000,
    expires_at: Date.now() / 1000 + 300,
    images: [
      {index: 0, name: "frontleft", width: 640, height: 480, url: `/api/test/${id}/0.png`},
      {index: 1, name: "frontright", width: 1280, height: 720, url: `/api/test/${id}/1.png`},
    ], selection: {image_index: 0, x: 100, y: 80}};
}

let root: Root | null = null;
let container: HTMLDivElement;
afterEach(() => {
  if (root) act(() => root?.unmount());
  root = null;
  container?.remove();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
});

function render(request = fixture(), onRespond = vi.fn().mockResolvedValue(undefined), onDismiss = vi.fn(),
  extra: {headingSelection?: boolean} = {}) {
  container = document.createElement("div");
  document.body.append(container);
  root = createRoot(container);
  act(() => root!.render(<ManipulationApprovalDialog key={request.id} request={request}
    onRespond={onRespond} onDismiss={onDismiss} {...extra} />));
  return {onRespond, onDismiss};
}

function bounds(image: HTMLImageElement, width: number, height: number) {
  image.getBoundingClientRect = () => ({left: 0, top: 0, width, height, x: 0, y: 0, right: width, bottom: height, toJSON: () => ({})});
}

function button(text: string) {
  const result = [...container.querySelectorAll("button")].find(item => item.textContent?.includes(text));
  if (!result) throw new Error("Button missing: " + text);
  return result;
}

function imageLoaded() {
  const image = container.querySelector("img")!;
  act(() => image.dispatchEvent(new Event("load", {bubbles: true})));
  return image;
}

describe("manipulation image selection", () => {
  it("maps scaled image pixels and refuses clicks outside the image", () => {
    const bounds = {left: 10, top: 20, width: 320, height: 180};
    expect(imagePixel(170, 110, bounds, 1280, 720)).toEqual({x: 640, y: 360});
    expect(imagePixel(329.9, 199.9, bounds, 1280, 720)).toEqual({x: 1279, y: 719});
    expect(imagePixel(330, 200, bounds, 1280, 720)).toBeNull();
    expect(imagePixel(5, 30, bounds, 1280, 720)).toBeNull();
  });

  it("validates detection coordinates against the chosen image", () => {
    const request = fixture();
    expect(validSelection(request, request.selection)).toBe(true);
    expect(validSelection(request, {image_index: 0, x: 640, y: 80})).toBe(false);
    expect(validSelection(request, {image_index: 8, x: 5, y: 5})).toBe(false);
    expect(validSelection(request, {image_index: 0, x: 5.5, y: 5})).toBe(false);
  });
});

describe("manipulation approval dialog", () => {
  it("never approves on arrival or Enter, then sends the reviewed detection explicitly", async () => {
    const {onRespond} = render();
    expect(button("Approve selected").disabled).toBe(true);
    expect(onRespond).not.toHaveBeenCalled();
    imageLoaded();
    act(() => container.querySelector('[role="dialog"]')!.dispatchEvent(new KeyboardEvent("keydown", {key: "Enter", bubbles: true})));
    expect(onRespond).not.toHaveBeenCalled();
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent).toContain("pixel (100, 80)");
    await act(async () => button("Approve selected").click());
    expect(onRespond).toHaveBeenCalledExactlyOnceWith({approve: true, image_index: 0, image_x: 100, image_y: 80});
  });

  it("requires an explicit override and uses the full-resolution selected pixel", async () => {
    const request = fixture();
    request.selection = null;
    const {onRespond} = render(request);
    const image = imageLoaded();
    image.getBoundingClientRect = () => ({left: 10, top: 20, width: 320, height: 240,
      x: 10, y: 20, right: 330, bottom: 260, toJSON: () => ({})});
    act(() => image.dispatchEvent(new MouseEvent("click", {clientX: 170, clientY: 140, bubbles: true})));
    expect(button("Approve selected").disabled).toBe(true);
    act(() => container.querySelector<HTMLInputElement>('input[type="checkbox"]')!.click());
    act(() => image.dispatchEvent(new MouseEvent("click", {clientX: 170, clientY: 140, bubbles: true})));
    await act(async () => button("Approve selected").click());
    expect(onRespond).toHaveBeenCalledExactlyOnceWith({approve: true, image_index: 0, image_x: 320, image_y: 240});
  });

  it("closing sends nothing and rejection needs no selected pixel", async () => {
    const request = fixture();
    request.selection = null;
    const {onRespond, onDismiss} = render(request);
    act(() => button("Close without response").click());
    expect(onDismiss).toHaveBeenCalledOnce();
    expect(onRespond).not.toHaveBeenCalled();
    await act(async () => button("Reject grasp").click());
    expect(onRespond).toHaveBeenCalledExactlyOnceWith({approve: false});
  });

  it("a new pending request resets selection and never reuses an override", () => {
    render();
    imageLoaded();
    const next = fixture("grasp-2");
    next.selection = {image_index: 1, x: 50, y: 60};
    act(() => root!.render(<ManipulationApprovalDialog key={next.id} request={next}
      onRespond={vi.fn()} onDismiss={vi.fn()} />));
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent).toContain("frontright · pixel (50, 60)");
    expect(button("Approve selected").disabled).toBe(true);
  });

  it("walk-to sends the selected point without approving; heading controls stay hidden by default", async () => {
    const {onRespond} = render();
    expect(button("Walk to selected point").disabled).toBe(true);
    imageLoaded();
    expect(container.textContent).not.toContain("Choose stand point");
    expect(container.querySelectorAll('input[type="checkbox"]').length).toBe(1);
    await act(async () => button("Walk to selected point").click());
    expect(onRespond).toHaveBeenCalledExactlyOnceWith({approve: false, image_index: 0, image_x: 100, image_y: 80, walk_to: true});
  });

  it("with heading selection enabled, a stand point on the same image rides along with walk-to only", async () => {
    const {onRespond} = render(fixture(), vi.fn().mockResolvedValue(undefined), vi.fn(), {headingSelection: true});
    const image = imageLoaded();
    bounds(image, 640, 480);
    const checkboxes = container.querySelectorAll<HTMLInputElement>('input[type="checkbox"]');
    expect(checkboxes.length).toBe(2);
    act(() => checkboxes[1].click());
    expect(container.textContent).toContain("Click the ground pixel the robot should stand on");
    act(() => image.dispatchEvent(new MouseEvent("click", {clientX: 200, clientY: 400, bubbles: true})));
    expect(container.querySelector('[data-testid="stand-selection"]')!.textContent).toContain("(200, 400)");
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent).toContain("pixel (100, 80)");
    expect(container.querySelector('[data-testid="stand-marker"]')).not.toBeNull();
    await act(async () => button("Approve selected").click());
    expect(onRespond).toHaveBeenLastCalledWith({approve: true, image_index: 0, image_x: 100, image_y: 80});
    await act(async () => button("Walk to selected point").click());
    expect(onRespond).toHaveBeenLastCalledWith({approve: false, image_index: 0, image_x: 100, image_y: 80,
      walk_to: true, stand_x: 200, stand_y: 400});
  });

  it("a stand point on another image than the target is ignored", async () => {
    const {onRespond} = render(fixture(), vi.fn().mockResolvedValue(undefined), vi.fn(), {headingSelection: true});
    act(() => button("Next").click());
    const image = imageLoaded();
    bounds(image, 1280, 720);
    act(() => container.querySelectorAll<HTMLInputElement>('input[type="checkbox"]')[1].click());
    act(() => image.dispatchEvent(new MouseEvent("click", {clientX: 640, clientY: 360, bubbles: true})));
    expect(container.querySelector('[data-testid="stand-selection"]')!.textContent).toContain("ignored");
    act(() => button("Previous").click());
    imageLoaded();
    await act(async () => button("Walk to selected point").click());
    expect(onRespond).toHaveBeenCalledExactlyOnceWith({approve: false, image_index: 0, image_x: 100, image_y: 80, walk_to: true});
  });

  it("overlays depth, reads it under the cursor, and reports depth at the selected point", async () => {
    const request = fixture();
    request.images[0] = {...request.images[0], depth_url: "/api/test/d/0.png", point_query_url: "/api/test/p/0",
      ground_estimate: true};
    request.depth_range_m = [0.3, 6];
    request.walk_to = {standoff_m: 1, max_distance_m: 5};
    const fetchMock = vi.fn(async (url: string) => {
      const query = new URL(url, "http://x").searchParams;
      const x = Number(query.get("x")), y = Number(query.get("y"));
      const body = x === 100
        ? {image_index: 0, x, y, has_depth: true, depth_m: 1.42, ground: {hit: true, distance_m: 6.1, walk_m: 5, capped: true}}
        : {image_index: 0, x, y, has_depth: true, depth_m: null, ground: {hit: false}};
      return new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});
    });
    vi.stubGlobal("fetch", fetchMock);
    render(request);
    const image = imageLoaded();
    bounds(image, 640, 480);
    expect(container.querySelector('[data-testid="depth-overlay"]')).not.toBeNull();
    await act(async () => { await Promise.resolve(); await Promise.resolve(); });
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent)
      .toContain("Walk To (ground plane): 6.10 m away → walks 5.00 m (capped) · measured depth 1.42 m (display only)");
    expect(button("Walk to selected point").textContent).toBe("Walk to selected point (5.0 m, ground plane)");
    expect(container.querySelector('[data-testid="walk-note"]')!.textContent).toContain("are not used");
    expect(container.querySelector('[data-testid="walk-note"]')!.textContent).toContain("moves at most 5.0 m");

    await act(async () => { image.dispatchEvent(new MouseEvent("mousemove", {clientX: 300, clientY: 200, bubbles: true})); });
    await act(async () => { await Promise.resolve(); await Promise.resolve(); });
    expect(container.querySelector('[data-testid="depth-readout"]')!.textContent)
      .toBe("Cursor (300, 200): Walk To (ground plane): ray misses the floor · measured depth: none");
    expect(fetchMock.mock.calls.some(([url]) => String(url).includes("/api/test/p/0?x=300&y=200"))).toBe(true);

    const toggle = [...container.querySelectorAll<HTMLInputElement>('input[type="checkbox"]')]
      .find(item => item.parentElement?.textContent?.includes("Show depth"))!;
    act(() => toggle.click());
    expect(container.querySelector('[data-testid="depth-overlay"]')).toBeNull();
  });

  it("shows the ground-plane estimate even when an image has no depth", async () => {
    const request = fixture();
    request.images[0] = {...request.images[0], point_query_url: "/api/test/p/0", ground_estimate: true};
    request.walk_to = {standoff_m: 1, max_distance_m: 5};
    vi.stubGlobal("fetch", vi.fn(async () => new Response(JSON.stringify(
      {image_index: 0, x: 100, y: 80, has_depth: false, depth_m: null, ground: {hit: true, distance_m: 0.9, walk_m: 0, capped: false}}),
      {status: 200, headers: {"Content-Type": "application/json"}})));
    render(request);
    imageLoaded();
    await act(async () => { await Promise.resolve(); await Promise.resolve(); });
    expect(container.textContent).not.toContain("Show depth");
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent).toContain("Walk To (ground plane): 0.90 m away → turns only");
    expect(button("Walk to selected point").textContent).toContain("(turn only, ground plane)");
  });

  it("warns that hand-camera walk estimates are unreliable", async () => {
    const request = fixture();
    request.images[0] = {...request.images[0], name: "hand_color_image", point_query_url: "/api/test/p/0", ground_estimate: true,
      ground_estimate_warning: "Hand camera: walk distances from this camera are unreliable. Pick walk-to points in a body camera."};
    request.walk_to = {standoff_m: 1.5, max_distance_m: 5};
    vi.stubGlobal("fetch", vi.fn(async () => new Response(JSON.stringify(
      {image_index: 0, x: 100, y: 80, has_depth: false, depth_m: null, ground: {hit: true, distance_m: 3.58, walk_m: 2.08, capped: false}}),
      {status: 200, headers: {"Content-Type": "application/json"}})));
    render(request);
    imageLoaded();
    await act(async () => { await Promise.resolve(); await Promise.resolve(); });
    expect(container.querySelector('[data-testid="estimate-warning"]')!.textContent).toContain("unreliable");
    expect(container.querySelector('[data-testid="grasp-selection"]')!.textContent).toContain("(unreliable for this camera)");
    expect(button("Walk to selected point").textContent).toBe("Walk to selected point (2.1 m, ground plane, unreliable)");
    expect(container.querySelector('[data-testid="walk-note"]')!.textContent).toContain("stops 1.5 m short");
  });

  it("shows no depth controls for images without depth", () => {
    render();
    imageLoaded();
    expect(container.textContent).not.toContain("Show depth");
    expect(container.querySelector('[data-testid="depth-overlay"]')).toBeNull();
  });

  it("shows response failures and clearly identifies simulation", async () => {
    const request = {...fixture(), simulated: true};
    render(request, vi.fn().mockRejectedValue(new Error("Request was superseded")));
    imageLoaded();
    expect(container.textContent).toContain("SIMULATION — no robot response");
    expect(container.textContent).toContain("including pick or drag");
    await act(async () => button("Approve selected").click());
    expect(container.querySelector('[role="alert"]')!.textContent).toBe("Request was superseded");
  });
});

describe("simulated manipulation request", () => {
  it("opens the test dialog without a robot connection or any automatic response", async () => {
    const request = {...fixture(), simulated: true};
    let demo = false;
    const fetchMock = vi.fn(async (url: string, init?: RequestInit) => {
      if (url === "api/manipulation/demo") demo = true;
      return new Response(JSON.stringify(url.endsWith("pending")
        ? {available: false, error: "ROS unavailable", requests: demo ? [request] : []}
        : {id: request.id}), {status: 200});
    });
    vi.stubGlobal("fetch", fetchMock);
    container = document.createElement("div");
    document.body.append(container);
    root = createRoot(container);
    await act(async () => root!.render(<ManipulationApprovalHost />));
    await act(async () => button("Test manipulation dialog").click());
    expect(container.textContent).toContain("SIMULATION — no robot response");
    expect(container.textContent).not.toContain("ROS unavailable");
    imageLoaded();
    expect(button("Approve selected").disabled).toBe(false);
    expect(fetchMock.mock.calls.some(([url]) => url.endsWith("/respond"))).toBe(false);
  });
});
