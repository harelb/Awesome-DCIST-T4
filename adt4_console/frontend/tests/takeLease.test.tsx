import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {TakeLeaseButton} from "../src/viewer/LeaseControl";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

describe("TakeLeaseButton", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
  });

  it("asks for the robot name before posting take_lease", async () => {
    const fetchMock = vi.fn(async (url: string) =>
      String(url).startsWith("api/time")
        ? new Response(JSON.stringify({t: Date.now() / 1000}), {status: 200, headers: {"Content-Type": "application/json"}})
        : new Response(JSON.stringify({ok: true, message: "lease owner before: bosdyn.android.spotapp; after: understanding-h"}), {status: 200, headers: {"Content-Type": "application/json"}}),
    );
    vi.stubGlobal("fetch", fetchMock);
    const onTaken = vi.fn();
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<TakeLeaseButton robot="hamilton" owner="bosdyn.android.spotapp x" onTaken={onTaken} />));
    act(() => (document.querySelector("button") as HTMLButtonElement).click());
    expect(document.body.textContent).toContain("from tablet");
    expect(document.body.textContent).toContain("planner may then command the robot");
    const yes = document.querySelector('[data-testid="take-lease-yes"]') as HTMLButtonElement;
    const input = document.querySelector('input[aria-label="Robot name to confirm"]') as HTMLInputElement;
    const setValue = (v: string) => {
      Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!.call(input, v);
      input.dispatchEvent(new Event("input", {bubbles: true}));
    };
    expect(yes.disabled).toBe(true);
    act(() => setValue("hamilto"));
    expect(yes.disabled).toBe(true);
    act(() => setValue("hamilton"));
    expect(yes.disabled).toBe(false);
    await act(async () => yes.click());
    const posts = fetchMock.mock.calls.filter(c => String(c[0]).includes("take_lease"));
    expect(posts.length).toBe(1);
    expect(String(posts[0][0])).toBe("api/robots/hamilton/take_lease");
    expect(onTaken).toHaveBeenCalled();
    expect(document.querySelector('[data-testid="take-lease-yes"]')).toBeNull();
  });
});
