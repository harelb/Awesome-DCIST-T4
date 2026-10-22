import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {useLidarReenable, useOverlays} from "../src/stores/overlays";
import {useRobots} from "../src/stores/robots";
import {LidarToggle} from "../src/viewer/LivePanel";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

type L = {available: boolean; enabled: boolean} | null;
function Probe({lidar}: {lidar: L}) {
  useLidarReenable("gauss", lidar, 10);
  return null;
}

describe("useLidarReenable", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  let posts: unknown[] = [];

  beforeEach(() => {
    vi.useFakeTimers();
    posts = [];
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string, init?: RequestInit) => {
        posts.push([String(url), JSON.parse(String(init?.body ?? "null"))]);
        return new Response(JSON.stringify({ok: true}), {status: 200, headers: {"Content-Type": "application/json"}});
      }),
    );
    useOverlays.setState({lidarOn: {}, lidarError: {}});
    host = document.createElement("div");
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root?.unmount());
    vi.useRealTimers();
    vi.unstubAllGlobals();
  });

  const render = (lidar: L) => act(() => root!.render(<Probe lidar={lidar} />));

  it("a remount that finds the robot off asks at once, then every 10 s until enabled", () => {
    render({available: true, enabled: false});
    expect(posts).toEqual([["api/live/gauss/lidar", {enabled: true}]]);
    act(() => void vi.advanceTimersByTime(9_000));
    expect(posts.length).toBe(1);
    act(() => void vi.advanceTimersByTime(1_000));
    expect(posts.length).toBe(2);
    act(() => void vi.advanceTimersByTime(20_000));
    expect(posts.length).toBe(4);
    render({available: true, enabled: true});
    act(() => void vi.advanceTimersByTime(60_000));
    expect(posts.length).toBe(4);
    expect(useOverlays.getState().lidarOn.gauss).toBe(true);
  });

  it("asks nothing without a lidar, or once unmounted", () => {
    render(null);
    render({available: false, enabled: false});
    act(() => void vi.advanceTimersByTime(30_000));
    expect(posts.length).toBe(0);
    render({available: true, enabled: false});
    act(() => root!.unmount());
    root = null;
    act(() => void vi.advanceTimersByTime(30_000));
    expect(posts.length).toBe(1);
  });

  it("the toggle stays when the robot reports a lidar error, and shows it", () => {
    useRobots.setState({state: {gauss: {lidar: {available: true, enabled: true, error: "ValueError: bad cloud"}}}});
    act(() => root!.render(<LidarToggle robot="gauss" />));
    expect(host!.textContent).toContain("Lidar");
    expect(host!.querySelector('[data-testid="lidar-error-gauss"]')?.textContent).toContain("bad cloud");
    act(() => useRobots.setState({state: {gauss: {lidar: {available: true, enabled: true, error: null}}}}));
    expect(host!.querySelector('[data-testid="lidar-error-gauss"]')).toBeNull();
    act(() => useRobots.setState({state: {gauss: {lidar: {available: false, enabled: false}}}}));
    expect(host!.textContent).toBe("");
  });
});
