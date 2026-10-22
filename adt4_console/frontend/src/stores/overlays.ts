// Live RViz-replacement overlays: per-robot occupancy grid (fetched over REST,
// refreshed on `grid/<r>` frames) and executor-path display toggles. Paths
// themselves arrive on `robot/<r>/paths` and live in the robots store.
// Lidar: per-robot on/off (default off); on asks the robot to stream its point
// cloud and LiveOverlays polls it, off stops both.
import {useEffect} from "react";
import {create} from "zustand";
import {errorText, getMsgpack, postJSON} from "../lib/api";
import {decodeGrid, type DecodedGrid} from "../lib/overlays";
import {ws} from "../ws";

export const DEFAULT_GRID_OPACITY = 0.7;

interface OverlayStore {
  grids: Record<string, DecodedGrid>;
  gridError: Record<string, string | null>;
  gridOn: Record<string, boolean>; // robot -> shown (default on)
  gridOpacity: Record<string, number>;
  showSmooth: boolean;
  showMlp: boolean;
  showTarget: boolean;
  /** robot -> lidar wanted (default off) */
  lidarOn: Record<string, boolean>;
  lidarError: Record<string, string | null>;
  set: (patch: Partial<OverlayStore>) => void;
}

export const useOverlays = create<OverlayStore>(set => ({
  grids: {},
  gridError: {},
  gridOn: {},
  gridOpacity: {},
  showSmooth: true,
  showMlp: true,
  showTarget: true,
  lidarOn: {},
  lidarError: {},
  set: patch => set(patch as Partial<OverlayStore>),
}));

/** Ask ``robot`` to start / stop its lidar stream (POST /api/live/<r>/lidar)
 *  and remember the choice; polling follows ``lidarOn``. */
export async function requestLidar(robot: string, enabled: boolean): Promise<void> {
  useOverlays.setState(s => ({lidarOn: {...s.lidarOn, [robot]: enabled}, lidarError: {...s.lidarError, [robot]: null}}));
  try {
    await postJSON(`api/live/${encodeURIComponent(robot)}/lidar`, {enabled});
  } catch (e) {
    useOverlays.setState(s => ({lidarError: {...s.lidarError, [robot]: errorText(e)}}));
  }
}

/** While mounted (the operator wants ``robot``'s lidar) and the robot reports
 *  a lidar that is not streaming: ask it to start at once, then again every
 *  ``everyS`` s until it reports ``enabled`` (a console restart, a robot agent
 *  restart or a lost request all leave it off). */
export function useLidarReenable(robot: string, lidar: {available?: boolean; enabled?: boolean} | null | undefined, everyS: number) {
  const off = !!lidar?.available && !lidar.enabled;
  useEffect(() => {
    if (!off) return;
    const ask = () => void requestLidar(robot, true);
    ask();
    const h = window.setInterval(ask, everyS * 1000);
    return () => window.clearInterval(h);
  }, [robot, off, everyS]);
}

export const gridShown = (s: OverlayStore, r: string) => s.gridOn[r] ?? true;
export const gridOpacityOf = (s: OverlayStore, r: string) => s.gridOpacity[r] ?? DEFAULT_GRID_OPACITY;

const inflight = new Map<string, Promise<void>>();

export function fetchGrid(robot: string): Promise<void> {
  const cur = inflight.get(robot);
  if (cur) return cur;
  const p = (async () => {
    try {
      const g = await decodeGrid(await getMsgpack(`api/live/${encodeURIComponent(robot)}/grid`));
      useOverlays.setState(s => ({grids: {...s.grids, [robot]: g}, gridError: {...s.gridError, [robot]: null}}));
    } catch (e) {
      useOverlays.setState(s => ({gridError: {...s.gridError, [robot]: errorText(e)}}));
    } finally {
      inflight.delete(robot);
    }
  })();
  inflight.set(robot, p);
  return p;
}

/** Fetch whenever `grid/<r>` announces a different hash. Returns the unsubscribe.
 * No unconditional first fetch: the hub replays the latest `grid/<r>` on subscribe,
 * and robots without an occupancy grid would otherwise answer 404 (a console error). */
export function watchGrid(robot: string): () => void {
  return ws.on(`grid/${robot}`, data => {
    const msg = data as {hash?: string | null} | null;
    if (msg && msg.hash === null) {
      // robot restarted without a grid yet: drop the previous incarnation's
      useOverlays.setState(s => {
        const grids = {...s.grids};
        delete grids[robot];
        return {grids};
      });
      return;
    }
    if (!msg?.hash || msg.hash !== useOverlays.getState().grids[robot]?.hash) void fetchGrid(robot);
  });
}
