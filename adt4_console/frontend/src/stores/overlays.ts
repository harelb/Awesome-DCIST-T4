// Live RViz-replacement overlays: per-robot occupancy grid (fetched over REST,
// refreshed on `grid/<r>` frames) and executor-path display toggles. Paths
// themselves arrive on `robot/<r>/paths` and live in the robots store.
import {create} from "zustand";
import {errorText, getMsgpack} from "../lib/api";
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
  /** change detection: added (green) / removed (red) boxes, their labels */
  showAdded: boolean;
  showRemoved: boolean;
  showChangeLabels: boolean;
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
  showAdded: true,
  showRemoved: true,
  showChangeLabels: true,
  set: patch => set(patch as Partial<OverlayStore>),
}));

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
