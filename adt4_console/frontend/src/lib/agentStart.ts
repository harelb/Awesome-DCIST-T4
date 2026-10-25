// Where a mission agent starts (API.md §15.1 Agent.start): the pure parts of
// relocating it — typed map x/y or lat/lon (through the GPS anchor's affine),
// validation (numbers must be finite; a start outside the map is only a warning),
// snapping to the nearest place node, when a run forbids moving a start, and the
// drag gesture's state (grab → move → commit on release / cancel on Esc).
import {lonLatToMap, mapToLonLat, type GeoInfo} from "./geo";

/** map metres, 2 decimals (1 cm: what the inputs show and the mission stores) */
export const r2 = (v: number) => Math.round(v * 100) / 100;
export const roundXY = (p: readonly number[]): number[] => [r2(p[0]), r2(p[1])];

/** A typed coordinate: a finite number (blank, "1e999", "abc" are not). */
export function parseCoord(s: string): number | null {
  const t = s.trim().replace(",", ".");
  if (!t || !/^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(t)) return null;
  const v = Number(t);
  return Number.isFinite(v) ? v : null;
}

export type StartInput = {mode: "xy"; x: string; y: string} | {mode: "lonlat"; lat: string; lon: string};

/** The typed fields → a map-frame start, or the problem with them. */
export function startFromInput(inp: StartInput, geo: GeoInfo | null): {start: number[]} | {error: string} {
  if (inp.mode === "xy") {
    const x = parseCoord(inp.x);
    const y = parseCoord(inp.y);
    if (x === null) return {error: "x must be a number (metres)"};
    if (y === null) return {error: "y must be a number (metres)"};
    return {start: roundXY([x, y])};
  }
  if (!geo) return {error: "this map has no GPS anchor: enter map x / y"};
  const lat = parseCoord(inp.lat);
  const lon = parseCoord(inp.lon);
  if (lat === null || lat < -90 || lat > 90) return {error: "latitude must be a number in −90…90"};
  if (lon === null || lon < -180 || lon > 180) return {error: "longitude must be a number in −180…180"};
  return {start: roundXY(lonLatToMap(geo, lon, lat))};
}

/** The fields for a start (lat/lon to 7 decimals ≈ 1 cm). */
export function inputOf(start: readonly number[] | null | undefined, mode: "xy" | "lonlat", geo: GeoInfo | null): StartInput {
  if (mode === "lonlat" && geo) {
    if (!start) return {mode, lat: "", lon: ""};
    const [lon, lat] = mapToLonLat(geo, start[0], start[1]);
    return {mode, lat: lat.toFixed(7), lon: lon.toFixed(7)};
  }
  return {mode: "xy", x: start ? start[0].toFixed(2) : "", y: start ? start[1].toFixed(2) : ""};
}

/** Advice about a valid start (not a block): outside the map's extent (plus a margin). */
export function startWarning(start: readonly number[], bounds: readonly number[] | null | undefined, margin = 5): string | null {
  if (!bounds || bounds.length < 4) return null;
  const [x0, y0, x1, y1] = bounds;
  if (start[0] < x0 - margin || start[0] > x1 + margin || start[1] < y0 - margin || start[1] > y1 + margin)
    return `outside the map (x ${x0.toFixed(0)}…${x1.toFixed(0)}, y ${y0.toFixed(0)}…${y1.toFixed(0)})`;
  return null;
}

export interface PlacePoint {
  sym: string;
  x: number;
  y: number;
}

/** The place node nearest ``p`` within ``maxD`` metres (snapping a dragged start). */
export function nearestPlace(p: readonly number[], places: Iterable<PlacePoint>, maxD = 4): PlacePoint | null {
  let best: PlacePoint | null = null;
  let bd = maxD;
  for (const q of places) {
    const d = Math.hypot(q.x - p[0], q.y - p[1]);
    if (d <= bd) {
      bd = d;
      best = q;
    }
  }
  return best;
}

export interface RunLike {
  kind: "sim" | "live";
  playing: boolean;
  finished: boolean;
}

/** Why the open mission's starts cannot move now (null: they can). A live run
 *  that has not ended locks them; so does a simulation while it plays. A paused,
 *  reset or finished simulation does not: moving a start makes a new revision,
 *  and a new revision drops its simulation (see ``moveResetsRun``). */
export function startLockReason(run: RunLike | null | undefined): string | null {
  if (!run) return null;
  if (run.kind === "live" && !run.finished) return "A live run is in progress: starts are locked until it ends (or is aborted).";
  if (run.kind === "sim" && run.playing) return "The simulation is playing: pause it to move a start (that resets the simulation).";
  return null;
}

/** Moving a start of the open mission discards its run (paused / finished simulation). */
export const moveResetsRun = (run: RunLike | null | undefined) => !!run && startLockReason(run) === null;

// ------------------------------------------------------------- dragging
export interface StartDrag {
  agent: string;
  pointerId: number;
  /** where the agent was drawn when grabbed */
  from: number[];
  /** the start it would get now (snapped when ``snap`` is set) */
  at: number[];
  snap: string | null;
  /** screen position of the grab (a release within the tap slop is not a move) */
  sx: number;
  sy: number;
  moved: boolean;
}

export function grab(agent: string, pointerId: number, from: readonly number[], sx: number, sy: number): StartDrag {
  return {agent, pointerId, from: roundXY(from), at: roundXY(from), snap: null, sx, sy, moved: false};
}

/** The pointer moved to ground point ``p`` (screen ``sx``, ``sy``); with a place
 *  to snap to, the start is that place. */
export function dragTo(d: StartDrag, p: readonly number[], sx: number, sy: number, slopPx: number, snap: PlacePoint | null = null): StartDrag {
  const moved = d.moved || Math.hypot(sx - d.sx, sy - d.sy) > slopPx;
  return {...d, moved, at: snap ? roundXY([snap.x, snap.y]) : roundXY(p), snap: snap?.sym ?? null};
}

/** Release: the start to commit, or null (a tap, or it did not move). */
export function release(d: StartDrag): number[] | null {
  if (!d.moved) return null;
  if (d.at[0] === d.from[0] && d.at[1] === d.from[1]) return null;
  return d.at;
}
