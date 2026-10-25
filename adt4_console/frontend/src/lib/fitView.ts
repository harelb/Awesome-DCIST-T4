// "Fit mission": frame a set of map points (the open mission's agents, paths
// and areas) in the top view, inside the part of the map the overlays leave
// free (toolbars top-left / top-right, banners, the legend). Pure math here;
// stores/mission.ts fitMission() gathers the points, Scene.tsx CameraRig moves
// the camera.

export interface Insets {
  top: number;
  right: number;
  bottom: number;
  left: number;
}

/** A top-view framing: the camera target (map x/y) and its height above it. */
export interface TopFit {
  target: [number, number];
  height: number;
  /** metres per screen pixel at that height */
  mpp: number;
}

/** smallest framed extent (m): one agent alone is not zoomed into the ground */
export const FIT_MIN_SPAN = 20;
/** margin around the framed points, as a fraction of their extent */
export const FIT_PAD = 0.12;
/** the free part is never taken as less than this (px): a phone's half-height
 *  sheet plus the toolbars and a notice can leave a strip of ~100 px, and the
 *  points must land in that strip, not under the sheet */
export const FIT_MIN_FREE = 48;

/** Frame `pts` (map x/y) seen straight down by a camera whose screen axes are
 *  `right` / `up` (unit map-plane vectors: the view may be rotated), in a
 *  `w`×`h` px canvas with a vertical field of view `fovY` (radians), keeping
 *  them inside the canvas minus `insets` (px). Null without points. */
export function fitTopView(pts: number[][], right: number[], up: number[], w: number, h: number, fovY: number, insets: Insets): TopFit | null {
  const ok = pts.filter(p => p && Number.isFinite(p[0]) && Number.isFinite(p[1]));
  if (!ok.length || w <= 0 || h <= 0) return null;
  let x0 = Infinity;
  let x1 = -Infinity;
  let y0 = Infinity;
  let y1 = -Infinity;
  for (const p of ok) {
    const sx = p[0] * right[0] + p[1] * right[1];
    const sy = p[0] * up[0] + p[1] * up[1];
    x0 = Math.min(x0, sx);
    x1 = Math.max(x1, sx);
    y0 = Math.min(y0, sy);
    y1 = Math.max(y1, sy);
  }
  const bw = Math.max(x1 - x0, FIT_MIN_SPAN) * (1 + 2 * FIT_PAD);
  const bh = Math.max(y1 - y0, FIT_MIN_SPAN) * (1 + 2 * FIT_PAD);
  // the free part of the canvas
  const aw = Math.max(w - insets.left - insets.right, FIT_MIN_FREE);
  const ah = Math.max(h - insets.top - insets.bottom, FIT_MIN_FREE);
  const mpp = Math.max(bw / aw, bh / ah);
  const height = (mpp * h) / (2 * Math.tan(fovY / 2));
  // the box centre goes to the free part's centre: the canvas centre (the
  // camera target) sits that many pixels away from it
  const dx = (insets.left - insets.right) / 2;
  const dy = (insets.top - insets.bottom) / 2;
  const cx = (x0 + x1) / 2 - dx * mpp;
  const cy = (y0 + y1) / 2 + dy * mpp;
  return {target: [right[0] * cx + up[0] * cy, right[1] * cx + up[1] * cy], height, mpp};
}

/** Pixels the map overlays cover along each edge of `canvas`: every overlay
 *  (`selector`) in the upper half of the visible map pushes the top inset
 *  down, in the lower half the bottom one up; `margin` px on every side.
 *  `outside`: overlays that are not in the canvas but cover its bottom (a
 *  phone's sheet); they go first, so what sits just above the sheet (the
 *  status line) counts as a bottom overlay, not a top one. */
export function overlayInsets(canvas: Element, selector: string, margin = 16, outside: readonly Element[] = []): Insets {
  const c = canvas.getBoundingClientRect();
  const out: Insets = {top: margin, right: margin, bottom: margin, left: margin};
  const hits = (r: DOMRect) => r.width > 0 && r.height > 0 && r.bottom > c.top && r.top < c.bottom;
  for (const el of outside) {
    const r = el.getBoundingClientRect();
    if (hits(r)) out.bottom = Math.max(out.bottom, c.bottom - r.top + margin);
  }
  const mid = (c.top + c.bottom - (out.bottom - margin)) / 2;
  canvas.querySelectorAll<HTMLElement>(selector).forEach(el => {
    const r = el.getBoundingClientRect();
    if (!hits(r)) return;
    if ((r.top + r.bottom) / 2 < mid) out.top = Math.max(out.top, r.bottom - c.top + margin);
    else out.bottom = Math.max(out.bottom, c.bottom - r.top + margin);
  });
  return out;
}

/** What covers the map: toolbars, banners, notices, the legend and status line. */
export const OVERLAY_SELECTOR = ".vtoolbar, .live-edit-strip, .pick-banner, .canvas-notices > *, .robot-legend, .vstatus";
