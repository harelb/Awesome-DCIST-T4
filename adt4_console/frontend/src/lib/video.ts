// Live camera video (API.md §12): codec choice, key-frame gating and rate
// meters. Pure logic, no React / DOM, so it is unit-testable.

export type VideoCodec = "h264" | "vp8" | "jpeg";

/** One `video/<robot>/<camera>` WebSocket frame's data. */
export interface VideoFrame {
  camera: string;
  codec: VideoCodec;
  seq: number;
  t: number; // robot wall clock when the frame was sampled (s)
  tBase?: number | null; // the same instant in the console's clock (clock-offset corrected)
  stamp?: number | null; // image header stamp
  key: boolean;
  width: number;
  height: number;
  fps: number;
  data: Uint8Array;
  robot?: string;
  /** mosaic stream only: where each camera sits in the picture */
  tiles?: MosaicTile[];
}

/** One camera's rectangle in a mosaic frame (API.md §12.4); w = h = 0 while
 * the robot has no frame of it yet. */
export interface MosaicTile {
  camera: string;
  x: number;
  y: number;
  w: number;
  h: number;
  stamp: number | null;
  /** how old the camera's frame was when the mosaic was sampled (s) */
  age: number | null;
}

/** The robot's pseudo camera that tiles several cameras into one stream. */
export const MOSAIC = "mosaic";

export type GridVideoMode = "mosaic" | "tiles";
export type GridQuality = "low" | "medium";

/** Grid video presets: tile width (px), fps, total kbit/s for the whole grid.
 * medium = ~100 kB/s, a quarter of the default 400 kB/s C5 budget. */
export const GRID_QUALITY: Record<GridQuality, {width: number; fps: number; kbps: number}> = {
  low: {width: 240, fps: 3, kbps: 300},
  medium: {width: 320, fps: 5, kbps: 800},
};
export const MIN_STREAM_KBPS = 50;

export interface GridStreamParams {
  camera: string;
  cameras?: string[];
  width: number;
  fps: number;
  kbps: number;
}

/** What to request for the grid: one mosaic stream of the visible cameras, or
 * one stream per visible camera sharing the preset's total bitrate (each at
 * least MIN_STREAM_KBPS). */
export function gridStreams(mode: GridVideoMode, quality: GridQuality, cams: string[], solo = false): GridStreamParams[] {
  if (!cams.length) return [];
  const p = GRID_QUALITY[quality];
  // a soloed camera fills the whole grid: twice the tile width, same bitrate
  const q = solo ? {...p, width: Math.min(640, 2 * p.width)} : p;
  if (mode === "mosaic") return [{camera: MOSAIC, cameras: [...cams], width: q.width, fps: q.fps, kbps: q.kbps}];
  const kbps = Math.max(MIN_STREAM_KBPS, Math.round(q.kbps / cams.length));
  return cams.map(camera => ({camera, width: q.width, fps: q.fps, kbps}));
}

/** Per-camera rate of NEW images inside a mosaic (distinct stamps over a
 * sliding window): the mosaic repeats a camera's last image when it has no
 * newer one, so mosaic frames/s says nothing about one camera's freshness. */
export class StampRate {
  private last = new Map<string, number | null>();
  private seen = new Map<string, number[]>();
  constructor(private windowMs = 3000) {}

  add(camera: string, stamp: number | null, now: number) {
    if (stamp === null) return;
    if (this.last.get(camera) === stamp) return;
    this.last.set(camera, stamp);
    const l = this.seen.get(camera) ?? [];
    l.push(now);
    this.seen.set(camera, l);
    this.trim(l, now);
  }

  private trim(l: number[], now: number) {
    while (l.length && now - l[0] > this.windowMs) l.shift();
  }

  fps(camera: string, now: number): number {
    const l = this.seen.get(camera);
    if (!l) return 0;
    this.trim(l, now);
    return l.length / (this.windowMs / 1000);
  }
}

function parseTiles(v: unknown): MosaicTile[] | undefined {
  if (!Array.isArray(v)) return undefined;
  const num = (x: unknown) => (typeof x === "number" && Number.isFinite(x) ? x : null);
  return v
    .filter((t): t is unknown[] => Array.isArray(t) && t.length >= 5)
    .map(t => ({camera: String(t[0]), x: num(t[1]) ?? 0, y: num(t[2]) ?? 0, w: num(t[3]) ?? 0, h: num(t[4]) ?? 0, stamp: num(t[5]), age: num(t[6])}));
}

export const FPS_OPTIONS = [1, 2, 5, 10, 15];
export const BITRATE_PRESETS: {label: string; kbps: number}[] = [
  {label: "low", kbps: 150},
  {label: "medium", kbps: 300},
  {label: "high", kbps: 800},
];
export const VIDEO_WIDTHS = [320, 480, 640, 960];

/** H.264 Constrained Baseline 3.0 — what the robot's x264 `-profile:v baseline` produces. */
export const H264_CODEC = "avc1.42E01E";

/** WebCodecs decoder config. Annex-B H.264 (SPS/PPS in band): no `description`. */
export function decoderConfig(codec: VideoCodec): VideoDecoderConfig | null {
  if (codec === "h264") return {codec: H264_CODEC, optimizeForLatency: true};
  if (codec === "vp8") return {codec: "vp8", optimizeForLatency: true};
  return null;
}

type DecoderProbe = {isConfigSupported?: (c: VideoDecoderConfig) => Promise<{supported?: boolean}>} | undefined;
const webCodecs = (): DecoderProbe => (globalThis as {VideoDecoder?: typeof VideoDecoder}).VideoDecoder;

/** The WASM h264 decoder (h264.worker.ts) needs only WebAssembly + module
 * workers, so it also runs on plain-http pages, where WebCodecs is missing. */
export const wasmH264Available = () => typeof WebAssembly === "object" && typeof Worker === "function";

async function supported(dec: DecoderProbe, codec: VideoCodec): Promise<boolean> {
  if (!dec?.isConfigSupported) return false;
  try {
    return Boolean((await dec.isConfigSupported(decoderConfig(codec)!)).supported);
  } catch {
    return false; // unsupported config shape
  }
}

export type H264Decoder = "webcodecs" | "wasm";

/** How this browser decodes h264: WebCodecs (hardware) when it can, else WASM. */
export async function h264Decoder(dec: DecoderProbe = webCodecs(), wasm = wasmH264Available()): Promise<H264Decoder | null> {
  if (await supported(dec, "h264")) return "webcodecs";
  return wasm ? "wasm" : null;
}

/** h264 when this browser can decode it (WebCodecs, else WASM), else vp8
 * (WebCodecs), else jpeg (one <img> per frame). */
export async function pickCodec(dec: DecoderProbe = webCodecs(), wasm = wasmH264Available()): Promise<VideoCodec> {
  if (await h264Decoder(dec, wasm)) return "h264";
  if (await supported(dec, "vp8")) return "vp8";
  return "jpeg";
}

export type GateAction = "decode" | "skip";

/**
 * Decides which frames may reach the decoder. A delta frame is only decodable
 * when every frame since the last key frame was decoded, so the gate skips
 * frames until a key frame:
 *  - at the start,
 *  - after a gap in the WebSocket topic seq (frames lost on the way),
 *  - after `broken()` (decoder error or backlog),
 *  - when the stream's codec / size changes (the new encoder starts with a key
 *    frame; `reconfigure` is set on that key frame).
 */
export class KeyGate {
  waitingKey = true;
  skipped = 0;
  private lastSeq: number | null = null;
  private config: string | null = null;

  accept(f: Pick<VideoFrame, "key" | "codec" | "width" | "height">, seq: number): {action: GateAction; reconfigure: boolean} {
    const gap = this.lastSeq !== null && seq !== this.lastSeq + 1;
    this.lastSeq = seq;
    const cfg = `${f.codec}/${f.width}x${f.height}`;
    if (gap || (this.config !== null && cfg !== this.config)) this.waitingKey = true;
    if (this.waitingKey && !f.key) {
      this.skipped++;
      return {action: "skip", reconfigure: false};
    }
    const reconfigure = cfg !== this.config;
    this.config = cfg;
    this.waitingKey = false;
    return {action: "decode", reconfigure};
  }

  /** The decoder lost the chain (or failed): wait for the next key frame and
   * reconfigure the decoder on it. */
  broken() {
    this.waitingKey = true;
    this.config = null;
  }

  reset() {
    this.waitingKey = true;
    this.lastSeq = null;
    this.config = null;
  }
}

/** Frames and bytes over a fixed sliding window → fps and kbps. The divisor is
 * the window (or the time since the first frame, while that is shorter), not
 * the age of the oldest sample, so a burst after a quiet spell is not inflated. */
export class RateMeter {
  private samples: [number, number][] = [];
  private start: number | null = null;
  constructor(private windowMs = 2000) {}

  add(bytes: number, now: number) {
    if (this.start === null) this.start = now;
    this.samples.push([now, bytes]);
    this.trim(now);
  }

  private trim(now: number) {
    while (this.samples.length && now - this.samples[0][0] > this.windowMs) this.samples.shift();
  }

  rates(now: number): {fps: number; kbps: number} {
    this.trim(now);
    if (!this.samples.length) return {fps: 0, kbps: 0};
    const span = Math.max(this.windowMs / 4, Math.min(this.windowMs, now - (this.start ?? now))) / 1000;
    const bytes = this.samples.reduce((a, s) => a + s[1], 0);
    return {fps: this.samples.length / span, kbps: (bytes * 8) / 1000 / span};
  }
}

/** EncodedVideoChunk init for a frame (timestamps only need to be unique per stream). */
export function chunkInit(f: VideoFrame): EncodedVideoChunkInit {
  return {type: f.key ? "key" : "delta", timestamp: Math.round(f.seq * 1000), data: f.data};
}

/** Normalise a decoded msgpack frame (bin → Uint8Array) or return null. */
export function parseVideoFrame(d: unknown): VideoFrame | null {
  if (!d || typeof d !== "object") return null;
  const o = d as Record<string, unknown>;
  const data = o.data instanceof Uint8Array ? o.data : o.data instanceof ArrayBuffer ? new Uint8Array(o.data) : null;
  if (!data || typeof o.seq !== "number" || typeof o.codec !== "string") return null;
  return {
    camera: String(o.camera ?? ""),
    codec: o.codec as VideoCodec,
    seq: o.seq,
    t: Number(o.t ?? 0),
    tBase: typeof o.t_base === "number" ? o.t_base : null,
    stamp: (o.stamp as number | null | undefined) ?? null,
    key: Boolean(o.key),
    width: Number(o.width ?? 0),
    height: Number(o.height ?? 0),
    fps: Number(o.fps ?? 0),
    data,
    robot: o.robot === undefined ? undefined : String(o.robot),
    tiles: parseTiles(o.tiles),
  };
}

/** End-to-end lag (ms) from robot capture to now, skew-corrected: the console
 * maps the capture time into its clock (``tBase``), ``offset`` maps ours. */
export function frameLagMs(f: Pick<VideoFrame, "tBase">, nowMs: number, offset: number | null): number | null {
  if (f.tBase == null || offset == null) return null;
  return (nowMs / 1000 + offset - f.tBase) * 1000;
}

/** Operator's grid camera selection (per robot): tile order and the cameras
 * switched off. Stored as the "off" list so cameras the robot adds later
 * appear switched on. */
export interface GridSelection {
  order: string[];
  off: string[];
}

/** Fit a saved selection to the robot's current cameras: unknown cameras go,
 * new ones are appended (on) in the robot's order. */
export function normalizeSelection(saved: Partial<GridSelection> | null | undefined, cams: string[]): GridSelection {
  const known = new Set(cams);
  const order = (Array.isArray(saved?.order) ? saved.order : []).filter((c, i, a) => known.has(c) && a.indexOf(c) === i);
  for (const c of cams) if (!order.includes(c)) order.push(c);
  const off = (Array.isArray(saved?.off) ? saved.off : []).filter(c => known.has(c));
  return {order, off: [...new Set(off)]};
}

/** The cameras to show, in tile order: just the solo camera while one is
 * soloed (and still exists), else every camera that is switched on. */
export function selectedCameras(sel: GridSelection, solo: string | null = null): string[] {
  if (solo && sel.order.includes(solo)) return [solo];
  const off = new Set(sel.off);
  return sel.order.filter(c => !off.has(c));
}

/** What to stream: the selected cameras that are also in view, in tile order. */
export function streamCameras(selected: string[], inView: Iterable<string>): string[] {
  const v = new Set(inView);
  return selected.filter(c => v.has(c));
}

export function toggleCamera(sel: GridSelection, cam: string): GridSelection {
  return {...sel, off: sel.off.includes(cam) ? sel.off.filter(c => c !== cam) : [...sel.off, cam]};
}

export function setAllCameras(sel: GridSelection, on: boolean): GridSelection {
  return {...sel, off: on ? [] : [...sel.order]};
}

/** Move `cam` by `delta` places (up/down buttons), clamped to the ends. */
export function moveCamera(sel: GridSelection, cam: string, delta: number): GridSelection {
  const i = sel.order.indexOf(cam);
  if (i < 0) return sel;
  const j = Math.max(0, Math.min(sel.order.length - 1, i + delta));
  if (i === j) return sel;
  const order = [...sel.order];
  order.splice(i, 1);
  order.splice(j, 0, cam);
  return {...sel, order};
}

/** Drag-to-reorder: put `cam` where `target` is. */
export function dropCamera(sel: GridSelection, cam: string, target: string): GridSelection {
  const j = sel.order.indexOf(target);
  return j < 0 ? sel : moveCamera(sel, cam, j - sel.order.indexOf(cam));
}
