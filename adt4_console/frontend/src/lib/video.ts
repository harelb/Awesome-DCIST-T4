// Live camera video (API.md §12): codec choice, key-frame gating and rate
// meters. Pure logic, no React / DOM, so it is unit-testable.

export type VideoCodec = "h264" | "vp8" | "jpeg";

/** One `video/<robot>/<camera>` WebSocket frame's data. */
export interface VideoFrame {
  camera: string;
  codec: VideoCodec;
  seq: number;
  t: number; // robot wall clock when the frame was sampled (s)
  stamp?: number | null; // image header stamp
  key: boolean;
  width: number;
  height: number;
  fps: number;
  data: Uint8Array;
  robot?: string;
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

/** h264 when this browser can decode it, else vp8, else jpeg (one <img> per frame). */
export async function pickCodec(
  dec: {isConfigSupported?: (c: VideoDecoderConfig) => Promise<{supported?: boolean}>} | undefined = (globalThis as {VideoDecoder?: typeof VideoDecoder}).VideoDecoder,
): Promise<VideoCodec> {
  if (!dec?.isConfigSupported) return "jpeg";
  for (const c of ["h264", "vp8"] as const) {
    try {
      if ((await dec.isConfigSupported(decoderConfig(c)!)).supported) return c;
    } catch {
      /* unsupported config shape */
    }
  }
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
    stamp: (o.stamp as number | null | undefined) ?? null,
    key: Boolean(o.key),
    width: Number(o.width ?? 0),
    height: Number(o.height ?? 0),
    fps: Number(o.fps ?? 0),
    data,
    robot: o.robot === undefined ? undefined : String(o.robot),
  };
}
