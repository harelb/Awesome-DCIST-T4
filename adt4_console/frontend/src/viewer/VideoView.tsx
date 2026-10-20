// Camera tab, "Video" mode: a live C5 stream from the robot's fleetlink agent
// (API.md §12). WebCodecs VideoDecoder for h264 / vp8 drawn to a canvas, an
// <img> per frame for jpeg. Without WebCodecs (plain-http pages opened from
// another machine) h264 decodes in a WASM worker instead, drawn with WebGL;
// `?h264=wasm` forces that path. Keepalive every 2 s while the tab is visible.
import {useEffect, useRef, useState} from "react";
import {getToken} from "../lib/auth";
import {ws} from "../ws";
import {consoleClockOffset} from "../lib/clock";
import {BITRATE_PRESETS, FPS_OPTIONS, KeyGate, RateMeter, VIDEO_WIDTHS, chunkInit, decoderConfig, frameLagMs, h264Decoder, parseVideoFrame, pickCodec, type H264Decoder, type VideoCodec, type VideoFrame as StreamFrame} from "../lib/video";
import type {WasmH264Decoder} from "../lib/wasmH264";
import type {YuvRenderer} from "../lib/yuvRenderer";

const START_SPACING_MS = 1100;
const WASM_BACKLOG = 4; // access units queued in the worker before dropping to the next key frame
const forceWasm = () => typeof location !== "undefined" && new URLSearchParams(location.search).get("h264") === "wasm";
const KEEPALIVE_MS = 2000;

interface ServerStatus {
  params?: {fps: number; kbps: number; width: number; codec: VideoCodec};
  relay?: {frames: number; withheld: number; waiting_key: boolean};
  rx?: {fps?: number; kbps?: number; lost?: number};
  robot?: {codec: string; fps: number; kbps: number; width: number; height: number; sent_kbps: number; frames: number; dropped: number; error?: string; requested_codec?: string} | null;
}

interface Stats {
  fps: number;
  kbps: number;
  decodeMs: number | null;
  lagMs: number | null;
  skipped: number;
  waitingKey: boolean;
  codec: string;
  decoder: H264Decoder | null;
  width: number;
  height: number;
}

export const viewerId = () => (globalThis.crypto?.randomUUID?.() ?? `v${Math.random().toString(36).slice(2)}`).slice(0, 36);

export async function post(path: string, body: unknown, keepalive = false): Promise<Response> {
  const headers = new Headers({"Content-Type": "application/json"});
  const t = getToken();
  if (t) headers.set("Authorization", `Bearer ${t}`);
  return fetch(path, {method: "POST", headers, body: JSON.stringify(body), keepalive});
}

export async function detail(res: Response): Promise<string> {
  try {
    const j = await res.json();
    if (typeof j.detail === "string") return j.detail;
  } catch {
    /* not JSON */
  }
  return `HTTP ${res.status}`;
}

export const usePageVisible = () => {
  const [visible, setVisible] = useState(typeof document === "undefined" || document.visibilityState !== "hidden");
  useEffect(() => {
    const on = () => setVisible(document.visibilityState !== "hidden");
    document.addEventListener("visibilitychange", on);
    return () => document.removeEventListener("visibilitychange", on);
  }, []);
  return visible;
};

export function VideoView({robot, cam}: {robot: string; cam: string}) {
  const [fps, setFps] = useState(5);
  const [kbps, setKbps] = useState(300);
  const [width, setWidth] = useState(640);
  const [codecChoice, setCodecChoice] = useState<"auto" | VideoCodec>("auto");
  const [autoCodec, setAutoCodec] = useState<VideoCodec | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [server, setServer] = useState<ServerStatus | null>(null);
  const [stats, setStats] = useState<Stats | null>(null);
  const [jpegUrl, setJpegUrl] = useState<string | null>(null);
  const visible = usePageVisible();
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const glCanvasRef = useRef<HTMLCanvasElement | null>(null); // WASM path (a canvas holds one context type)
  const [h264Path, setH264Path] = useState<H264Decoder | null>(null);
  const viewer = useRef(viewerId());

  useEffect(() => {
    let alive = true;
    const wasm = forceWasm();
    void (wasm ? Promise.resolve("h264" as const) : pickCodec()).then(c => alive && setAutoCodec(c));
    void (wasm ? Promise.resolve("wasm" as const) : h264Decoder()).then(d => alive && setH264Path(d));
    return () => {
      alive = false;
    };
  }, []);
  const codec: VideoCodec | null = codecChoice === "auto" ? autoCodec : codecChoice;
  const ready = codec !== null && (codec !== "h264" || h264Path !== null);
  const webcodecs = typeof VideoDecoder !== "undefined" && !forceWasm();
  const base = `api/robots/${encodeURIComponent(robot)}/video/${encodeURIComponent(cam)}`;
  const paramsRef = useRef({fps, kbps, width, codec});
  paramsRef.current = {fps, kbps, width, codec};
  const startRef = useRef<(() => Promise<void>) | null>(null);
  const lastStartRef = useRef(0); // the console rate-limits parameter changes to 1/s

  // stream lifecycle: start + keepalive every 2 s; stop when hidden or closed
  useEffect(() => {
    if (!ready || !visible) return;
    let alive = true;
    let timer: number | undefined;
    const start = async (): Promise<void> => {
      window.clearTimeout(timer);
      // pace starts instead of provoking a 429 when settings change quickly
      const wait = START_SPACING_MS - (Date.now() - lastStartRef.current);
      if (wait > 0) {
        timer = window.setTimeout(() => void start(), wait);
        return;
      }
      lastStartRef.current = Date.now();
      try {
        const res = await post(`${base}/start`, {...paramsRef.current, viewer: viewer.current});
        if (!alive) return;
        if (res.status === 429) {
          // parameters changed too quickly: retry once the rate limit has passed
          timer = window.setTimeout(() => void start(), 1100);
          return;
        }
        if (!res.ok) throw new Error(await detail(res));
        setErr(null);
      } catch (e) {
        if (alive) setErr(e instanceof Error ? e.message : String(e));
      }
    };
    const keepalive = async () => {
      try {
        const res = await post(`${base}/keepalive`, {viewer: viewer.current});
        if (!alive) return;
        if (res.status === 404) return void start(); // console restarted / stream expired
        if (!res.ok) throw new Error(await detail(res));
        setServer((await res.json()) as ServerStatus);
        setErr(null);
      } catch (e) {
        if (alive) setErr(e instanceof Error ? e.message : String(e));
      }
    };
    startRef.current = start;
    void start();
    const id = window.setInterval(() => void keepalive(), KEEPALIVE_MS);
    return () => {
      alive = false;
      startRef.current = null;
      window.clearInterval(id);
      window.clearTimeout(timer);
      void post(`${base}/stop`, {viewer: viewer.current}, true).catch(() => undefined);
    };
  }, [base, ready, visible]);

  // retune a running stream (no stop: the robot just restarts its encoder)
  useEffect(() => {
    void startRef.current?.();
  }, [fps, kbps, width, codec]);

  // receive + decode
  useEffect(() => {
    if (!ready || !visible) return;
    const topic = `video/${robot}/${cam}`;
    const gate = new KeyGate();
    const meter = new RateMeter();
    const submitted = new Map<number, number>();
    let decodeMs: number | null = null;
    let lagMs: number | null = null;
    let last: StreamFrame | null = null;
    let decoder: VideoDecoder | null = null;
    let wasm: WasmH264Decoder | null = null;
    let wasmLoading = false;
    let renderer: YuvRenderer | null = null;
    let alive = true;
    let lastUrl: string | null = null;
    let lastKfReq = -1e9;
    const useWasmPath = () => h264Path === "wasm";

    const draw = (frame: VideoFrame) => {
      const c = canvasRef.current;
      if (c) {
        if (c.width !== frame.displayWidth || c.height !== frame.displayHeight) {
          c.width = frame.displayWidth;
          c.height = frame.displayHeight;
        }
        c.getContext("2d")?.drawImage(frame, 0, 0);
      }
      const t0 = submitted.get(frame.timestamp);
      if (t0 !== undefined) {
        const d = performance.now() - t0;
        decodeMs = decodeMs === null ? d : 0.8 * decodeMs + 0.2 * d;
        submitted.delete(frame.timestamp);
      }
      frame.close();
    };
    const loadWasm = () => {
      if (wasmLoading) return;
      wasmLoading = true;
      void Promise.all([import("../lib/wasmH264"), import("../lib/yuvRenderer")]).then(
        ([{WasmH264Decoder}, {YuvRenderer}]) => {
          if (!alive) return;
          wasm = new WasmH264Decoder(
            pic => {
              const c = glCanvasRef.current;
              if (c && alive) (renderer ??= new YuvRenderer(c)).draw(pic.data, pic.width, pic.height);
              const t0 = submitted.get(pic.id);
              if (t0 !== undefined) {
                const d = performance.now() - t0;
                decodeMs = decodeMs === null ? d : 0.8 * decodeMs + 0.2 * d;
                submitted.delete(pic.id);
              }
            },
            (message, id) => {
              if (id !== undefined) submitted.delete(id);
              setErr(`decoder: ${message}`);
              gate.broken();
            },
          );
        },
        e => setErr(`decoder: ${e instanceof Error ? e.message : String(e)}`),
      );
    };
    const newDecoder = () => {
      if (!webcodecs) return null;
      return new VideoDecoder({
        output: draw,
        error: e => {
          setErr(`decoder: ${e.message}`);
          gate.broken();
          decoder = null; // recreated on the next key frame
        },
      });
    };

    const off = ws.on(topic, (d, f) => {
      if (f.topic !== topic) return; // the prefix also matches e.g. rgb2
      const fr = parseVideoFrame(d);
      if (!fr) return;
      const now = performance.now();
      meter.add(fr.data.byteLength, now);
      last = fr;
      lagMs = frameLagMs(fr, Date.now(), consoleClockOffset());
      if (fr.codec === "jpeg") {
        const url = URL.createObjectURL(new Blob([fr.data as BlobPart], {type: "image/jpeg"}));
        if (lastUrl) URL.revokeObjectURL(lastUrl);
        lastUrl = url;
        setJpegUrl(url);
        return;
      }
      const cfg = decoderConfig(fr.codec);
      const useWasm = fr.codec === "h264" && h264Path === "wasm";
      if (!cfg || (!webcodecs && !useWasm)) {
        gate.skipped++;
        return;
      }
      if (useWasm && !wasm) {
        loadWasm(); // first h264 frame: fetch the decoder, then start at a key frame
        gate.broken();
      }
      if (decoder && decoder.decodeQueueSize > 8) gate.broken(); // decoder backlog: drop to the next key frame
      if (wasm && wasm.queueSize > WASM_BACKLOG) gate.broken();
      const {action, reconfigure} = gate.accept(fr, f.seq);
      if (action === "skip") {
        // lost sync: ask for a key frame now instead of freezing until the
        // next GOP boundary (up to 2 s); at most once a second
        if (performance.now() - lastKfReq > 1000) {
          lastKfReq = performance.now();
          void post(`${base}/keyframe`, {}).catch(() => undefined);
        }
        return;
      }
      if (useWasm) {
        if (!wasm) return;
        if (reconfigure) wasm.reset();
        const id = Math.round(fr.seq * 1000);
        submitted.set(id, now);
        if (submitted.size > 64) submitted.delete(submitted.keys().next().value as number);
        wasm.decode(fr.data, fr.width, fr.height, id);
        return;
      }
      try {
        if (!decoder || decoder.state === "closed") decoder = newDecoder();
        if (!decoder) return;
        if (reconfigure || decoder.state !== "configured") {
          if (decoder.state === "configured") decoder.reset();
          decoder.configure(cfg);
        }
        submitted.set(Math.round(fr.seq * 1000), now);
        if (submitted.size > 64) submitted.delete(submitted.keys().next().value as number);
        decoder.decode(new EncodedVideoChunk(chunkInit(fr)));
      } catch (e) {
        setErr(`decoder: ${e instanceof Error ? e.message : String(e)}`);
        gate.broken();
        try {
          decoder?.close();
        } catch {
          /* already closed */
        }
        decoder = null;
      }
    });
    const id = window.setInterval(() => {
      const r = meter.rates(performance.now());
      setStats({
        ...r,
        decodeMs,
        lagMs,
        skipped: gate.skipped,
        waitingKey: gate.waitingKey && last?.codec !== "jpeg",
        codec: last?.codec ?? "",
        decoder: last?.codec === "h264" ? (useWasmPath() ? "wasm" : "webcodecs") : null,
        width: last?.width ?? 0,
        height: last?.height ?? 0,
      });
    }, 500);
    return () => {
      alive = false;
      off();
      window.clearInterval(id);
      wasm?.close();
      try {
        decoder?.close();
      } catch {
        /* already closed */
      }
      if (lastUrl) URL.revokeObjectURL(lastUrl);
      setJpegUrl(null);
      setStats(null);
    };
  }, [robot, cam, ready, visible, webcodecs, h264Path]);

  const rob = server?.robot;
  const showJpeg = stats?.codec === "jpeg";
  const showWasm = stats?.decoder === "wasm";
  return (
    <>
      <div className="row gap-sm wrap">
        <select className="input input-sm" value={fps} onChange={e => setFps(Number(e.target.value))} aria-label="Video frame rate">
          {FPS_OPTIONS.map(f => (
            <option key={f} value={f}>
              {f} fps
            </option>
          ))}
        </select>
        <div className="segmented" role="group" aria-label="Bitrate">
          {BITRATE_PRESETS.map(p => (
            <button key={p.kbps} className={kbps === p.kbps ? "on" : ""} onClick={() => setKbps(p.kbps)} title={`${p.kbps} kbit/s`}>
              {p.label}
            </button>
          ))}
        </div>
        <select className="input input-sm" value={width} onChange={e => setWidth(Number(e.target.value))} aria-label="Video width">
          {VIDEO_WIDTHS.map(w => (
            <option key={w} value={w}>
              {w} px
            </option>
          ))}
        </select>
        <select className="input input-sm" value={codecChoice} onChange={e => setCodecChoice(e.target.value as "auto" | VideoCodec)} aria-label="Codec">
          <option value="auto">auto{autoCodec ? ` (${autoCodec})` : ""}</option>
          <option value="h264" disabled={!h264Path}>
            h264{h264Path === "wasm" ? " (wasm)" : ""}
          </option>
          <option value="vp8" disabled={!webcodecs}>
            vp8
          </option>
          <option value="jpeg">jpeg</option>
        </select>
      </div>
      <div className="camera-frame">
        <canvas ref={canvasRef} className="camera-canvas" style={{display: showJpeg || !stats?.codec || showWasm ? "none" : "block"}} aria-label={`${robot} ${cam} video`} />
        <canvas ref={glCanvasRef} className="camera-canvas" style={{display: showWasm ? "block" : "none"}} aria-label={`${robot} ${cam} video (wasm)`} />
        {showJpeg && jpegUrl && <img src={jpegUrl} alt={`${robot} ${cam}`} />}
        {!stats?.codec && <div className="muted small camera-empty">{visible ? "Waiting for video..." : "Paused (tab hidden)"}</div>}
      </div>
      {stats && (
        <div className="mono small camera-meta">
          {stats.codec}
          {stats.decoder === "wasm" ? " (wasm)" : ""} · {stats.fps.toFixed(1)} fps · {Math.round(stats.kbps)} kbit/s
          {stats.decodeMs !== null && ` · decode ${stats.decodeMs.toFixed(0)} ms`}
          {stats.lagMs !== null && stats.lagMs > -5000 && stats.lagMs < 60000 && (
            <span title="Robot capture to browser display, corrected for robot/console/browser clock offsets"> · lag {Math.max(0, stats.lagMs).toFixed(0)} ms</span>
          )}
          {stats.skipped > 0 && ` · skipped ${stats.skipped}`}
          {stats.waitingKey && stats.codec && <span className="tone-warn"> · waiting for key frame</span>}
        </div>
      )}
      {(rob || server?.params) && (
        <div className="mono small camera-meta">
          {rob
            ? `${rob.codec} ${rob.width}x${rob.height} @ ${rob.fps} fps, target ${rob.kbps} kbit/s, sent ${Math.round(rob.sent_kbps)} kbit/s, robot dropped ${rob.dropped}`
            : `requested ${server?.params?.codec} ${server?.params?.width} px @ ${server?.params?.fps} fps, ${server?.params?.kbps} kbit/s`}
          {server?.rx?.lost ? ` · lost in transit ${server.rx.lost}` : ""}
        </div>
      )}
      {rob?.error && (
        <div className="inline-err small">
          Robot: {rob.error}
          {rob.requested_codec ? ` (fell back from ${rob.requested_codec} to ${rob.codec})` : ""}
        </div>
      )}
      {err && <div className="inline-err small">{err}</div>}
    </>
  );
}
