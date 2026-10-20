// Camera tab, "All cameras" + Video: live video of every camera at once
// (API.md §12.4). Default "mosaic": the robot tiles the cameras into ONE
// stream (one encoder there, one decoder here) and each tile's rectangle is
// drawn into that tile's canvas. "Per camera": one small stream per tile.
// The operator picks which cameras show (chips: on/off, drag or ‹ › to
// reorder = mosaic tile order, All / None; saved per robot) and can solo one
// (click a tile). What streams = selected ∩ in view, only while the page is
// visible; changes are debounced 1 s.
import {useEffect, useMemo, useRef, useState} from "react";
import {ws} from "../ws";
import {consoleClockOffset} from "../lib/clock";
import {GRID_QUALITY, MOSAIC, StampRate, dropCamera, frameLagMs, gridStreams, h264Decoder, moveCamera, normalizeSelection, parseVideoFrame, pickCodec, selectedCameras, setAllCameras, streamCameras, toggleCamera, type GridQuality, type GridSelection, type GridStreamParams, type GridVideoMode, type H264Decoder, type VideoCodec, type VideoFrame as StreamFrame} from "../lib/video";
import {StreamDecoder} from "../lib/videoDecode";
import {detail, post, usePageVisible, viewerId} from "./VideoView";

const START_SPACING_MS = 1100; // the console rate-limits parameter changes to 1/s
const KEEPALIVE_MS = 2000;
const VIEW_DEBOUNCE_MS = 1000; // scrolling must not retune the mosaic on every pixel
const forceWasm = () => typeof location !== "undefined" && new URLSearchParams(location.search).get("h264") === "wasm";

interface TileStat {
  fps: number;
  kbps: number | null;
  /** capture-to-display age at the last picture (s) and when that was (ms) */
  age: number | null;
  drawnAt: number;
  noFrame: boolean;
}

interface RobotSide {
  codec?: string;
  width?: number;
  height?: number;
  kbps?: number;
  kbps_effective?: number;
  sent_kbps?: number;
  dropped?: number;
  error?: string;
  requested_codec?: string;
}

const store = {
  get<T extends string>(k: string, ok: readonly T[], dflt: T): T {
    try {
      const v = localStorage.getItem(k);
      return v && (ok as readonly string[]).includes(v) ? (v as T) : dflt;
    } catch {
      return dflt;
    }
  },
  set(k: string, v: string) {
    try {
      localStorage.setItem(k, v);
    } catch {
      /* private window */
    }
  },
};

/** Start + keepalive + retune + stop of one console video stream. */
function useStreamSession(robot: string, p: GridStreamParams, codec: VideoCodec | null, enabled: boolean, onStatus?: (s: {robot?: RobotSide | null}) => void) {
  const base = `api/robots/${encodeURIComponent(robot)}/video/${encodeURIComponent(p.camera)}`;
  const viewer = useRef(viewerId());
  const body = {fps: p.fps, kbps: p.kbps, width: p.width, codec, ...(p.cameras ? {cameras: p.cameras} : {})};
  const bodyKey = JSON.stringify(body);
  const bodyRef = useRef(body);
  bodyRef.current = body;
  const startRef = useRef<(() => Promise<void>) | null>(null);
  const lastStart = useRef(0);
  const statusRef = useRef(onStatus);
  statusRef.current = onStatus;
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    if (!enabled || !codec) return;
    let alive = true;
    let timer: number | undefined;
    const start = async (): Promise<void> => {
      window.clearTimeout(timer);
      const wait = START_SPACING_MS - (Date.now() - lastStart.current);
      if (wait > 0) {
        timer = window.setTimeout(() => void start(), wait);
        return;
      }
      lastStart.current = Date.now();
      try {
        const res = await post(`${base}/start`, {...bodyRef.current, viewer: viewer.current});
        if (!alive) return;
        if (res.status === 429) {
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
        if (res.status === 404) return void start();
        if (!res.ok) throw new Error(await detail(res));
        statusRef.current?.(await res.json());
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
  }, [base, enabled, codec]);

  // retune a running stream (e.g. the visible camera list changed)
  useEffect(() => {
    void startRef.current?.();
  }, [bodyKey]);

  return {err, base};
}

/** Subscribe to a stream's WebSocket topic and decode it. */
function useDecodedStream(robot: string, camera: string, h264: H264Decoder | null, enabled: boolean, keyframeUrl: string, onPicture: (src: CanvasImageSource, w: number, h: number, fr: StreamFrame, dec: StreamDecoder) => void, onError: (m: string) => void) {
  const pictureRef = useRef(onPicture);
  pictureRef.current = onPicture;
  const errRef = useRef(onError);
  errRef.current = onError;
  const decRef = useRef<StreamDecoder | null>(null);
  useEffect(() => {
    if (!enabled) return;
    const topic = `video/${robot}/${camera}`;
    const dec: StreamDecoder = new StreamDecoder({
      h264,
      onPicture: (src, w, h, fr) => pictureRef.current(src, w, h, fr, dec),
      onError: m => errRef.current(m),
      onNeedKeyframe: () => void post(keyframeUrl, {}).catch(() => undefined),
    });
    decRef.current = dec;
    const off = ws.on(topic, (d, f) => {
      if (f.topic !== topic) return; // the prefix also matches e.g. rgb2
      const fr = parseVideoFrame(d);
      if (fr) dec.push(fr, f.seq);
    });
    return () => {
      off();
      dec.close();
      decRef.current = null;
    };
  }, [robot, camera, h264, enabled, keyframeUrl]);
  return decRef;
}

function draw(canvas: HTMLCanvasElement | undefined, src: CanvasImageSource, sx: number, sy: number, w: number, h: number) {
  if (!canvas) return;
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  canvas.getContext("2d")?.drawImage(src, sx, sy, w, h, 0, 0, w, h);
}

type Shared = {
  canvases: Map<string, HTMLCanvasElement>;
  stats: Map<string, TileStat>;
};

/** The mosaic: one stream, cropped into the tiles. Renders nothing itself. */
function MosaicStream({robot, p, codec, h264, enabled, shared, onInfo}: {robot: string; p: GridStreamParams; codec: VideoCodec | null; h264: H264Decoder | null; enabled: boolean; shared: Shared; onInfo: (i: StreamInfo) => void}) {
  const camsRef = useRef<string[]>([]);
  camsRef.current = p.cameras ?? [];
  const robotSide = useRef<RobotSide | null>(null);
  const [decErr, setDecErr] = useState<string | null>(null);
  const {err, base} = useStreamSession(robot, p, codec, enabled, s => (robotSide.current = s.robot ?? null));
  const stamps = useMemo(() => new StampRate(), []);
  const dec = useDecodedStream(
    robot,
    MOSAIC,
    h264,
    enabled && codec !== null,
    `${base}/keyframe`,
    (src, _w, _h, fr) => {
      const now = performance.now();
      const lag = frameLagMs(fr, Date.now(), consoleClockOffset());
      for (const t of fr.tiles ?? []) {
        stamps.add(t.camera, t.stamp, now);
        const canvas = shared.canvases.get(t.camera);
        const prev = shared.stats.get(t.camera);
        if (!t.w || !t.h) {
          shared.stats.set(t.camera, {fps: 0, kbps: null, age: null, drawnAt: now, noFrame: true});
          continue;
        }
        draw(canvas, src, t.x, t.y, t.w, t.h);
        const age = (lag !== null && lag > -5000 && lag < 60000 ? Math.max(0, lag) / 1000 : 0) + (t.age ?? 0);
        shared.stats.set(t.camera, {...(prev ?? {kbps: null}), fps: stamps.fps(t.camera, now), kbps: null, age, drawnAt: now, noFrame: false});
      }
    },
    m => setDecErr(m),
  );
  useEffect(() => {
    const id = window.setInterval(() => {
      const d = dec.current;
      const now = performance.now();
      for (const c of camsRef.current) {
        const s = shared.stats.get(c);
        if (s) s.fps = stamps.fps(c, now);
      }
      const r = d?.meter.rates(now) ?? {fps: 0, kbps: 0};
      onInfo({fps: r.fps, kbps: r.kbps, codec: d?.last?.codec ?? "", width: d?.last?.width ?? 0, height: d?.last?.height ?? 0, waitingKey: Boolean(d?.gate.waitingKey && d.last && d.last.codec !== "jpeg"), robot: robotSide.current, err: err ?? decErr});
    }, 500);
    return () => window.clearInterval(id);
  }, [dec, err, decErr, onInfo, shared, stamps]);
  return null;
}

/** Per-camera mode: one small stream for one tile. */
function TileStream({robot, p, codec, h264, enabled, shared, onErr}: {robot: string; p: GridStreamParams; codec: VideoCodec | null; h264: H264Decoder | null; enabled: boolean; shared: Shared; onErr: (cam: string, e: string | null) => void}) {
  const cam = p.camera;
  const {err, base} = useStreamSession(robot, p, codec, enabled);
  const dec = useDecodedStream(
    robot,
    cam,
    h264,
    enabled && codec !== null,
    `${base}/keyframe`,
    (src, w, h, fr, d) => {
      const now = performance.now();
      draw(shared.canvases.get(cam), src, 0, 0, w, h);
      const lag = frameLagMs(fr, Date.now(), consoleClockOffset());
      const r = d.meter.rates(now);
      shared.stats.set(cam, {fps: r.fps, kbps: r.kbps, age: lag !== null && lag > -5000 && lag < 60000 ? Math.max(0, lag) / 1000 : null, drawnAt: now, noFrame: false});
    },
    m => onErr(cam, m),
  );
  useEffect(() => onErr(cam, err), [cam, err, onErr]);
  useEffect(() => {
    const id = window.setInterval(() => {
      const s = shared.stats.get(cam);
      const d = dec.current;
      if (s && d) Object.assign(s, d.meter.rates(performance.now()));
    }, 500);
    return () => window.clearInterval(id);
  }, [cam, dec, shared]);
  return null;
}

interface StreamInfo {
  fps: number;
  kbps: number;
  codec: string;
  width: number;
  height: number;
  waitingKey: boolean;
  robot: RobotSide | null;
  err: string | null;
}

/** Tiles in view (IntersectionObserver); every tile without one. */
function useInView(cams: string[]) {
  const els = useRef(new Map<string, Element>());
  const raw = useRef(new Set<string>());
  const [inView, setInView] = useState<string[]>(typeof IntersectionObserver === "undefined" ? cams : []);
  const obs = useRef<IntersectionObserver | null>(null);
  const camsKey = cams.join("\u0000");
  useEffect(() => {
    if (typeof IntersectionObserver === "undefined") {
      setInView(cams);
      return;
    }
    const o = new IntersectionObserver(entries => {
      for (const e of entries) {
        const cam = (e.target as HTMLElement).dataset.cam;
        if (!cam) continue;
        if (e.isIntersecting) raw.current.add(cam);
        else raw.current.delete(cam);
      }
      const next = cams.filter(c => raw.current.has(c));
      setInView(prev => (prev.join("\u0000") === next.join("\u0000") ? prev : next));
    });
    obs.current = o;
    for (const el of els.current.values()) o.observe(el);
    return () => {
      o.disconnect();
      obs.current = null;
    };
  }, [camsKey]); // eslint-disable-line react-hooks/exhaustive-deps
  const ref = (cam: string) => (el: Element | null) => {
    const old = els.current.get(cam);
    if (old && old !== el) obs.current?.unobserve(old);
    if (el) {
      els.current.set(cam, el);
      obs.current?.observe(el);
    } else {
      els.current.delete(cam);
      raw.current.delete(cam);
    }
  };
  return {inView, ref};
}

/** The camera list to stream, debounced: scrolling or clicking through the
 * selection chips must not retune the mosaic on every change. The first list
 * (nothing streaming yet) and "nothing" apply at once. */
function useDebouncedList(list: string[], ms: number): string[] {
  const key = list.join("\u0000");
  const [out, setOut] = useState(list);
  const outKey = out.join("\u0000");
  useEffect(() => {
    if (key === outKey) return;
    if (!outKey || !key) return void setOut(list);
    const t = window.setTimeout(() => setOut(list), ms);
    return () => window.clearTimeout(t);
  }, [key, outKey, ms]); // eslint-disable-line react-hooks/exhaustive-deps
  return out;
}

const selKey = (robot: string) => `adt4.gridVideo.sel.${robot}`;
function loadSelection(robot: string, cams: string[]): GridSelection {
  try {
    const raw = localStorage.getItem(selKey(robot));
    return normalizeSelection(raw ? (JSON.parse(raw) as Partial<GridSelection>) : null, cams);
  } catch {
    return normalizeSelection(null, cams);
  }
}

/** Per-camera on/off chips (drag or ‹ › to reorder = mosaic tile order). */
function SelectionChips({sel, onChange}: {sel: GridSelection; onChange: (s: GridSelection) => void}) {
  const drag = useRef<string | null>(null);
  const off = new Set(sel.off);
  return (
    <div className="row gap-sm wrap grid-cam-chips" role="group" aria-label="Grid cameras">
      <button className="btn btn-xs" onClick={() => onChange(setAllCameras(sel, true))} disabled={!sel.off.length}>
        All
      </button>
      <button className="btn btn-xs" onClick={() => onChange(setAllCameras(sel, false))} disabled={sel.off.length === sel.order.length}>
        None
      </button>
      {sel.order.map((c, i) => (
        <span
          key={c}
          className={`grid-cam-chip ${off.has(c) ? "off" : ""}`}
          draggable
          onDragStart={e => {
            drag.current = c;
            e.dataTransfer.effectAllowed = "move";
          }}
          onDragOver={e => e.preventDefault()}
          onDrop={e => {
            e.preventDefault();
            if (drag.current && drag.current !== c) onChange(dropCamera(sel, drag.current, c));
            drag.current = null;
          }}
          title="Drag to reorder (= tile order)"
        >
          <button className="chip-move" onClick={() => onChange(moveCamera(sel, c, -1))} disabled={i === 0} aria-label={`Move ${c} earlier`}>
            ‹
          </button>
          <label>
            <input type="checkbox" checked={!off.has(c)} onChange={() => onChange(toggleCamera(sel, c))} aria-label={`Show ${c}`} />
            <span className="mono small">{c}</span>
          </label>
          <button className="chip-move" onClick={() => onChange(moveCamera(sel, c, 1))} disabled={i === sel.order.length - 1} aria-label={`Move ${c} later`}>
            ›
          </button>
        </span>
      ))}
    </div>
  );
}

const MODES: readonly GridVideoMode[] = ["mosaic", "tiles"];
const QUALITIES: readonly GridQuality[] = ["low", "medium"];

export function GridVideo({robot, cams, onOpen}: {robot: string; cams: string[]; onOpen: (cam: string) => void}) {
  const [mode, setModeRaw] = useState<GridVideoMode>(() => store.get("adt4.gridVideo.mode", MODES, "mosaic"));
  const [quality, setQualityRaw] = useState<GridQuality>(() => store.get("adt4.gridVideo.quality", QUALITIES, "medium"));
  const setMode = (m: GridVideoMode) => (setModeRaw(m), store.set("adt4.gridVideo.mode", m));
  const setQuality = (q: GridQuality) => (setQualityRaw(q), store.set("adt4.gridVideo.quality", q));
  const camsKey = cams.join("\u0000");
  const [sel, setSelRaw] = useState<GridSelection>(() => loadSelection(robot, cams));
  useEffect(() => setSelRaw(loadSelection(robot, cams)), [robot, camsKey]); // eslint-disable-line react-hooks/exhaustive-deps
  const setSel = (s: GridSelection) => {
    setSelRaw(s);
    store.set(selKey(robot), JSON.stringify(s));
  };
  const [solo, setSolo] = useState<string | null>(null);
  const visible = usePageVisible();
  const [codec, setCodec] = useState<VideoCodec | null>(null);
  const [h264, setH264] = useState<H264Decoder | null>(null);
  const [info, setInfo] = useState<StreamInfo | null>(null);
  const [tileErr, setTileErr] = useState<Record<string, string | null>>({});
  const [, setTick] = useState(0);
  const shared = useMemo<Shared>(() => ({canvases: new Map(), stats: new Map()}), []);
  const shown = selectedCameras(sel, solo);
  const soloOn = solo !== null && shown.length === 1 && shown[0] === solo;
  const {inView, ref} = useInView(shown);
  const streamCams = useDebouncedList(streamCameras(shown, inView), VIEW_DEBOUNCE_MS);

  useEffect(() => {
    let alive = true;
    const wasm = forceWasm();
    void (wasm ? Promise.resolve("h264" as const) : pickCodec()).then(c => alive && setCodec(c));
    void (wasm ? Promise.resolve("wasm" as const) : h264Decoder()).then(d => alive && setH264(d));
    return () => {
      alive = false;
    };
  }, []);
  useEffect(() => {
    const id = window.setInterval(() => setTick(t => t + 1), 500);
    return () => window.clearInterval(id);
  }, []);
  // a new robot / mode starts from blank tiles
  useEffect(() => {
    shared.stats.clear();
    setInfo(null);
    setTileErr({});
    setSolo(null);
  }, [robot, mode, shared]);

  const ready = codec !== null && (codec !== "h264" || h264 !== null);
  const enabled = visible && ready;
  const streams = gridStreams(mode, quality, streamCams, soloOn);
  const onTileErr = useMemo(() => (cam: string, e: string | null) => setTileErr(m => (m[cam] === e ? m : {...m, [cam]: e})), []);
  const q = GRID_QUALITY[quality];
  const now = performance.now();

  return (
    <>
      <div className="row gap-sm wrap camera-grid-controls">
        <div className="segmented" role="group" aria-label="Grid video mode">
          <button className={mode === "mosaic" ? "on" : ""} onClick={() => setMode("mosaic")} title="The robot tiles the selected cameras into one stream: one encoder, least CPU and bandwidth">
            Mosaic
          </button>
          <button className={mode === "tiles" ? "on" : ""} onClick={() => setMode("tiles")} title="One stream per selected camera (one encoder each on the robot)">
            Per camera
          </button>
        </div>
        <div className="segmented" role="group" aria-label="Grid video quality">
          {QUALITIES.map(k => (
            <button key={k} className={quality === k ? "on" : ""} onClick={() => setQuality(k)} title={`${GRID_QUALITY[k].width} px tiles, ${GRID_QUALITY[k].fps} fps, ${GRID_QUALITY[k].kbps} kbit/s for the whole grid`}>
              {k}
            </button>
          ))}
        </div>
        <span className="mono small muted">
          {mode === "mosaic" && info?.codec
            ? `${info.codec} ${info.width}x${info.height} · ${info.fps.toFixed(1)} fps · ${Math.round(info.kbps)} kbit/s`
            : `${q.width} px · ${q.fps} fps · ${q.kbps} kbit/s total`}
          {mode === "mosaic" && info?.robot?.kbps_effective !== undefined && info.robot.kbps_effective < (info.robot.kbps ?? 0) ? ` · capped to ${info.robot.kbps_effective} kbit/s by the C5 budget` : ""}
          {` · streaming ${streamCams.length}/${shown.length}`}
          {!visible ? " · paused (tab hidden)" : ""}
        </span>
      </div>
      {soloOn ? (
        <div className="row gap-sm">
          <span className="small">
            Solo: <span className="mono">{solo}</span>
          </span>
          <button className="btn btn-xs" onClick={() => setSolo(null)}>
            Back to all
          </button>
        </div>
      ) : (
        <SelectionChips sel={sel} onChange={setSel} />
      )}
      {mode === "mosaic" && info?.waitingKey && <div className="small tone-warn">waiting for key frame</div>}
      {mode === "mosaic" && (info?.err || info?.robot?.error) && (
        <div className="inline-err small">
          {info?.robot?.error ? `Robot: ${info.robot.error}${info.robot.requested_codec ? ` (fell back from ${info.robot.requested_codec} to ${info.robot.codec})` : ""}` : info?.err}
        </div>
      )}
      {!shown.length && <div className="muted small">No camera selected. Switch some on above.</div>}
      <div className={`camera-grid ${soloOn ? "solo" : ""}`}>
        {shown.map(c => {
          const s = shared.stats.get(c);
          const age = s && s.age !== null ? s.age + (now - s.drawnAt) / 1000 : null;
          const err = mode === "tiles" ? tileErr[c] : null;
          const live = streamCams.includes(c);
          return (
            <div className="camera-tile" key={`${robot}/${c}`} ref={ref(c)} data-cam={c}>
              <button className="camera-tile-img" onClick={() => setSolo(soloOn ? null : c)} title={soloOn ? "Back to all cameras" : `Solo ${c} (only this camera, larger)`}>
                <canvas
                  ref={el => {
                    if (el) shared.canvases.set(c, el);
                    else shared.canvases.delete(c);
                  }}
                  aria-label={`${robot} ${c} video`}
                  style={{display: s && !s.noFrame ? "block" : "none"}}
                />
                {(!s || s.noFrame) && <span className="muted small">{!live ? "not in view" : s?.noFrame ? "no frame yet" : "..."}</span>}
              </button>
              <div className="camera-tile-bar row between">
                <span className="mono small">{c}</span>
                <span className="row gap-sm">
                  {err ? (
                    <span className="small tone-err" title={err}>
                      error
                    </span>
                  ) : !live && s ? (
                    <span className="small muted">paused</span>
                  ) : s && !s.noFrame ? (
                    <span className="mono small muted" title={mode === "mosaic" ? "new images per second · capture-to-display age (the bitrate is shared, see above)" : "frames per second · bitrate · capture-to-display age"}>
                      {s.fps.toFixed(1)} fps{s.kbps !== null ? ` · ${Math.round(s.kbps)} kb/s` : ""}
                      {age !== null && <span className={age > 3 ? "tone-warn" : ""}> · {age.toFixed(1)} s</span>}
                    </span>
                  ) : null}
                  <button className="btn btn-xs" onClick={() => onOpen(c)} title={`Full live video of ${c} (Video mode)`}>
                    full
                  </button>
                </span>
              </div>
            </div>
          );
        })}
      </div>
      {streams.map(p =>
        mode === "mosaic" ? (
          <MosaicStream key={robot} robot={robot} p={p} codec={codec} h264={h264} enabled={enabled} shared={shared} onInfo={setInfo} />
        ) : (
          <TileStream key={`${robot}/${p.camera}`} robot={robot} p={p} codec={codec} h264={h264} enabled={enabled} shared={shared} onErr={onTileErr} />
        ),
      )}
    </>
  );
}
