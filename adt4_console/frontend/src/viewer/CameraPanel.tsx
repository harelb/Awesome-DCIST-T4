// Camera tab: on-demand frames from a robot's fleetlink agent (C4 request/reply).
// One request at a time; auto-refresh waits for the previous frame to land.
import {useEffect, useRef, useState} from "react";
import {getJSON, errorText} from "../lib/api";
import {getToken} from "../lib/auth";
import {Empty, Toggle} from "../components/ui";
import {useLiveRobots} from "../stores/robots";

export interface CameraFrame {
  url: string;
  bytes: number;
  width: number | null;
  height: number | null;
  srcWidth: number | null;
  srcHeight: number | null;
  /** robot-side age when the reply was built (s) */
  age: number | null;
  /** browser time the frame arrived (ms) */
  at: number;
  /** round trip of the request (ms) */
  rtt: number;
}

const num = (v: string | null) => (v === null || v === "" || Number.isNaN(Number(v)) ? null : Number(v));

export const MIN_HZ = 0.2;
export const MAX_HZ = 5;
export const clampHz = (hz: number) => Math.min(MAX_HZ, Math.max(MIN_HZ, hz));

/** Frame age now: robot-side age at reply time + time since it arrived. */
export function frameAge(f: Pick<CameraFrame, "age" | "at">, now: number): number | null {
  return f.age === null ? null : f.age + Math.max(0, now - f.at) / 1000;
}

async function fetchFrame(robot: string, cam: string, w: number, signal: AbortSignal): Promise<CameraFrame> {
  // plain fetch, not lib/api `request`: a 504 here is a slow robot, not a dead console
  const headers = new Headers();
  const t = getToken();
  if (t) headers.set("Authorization", `Bearer ${t}`);
  const t0 = performance.now();
  const res = await fetch(`api/robots/${encodeURIComponent(robot)}/camera/${encodeURIComponent(cam)}.jpg?w=${w}&q=70`, {headers, signal, cache: "no-store"});
  if (!res.ok) {
    let detail = `HTTP ${res.status}`;
    try {
      const j = await res.json();
      if (typeof j.detail === "string") detail = j.detail;
    } catch {
      /* not JSON */
    }
    throw new Error(res.status === 504 ? `No reply from ${robot} (timeout)` : detail);
  }
  const blob = await res.blob();
  const h = res.headers;
  return {
    url: URL.createObjectURL(blob),
    bytes: num(h.get("X-Frame-Bytes")) ?? blob.size,
    width: num(h.get("X-Frame-Width")),
    height: num(h.get("X-Frame-Height")),
    srcWidth: num(h.get("X-Source-Width")),
    srcHeight: num(h.get("X-Source-Height")),
    age: num(h.get("X-Frame-Age")),
    at: Date.now(),
    rtt: performance.now() - t0,
  };
}

export function CameraPanel() {
  const robots = useLiveRobots();
  const [robot, setRobot] = useState<string>("");
  const [cams, setCams] = useState<string[]>([]);
  const [cam, setCam] = useState<string>("");
  const [auto, setAuto] = useState(true);
  const [hz, setHz] = useState(1);
  const [width, setWidth] = useState(640);
  const [frame, setFrame] = useState<CameraFrame | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [now, setNow] = useState(Date.now());
  const [nonce, setNonce] = useState(0);
  const frameRef = useRef<CameraFrame | null>(null);

  const r = robot && robots.includes(robot) ? robot : (robots[0] ?? "");

  // camera list per robot
  useEffect(() => {
    if (!r) return;
    let alive = true;
    getJSON<{cameras: string[]} | string[]>(`api/robots/${encodeURIComponent(r)}/cameras`)
      .then(d => {
        if (!alive) return;
        const list = Array.isArray(d) ? d : (d.cameras ?? []);
        setCams(list);
        setCam(c => (list.includes(c) ? c : (list[0] ?? "")));
      })
      .catch(e => alive && setErr(errorText(e)));
    return () => {
      alive = false;
    };
  }, [r]);

  // fetch loop: one request in flight; next one `1/hz` after the previous started
  useEffect(() => {
    if (!r || !cam) return;
    const ctl = new AbortController();
    let timer: number | undefined;
    const run = async () => {
      const started = Date.now();
      setBusy(true);
      try {
        const f = await fetchFrame(r, cam, width, ctl.signal);
        if (ctl.signal.aborted) {
          URL.revokeObjectURL(f.url);
          return;
        }
        if (frameRef.current) URL.revokeObjectURL(frameRef.current.url);
        frameRef.current = f;
        setFrame(f);
        setErr(null);
      } catch (e) {
        if (!ctl.signal.aborted) setErr(e instanceof Error ? e.message : String(e));
      } finally {
        if (!ctl.signal.aborted) setBusy(false);
      }
      if (auto && !ctl.signal.aborted) {
        const wait = Math.max(0, 1000 / clampHz(hz) - (Date.now() - started));
        timer = window.setTimeout(run, wait);
      }
    };
    void run();
    return () => {
      ctl.abort();
      window.clearTimeout(timer);
    };
  }, [r, cam, auto, hz, width, nonce]);

  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 250);
    return () => window.clearInterval(id);
  }, []);
  useEffect(
    () => () => {
      if (frameRef.current) URL.revokeObjectURL(frameRef.current.url);
    },
    [],
  );

  if (!robots.length) return <Empty title="No live robots">Camera frames come from a robot's fleetlink agent. None has reported yet.</Empty>;
  const age = frame ? frameAge(frame, now) : null;
  return (
    <div className="camera-panel">
      <div className="row gap-sm wrap">
        <select className="input input-sm" value={r} onChange={e => setRobot(e.target.value)} aria-label="Robot">
          {robots.map(x => (
            <option key={x} value={x}>
              {x}
            </option>
          ))}
        </select>
        <select className="input input-sm" value={cam} onChange={e => setCam(e.target.value)} aria-label="Camera stream" disabled={!cams.length}>
          {!cams.length && <option value="">no cameras</option>}
          {cams.map(c => (
            <option key={c} value={c}>
              {c}
            </option>
          ))}
        </select>
        <select className="input input-sm" value={width} onChange={e => setWidth(Number(e.target.value))} aria-label="Frame width">
          {[320, 640, 960, 1280].map(w => (
            <option key={w} value={w}>
              {w} px
            </option>
          ))}
        </select>
        <button className="btn btn-sm" onClick={() => setNonce(n => n + 1)} disabled={!cam}>
          Refresh
        </button>
      </div>
      <div className="row gap-sm wrap">
        <Toggle checked={auto} onChange={setAuto} label="Auto refresh" />
        <label className="slider-row camera-hz">
          <input type="range" min={MIN_HZ} max={MAX_HZ} step={0.1} value={hz} disabled={!auto} aria-label="Refresh rate" onChange={e => setHz(clampHz(Number(e.target.value)))} />
          <span className="mono small">{hz.toFixed(1)} Hz</span>
        </label>
      </div>
      <div className={`camera-frame ${busy ? "busy" : ""}`}>
        {frame ? <img src={frame.url} alt={`${r} ${cam}`} /> : <div className="muted small camera-empty">{busy ? "Requesting frame..." : "No frame"}</div>}
      </div>
      {frame && (
        <div className="mono small camera-meta">
          <span className={age !== null && age > 5 ? "tone-warn" : ""}>age {age === null ? "?" : `${age.toFixed(1)} s`}</span>
          {" · "}
          {(frame.bytes / 1024).toFixed(1)} KB
          {frame.width ? ` · ${frame.width}x${frame.height}` : ""}
          {frame.srcWidth ? ` (of ${frame.srcWidth}x${frame.srcHeight})` : ""}
          {` · ${Math.round(frame.rtt)} ms`}
        </div>
      )}
      {err && <div className="inline-err small">{err}</div>}
    </div>
  );
}
