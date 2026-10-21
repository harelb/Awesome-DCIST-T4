// Full-screen drive view for phones (and touch tablets): the robot's camera
// fills the screen, with two translucent sticks over it like Spot's tablet
// app. Left half of the screen = move stick (up/down forward/back, sideways
// strafes), right half = turn stick (sideways yaws); both work at once. A stick
// centres where the finger lands. The view menu switches cameras, all cameras
// at once, or the map.
//
// Safety is the Teleop tab's: Drive arms (confirmed once per browser session),
// the stream is TeleopDriver's 10 Hz with the console's and the robot's 0.5 s
// deadman behind it, lifting both fingers commands zero, and hiding the page,
// leaving the browser, changing the robot focus or closing the view disarms.
// While this view is open the Teleop tab stands down (viewer store driveFull).
import {useCallback, useEffect, useLayoutEffect, useRef, useState, type PointerEvent as ReactPointerEvent} from "react";
import {getJSON, postJSON, errorText} from "../lib/api";
import {CAPS, StickTracker, TeleopDriver, ZERO, armConfirmed, captureTouches, isZero, setArmConfirmed, teleopWsUrl, type DriverStatus, type StickId, type Twist} from "../lib/teleop";
import {useRobots} from "../stores/robots";
import {useViewer} from "../stores/viewer";
import type {TeleopView} from "../types";
import {Modal, Spinner} from "../components/ui";
import {useStillFrames} from "./CameraPanel";
import {VideoView} from "./VideoView";
import {GridVideo} from "./GridVideo";
import "./driveFullscreen.css";

/** Stick radius as a fraction of the shorter screen side (clamped, px). */
const stickRadius = (w: number, h: number) => Math.max(48, Math.min(90, Math.min(w, h) * 0.17));
const POLL_MS = 3000;
const VIEW_KEY = "adt4.driveFull.view";
const MEDIA_KEY = "adt4.driveFull.media";

export type DriveView = {kind: "cam"; cam: string} | {kind: "all"} | {kind: "map"};
type Media = "video" | "still";
type Cmd = "stand" | "sit" | "stop";

const load = (k: string) => {
  try {
    return localStorage.getItem(k);
  } catch {
    return null;
  }
};
const save = (k: string, v: string) => {
  try {
    localStorage.setItem(k, v);
  } catch {
    /* private window */
  }
};

/** Parse a remembered view, falling back to the robot's first camera. */
export function pickView(saved: string | null, cams: string[]): DriveView {
  if (saved === "all" && cams.length > 1) return {kind: "all"};
  if (saved === "map") return {kind: "map"};
  if (saved?.startsWith("cam:") && cams.includes(saved.slice(4))) return {kind: "cam", cam: saved.slice(4)};
  return cams.length ? {kind: "cam", cam: cams[0]} : {kind: "map"};
}
const viewKey = (v: DriveView) => (v.kind === "cam" ? `cam:${v.cam}` : v.kind);
const viewLabel = (v: DriveView) => (v.kind === "cam" ? v.cam : v.kind === "all" ? "All cameras" : "Map");

/** Enter browser full screen (Android Chrome; iPhone Safari has no element
 *  full screen, the view is then just fixed over the page). Call from the
 *  tap handler: browsers allow it only right after a user gesture. */
export function requestPageFullscreen() {
  const el = document.documentElement as HTMLElement & {webkitRequestFullscreen?: () => void};
  try {
    if (document.fullscreenElement) return;
    if (el.requestFullscreen) void el.requestFullscreen({navigationUI: "hide"}).catch(() => undefined);
    else el.webkitRequestFullscreen?.();
  } catch {
    /* not allowed: the CSS overlay still covers the page */
  }
}

function exitPageFullscreen() {
  try {
    if (document.fullscreenElement) void document.exitFullscreen().catch(() => undefined);
  } catch {
    /* already out */
  }
}

function StillPicture({robot, cam}: {robot: string; cam: string}) {
  const {frame, err} = useStillFrames(robot, cam, 960, 2);
  return <div className="camera-frame bare">{frame ? <img src={frame.url} alt={`${robot} ${cam}`} /> : <div className="muted small camera-empty">{err ?? "Requesting frame..."}</div>}</div>;
}

function StillTile({robot, cam}: {robot: string; cam: string}) {
  const {frame} = useStillFrames(robot, cam, 320, 1);
  return (
    <div className="dfs-tile">
      {frame ? <img src={frame.url} alt={`${robot} ${cam}`} /> : <span className="muted small">...</span>}
      <span className="dfs-tile-name mono">{cam}</span>
    </div>
  );
}

export function DriveFullscreen({robot, onExit}: {robot: string; onExit: () => void}) {
  const [cams, setCams] = useState<string[] | null>(null);
  const [view, setViewRaw] = useState<DriveView>({kind: "map"});
  const [media, setMediaRaw] = useState<Media>(() => (load(MEDIA_KEY) === "still" ? "still" : "video"));
  const [menu, setMenu] = useState(false);
  const setView = (v: DriveView) => {
    setViewRaw(v);
    save(VIEW_KEY, viewKey(v));
    setMenu(false);
  };
  const setMedia = (m: Media) => {
    setMediaRaw(m);
    save(MEDIA_KEY, m);
  };
  useEffect(() => {
    let alive = true;
    getJSON<{cameras: string[]} | string[]>(`api/robots/${encodeURIComponent(robot)}/cameras`)
      .then(d => {
        if (!alive) return;
        const list = Array.isArray(d) ? d : (d.cameras ?? []);
        setCams(list);
        setViewRaw(pickView(load(VIEW_KEY), list));
      })
      .catch(() => alive && setCams([]));
    return () => {
      alive = false;
    };
  }, [robot]);

  // executor state (the hub also pushes it), as the Teleop tab does
  const tv = useRobots(s => s.teleop[robot]);
  useEffect(() => {
    let alive = true;
    const pull = () =>
      getJSON<TeleopView>(`api/robots/${encodeURIComponent(robot)}/teleop`)
        .then(v => alive && useRobots.setState(s => ({teleop: {...s.teleop, [robot]: {...s.teleop[robot], ...v, state: v.state ?? null}}})))
        .catch(() => undefined);
    void pull();
    const h = window.setInterval(pull, POLL_MS);
    return () => {
      alive = false;
      window.clearInterval(h);
    };
  }, [robot]);

  // the velocity stream
  const driver = useRef<TeleopDriver | null>(null);
  const [drv, setDrv] = useState<DriverStatus>({connected: false, rttMs: null, acked: null, error: null, sent: 0});
  const [armed, setArmed] = useState(false);
  const [confirming, setConfirming] = useState(false);
  const [speed, setSpeed] = useState(0.25);
  const speedRef = useRef(speed);
  speedRef.current = speed;
  const [cmd, setCmd] = useState<Twist>(ZERO);
  const [note, setNote] = useState<string | null>(null);
  const [busy, setBusy] = useState<Cmd | null>(null);
  const tracker = useRef(new StickTracker(70));
  const [, setTick] = useState(0); // re-render the sticks

  useLayoutEffect(() => {
    const d = new TeleopDriver(teleopWsUrl(robot), s => setDrv(s));
    driver.current = d;
    return () => {
      d.disarm();
      driver.current = null;
      tracker.current.clear();
    };
  }, [robot]);

  const disarm = useCallback((why?: string) => {
    driver.current?.disarm();
    tracker.current.clear();
    setArmed(false);
    setCmd(ZERO);
    setTick(t => t + 1);
    if (why) setNote(`Disarmed: ${why}`);
  }, []);
  const doArm = () => {
    if (!driver.current) return;
    driver.current.arm();
    setArmed(true);
    setNote(null);
  };
  const toggleArm = () => {
    if (armed) return disarm();
    if (armConfirmed()) doArm();
    else setConfirming(true);
  };

  useEffect(() => {
    if (!armed) return;
    const vis = () => document.visibilityState !== "visible" && disarm("page hidden");
    const hide = () => disarm("page closed");
    const blur = () => disarm("browser lost focus");
    document.addEventListener("visibilitychange", vis);
    window.addEventListener("pagehide", hide);
    window.addEventListener("blur", blur);
    return () => {
      document.removeEventListener("visibilitychange", vis);
      window.removeEventListener("pagehide", hide);
      window.removeEventListener("blur", blur);
    };
  }, [armed, disarm]);

  // the page under the view must not scroll or bounce
  useEffect(() => {
    document.documentElement.classList.add("drive-full-open");
    return () => document.documentElement.classList.remove("drive-full-open");
  }, []);

  // map view: the viewer's 3D canvas alone fills the screen under the sticks
  useEffect(() => {
    if (view.kind !== "map") return;
    document.documentElement.classList.add("drive-full-map");
    return () => document.documentElement.classList.remove("drive-full-map");
  }, [view.kind]);

  // keep the screen on while the view is open (secure contexts only)
  useEffect(() => {
    let lock: {release: () => Promise<void>} | null = null;
    const nav = navigator as Navigator & {wakeLock?: {request: (t: "screen") => Promise<{release: () => Promise<void>}>}};
    nav.wakeLock
      ?.request("screen")
      .then(l => (lock = l))
      .catch(() => undefined);
    return () => void lock?.release().catch(() => undefined);
  }, []);

  const recompute = () => {
    const t = armed ? tracker.current.twist(speedRef.current) : ZERO;
    driver.current?.command(t);
    setCmd(t);
    setTick(n => n + 1);
  };
  useEffect(() => {
    if (armed) recompute();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [speed]);

  // the stick area: native non-passive touch listeners (iOS) + pointer events
  const zoneRef = useRef<HTMLDivElement>(null);
  useEffect(() => (zoneRef.current ? captureTouches(zoneRef.current) : undefined), []);
  const [size, setSize] = useState({w: window.innerWidth, h: window.innerHeight});
  useEffect(() => {
    const el = zoneRef.current;
    if (!el || typeof ResizeObserver === "undefined") return;
    const ro = new ResizeObserver(() => setSize({w: el.clientWidth, h: el.clientHeight}));
    ro.observe(el);
    return () => ro.disconnect();
  }, [view.kind]);
  const radius = stickRadius(size.w, size.h);
  tracker.current.radius = radius;
  const local = (e: ReactPointerEvent) => {
    const r = zoneRef.current!.getBoundingClientRect();
    return {x: e.clientX - r.left, y: e.clientY - r.top, w: r.width};
  };
  const onDown = (e: ReactPointerEvent) => {
    if (!armed) {
      setNote("Tap Drive to arm the sticks");
      return;
    }
    const p = local(e);
    if (tracker.current.down(e.pointerId, p.x, p.y, p.w) === null) return;
    e.currentTarget.setPointerCapture?.(e.pointerId);
    recompute();
  };
  const onMove = (e: ReactPointerEvent) => {
    const p = local(e);
    if (tracker.current.move(e.pointerId, p.x, p.y)) recompute();
  };
  const onUp = (e: ReactPointerEvent) => {
    if (tracker.current.up(e.pointerId)) recompute();
  };

  const post = async (c: Cmd) => {
    if (c === "stop") disarm();
    setBusy(c);
    try {
      const r = await postJSON<{ok: boolean; message?: string; error?: string}>(`api/robots/${encodeURIComponent(robot)}/teleop/${c}`);
      setNote(r.message || r.error || `${c}: sent`);
    } catch (e) {
      setNote(`${c}: ${errorText(e)}`);
    } finally {
      setBusy(null);
    }
  };

  const exit = () => {
    disarm();
    exitPageFullscreen();
    onExit();
  };

  const st = tv?.state ?? null;
  const blocked = st && !st.allowed ? st.interlock : "";
  const rosDown = tv?.ros === false;
  const stick = (id: StickId) => {
    const g = tracker.current.get(id);
    // resting place: lower corners
    const rest = {x: id === "move" ? size.w * 0.2 : size.w * 0.8, y: size.h - radius * 1.5 - 40};
    const o = g?.origin ?? rest;
    const v = g?.value ?? {x: 0, y: 0};
    return (
      <div
        key={id}
        className={`dfs-stick ${id} ${g ? "held" : ""}`}
        style={{left: o.x - radius, top: o.y - radius, width: radius * 2, height: radius * 2}}
        aria-hidden="true"
      >
        <div className="dfs-stick-ring" />
        <div className="dfs-stick-knob" style={{transform: `translate(${v.x * radius}px, ${-v.y * radius}px)`}} />
        <span className="dfs-stick-label">{id === "move" ? "move" : "turn"}</span>
      </div>
    );
  };

  const picture = () => {
    if (view.kind === "map") return null;
    if (view.kind === "all")
      return media === "video" ? (
        <div className="dfs-grid-video">
          <GridVideo robot={robot} cams={cams ?? []} onOpen={c => setView({kind: "cam", cam: c})} />
        </div>
      ) : (
        <div className="dfs-grid">
          {(cams ?? []).map(c => (
            <StillTile key={c} robot={robot} cam={c} />
          ))}
        </div>
      );
    return media === "video" ? <VideoView key={`${robot}/${view.cam}`} robot={robot} cam={view.cam} bare initialFps={10} /> : <StillPicture key={`${robot}/${view.cam}`} robot={robot} cam={view.cam} />;
  };

  const moving = armed && !isZero(cmd);
  return (
    <div className={`drive-full ${armed ? "armed" : ""} view-${view.kind}`} role="dialog" aria-label={`Drive ${robot} full screen`} data-testid="drive-full">
      <div className="dfs-picture">{picture()}</div>

      <div ref={zoneRef} className="dfs-zone" onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp} onLostPointerCapture={onUp} data-testid="drive-full-zone">
        {stick("move")}
        {stick("turn")}
        {!armed && <div className="dfs-zone-hint">Left: move · Right: turn{rosDown ? " · ROS is down on the console" : ""}</div>}
      </div>

      <div className="dfs-top">
        <button className="dfs-btn dfs-exit" onClick={exit} aria-label="Exit full-screen drive" title="Exit (disarms)">
          ✕
        </button>
        <button className="dfs-btn dfs-view" onClick={() => setMenu(m => !m)} aria-expanded={menu} aria-label="Switch view" data-testid="drive-full-view">
          {viewLabel(view)} ▾
        </button>
        <span className="dfs-robot mono">{robot}</span>
        <span className="grow" />
        <button
          className={`dfs-btn dfs-arm ${armed ? "on" : ""}`}
          onClick={toggleArm}
          disabled={rosDown}
          aria-pressed={armed}
          data-testid="drive-full-arm"
        >
          {armed ? "ARMED" : "Drive"}
        </button>
        <button className="dfs-btn dfs-stop" onClick={() => void post("stop")} disabled={busy === "stop"} title="Halt now: ends teleop and cancels the running plan" data-testid="drive-full-stop">
          {busy === "stop" ? <Spinner /> : "STOP"}
        </button>
      </div>

      {menu && (
        <div className="dfs-menu" role="menu" aria-label="Views">
          {(cams ?? []).map(c => (
            <button key={c} role="menuitemradio" aria-checked={view.kind === "cam" && view.cam === c} className={view.kind === "cam" && view.cam === c ? "on" : ""} onClick={() => setView({kind: "cam", cam: c})}>
              {c}
            </button>
          ))}
          {(cams?.length ?? 0) > 1 && (
            <button role="menuitemradio" aria-checked={view.kind === "all"} className={view.kind === "all" ? "on" : ""} onClick={() => setView({kind: "all"})}>
              All cameras
            </button>
          )}
          <button role="menuitemradio" aria-checked={view.kind === "map"} className={view.kind === "map" ? "on" : ""} onClick={() => setView({kind: "map"})}>
            Map
          </button>
          <div className="dfs-menu-row">
            <div className="segmented" role="group" aria-label="Picture">
              <button className={media === "video" ? "on" : ""} onClick={() => setMedia("video")}>
                Video
              </button>
              <button className={media === "still" ? "on" : ""} onClick={() => setMedia("still")} title="Still frames: lighter on the radio">
                Stills
              </button>
            </div>
          </div>
          <div className="dfs-menu-row">
            <button className="btn btn-sm" disabled={!!busy} onClick={() => void post("stand")}>
              {busy === "stand" && <Spinner />} Stand
            </button>
            <button className="btn btn-sm" disabled={!!busy} onClick={() => void post("sit")}>
              {busy === "sit" && <Spinner />} Sit
            </button>
          </div>
        </div>
      )}

      <div className="dfs-bottom">
        <label className="dfs-speed">
          <span className="mono">{(CAPS.vx * speed).toFixed(1)} m/s</span>
          <input type="range" min={0.1} max={1} step={0.05} value={speed} onChange={e => setSpeed(Number(e.target.value))} aria-label="Speed" />
        </label>
        <span className={`dfs-status mono ${moving ? "moving" : ""}`}>
          {cmd.vx.toFixed(2)} {cmd.vy.toFixed(2)} {cmd.wz.toFixed(2)}
          {drv.rttMs != null ? ` · ${drv.rttMs.toFixed(0)} ms` : ""}
          {armed && !drv.connected ? " · reconnecting" : ""}
        </span>
      </div>
      {(blocked || (armed && drv.error) || note) && <div className={`dfs-note ${blocked || drv.error ? "err" : ""}`}>{blocked ? `Executor refuses teleop: ${blocked}` : armed && drv.error ? drv.error : note}</div>}

      {confirming && (
        <Modal
          title={`Drive ${robot}?`}
          tone="err"
          onClose={() => setConfirming(false)}
          footer={
            <>
              <button className="btn" onClick={() => setConfirming(false)}>
                Cancel
              </button>
              <button
                className="btn btn-danger"
                data-testid="drive-full-arm-yes"
                onClick={() => {
                  setConfirming(false);
                  setArmConfirmed();
                  doArm();
                }}
              >
                Arm driving
              </button>
            </>
          }
        >
          <p className="confirm-msg">
            The sticks will move {robot} (up to {CAPS.vx} m/s). Left stick: forward/back and strafe; right stick: turn. Lifting your fingers stops it. Keep the robot in sight and the E-stop in reach. Leaving the page or this view disarms. The executor refuses while a plan
            is executing: pause it first. You are asked once per browser session.
          </p>
        </Modal>
      )}
    </div>
  );
}
