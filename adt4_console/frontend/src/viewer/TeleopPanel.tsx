// Teleop tab: drive a Spot through its executor (which holds the lease).
// Keys drive only while this panel has focus AND Drive is armed; arming asks
// for confirmation once per browser session and shows a red ARMED banner.
// Losing focus, hiding the page or changing the robot focus disarms. The
// console and the robot each run a 0.5 s deadman, so a dead browser or radio
// stops it.
// The panel drives the viewer's focused robot only (useFocusedRobot). With
// "all robots" in focus it drives nothing: it offers one button per robot,
// which focuses that robot; arming is still a separate, explicit step.
import {useCallback, useEffect, useLayoutEffect, useRef, useState, type KeyboardEvent as ReactKeyboardEvent, type PointerEvent as ReactPointerEvent} from "react";
import {useViewer} from "../stores/viewer";
import {useRobots} from "../stores/robots";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {
  CAPS,
  TeleopDriver,
  ZERO,
  armConfirmed,
  captureTouches,
  isDriveKey,
  joystickToTwist,
  keysToTwist,
  poseRate,
  setArmConfirmed,
  teleopWsUrl,
  yawOf,
  type DriverStatus,
  type Twist,
} from "../lib/teleop";
import type {PlanRobot, TeleopView} from "../types";
import {Badge, Empty, ErrorBox, Loading, Modal, Spinner} from "../components/ui";
import {LeaseControl} from "./LeaseControl";
import {FocusChip} from "./RobotFocusPicker";
import {useFocusedRobot} from "./useRobotFocus";
import "./teleop.css";

const POLL_MS = 3000;
const f2 = (v: number | null | undefined) => (v == null || !Number.isFinite(v) ? "–" : v.toFixed(2));
const fmtTwist = (t: Twist | null | undefined) => (t ? `${f2(t.vx)}, ${f2(t.vy)}, ${f2(t.wz)}` : "–");

type Cmd = "stand" | "sit" | "stop" | "pause" | "resume";

/** Robot-reported motion from successive live poses (C1 state, ~4 Hz). */
function useReportedRate(robot: string | null) {
  const pose = useRobots(s => (robot ? s.state[robot]?.pose : undefined));
  const at = useRobots(s => (robot ? s.stateAt[robot] : undefined));
  const prev = useRef<{x: number; y: number; yaw: number; t: number} | null>(null);
  const [rate, setRate] = useState<{v: number; wz: number} | null>(null);
  useEffect(() => {
    prev.current = null;
    setRate(null);
  }, [robot]);
  useEffect(() => {
    if (!pose || !at) return;
    const cur = {x: pose.x, y: pose.y, yaw: yawOf(pose), t: at / 1000};
    const r = poseRate(prev.current, cur);
    if (r) setRate(r);
    prev.current = cur;
  }, [pose, at]);
  return rate;
}

export function Joystick({enabled, onMove}: {enabled: boolean; onMove: (xy: {x: number; y: number} | null) => void}) {
  const ref = useRef<HTMLDivElement>(null);
  const [knob, setKnob] = useState<{x: number; y: number} | null>(null);
  const active = useRef<number | null>(null);
  const update = (e: ReactPointerEvent) => {
    const el = ref.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    const rad = r.width / 2;
    let x = (e.clientX - (r.left + rad)) / rad;
    let y = -(e.clientY - (r.top + rad)) / rad;
    const n = Math.hypot(x, y);
    if (n > 1) {
      x /= n;
      y /= n;
    }
    setKnob({x, y});
    onMove({x, y});
  };
  const end = () => {
    active.current = null;
    setKnob(null);
    onMove(null);
  };
  useEffect(() => {
    if (!enabled && active.current != null) end();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [enabled]);
  // phones: without this the touch scrolls the sheet (pointercancel ends the
  // drag at once) and its synthesized focus change disarms the panel
  useEffect(() => (ref.current ? captureTouches(ref.current) : undefined), []);
  return (
    <div
      ref={ref}
      className={`joystick ${enabled ? "" : "disabled"}`}
      aria-label="Drive joystick"
      role="application"
      onPointerDown={e => {
        if (!enabled || e.button !== 0) return;
        e.preventDefault();
        e.currentTarget.setPointerCapture?.(e.pointerId);
        active.current = e.pointerId;
        update(e);
      }}
      onPointerMove={e => active.current === e.pointerId && update(e)}
      onPointerUp={e => active.current === e.pointerId && end()}
      onPointerCancel={e => active.current === e.pointerId && end()}
      onLostPointerCapture={e => active.current === e.pointerId && end()}
    >
      <div className="joy-ring" />
      <div className="joy-knob" style={knob ? {transform: `translate(${knob.x * 50}%, ${-knob.y * 50}%)`} : undefined} />
    </div>
  );
}

export function TeleopPanel() {
  const robots = useApi<PlanRobot[]>("api/plan/robots");
  const focused = useFocusedRobot();
  const setFocus = useViewer(s => s.setFocus);
  const list = robots.data ?? [];
  // only ever the robot the operator focused; never a default pick
  const robot = focused && list.some(r => r.robot === focused) ? focused : null;
  const teleopViews = useRobots(s => s.teleop);

  const view = useRobots(s => (robot ? s.teleop[robot] : undefined));
  // initial fetch + slow poll (the hub pushes the executor's state as it comes)
  useEffect(() => {
    if (!robot) return;
    let alive = true;
    const load = () =>
      getJSON<TeleopView>(`api/robots/${encodeURIComponent(robot)}/teleop`)
        .then(v => alive && useRobots.setState(s => ({teleop: {...s.teleop, [robot]: {...s.teleop[robot], ...v, state: v.state ?? null}}})))
        .catch(() => undefined);
    void load();
    const h = window.setInterval(load, POLL_MS);
    return () => {
      alive = false;
      window.clearInterval(h);
    };
  }, [robot]);

  const rootRef = useRef<HTMLDivElement>(null);
  const driver = useRef<TeleopDriver | null>(null);
  const [drv, setDrv] = useState<DriverStatus>({connected: false, rttMs: null, acked: null, error: null, sent: 0});
  const [armed, setArmed] = useState(false);
  const [confirming, setConfirming] = useState(false);
  // starts at 0.4 m/s forward; the slider goes up to Spot's 1.6 m/s
  const [speed, setSpeed] = useState(0.25);
  const [strafe, setStrafe] = useState(false);
  const [cmd, setCmd] = useState<Twist>(ZERO);
  const [note, setNote] = useState<string | null>(null);
  const [busy, setBusy] = useState<Cmd | null>(null);
  const [result, setResult] = useState<{ok: boolean; text: string} | null>(null);
  const held = useRef(new Set<string>());
  const slow = useRef(false);
  const joy = useRef<{x: number; y: number} | null>(null);
  const speedRef = useRef(speed);
  const strafeRef = useRef(strafe);
  speedRef.current = speed;
  strafeRef.current = strafe;

  const armedRef = useRef(false);
  armedRef.current = armed;
  // one driver (velocity socket) per robot; a focus change disarms (zero +
  // release) before anything else can reach the new robot. Layout effect: the
  // cleanup runs in the same commit as the focus change, so no key handler
  // can still talk to the old driver in between.
  useLayoutEffect(() => {
    if (!robot) return;
    const d = new TeleopDriver(teleopWsUrl(robot), s => setDrv(s));
    driver.current = d;
    return () => {
      const was = armedRef.current || d.isArmed;
      d.disarm();
      driver.current = null;
      held.current.clear();
      joy.current = null;
      armedRef.current = false;
      setArmed(false);
      setCmd(ZERO);
      setDrv({connected: false, rttMs: null, acked: null, error: null, sent: 0});
      if (was) setNote(`Disarmed: robot focus changed (was driving ${robot})`);
    };
  }, [robot]);

  const recompute = useCallback(() => {
    const j = joy.current;
    const t = j ? joystickToTwist(j.x, j.y, speedRef.current, strafeRef.current) : keysToTwist(held.current, speedRef.current, slow.current);
    driver.current?.command(t);
    setCmd(t);
  }, []);
  useEffect(() => {
    if (armed) recompute();
  }, [speed, strafe, armed, recompute]);

  const disarm = useCallback((why?: string) => {
    driver.current?.disarm();
    held.current.clear();
    joy.current = null;
    slow.current = false;
    setArmed(false);
    setCmd(ZERO);
    if (why) setNote(`Disarmed: ${why}`);
  }, []);

  // the full-screen drive view takes over: one velocity stream at a time
  const driveFull = useViewer(s => s.driveFull);
  useEffect(() => {
    if (driveFull && armedRef.current) disarm("full-screen drive opened");
  }, [driveFull, disarm]);

  const doArm = () => {
    if (!driver.current || useViewer.getState().driveFull) return;
    driver.current.arm();
    setArmed(true);
    setNote(null);
    rootRef.current?.focus();
  };
  const requestArm = () => {
    if (armed) return disarm("Drive switched off");
    if (armConfirmed()) doArm();
    else setConfirming(true);
  };

  // disarm when the page is hidden or the browser window loses focus
  useEffect(() => {
    if (!armed) return;
    const vis = () => document.visibilityState !== "visible" && disarm("page hidden");
    const blur = () => disarm("browser window lost focus");
    document.addEventListener("visibilitychange", vis);
    window.addEventListener("blur", blur);
    return () => {
      document.removeEventListener("visibilitychange", vis);
      window.removeEventListener("blur", blur);
    };
  }, [armed, disarm]);

  const onKeyDown = (e: ReactKeyboardEvent) => {
    if (!armed) return;
    if (e.code === "Escape") {
      e.stopPropagation();
      return disarm("Esc");
    }
    if (e.key === "Shift") {
      slow.current = true;
      recompute();
      return;
    }
    if (!isDriveKey(e.code) || e.ctrlKey || e.metaKey || e.altKey) return;
    e.preventDefault();
    e.stopPropagation(); // Q/E must not rotate the map, Space must not scroll
    if (e.code === "Space") {
      held.current.clear();
      joy.current = null;
      setNote("Stopped (Space)");
    } else {
      held.current.add(e.code);
      slow.current = e.shiftKey;
    }
    recompute();
  };
  const onKeyUp = (e: ReactKeyboardEvent) => {
    if (!armed) return;
    if (e.key === "Shift") slow.current = false;
    else if (isDriveKey(e.code)) {
      e.preventDefault();
      e.stopPropagation();
      held.current.delete(e.code);
    } else return;
    recompute();
  };

  const post = async (c: Cmd) => {
    if (!robot) return;
    if (c === "stop") disarm();
    setBusy(c);
    setResult(null);
    try {
      const r = await postJSON<{ok: boolean; message?: string; topic?: string; delivered?: boolean; error?: string}>(`api/robots/${encodeURIComponent(robot)}/teleop/${c}`);
      const text = r.message || r.error || (r.delivered === false ? `${c}: sent, not acknowledged by the robot yet` : `${c}: sent`);
      setResult({ok: r.ok !== false && !r.error, text});
    } catch (e) {
      setResult({ok: false, text: `${c}: ${errorText(e)}`});
    } finally {
      setBusy(null);
    }
  };

  const reported = useReportedRate(robot);
  if (robots.loading && !robots.data) return <Loading label="Loading robots" />;
  if (robots.error && !robots.data) return <ErrorBox error={robots.error} onRetry={robots.reload} />;
  if (!list.length) return <Empty title="No robots">The console drives robots given with --robots.</Empty>;
  if (!focused)
    return (
      <div className="teleop-panel teleop-pick" aria-label="Teleop">
        <div className="teleop-pick-title">Pick a robot to drive</div>
        <div className="muted small">Teleop drives the focused robot only. Picking one focuses the whole viewer on it (camera, map, plan); arming Drive is a separate step.</div>
        {note && <div className="small muted">{note}</div>}
        <div className="teleop-pick-list">
          {list.map(r => {
            const st = teleopViews[r.robot]?.state;
            return (
              <button key={r.robot} className="btn teleop-pick-btn" onClick={() => setFocus(r.robot)} data-testid={`teleop-pick-${r.robot}`}>
                <span className="teleop-pick-name">{r.robot}</span>
                <span className="small muted">{st ? (st.allowed ? "executor ready" : `executor: ${st.interlock || "refuses"}`) : "no teleop state yet"}</span>
              </button>
            );
          })}
        </div>
      </div>
    );
  if (!robot)
    return (
      <Empty title={`${focused} has no executor here`}>
        The console drives robots given with --robots ({list.map(r => r.robot).join(", ")}).{" "}
        <button className="btn btn-xs" onClick={() => setFocus("all")}>
          All robots
        </button>
      </Empty>
    );

  const st = view?.state ?? null;
  const rosDown = view?.ros === false;
  const linkRtt = view?.link?.rtt;
  const standing = st?.standing == null ? "unknown" : st.standing ? "standing" : "sitting";
  const exec = st ? (st.plan_executing ? "plan executing" : st.plan_paused ? "plan paused" : st.executor || "idle") : "–";
  const blocked = st && !st.allowed ? st.interlock : "";

  return (
    <div
      ref={rootRef}
      className={`teleop-panel ${armed ? "armed" : ""}`}
      tabIndex={0}
      onKeyDown={onKeyDown}
      onKeyUp={onKeyUp}
      onBlur={e => {
        if (armed && !rootRef.current?.contains(e.relatedTarget as Node | null)) disarm("panel lost focus");
      }}
      aria-label="Teleop"
    >
      {armed ? (
        <div className="teleop-armed" role="alert">
          <strong>ARMED</strong> · driving {robot} · W/S A/D Q/E · Shift slow · Space stop · Esc disarm
        </div>
      ) : (
        <div className="teleop-hint muted small">Drive arms the keys and joystick. They work only while this panel has focus (clicking the map or camera disarms); the robot stops within 0.5 s when they stop.</div>
      )}

      <div className="row gap between">
        <FocusChip robot={robot} />
        <LeaseControl robot={robot} />
      </div>

      <div className="teleop-buttons">
        <button className="btn teleop-stop" disabled={!robot || busy === "stop"} onClick={() => void post("stop")} title="Halt the robot now: ends teleop and cancels the running plan">
          {busy === "stop" && <Spinner />} STOP
        </button>
        <button className="btn teleop-big" disabled={!robot || !!busy} onClick={() => void post("stand")}>
          {busy === "stand" && <Spinner />} Stand
        </button>
        <button className="btn teleop-big" disabled={!robot || !!busy} onClick={() => void post("sit")}>
          {busy === "sit" && <Spinner />} Sit
        </button>
        <button className="btn teleop-big" disabled={!robot || !!busy} onClick={() => void post("pause")} title="Pause the running plan (teleop is accepted while it is paused)">
          {busy === "pause" && <Spinner />} Pause
        </button>
        <button className="btn teleop-big" disabled={!robot || !!busy} onClick={() => void post("resume")} title="Resume the plan; it takes the robot back from teleop">
          {busy === "resume" && <Spinner />} Resume
        </button>
      </div>
      {result && (
        <div className={`inline-msg ${result.ok ? "tone-ok" : "tone-err"}`} role="status">
          {result.text}
        </div>
      )}

      <div className="row gap teleop-drive-row">
        <button
          className={`btn teleop-drive ${armed ? "btn-danger" : "btn-primary"}`}
          aria-pressed={armed}
          disabled={!robot || rosDown}
          onClick={requestArm}
          data-testid="teleop-drive"
          title={rosDown ? "ROS is not available on the console" : armed ? "Disarm" : "Arm keyboard / joystick driving"}
        >
          {armed ? "Disarm" : "Drive"}
        </button>
        <label className="teleop-speed">
          <span className="small muted">
            Speed {Math.round(speed * 100)} % · {(CAPS.vx * speed).toFixed(2)} m/s
          </span>
          <input type="range" min={0.1} max={1} step={0.05} value={speed} onChange={e => setSpeed(Number(e.target.value))} aria-label="Speed" />
        </label>
      </div>
      {note && <div className="small muted">{note}</div>}
      {drv.error && armed && <div className="inline-msg tone-err">{drv.error}</div>}
      {blocked && <div className="inline-msg tone-warn">Executor refuses teleop: {blocked}</div>}

      <div className="teleop-joy-row">
        <Joystick
          enabled={armed}
          onMove={xy => {
            joy.current = xy;
            recompute();
          }}
        />
        <label className="row gap-sm small">
          <input type="checkbox" checked={strafe} onChange={e => setStrafe(e.target.checked)} />
          Joystick strafes (instead of turning)
        </label>
      </div>

      <div className="kv-grid teleop-readouts">
        <div className="kv">
          <span className="kv-k">Commanded</span>
          <span className="kv-v mono" title="vx, vy (m/s), wz (rad/s) this panel sends">
            {fmtTwist(cmd)}
          </span>
        </div>
        <div className="kv">
          <span className="kv-k">Console ack</span>
          <span className="kv-v mono">{fmtTwist(drv.acked)}</span>
        </div>
        <div className="kv">
          <span className="kv-k">Robot applies</span>
          <span className="kv-v mono" title="The executor's last accepted command (after its caps)">
            {st ? fmtTwist(st.commanded) : "–"}
          </span>
        </div>
        <div className="kv">
          <span className="kv-k">Reported</span>
          <span className="kv-v mono" title="From the robot's live pose">
            {reported ? `${f2(reported.v)} m/s, ${f2(reported.wz)} rad/s` : "–"}
          </span>
        </div>
        <div className="kv">
          <span className="kv-k">Teleop</span>
          <span className="kv-v">
            <Badge tone={st?.active ? "warn" : "off"}>{st ? (st.active ? "active" : "idle") : "no state"}</Badge>
          </span>
        </div>
        <div className="kv">
          <span className="kv-k">Posture</span>
          <span className="kv-v">{st?.busy ? `${st.busy}…` : standing}</span>
        </div>
        <div className="kv">
          <span className="kv-k">Executor</span>
          <span className="kv-v">{exec}</span>
        </div>
        <div className="kv">
          <span className="kv-k">Link</span>
          <span className="kv-v mono" title="browser↔console round trip · console↔robot (fleetlink) round trip">
            {drv.rttMs == null ? "–" : `${drv.rttMs.toFixed(0)} ms`}
            {linkRtt != null ? ` · radio ${(linkRtt * 1000).toFixed(0)} ms` : ""}
            {armed && !drv.connected ? " · reconnecting" : ""}
          </span>
        </div>
        {st?.last_event && (
          <div className="kv">
            <span className="kv-k">Last event</span>
            <span className="kv-v small">{st.last_event}</span>
          </div>
        )}
        {st?.last_reject && (
          <div className="kv">
            <span className="kv-k">Rejected</span>
            <span className="kv-v small tone-warn">{st.last_reject}</span>
          </div>
        )}
      </div>
      {!st && robot && <div className="small muted">No teleop state from {robot}'s executor (not running, or built without teleop).</div>}

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
                data-testid="teleop-arm-yes"
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
            Keys W/S A/D Q/E and the joystick will move {robot} (up to {CAPS.vx} m/s). Keep the robot in sight and the E-stop in reach. Driving works only while this panel has focus; clicking elsewhere (the map, the camera) or changing the robot focus disarms.
            The executor refuses while a plan is executing: pause it first. You are asked once per browser session.
          </p>
        </Modal>
      )}
    </div>
  );
}
