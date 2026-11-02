// Active window change detection on one robot (API.md §19): the detector's
// latched state and Start / Stop / Reset.  The relocalize_awcd configs start it
// paused; Start is only offered once the robot is relocalized against the prior
// map (the same verdict as the legend's AlignStatus) — "Start anyway" overrides.
import {useEffect, useState} from "react";
import {ApiError, errorText, getJSON, postJSON} from "../lib/api";
import {relocStatus} from "../lib/frameAlign";
import {frameAlignOf, useRobots} from "../stores/robots";
import {ws} from "../ws";
import {useDistToPrior} from "./RobotLegend";
import {Badge, type Tone} from "../components/ui";

export interface CdState {
  robot: string;
  state: "running" | "paused" | "reset" | string | null;
  t: number | null;
  relocalized?: {ok: boolean; reason: string};
}

const STATE_TONE: Record<string, Tone> = {running: "ok", paused: "warn", reset: "off"};

/** Whether Start is offered: the legend's relocalization verdict must be "ok". */
export function canStart(tone: string): boolean {
  return tone === "ok";
}

export function ChangeDetectionControl({robot}: {robot: string}) {
  const fa = useRobots(s => frameAlignOf(robot, s));
  const reloc = useRobots(s => s.state[robot]?.reloc ?? s.link[robot]?.reloc);
  const pose = useRobots(s => s.state[robot]?.pose);
  const at = useRobots(s => s.stateAt[robot]);
  const dist = useDistToPrior(robot, pose, fa);
  const rs = relocStatus(fa, reloc, dist, at ? (Date.now() - at) / 1000 : 0);
  const [st, setSt] = useState<CdState | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [msg, setMsg] = useState<{tone: Tone; text: string} | null>(null);
  const [confirmOverride, setConfirmOverride] = useState(false);

  useEffect(() => {
    let live = true;
    getJSON<CdState>(`api/robots/${encodeURIComponent(robot)}/change_detection`).then(d => live && setSt(d), () => undefined);
    const off = ws.on(`robot/${robot}/change_detection`, d => {
      if (d && typeof d === "object") setSt(prev => ({...prev, ...(d as CdState)}));
    });
    return () => {
      live = false;
      off();
    };
  }, [robot]);

  const ready = canStart(rs.tone);
  const send = async (cmd: "start" | "stop" | "reset" | "reset_start", override = false) => {
    setBusy(cmd);
    setMsg(null);
    try {
      const r = await postJSON<CdState & {ok: boolean; results: {cmd: string; message?: string; error?: string}[]}>(
        `api/robots/${encodeURIComponent(robot)}/change_detection/${cmd}`, {override});
      setSt(r);
      const last = r.results[r.results.length - 1];
      setMsg(r.ok ? {tone: "ok", text: last?.message || "done"} : {tone: "err", text: last?.error || last?.message || "failed"});
    } catch (e) {
      setMsg({tone: "err", text: e instanceof ApiError && e.status === 409 ? `${robot} is not relocalized — use "Start anyway" to override` : errorText(e)});
    } finally {
      setBusy(null);
      setConfirmOverride(false);
    }
  };

  const state = st?.state ?? null;
  const running = state === "running";
  return (
    <div className="cd-control small" data-testid={`cd-${robot}`}>
      <div className="row gap-sm wrap">
        <span className="muted">change detection</span>
        <Badge tone={state ? STATE_TONE[state] ?? "off" : "off"}>{state ?? "not reported"}</Badge>
        {!running && (
          <button className="btn btn-xs btn-primary" disabled={!ready || !!busy} onClick={() => send("reset_start")}
            title={ready ? "Reset the detector and start it (robot relocalized)" : `Not relocalized: ${rs.text}`} data-testid={`cd-start-${robot}`}>
            {busy === "reset_start" ? "…" : "Start"}
          </button>
        )}
        {running && (
          <button className="btn btn-xs" disabled={!!busy} onClick={() => send("stop")} data-testid={`cd-stop-${robot}`}>
            {busy === "stop" ? "…" : "Stop"}
          </button>
        )}
        <button className="btn btn-xs" disabled={!!busy} onClick={() => send("reset")} title="Drop the detector's accumulated changes">
          {busy === "reset" ? "…" : "Reset"}
        </button>
      </div>
      {!running && !ready && (
        <div className="row gap-sm wrap cd-gate" data-testid={`cd-gate-${robot}`}>
          <span className="tone-warn">Start waits for relocalization: {rs.text || "no alignment reported"}</span>
          {!confirmOverride ? (
            <button className="btn btn-xs btn-danger-outline" disabled={!!busy} onClick={() => setConfirmOverride(true)}>Start anyway…</button>
          ) : (
            <>
              <span>Changes detected before relocalization are placed in the wrong frame.</span>
              <button className="btn btn-xs btn-danger" onClick={() => send("reset_start", true)} data-testid={`cd-override-${robot}`}>Start anyway</button>
              <button className="btn btn-xs" onClick={() => setConfirmOverride(false)}>Cancel</button>
            </>
          )}
        </div>
      )}
      {msg && <div className={`tone-${msg.tone}`}>{msg.text}</div>}
    </div>
  );
}
