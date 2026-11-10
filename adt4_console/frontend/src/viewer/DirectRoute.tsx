// "Send route directly": a previewed route goes straight onto the robot
// executor's input (POST api/robots/<robot>/path/direct -> compiled_plan_out),
// bypassing omniplanner. Each send needs an explicit confirmation.
import {useState} from "react";
import {errorText, postJSON} from "../lib/api";

export interface DirectPathResult {
  ok?: boolean;
  topic?: string;
  waypoints?: number;
  length_m?: number;
  lease?: Record<string, {ok: boolean; message?: string}> | null;
  warning?: string | null;
}

/** What the operator confirms before a direct send. */
export function directRouteWarning(robot: string): string {
  return `Sends this exact path to ${robot}'s executor. No plan repair, no constraints; a later omniplanner plan replaces it.`;
}

/** The send result in words. */
export function directRouteResultText(robot: string, r: DirectPathResult): string {
  const parts = [`Sent ${r.waypoints ?? "?"} waypoints${r.length_m != null ? ` (${r.length_m} m)` : ""} to ${robot}'s executor`];
  const lease = r.lease?.[robot];
  if (lease) parts.push(lease.ok ? "took the Spot lease" : `lease not taken: ${lease.message || "no answer"}`);
  if (r.warning) parts.push(r.warning);
  return parts.join("; ");
}

/** The routes worth sending: two or more finite points. */
export function sendableRoutes(routes: Record<string, number[][]> | null | undefined): [string, number[][]][] {
  return Object.entries(routes ?? {}).filter(
    ([, pts]) => Array.isArray(pts) && pts.filter(p => Array.isArray(p) && p.length >= 2 && Number.isFinite(p[0]) && Number.isFinite(p[1])).length >= 2,
  );
}

export function sendDirectRoute(robot: string, path: number[][], yaw?: number | null): Promise<DirectPathResult> {
  return postJSON<DirectPathResult>(`api/robots/${encodeURIComponent(robot)}/path/direct`, yaw == null ? {path} : {path, yaw});
}

type Status = {state: "confirm"} | {state: "busy"} | {state: "done"; tone: "ok" | "warn" | "err"; text: string};

/** One "Send route directly" button per robot route, each with a confirmation. */
export function DirectRouteButtons({routes, yaws}: {routes: Record<string, number[][]> | null | undefined; yaws?: Record<string, number | null | undefined>}) {
  const [st, setSt] = useState<Record<string, Status>>({});
  const list = sendableRoutes(routes);
  if (!list.length) return null;
  const put = (robot: string, s: Status | null) =>
    setSt(prev => {
      const next = {...prev};
      if (s) next[robot] = s;
      else delete next[robot];
      return next;
    });
  const send = async (robot: string, path: number[][]) => {
    put(robot, {state: "busy"});
    try {
      const r = await sendDirectRoute(robot, path, yaws?.[robot]);
      const leaseFailed = r.lease?.[robot] && !r.lease[robot].ok;
      put(robot, {state: "done", tone: r.ok === false ? "err" : r.warning || leaseFailed ? "warn" : "ok", text: directRouteResultText(robot, r)});
    } catch (e) {
      put(robot, {state: "done", tone: "err", text: `Not sent: ${errorText(e)}`});
    }
  };
  return (
    <div className="direct-route small" data-testid="direct-route">
      {list.map(([robot, path]) => {
        const s = st[robot];
        return (
          <div key={robot} className="row gap wrap">
            {s?.state === "confirm" ? (
              <>
                <span className="tone-warn" data-testid={`direct-route-warning-${robot}`}>{directRouteWarning(robot)}</span>
                <button className="btn btn-xs btn-danger" onClick={() => void send(robot, path)}>
                  Send to {robot}
                </button>
                <button className="btn btn-xs" onClick={() => put(robot, null)}>
                  Cancel
                </button>
              </>
            ) : (
              <button className="btn btn-xs" disabled={s?.state === "busy"} title={`Send this previewed route straight to ${robot}'s executor (bypasses omniplanner)`} onClick={() => put(robot, {state: "confirm"})}>
                {s?.state === "busy" ? "Sending…" : `Send route directly → ${robot}`}
              </button>
            )}
            {s?.state === "done" && (
              <span className={`tone-${s.tone}`} role="status">
                {s.text}
              </span>
            )}
          </div>
        );
      })}
    </div>
  );
}
