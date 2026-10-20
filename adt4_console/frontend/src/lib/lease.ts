// Spot lease state for the UI, from the executor's runtime guards
// (RobotPlan.lease over ROS, or fleetlink's live state guards), and omniplanner's
// lease_not_owned dispatch refusals.
import type {LeaseInfo, PlanStatus} from "../types";

/** Older than this, a lease view is not trusted (guards are published ~1 Hz and
 *  the console re-sends an unchanged view every 5 s). */
export const LEASE_STALE_S = 15;

/** A Spot lease client name, shortened for display. */
export function shortOwner(owner: string | null | undefined): string {
  const o = (owner ?? "").trim();
  if (!o) return "nobody";
  if (/^bosdyn\.android\.spotapp\b/i.test(o) || /tablet/i.test(o)) return "tablet";
  if (/^understanding/i.test(o)) return "planner";
  // e.g. "bosdyn.choreographer host" -> "choreographer"
  const first = o.split(/\s+/)[0];
  const parts = first.split(".");
  return parts[parts.length - 1] || first;
}

export type LeaseTone = "ok" | "warn" | "off";

export interface LeaseView {
  tone: LeaseTone;
  label: string;
  title: string;
}

/** Indicator for a lease snapshot; ``now`` and ``lease.t`` in console seconds. */
export function leaseView(lease: LeaseInfo | null | undefined, now?: number): LeaseView {
  if (!lease || !lease.known) return {tone: "off", label: "lease: unknown", title: "No lease information from the executor"};
  if (now !== undefined && lease.t !== undefined && now - lease.t > LEASE_STALE_S)
    return {tone: "off", label: "lease: unknown", title: `Last lease report ${Math.round(now - lease.t)} s ago`};
  if (lease.owned) return {tone: "ok", label: "lease: ours", title: lease.owner ? `Held by ${lease.owner}` : "The executor holds the lease"};
  return {tone: "warn", label: `lease: ${shortOwner(lease.owner)}`, title: lease.owner ? `Held by ${lease.owner}` : "Not held by the executor"};
}

/** Lease info from fleetlink's live-state guards (RobotState.guards). */
export function leaseFromGuards(g: Record<string, unknown> | undefined | null): LeaseInfo | null {
  if (!g || g.lease_known === undefined) return null;
  return {known: Boolean(g.lease_known), owned: Boolean(g.lease_owned), owner: String(g.lease_owner ?? ""), t: typeof g.t === "number" ? g.t : undefined};
}

/** The holder named by a lease refusal: "" when the refusal does not name one,
 *  null when ``reason`` is not a lease refusal. */
export function leaseRefusalOwner(reason: string | null | undefined): string | null {
  const m = /lease_not_owned(?:\(held by ([^)]*)\))?/.exec(reason ?? "");
  return m ? (m[1] ?? "").trim() : null;
}

/** Whether a plan status refused ``robot`` for its lease; returns the holder
 *  ("" if unnamed) or null. Uses the structured ``refusals`` when present, else
 *  the error text ("...: hamilton: lease_not_owned(held by ...)"). */
export function leaseRefusal(st: PlanStatus | null | undefined, robot: string): string | null {
  if (!st) return null;
  if (st.refusals) return leaseRefusalOwner(st.refusals[robot]);
  if (st.ok) return null;
  const err = st.error ?? "";
  const scoped = new RegExp(`(?:^|[:;]\\s*)${robot.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}:\\s*(lease_not_owned(?:\\([^)]*\\))?)`).exec(err);
  if (scoped) return leaseRefusalOwner(scoped[1]);
  // an unscoped refusal for this robot's own goal
  return st.robot === robot ? leaseRefusalOwner(err) : null;
}

/** The latest plan status for ``robot`` if it is a lease refusal not older than
 *  ``maxAgeS`` (console seconds) and newer than ``since`` (e.g. a lease take). */
export function recentLeaseRefusal(
  status: PlanStatus[] | undefined,
  robot: string,
  now: number,
  {maxAgeS = 600, since = 0}: {maxAgeS?: number; since?: number} = {},
): {owner: string; t: number} | null {
  const last = status?.length ? status[status.length - 1] : undefined;
  if (!last || now - last.t > maxAgeS || last.t <= since) return null;
  const owner = leaseRefusal(last, robot);
  return owner === null ? null : {owner, t: last.t};
}
