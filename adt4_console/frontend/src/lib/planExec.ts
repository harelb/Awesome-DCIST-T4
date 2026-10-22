// What a robot is doing with its plan, from what the console already has: the
// plan, the executor's per-step results, the planner's dispatch refusals, the
// executor's teleop state (plan_executing / plan_paused), the lease and the
// fleetlink link. Shown on the robot's legend chip and in the Plan panel.
// Phoenix (Husky) robots have no Spot executor: their status is the agent's
// manual-mode guard and its distance-to-goal report (phoenix_progress).
import type {LeaseInfo, PhoenixProgress, RobotPlan, TeleopState} from "../types";
import {currentActionIndex} from "../viewer/planUtil";
import {isSpot, progressMatches} from "./platform";

export interface PlanExec {
  /** none: no plan; ok: running or done; warn: needs a look; err: not moving */
  tone: "ok" | "warn" | "err" | "none";
  state: "none" | "refused" | "done" | "failed" | "offline" | "paused" | "lease_lost" | "idle" | "executing" | "stuck" | "manual";
  text: string;
  title: string;
}

/** On one step this long without a result: the executor is likely retrying. */
export const STUCK_S = 90;
/** The executor reports no plan running for this long while steps remain. */
export const IDLE_S = 8;

const FAIL = /fail|abort|error|reject|cancel/i;
const DONE = /succe|complete|done|finished/i;

export function fmtSince(s: number): string {
  s = Math.max(0, s);
  return s < 90 ? `${Math.round(s)} s` : s < 5400 ? `${Math.round(s / 60)} min` : `${(s / 3600).toFixed(1)} h`;
}

/** "lease_not_owned(held by bosdyn.android.spotapp 964e...)" -> "the tablet holds the lease". */
export function refusalText(reason: string): string {
  if (/spotapp|android|tablet/i.test(reason)) return "the tablet holds the lease";
  if (/lease_not_owned/i.test(reason)) return "the executor does not hold the lease";
  return reason;
}

export function ownerText(owner: string): string {
  return /spotapp|android/i.test(owner) ? "the tablet" : owner || "another client";
}

export function planExecStatus(
  robot: string,
  plan: RobotPlan | undefined,
  opts: {
    connected?: boolean | null;
    teleop?: TeleopState | null;
    lease?: LeaseInfo | null;
    now?: number;
    /** robot platform (lib/platform.ts); absent = spot */
    platform?: string | null;
    /** phoenix: agent guards' manual_mode and its phoenix_progress */
    manual?: boolean | null;
    progress?: PhoenixProgress | null;
  } = {},
): PlanExec {
  const now = opts.now ?? Date.now() / 1000;
  const p = plan?.plan;
  const spot = isSpot(opts.platform);
  // a goal whose plan the runtime guards refused: the robot never got it
  const refused = [...(plan?.status ?? [])].reverse().find(s => s.refusals && robot in s.refusals);
  if (refused && (!p || refused.t > p.t)) {
    const why = refusalText(refused.refusals![robot]);
    const then = spot ? "take the lease back on the robot, then send the goal again" : "fix the cause, then send the goal again";
    return {tone: "err", state: "refused", text: `plan not sent — ${why}`, title: `the planner refused to dispatch ${fmtSince(now - refused.t)} ago: ${refused.refusals![robot]}\n${then}`};
  }
  if (!p || !p.actions.length) return {tone: "none", state: "none", text: "", title: ""};
  if (!spot) return otherPlatformStatus(p, opts);
  const n = p.actions.length;
  const res = (plan!.results ?? []).filter(r => r.plan_id === p.plan_id);
  const last = res[res.length - 1];
  const cur = Math.min(currentActionIndex(plan), n);
  const step = `step ${Math.min(cur + 1, n)}/${n}`;
  const lastT = last?.t ?? p.t;
  if (cur >= n || (last && DONE.test(last.status ?? "") && (last.action_index ?? 0) >= n - 1)) {
    return {tone: "ok", state: "done", text: `plan done · ${n} steps · ${fmtSince(now - lastT)} ago`, title: `all ${n} steps succeeded; the last ${fmtSince(now - lastT)} ago`};
  }
  if (last && FAIL.test(last.status ?? "")) {
    return {tone: "err", state: "failed", text: `plan failed at step ${(last.action_index ?? 0) + 1}/${n}${last.detail ? ` — ${last.detail}` : ""}`, title: `${last.action_type ?? "step"} ${last.status}${last.detail ? `: ${last.detail}` : ""}, ${fmtSince(now - lastT)} ago`};
  }
  if (opts.connected === false) {
    return {tone: "warn", state: "offline", text: `robot offline · was on ${step}`, title: "no link to the robot: its executor may still be running the plan"};
  }
  if (opts.teleop?.plan_paused) return {tone: "warn", state: "paused", text: `paused · ${step}`, title: "the plan is paused: Resume continues it"};
  if (opts.lease?.known && !opts.lease.owned) {
    const who = ownerText(opts.lease.owner);
    return {tone: "err", state: "lease_lost", text: `stopped · ${who} holds the lease · ${step}`, title: `the executor lost the lease to ${opts.lease.owner || "another client"}: the robot will not move until the lease is taken back`};
  }
  const onStep = now - lastT;
  if (opts.teleop && !opts.teleop.plan_executing && onStep > IDLE_S) {
    return {tone: "warn", state: "idle", text: `not executing · stopped at ${step}`, title: "the executor reports no plan running, but steps remain (stopped, or the plan was replaced)"};
  }
  if (onStep > STUCK_S) {
    return {tone: "warn", state: "stuck", text: `still trying ${step} · ${fmtSince(onStep)}`, title: `no step finished for ${fmtSince(onStep)}: the executor is likely retrying (it gives up after 3 attempts)`};
  }
  return {tone: "ok", state: "executing", text: `executing ${step} · ${fmtSince(onStep)}`, title: `${res.length} of ${n} steps reported; this one started ${fmtSince(onStep)} ago`};
}

/** Phoenix and other non-Spot platforms: no per-step executor results, so the
 *  plan is "done" when the agent reports the pose within 1 m of the path end. */
function otherPlatformStatus(
  p: NonNullable<RobotPlan["plan"]>,
  opts: {connected?: boolean | null; manual?: boolean | null; progress?: PhoenixProgress | null},
): PlanExec {
  if (opts.connected === false) return {tone: "warn", state: "offline", text: "robot offline", title: "no link to the robot: it may still be driving the plan"};
  if (opts.manual) {
    return {tone: "warn", state: "manual", text: "manual mode — not executing", title: "the robot is in manual (RC) mode and will not execute plans: Resume (auto) hands it back to the planner"};
  }
  const pr = progressMatches(opts.progress, p.plan_id) ? opts.progress! : null;
  if (pr?.done) return {tone: "ok", state: "done", text: "plan done", title: "the robot is within 1 m of the end of the plan"};
  const d = pr?.dist_to_goal_m;
  if (typeof d === "number" && Number.isFinite(d)) {
    return {tone: "ok", state: "executing", text: `executing (${d < 10 ? d.toFixed(1) : Math.round(d)} m to goal)`, title: "straight-line distance from the robot to the end of the plan"};
  }
  return {tone: "ok", state: "executing", text: "executing", title: "no progress report for this plan yet"};
}
