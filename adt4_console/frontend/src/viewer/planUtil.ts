import type {RobotPlan} from "../types";

// Per-robot colours come from the topology (GET /api/network/colors) so the
// console matches RViz; SOURCE_TINTS stays for graph sources only.
export {robotColor, useRobotColorOf} from "../lib/palette";

export function currentActionIndex(plan: RobotPlan | undefined): number {
  if (!plan?.plan) return -1;
  const res = (plan.results ?? []).filter(r => !r.plan_id || r.plan_id === plan.plan!.plan_id);
  if (!res.length) return 0;
  const last = res[res.length - 1];
  const idx = last.action_index ?? 0;
  return /succe|complete|done|finished/i.test(last.status ?? "") ? idx + 1 : idx;
}

/** ``pts`` without consecutive repeats: a zero-length segment keeps drei's Line
 *  from drawing at all (plan paths repeat the point where two hops meet). */
export function distinctPoints(pts: [number, number, number][]): [number, number, number][] {
  return pts.filter((p, i) => i === 0 || p[0] !== pts[i - 1][0] || p[1] !== pts[i - 1][1] || p[2] !== pts[i - 1][2]);
}
