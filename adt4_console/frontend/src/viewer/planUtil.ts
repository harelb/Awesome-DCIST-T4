import {SOURCE_TINTS} from "../lib/palette";
import type {RobotPlan} from "../types";

export const robotColor = (name: string) => {
  let h = 0;
  for (const c of name) h = (h * 31 + c.charCodeAt(0)) >>> 0;
  return SOURCE_TINTS[h % SOURCE_TINTS.length];
};

export function currentActionIndex(plan: RobotPlan | undefined): number {
  if (!plan?.plan) return -1;
  const res = (plan.results ?? []).filter(r => !r.plan_id || r.plan_id === plan.plan!.plan_id);
  if (!res.length) return 0;
  const last = res[res.length - 1];
  const idx = last.action_index ?? 0;
  return /succe|complete|done|finished/i.test(last.status ?? "") ? idx + 1 : idx;
}
