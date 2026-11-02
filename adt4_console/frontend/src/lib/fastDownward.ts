// The planner's account of a plan (API.md §15.5): which planner (Fast Downward or
// GRSTAPS-X), its digest, and on a failure its last log lines and the problem file
// it was given. Responses carry it as ``planner_log`` (any engine); Fast Downward's
// is also under its older name ``fast_downward``.
export interface FastDownwardInfo {
  planner?: string;
  digest?: string;
  log?: string[];
  problem_file?: string | null;
}
export type PlannerLog = FastDownwardInfo;

/** The planner's account in a planner route's response or error body. */
export function fastDownwardOf(body: unknown): FastDownwardInfo | null {
  const b = body as {planner_log?: FastDownwardInfo; fast_downward?: FastDownwardInfo} | null;
  for (const fd of [b?.planner_log, b?.fast_downward]) if (fd && typeof fd === "object") return fd;
  return null;
}
export const plannerLogOf = fastDownwardOf;
