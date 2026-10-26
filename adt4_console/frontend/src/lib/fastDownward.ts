// Fast Downward's account of a plan (API.md §15.5): the digest, and on a failure
// its last log lines and the problem file it was given.
export interface FastDownwardInfo {
  digest?: string;
  log?: string[];
  problem_file?: string | null;
}

/** The ``fast_downward`` block of a planner route's response or error body. */
export function fastDownwardOf(body: unknown): FastDownwardInfo | null {
  const fd = (body as {fast_downward?: FastDownwardInfo} | null)?.fast_downward;
  return fd && typeof fd === "object" ? fd : null;
}
