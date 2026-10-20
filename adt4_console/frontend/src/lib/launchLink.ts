// Links into the Launch page with a prior map preselected (Maps → "Launch with map").
// Hash-router query: #/launch?prior=<path>&source=<machine>[&mode=mapping|relocalize]

export type LaunchMode = "mapping" | "relocalize";

export interface LaunchPreset {
  prior: string;
  source: string;
  mode: LaunchMode;
}

export function launchHref(source: string, path: string, mode: LaunchMode = "relocalize"): string {
  const q = new URLSearchParams({prior: path, source, mode});
  return `/launch?${q.toString()}`;
}

/** A preset from the Launch page's query string, or null when no prior map is given. */
export function parseLaunchPreset(search: string | URLSearchParams): LaunchPreset | null {
  const q = typeof search === "string" ? new URLSearchParams(search.startsWith("?") ? search.slice(1) : search) : search;
  const prior = q.get("prior") ?? "";
  const source = q.get("source") ?? "";
  if (!prior || !source) return null;
  // a prior map almost always means relocalizing against it; mapping is kept when asked for
  const mode: LaunchMode = q.get("mode") === "mapping" ? "mapping" : "relocalize";
  return {prior, source, mode};
}
