// Run identity on the client: copy badges, Verify pairing, "prefer the base's copy"
// for Launch with map, and the viewer's default sources for the active launch.
// Pure functions (unit tested); the backend does the pairing (run_identity.pair_copies).
import type {CopyState, MapEntry, MapsResponse} from "../types";
import type {LaunchPreset} from "./launchLink";

export type Tone = "ok" | "warn" | "off" | "info";

/** Badge for a robot run: is it on the base station, and is that copy current? */
export function copyBadge(map: MapEntry, base: string): {tone: Tone; text: string; title: string} | null {
  const c = map.copy;
  if (!c) return null;
  const on = c.machine || base;
  const how = c.matched_by === "contents" ? " (matched by its files; copied before run ids)" : "";
  switch (c.state) {
    case "in_sync":
      return {tone: "ok", text: `on ${on} ✓ in sync`, title: `${on}:${c.path ?? c.name}${how}${c.transferred_at ? `\ncopied ${c.transferred_at}` : ""}`};
    case "outdated":
      return {tone: "warn", text: `on ${on} – outdated`, title: `robot changed since the copy ${on}:${c.path ?? c.name}${how}`};
    case "not_copied":
      return {tone: "off", text: "not copied", title: `no copy of this run on ${on}`};
    default:
      return null;
  }
}

export const COPY_STATE_TEXT: Record<CopyState, string> = {
  in_sync: "in sync",
  outdated: "outdated (robot changed since copy)",
  not_copied: "not copied",
  unknown: "source offline",
};

/** "copy of hamilton:/home/swarm/adt4_output/map_…" for a base copy, else null. */
export function copyOfText(map: MapEntry): string | null {
  const c = map.copy_of;
  if (!c || !c.machine) return null;
  return `copy of ${c.machine}:${c.path || c.name}`;
}

export function entriesOf(maps: MapsResponse | null | undefined, machine: string): MapEntry[] {
  const list = maps?.machines?.[machine];
  return Array.isArray(list) ? list : [];
}

/** The base-station entry a robot run's copy points at (for "Open copy"). */
export function copyEntry(maps: MapsResponse | null | undefined, map: MapEntry, base: string): MapEntry | null {
  const c = map.copy;
  if (!c?.name || c.state === "not_copied") return null;
  const found = entriesOf(maps, c.machine || base).find(e => e.name === c.name);
  return found ?? {name: c.name, size: "?", path: c.path};
}

/**
 * Verify pairing: machine -> run name there.  A robot run with a base copy compares
 * against that copy (named <experiment>__<robot>); a base copy compares against its
 * source.  null = no pairing (compare the same name everywhere, as before).
 */
export function verifyNames(machine: string, map: MapEntry, base: string): Record<string, string> | null {
  if (map.copy?.name && map.copy.state !== "not_copied") return {[machine]: map.name, [map.copy.machine || base]: map.copy.name};
  if (map.copy_of?.machine && map.copy_of.name) return {[map.copy_of.machine]: map.copy_of.name, [machine]: map.name};
  return null;
}

/**
 * Launch with map: a robot run whose base copy is in sync launches from the base's
 * copy (the robots that hold the original are skipped by run_id; nothing is re-sent
 * to the base).  Returns the preset unchanged otherwise, with a note explaining why.
 */
export function preferBaseCopy(preset: LaunchPreset, maps: MapsResponse | null | undefined, base: string): {preset: LaunchPreset; note: string | null} {
  if (!base || preset.source === base) return {preset, note: null};
  const run = entriesOf(maps, preset.source).find(e => e.path === preset.prior || e.abs_path === preset.prior);
  const c = run?.copy;
  if (!run || !c) return {preset, note: null};
  if (c.state === "in_sync" && c.path) {
    return {
      preset: {...preset, source: c.machine || base, prior: c.path},
      note: `Using ${c.machine || base}'s in-sync copy ${c.name} of ${preset.source}:${run.name}.`,
    };
  }
  if (c.state === "outdated") return {preset, note: `${c.machine || base}'s copy ${c.name} is outdated; launching from ${preset.source}'s run.`};
  return {preset, note: null};
}

/** Prior selection hint: is the selected prior available on this machine for the viewer? */
export function priorAvailability(maps: MapsResponse | null | undefined, machine: string, path: string, base: string): {local: boolean; copy: string | null; state: CopyState | null; entry: MapEntry | null} {
  if (!machine || machine === base) return {local: true, copy: null, state: null, entry: null};
  const run = entriesOf(maps, machine).find(e => e.path === path || e.abs_path === path) ?? null;
  const c = run?.copy;
  if (c && c.state !== "not_copied" && c.name) return {local: true, copy: c.name, state: c.state, entry: run};
  return {local: false, copy: null, state: c?.state ?? null, entry: run};
}

/**
 * Viewer default for the active launch: enable the defaults that already exist in the
 * catalog now; the rest (a live:<robot> that has not sent anything yet) are enabled
 * when they appear.
 */
export function pickDefaultSources(catalog: string[], defaults: string[]): {enable: string[]; pending: string[]} {
  const have = new Set(catalog);
  return {enable: defaults.filter(id => have.has(id)), pending: defaults.filter(id => !have.has(id))};
}
