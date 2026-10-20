// Robot viewer focus: "all" robots or one robot. Every robot-specific part of
// the viewer (camera, teleop, plan, live link, stats, 3D dimming, legend)
// follows this one value (stores/viewer.ts `focus`). Persisted per browser;
// a stored robot the console does not know (any more) reads as "all".
// Pure helpers (no store imports: stores/viewer.ts loads the focus from here);
// the hooks live in useRobotFocus.ts.
import {safeStorage} from "./paneLayout";

export type Focus = "all" | string;
export const FOCUS_KEY = "adt4.viewer.focus.v1";
/** state older than this reads as stale (RobotGlyphs uses the same 5 s) */
export const STALE_MS = 5000;

export function loadFocus(storage: Pick<Storage, "getItem"> | null | undefined = safeStorage()): Focus {
  try {
    const v = storage?.getItem(FOCUS_KEY);
    return typeof v === "string" && v.trim() ? v : "all";
  } catch {
    return "all";
  }
}

export function saveFocus(f: Focus, storage: Pick<Storage, "setItem"> | null | undefined = safeStorage()): boolean {
  try {
    storage?.setItem(FOCUS_KEY, f);
    return !!storage;
  } catch {
    return false;
  }
}

/** The focus in effect: a robot only while the roster knows it. An empty
 *  roster (nothing heard yet) keeps the stored robot so a reload does not
 *  flash "all" before the WebSocket delivers the fleet. */
export function resolveFocus(f: Focus, roster: readonly string[]): Focus {
  if (f === "all") return "all";
  if (!roster.length) return f;
  return roster.includes(f) ? f : "all";
}

/** Next / previous focus in the cycle all -> robot 1 -> ... -> robot n -> all. */
export function cycleFocus(f: Focus, roster: readonly string[], dir: 1 | -1 = 1): Focus {
  const ring: Focus[] = ["all", ...roster];
  const i = Math.max(0, ring.indexOf(f));
  return ring[(i + dir + ring.length) % ring.length];
}

export type RobotLiveness = "online" | "stale" | "offline";

/** online: C1 state within STALE_MS; stale: heard before, not lately (or the
 *  link says connected without a fresh state); offline: never heard, or the
 *  link is down. */
export function robotLiveness(stateAt: number | undefined, linkConnected: boolean | undefined, now: number): RobotLiveness {
  if (linkConnected === false) return "offline";
  if (stateAt !== undefined && now - stateAt <= STALE_MS) return "online";
  if (stateAt !== undefined || linkConnected) return "stale";
  return "offline";
}

/** Every robot the console knows: topology robots, plus any robot that sent
 *  state, link stats, a plan or teleop state. */
export function rosterOf(machines: {name: string; role: string}[] | undefined, ...names: string[][]): string[] {
  const out = new Set<string>();
  for (const m of machines ?? []) if (m.role === "robot") out.add(m.name);
  for (const list of names) for (const n of list) out.add(n);
  return [...out].sort();
}
