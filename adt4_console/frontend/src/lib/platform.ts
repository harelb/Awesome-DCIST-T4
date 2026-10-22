// Robot platform: which controls and status a robot has. One rule everywhere
// (fleetlink.relay_ros.platform_of mirrors it): the topology's platform_type,
// missing = "spot", "husky" = "phoenix" (ARL's phoenix stack on a Clearpath
// Husky). Spot-only UI (lease, stand/sit, strafe, pause) is gated with isSpot().
// Phoenix status comes from the fleetlink agent's guards (battery, e-stop,
// safety stop, manual mode).
import type {PhoenixProgress} from "../types";

export type Platform = "spot" | "phoenix" | (string & {});

type PlatformLike = string | null | undefined | {platform?: string | null; platform_type?: string | null};

/** "spot" / "phoenix" / other platform_type, from a platform name or a robot
 *  record (``platform`` from the agent / backend wins over ``platform_type``). */
export function platformOf(x: PlatformLike): Platform {
  const raw = typeof x === "string" ? x : (x?.platform ?? x?.platform_type ?? "");
  const p = String(raw ?? "").trim().toLowerCase();
  if (!p) return "spot";
  return p === "husky" ? "phoenix" : p;
}

export const isSpot = (x: PlatformLike) => platformOf(x) === "spot";
export const isPhoenix = (x: PlatformLike) => platformOf(x) === "phoenix";

/** A robot's platform from what the console knows, most specific first: the
 *  agent's C1 state, the backend's plan-robot entry, the fleet topology. */
export function resolvePlatform(...hints: (string | null | undefined)[]): Platform {
  return platformOf(hints.find(h => typeof h === "string" && h.trim()) ?? null);
}

/** Phoenix guards (agent C1 ``guards``, ``source: "phoenix"``); null = unknown. */
export interface PhoenixGuards {
  manual: boolean | null;
  battery: number | null;
  estop: boolean | null;
  safety: boolean | null;
}

const bool = (v: unknown): boolean | null => (typeof v === "boolean" ? v : null);

export function phoenixGuards(g: Record<string, unknown> | null | undefined): PhoenixGuards {
  const b = g?.battery_percent;
  return {
    manual: bool(g?.manual_mode),
    battery: typeof b === "number" && Number.isFinite(b) ? b : null,
    estop: bool(g?.estop_pressed),
    safety: bool(g?.safety_stop),
  };
}

export interface StatusChip {
  key: "mode" | "battery" | "estop" | "safety";
  tone: "ok" | "warn" | "err" | "off";
  text: string;
  title: string;
}

/** AUTO / MANUAL, battery %, E-STOP, SAFETY-STOP chips (stops only while set). */
export function phoenixChips(g: PhoenixGuards): StatusChip[] {
  const out: StatusChip[] = [];
  if (g.manual === true) out.push({key: "mode", tone: "warn", text: "MANUAL", title: "Manual (RC) mode: the robot will not execute plans. Resume (auto) hands it back to the planner."});
  else if (g.manual === false) out.push({key: "mode", tone: "ok", text: "AUTO", title: "Autonomous mode: the robot executes plans"});
  if (g.battery !== null) {
    const pct = Math.round(g.battery);
    out.push({key: "battery", tone: pct < 15 ? "err" : pct < 30 ? "warn" : "off", text: `${pct} %`, title: `Battery ${pct} %`});
  }
  if (g.estop) out.push({key: "estop", tone: "err", text: "E-STOP", title: "Emergency stop is pressed: the platform will not move"});
  if (g.safety) out.push({key: "safety", tone: "err", text: "SAFETY-STOP", title: "Safety stop is active: the platform will not move"});
  return out;
}

/** The progress report belongs to plan ``planId`` (string ids, or the same
 *  "sec.nanosec" stamp written differently). */
export function progressMatches(p: PhoenixProgress | null | undefined, planId: string | null | undefined): boolean {
  if (!p || planId == null || p.plan_id == null) return false;
  if (String(p.plan_id) === String(planId)) return true;
  const a = Number(p.plan_id);
  const b = Number(planId);
  return Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) < 1e-6;
}
