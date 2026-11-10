// Chat auto-send: the rules for which robots the chat may plan for and send to
// without the operator pressing Send. Pure (the store and ChatPanel use them).
//
// A robot is "online" for auto-send when its fleetlink link is up and its state
// is fresh (robotLiveness), its executor publishes runtime guards, it is not
// E-stopped, and (phoenix) it is not in manual mode. A Spot whose lease another
// client holds is still online: sending a plan takes the lease (the backend's
// RosBridge.auto_take_leases), and the send result reports it. GRSTAPS-X leaves
// Huskies out of its plans, so with that engine a Husky is skipped too.
import {platformOf} from "./platform";
import {robotLiveness} from "../viewer/robotFocus";
import type {RobotState} from "../types";

export type AutoKind = "spot" | "husky";

export interface AutoRobot {
  name: string;
  kind: AutoKind;
  /** Spot: who holds the lease when the executor does not (sending takes it) */
  leaseHolder?: string;
}

export interface Skipped {
  name: string;
  reason: string;
}

export interface Fleet {
  state: Record<string, RobotState | undefined>;
  stateAt: Record<string, number | undefined>;
  link: Record<string, {connected?: boolean} | undefined>;
  /** topology platform_type by robot (absent: from the robot's own state) */
  platforms?: Record<string, string | null | undefined>;
}

export function kindOf(robot: string, f: Fleet): AutoKind {
  const p = platformOf(f.state[robot]?.platform ?? f.platforms?.[robot] ?? null);
  return p === "phoenix" ? "husky" : "spot";
}

/** Which of ``names`` the chat may send to now, and why the others are skipped. */
export function onlineRobots(names: readonly string[], f: Fleet, now: number, engine?: string | null): {online: AutoRobot[]; skipped: Skipped[]} {
  const online: AutoRobot[] = [];
  const skipped: Skipped[] = [];
  for (const name of [...new Set(names)].sort()) {
    const live = robotLiveness(f.stateAt[name], f.link[name]?.connected, now);
    if (live !== "online") {
      skipped.push({name, reason: live});
      continue;
    }
    const kind = kindOf(name, f);
    const g = f.state[name]?.guards as Record<string, unknown> | undefined;
    if (!g || Object.keys(g).length === 0) {
      skipped.push({name, reason: "no executor status (runtime guards)"});
      continue;
    }
    if (g.estop_pressed === true) {
      skipped.push({name, reason: "E-stopped"});
      continue;
    }
    if (kind === "husky" && g.manual_mode === true) {
      skipped.push({name, reason: "in manual mode"});
      continue;
    }
    if (kind === "husky" && engine === "grstaps") {
      skipped.push({name, reason: "GRSTAPS-X leaves Huskies out"});
      continue;
    }
    const holder = kind === "spot" && g.lease_known === true && g.lease_owned === false ? String(g.lease_owner ?? "").trim() || "another client" : undefined;
    online.push(holder ? {name, kind, leaseHolder: holder} : {name, kind});
  }
  return {online, skipped};
}

/** The chat's agent roster for these robots: bound, starting at their live pose. */
export function autoRoster(online: readonly AutoRobot[]) {
  return online.map(r => ({id: r.name, kind: r.kind, start: null, binding: r.name}));
}

/** The robots a PDDL goal names with its -by forms (visited-poi-by gauss t1, ...). */
export function goalRobots(goal: string): string[] {
  const out: string[] = [];
  for (const m of goal.matchAll(/\(\s*[a-z][a-z0-9_-]*-by\s+([A-Za-z_][\w-]*)/g)) if (!out.includes(m[1])) out.push(m[1]);
  return out;
}

export interface AutoSendPlan {
  /** the robot the goal is sent through (one send covers every -by robot) */
  via: string | null;
  /** the robots the goal names (or ``via`` alone for a goal without -by forms) */
  robots: string[];
  skipped: Skipped[];
  /** why nothing is sent, when ``via`` is null */
  blocked?: string;
}

/** Whether, and through which robot, to auto-send ``goal``. A goal naming a
 *  robot that is not online is not sent at all (the planner would assign it
 *  work it cannot do). */
export function planAutoSend(goal: string, online: readonly AutoRobot[], skipped: readonly Skipped[]): AutoSendPlan {
  const ok = new Set(online.map(r => r.name));
  const named = goalRobots(goal);
  const bad = named.filter(r => !ok.has(r));
  if (bad.length) {
    const why = (r: string) => skipped.find(s => s.name === r)?.reason ?? "not online";
    return {via: null, robots: named, skipped: [...skipped], blocked: `not sent: ${bad.map(r => `${r} (${why(r)})`).join(", ")}`};
  }
  const via = named[0] ?? online[0]?.name ?? null;
  return {via, robots: named.length ? named : via ? [via] : [], skipped: [...skipped], blocked: via ? undefined : "not sent: no robot is online"};
}
