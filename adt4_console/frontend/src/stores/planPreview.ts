// A PDDL goal's plan, previewed before it is sent (API.md §15.5): omniplanner's
// per-robot routes, drawn on the map by Scene's PreviewOverlay until cleared.
import {create} from "zustand";
import {ApiError, errorText, postJSON} from "../lib/api";
import {fastDownwardOf, type FastDownwardInfo} from "../lib/fastDownward";
import type {PlannerScheduleData} from "../viewer/PlannerSchedule";
import {useMission} from "./mission";
import {useRobots} from "./robots";

/** A region goal as GRSTAPS-X planned it: the place(s) its visits go to (an
 *  explore's coverage waypoints still to do) and where they are. */
export interface RegionPlan {
  kind: "visit" | "explore";
  places: string[];
  points: number[][];
  total?: number | null;
  spacing_m?: number | null;
}

/** A drawn mission area's symbol in a goal (area1, area2, ...: the mission's areas in order). */
export const AREA_SYMBOL = /\barea\d+\b/i;

export interface PddlPreview {
  routes: Record<string, number[][]>;
  driven_m: Record<string, number>;
  actions: {robot: string; action: string}[];
  robots: string[];
  ground_s: number;
  solve_s: number;
  pois: number;
  fast_downward?: FastDownwardInfo | null;
  /** which engine planned it, its account, and GRSTAPS-X's timed schedule */
  engine?: string;
  planner_log?: FastDownwardInfo | null;
  schedule?: PlannerScheduleData | null;
  /** each robot's goals as node symbols, in order (the chat shows their images) */
  goals?: Record<string, {sym: string; xy?: number[] | null; yaw?: number | null}[]>;
  /** GRSTAPS-X's region goals: {region: its visits} */
  regions?: Record<string, RegionPlan>;
}

/** A failed preview: the reason, and Fast Downward's account when it ran. */
export class PreviewError extends Error {
  fd: FastDownwardInfo | null;
  constructor(message: string, fd: FastDownwardInfo | null) {
    super(message);
    this.fd = fd;
  }
}

interface PreviewStore {
  /** the goal whose routes are drawn */
  goal: string | null;
  preview: PddlPreview | null;
  set: (goal: string | null, preview: PddlPreview | null) => void;
}

export const usePlanPreview = create<PreviewStore>(set => ({
  goal: null,
  preview: null,
  set: (goal, preview) => set({goal, preview}),
}));

export function clearPreview() {
  usePlanPreview.getState().set(null, null);
}

/** Plan ``goal`` for ``robots`` ([{name, start?}]: no start = the live pose) and draw it. */
export async function previewPddl(
  goal: string,
  robots: {name: string; start?: number[] | null}[],
  src: string | null,
  constraints?: {predicate: string; symbols: string[]}[],
  engine?: string,
): Promise<PddlPreview> {
  try {
    // a drawn mission area is named by its position in the mission's list (API.md §15.5.1)
    const areas = AREA_SYMBOL.test(goal) ? useMission.getState().areas.map(a => ({name: a.name, polygon: a.polygon})) : undefined;
    const p = await postJSON<PddlPreview>("api/missions/plan/preview_pddl", {pddl_goal: goal, robots, src, constraints, engine, areas});
    usePlanPreview.getState().set(goal, p);
    return p;
  } catch (e) {
    clearPreview();
    throw new PreviewError(errorText(e), e instanceof ApiError ? fastDownwardOf(e.body) : null);
  }
}

/** The robots a goal preview plans for: the connected ones (their live poses) and
 *  the goal's robot; with none connected, the mission planner's ground agents. */
export function previewRobots(robot: string | null): {name: string; start?: number[] | null}[] {
  const live = Object.keys(useRobots.getState().state);
  if (live.length > 0) return [...new Set([...live, ...(robot ? [robot] : [])])].map(name => ({name}));
  return useMission
    .getState()
    .roster.filter(a => a.kind !== "quad" && (a.start || a.binding))
    .map(a => ({name: a.id, start: a.start ?? null}));
}
