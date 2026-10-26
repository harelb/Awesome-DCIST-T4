// A PDDL goal's plan, previewed before it is sent (API.md §15.5): omniplanner's
// per-robot routes, drawn on the map by Scene's PreviewOverlay until cleared.
import {create} from "zustand";
import {ApiError, errorText, postJSON} from "../lib/api";
import {fastDownwardOf, type FastDownwardInfo} from "../lib/fastDownward";

export interface PddlPreview {
  routes: Record<string, number[][]>;
  driven_m: Record<string, number>;
  actions: {robot: string; action: string}[];
  robots: string[];
  ground_s: number;
  solve_s: number;
  pois: number;
  fast_downward?: FastDownwardInfo | null;
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
): Promise<PddlPreview> {
  try {
    const p = await postJSON<PddlPreview>("api/missions/plan/preview_pddl", {pddl_goal: goal, robots, src, constraints});
    usePlanPreview.getState().set(goal, p);
    return p;
  } catch (e) {
    clearPreview();
    throw new PreviewError(errorText(e), e instanceof ApiError ? fastDownwardOf(e.body) : null);
  }
}
