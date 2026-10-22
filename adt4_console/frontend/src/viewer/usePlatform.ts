// A robot's platform (lib/platform.ts) from what the console knows: the
// agent's C1 state, the backend's /api/plan/robots entry (``hint``), the fleet
// topology's platform_type. Nothing known = "spot".
import {useRobots} from "../stores/robots";
import {useFleet} from "../stores/fleet";
import {phoenixGuards, resolvePlatform, type Platform, type PhoenixGuards} from "../lib/platform";

export function usePlatform(robot: string | null | undefined, hint?: string | null): Platform {
  const fromState = useRobots(s => (robot ? s.state[robot]?.platform : undefined));
  const fromFleet = useFleet(s => (robot ? s.machines?.machines.find(m => m.name === robot)?.platform_type : undefined));
  return resolvePlatform(fromState, hint, fromFleet);
}

const NONE: PhoenixGuards = {manual: null, battery: null, estop: null, safety: null};

/** Phoenix guards from the robot's C1 state (all null when unknown). */
export function usePhoenixGuards(robot: string | null | undefined): PhoenixGuards {
  const g = useRobots(s => (robot ? s.state[robot]?.guards : undefined));
  return g ? phoenixGuards(g) : NONE;
}
