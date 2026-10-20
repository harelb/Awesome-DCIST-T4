// Hooks over the robot focus (robotFocus.ts has the pure rules).
import {useEffect, useMemo, useState} from "react";
import {useFleet} from "../stores/fleet";
import {useRobots} from "../stores/robots";
import {useViewer} from "../stores/viewer";
import {resolveFocus, robotLiveness, rosterOf, type RobotLiveness} from "./robotFocus";

export function useRobotRoster(): string[] {
  const machines = useFleet(s => s.machines?.machines);
  const state = useRobots(s => s.state);
  const link = useRobots(s => s.link);
  const plan = useRobots(s => s.plan);
  const teleop = useRobots(s => s.teleop);
  const key = [machines?.map(m => `${m.name}:${m.role}`).join(","), Object.keys(state).join(","), Object.keys(link).join(","), Object.keys(plan).join(","), Object.keys(teleop).join(",")].join("|");
  // eslint-disable-next-line react-hooks/exhaustive-deps
  return useMemo(() => rosterOf(machines, Object.keys(state), Object.keys(link), Object.keys(plan), Object.keys(teleop)), [key]);
}

/** The focused robot, or null for "all robots". */
export function useFocusedRobot(): string | null {
  const focus = useViewer(s => s.focus);
  const roster = useRobotRoster();
  const f = resolveFocus(focus, roster);
  return f === "all" ? null : f;
}

/** Liveness of every roster robot, re-evaluated each second. */
export function useRobotLiveness(roster: readonly string[]): Record<string, RobotLiveness> {
  const stateAt = useRobots(s => s.stateAt);
  const link = useRobots(s => s.link);
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);
  return useMemo(() => Object.fromEntries(roster.map(r => [r, robotLiveness(stateAt[r], link[r]?.connected, now)])), [roster, stateAt, link, now]);
}
