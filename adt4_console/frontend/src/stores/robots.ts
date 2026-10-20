import {useMemo} from "react";
import {create} from "zustand";
import type {Cls, LinkInfo, RobotPlan, RobotState} from "../types";
import {parsePaths, type RobotPaths} from "../lib/overlays";
import {ws} from "../ws";

export const HISTORY = 60;

interface RobotsStore {
  state: Record<string, RobotState>;
  stateAt: Record<string, number>;
  plan: Record<string, RobotPlan>;
  paths: Record<string, RobotPaths>;
  link: Record<string, LinkInfo>;
  rateHistory: Record<string, Record<Cls, number[]>>;
  setLink: (r: string, l: LinkInfo) => void;
}

export const useRobots = create<RobotsStore>(set => ({
  state: {},
  stateAt: {},
  plan: {},
  paths: {},
  link: {},
  rateHistory: {},
  setLink: (r, l) =>
    set(s => {
      const prev = s.rateHistory[r] ?? {};
      const next = {} as Record<Cls, number[]>;
      const classes = new Set<Cls>(["C1", "C2", "C3", ...Object.keys(l.rates ?? {}), ...Object.keys(l.budgets ?? {})]);
      for (const c of classes) {
        next[c] = [...(prev[c] ?? []), Number(l.rates?.[c] ?? 0)].slice(-HISTORY);
      }
      return {link: {...s.link, [r]: l}, rateHistory: {...s.rateHistory, [r]: next}};
    }),
}));

export function bindRobots() {
  ws.on("robot/", (d, f) => {
    const m = /^robot\/([^/]+)\/(state|plan|paths)$/.exec(f.topic);
    if (!m) return;
    const [, r, kind] = m;
    if (kind === "paths") {
      const p = d == null ? null : parsePaths(d);
      if (p) useRobots.setState(s => ({paths: {...s.paths, [r]: p}}));
      else if (d == null)
        // the robot restarted (new epoch) and has no paths: drop the stale ones
        useRobots.setState(s => {
          const paths = {...s.paths};
          delete paths[r];
          return {paths};
        });
    } else if (kind === "state")
      useRobots.setState(s => ({state: {...s.state, [r]: d as RobotState}, stateAt: {...s.stateAt, [r]: Date.now()}}));
    else useRobots.setState(s => ({plan: {...s.plan, [r]: d as RobotPlan}}));
  });
  ws.on("link/", (d, f) => {
    const r = f.topic.slice(5);
    if (r) useRobots.getState().setLink(r, d as LinkInfo);
  });
}

/** Robots the console has heard from (state or link). */
export function useLiveRobots(): string[] {
  const state = useRobots(s => s.state);
  const link = useRobots(s => s.link);
  return useMemo(() => [...new Set([...Object.keys(link), ...Object.keys(state)])].sort(), [state, link]);
}
