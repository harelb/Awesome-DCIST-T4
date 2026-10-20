// Dims (or drops) everything under it: the robot viewer draws the robots that
// are not in focus at DIM opacity. Works on any three.js material: before
// each rendered frame it scales the material's own opacity, remembering the
// owner's value (and re-reading it when the owner changes it), and restores
// it once the group is no longer dimmed.
import {useEffect, useRef, type ReactNode} from "react";
import type * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {useViewer} from "../stores/viewer";
import {useFocusedRobot} from "./useRobotFocus";

export const DIM = 0.3;

export type OthersMode = "dim" | "hide" | "show";

/** How a robot's layers are drawn under the current focus. */
export function focusLook(robot: string | null, focused: string | null, others: OthersMode): "normal" | "dim" | "hidden" {
  if (!focused || !robot || robot === focused || others === "show") return "normal";
  return others === "hide" ? "hidden" : "dim";
}

export function useFocusLook(robot: string | null): "normal" | "dim" | "hidden" {
  const focused = useFocusedRobot();
  const others = useViewer(s => s.focusOthers);
  return focusLook(robot, focused, others);
}

interface DimData {
  dimBase?: number;
  dimBaseT?: boolean;
  dimApplied?: number;
}

/** Scale one material's opacity by `f` (1 restores the owner's value). */
export function applyDim(m: THREE.Material, f: number) {
  const u = m.userData as DimData;
  if (u.dimApplied === undefined) {
    u.dimBase = m.opacity;
    u.dimBaseT = m.transparent;
  } else if (m.opacity !== u.dimApplied) u.dimBase = m.opacity; // the owner set a new opacity
  const target = (u.dimBase ?? 1) * f;
  if (m.opacity !== target) m.opacity = target;
  const t = f < 1 ? true : (u.dimBaseT ?? m.transparent);
  if (m.transparent !== t) {
    m.transparent = t;
    m.needsUpdate = true;
  }
  if (f < 1) u.dimApplied = m.opacity;
  else {
    delete u.dimApplied;
    delete u.dimBase;
    delete u.dimBaseT;
  }
}

export function DimGroup({robot, children}: {robot: string | null; children: ReactNode}) {
  const look = useFocusLook(robot);
  const ref = useRef<THREE.Group>(null);
  const applied = useRef(false);
  const invalidate = useThree(s => s.invalidate);
  useEffect(() => invalidate(), [look, invalidate]);
  useFrame(() => {
    const g = ref.current;
    if (!g) return;
    const dim = look === "dim";
    if (!dim && !applied.current) return;
    const f = dim ? DIM : 1;
    g.traverse(o => {
      const mat = (o as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
      if (!mat) return;
      for (const m of Array.isArray(mat) ? mat : [mat]) applyDim(m, f);
    });
    applied.current = dim;
  });
  // hidden: not drawn at all (also out of the click picker's registry)
  if (look === "hidden") return null;
  return (
    <group ref={ref} userData={{focusLook: look}}>
      {children}
    </group>
  );
}
