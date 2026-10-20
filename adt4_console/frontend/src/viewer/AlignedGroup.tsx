// Draws a robot's live layer in the global frame: a three.js group carrying the
// robot's `frame_align` transform (see lib/frameAlign.ts for the policy). When
// the alignment changes (a loop closure moves map -> odom) the group glides to
// the new pose instead of jumping, and nothing is re-created, so no flicker.
import {useEffect, useRef, type ReactNode} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {layerKey, layerMatrix} from "../lib/frameAlign";
import {useFrameAlign} from "../stores/robots";

const GLIDE_MS = 600;

interface Glide {
  fromP: THREE.Vector3;
  fromQ: THREE.Quaternion;
  toP: THREE.Vector3;
  toQ: THREE.Quaternion;
  t0: number;
}

/** `frame` = the layer's frame (empty: the robot's scene-graph frame). */
export function AlignedGroup({robot, frame, children}: {robot: string; frame?: string | null; children: ReactNode}) {
  const fa = useFrameAlign(robot);
  const key = layerKey(fa, frame);
  const ref = useRef<THREE.Group>(null);
  const glide = useRef<Glide | null>(null);
  const placed = useRef(false);
  const invalidate = useThree(s => s.invalidate);
  useEffect(() => {
    const g = ref.current;
    if (!g) return;
    const m = layerMatrix(fa, frame);
    const toP = new THREE.Vector3();
    const toQ = new THREE.Quaternion();
    if (m) m.decompose(toP, toQ, new THREE.Vector3());
    if (!placed.current) {
      // first placement: snap (nothing drawn yet to glide from)
      placed.current = true;
      g.position.copy(toP);
      g.quaternion.copy(toQ);
      glide.current = null;
    } else {
      glide.current = {fromP: g.position.clone(), fromQ: g.quaternion.clone(), toP, toQ, t0: performance.now()};
    }
    invalidate();
    // fa is read through `key` (its matrix); the object identity changes at 4 Hz
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key, invalidate]);
  useFrame(() => {
    const g = ref.current;
    const gl = glide.current;
    if (!g || !gl) return;
    const a = Math.min(1, (performance.now() - gl.t0) / GLIDE_MS);
    const e = a * a * (3 - 2 * a); // smoothstep
    g.position.lerpVectors(gl.fromP, gl.toP, e);
    g.quaternion.slerpQuaternions(gl.fromQ, gl.toQ, e);
    if (a < 1) invalidate();
    else glide.current = null;
  });
  return (
    <group ref={ref} userData={{alignedRobot: robot}}>
      {children}
    </group>
  );
}
