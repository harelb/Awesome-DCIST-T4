// A robot's platform model (lib/robotModel) in its pose frame, optionally
// highlighted in the robot's colour: an outline (each URDF part's convex hull,
// grown by OUTLINE_M and drawn back faces only, so it shows as a halo around the
// part), an emissive tint, or the whole model painted (shaded) in that colour. Hulls, not the parts themselves: the decimated parts
// are many small disconnected pieces, and a shell grown from them tears apart.
// Given a ``motion`` (a pose track), Spot's legs trot and the Husky's wheels roll
// at its ground speed (lib/gait): each part and its hull move by the same matrix.
import {Component, useEffect, useMemo, useRef, type ReactNode} from "react";
import * as THREE from "three";
import {useFrame, useLoader, useThree} from "@react-three/fiber";
import {GLTFLoader} from "three/examples/jsm/loaders/GLTFLoader.js";
import {ConvexGeometry} from "three/examples/jsm/geometries/ConvexGeometry.js";
import {OUTLINE_M, PAINT_EMISSIVE, TINT_INTENSITY, type RobotHighlight} from "../lib/robotModel";
import {GaitState, HUSKY_WHEEL_R, rigKind, spotPartDeltas, wheelPartDeltas, type PoseTrack} from "../lib/gait";
import {useViewStyle} from "../stores/viewStyle";

/** Renders ``fallback`` (the arrow) when the model fails to load, so a missing or
 *  broken GLB never takes the 3D view down. */
export class ModelBoundary extends Component<{fallback: ReactNode; children: ReactNode}, {failed: boolean}> {
  state = {failed: false};
  static getDerivedStateFromError() {
    return {failed: true};
  }
  componentDidCatch(err: unknown) {
    console.warn("robot model failed to load; drawing the arrow", err);
  }
  render() {
    return this.state.failed ? this.props.fallback : this.props.children;
  }
}

/** The model's parts (by GLB node name) with their transforms baked in (one-time per URL). */
function useModelParts(url: string): {names: string[]; parts: THREE.BufferGeometry[]} {
  const gltf = useLoader(GLTFLoader, url);
  return useMemo(() => {
    const names: string[] = [];
    const parts: THREE.BufferGeometry[] = [];
    gltf.scene.updateMatrixWorld(true);
    gltf.scene.traverse(o => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      const g = m.geometry.clone();
      g.applyMatrix4(m.matrixWorld);
      if (!g.getAttribute("normal")) g.computeVertexNormals();
      names.push(m.name);
      parts.push(g);
    });
    return {names, parts};
  }, [gltf]);
}

const IDENTITY = new THREE.Matrix4();

/** Moves the model's parts (meshes + hull meshes, by part index) for ``motion``'s
 *  ground speed each frame, rendering frames only while something moves. */
function useGait(names: string[], meshes: React.RefObject<(THREE.Mesh | null)[]>, shells: React.RefObject<(THREE.Mesh | null)[]>, bob: React.RefObject<THREE.Group | null>, motion: PoseTrack | null | undefined) {
  const enabled = useViewStyle(s => s.robotGait) && !!motion;
  const kind = useMemo(() => rigKind(names), [names]);
  const gait = useRef(new GaitState());
  const posed = useRef(false);
  const invalidate = useThree(s => s.invalidate);
  useEffect(() => invalidate(), [enabled, invalidate]);
  useFrame((_, dt) => {
    if (!kind) return;
    const set = (deltas: Record<string, THREE.Matrix4> | null) => {
      names.forEach((n, i) => {
        const m = deltas?.[n] ?? IDENTITY;
        for (const o of [meshes.current?.[i], shells.current?.[i]]) {
          if (!o) continue;
          o.matrix.copy(m);
          o.matrixWorldNeedsUpdate = true;
        }
      });
    };
    if (!enabled || !motion) {
      if (posed.current) {
        gait.current = new GaitState();
        set(null);
        bob.current?.position.set(0, 0, 0);
        posed.current = false;
      }
      return;
    }
    const s = motion.sample(performance.now());
    const g = gait.current;
    let moving: boolean;
    if (kind === "legs") {
      moving = g.step(dt, s?.speed ?? 0, s?.yawRate ?? 0);
      const pose = g.pose();
      set(spotPartDeltas(pose));
      bob.current?.position.set(0, 0, pose.bob);
    } else {
      moving = g.roll(dt, s?.speed ?? 0, HUSKY_WHEEL_R);
      set(wheelPartDeltas(g.wheel));
    }
    posed.current = true;
    if (moving || s?.moving) invalidate();
  });
}

/** ``part``'s convex hull grown by ``d`` on every side (scaled per axis about the
 *  hull's centre, so thin legs get the same margin as the body). */
export function hullShell(part: THREE.BufferGeometry, d: number): THREE.BufferGeometry {
  const pos = part.getAttribute("position");
  const pts: THREE.Vector3[] = [];
  for (let i = 0; i < pos.count; i++) pts.push(new THREE.Vector3(pos.getX(i), pos.getY(i), pos.getZ(i)));
  const hull = new ConvexGeometry(pts);
  hull.computeBoundingBox();
  const box = hull.boundingBox!;
  const c = box.getCenter(new THREE.Vector3());
  const s = box.getSize(new THREE.Vector3());
  const k = (n: number) => (n > 1e-6 ? (n + 2 * d) / n : 1);
  hull.translate(-c.x, -c.y, -c.z).scale(k(s.x), k(s.y), k(s.z)).translate(c.x, c.y, c.z);
  return hull;
}

/** ``onTop``: drawn over the scene (the mission overlay) - the model goes last
 *  (transparent pass, renderOrder from onTop) and its first part clears the depth
 *  buffer, so the model is depth-tested against itself only. */
export function RobotModel({url, color, highlight, opacity, onTop, motion}: {url: string; color: string; highlight: RobotHighlight; opacity: number; onTop?: number; motion?: PoseTrack | null}) {
  const {names, parts} = useModelParts(url);
  const meshes = useRef<(THREE.Mesh | null)[]>([]);
  const shellMeshes = useRef<(THREE.Mesh | null)[]>([]);
  const bob = useRef<THREE.Group>(null);
  useGait(names, meshes, shellMeshes, bob, motion);
  const shells = useMemo(() => (highlight === "outline" ? parts.map(p => hullShell(p, OUTLINE_M)) : []), [parts, highlight]);
  useEffect(() => () => shells.forEach(g => g.dispose()), [shells]);

  const body = useMemo(() => {
    const paint = highlight === "paint";
    const glow = highlight === "tint" ? TINT_INTENSITY : paint ? PAINT_EMISSIVE : 0;
    return new THREE.MeshStandardMaterial({
      vertexColors: !paint,
      color: paint ? color : "#ffffff",
      roughness: paint ? 0.55 : 0.7,
      metalness: 0.05,
      transparent: opacity < 1 || onTop !== undefined,
      opacity,
      emissive: glow ? new THREE.Color(color) : new THREE.Color(0, 0, 0),
      emissiveIntensity: glow,
    });
  }, [color, highlight, opacity, onTop]);
  const halo = useMemo(
    () => new THREE.MeshBasicMaterial({color, side: THREE.BackSide, transparent: opacity < 1 || onTop !== undefined, opacity}),
    [color, opacity, onTop],
  );
  useEffect(() => () => body.dispose(), [body]);
  useEffect(() => () => halo.dispose(), [halo]);

  // three calls onBeforeRender unconditionally, so it is only ever set, never undefined
  const first = onTop === undefined ? {} : {onBeforeRender: (gl: THREE.WebGLRenderer) => gl.clearDepth(), frustumCulled: false};
  // likewise renderOrder: only set on top (undefined would break three's sort)
  const order = (i: number) => (onTop === undefined ? {} : {renderOrder: i === 0 ? onTop : onTop + 1});
  // parts are placed by matrix (identity = as baked; the gait moves the legs / wheels)
  return (
    <group ref={bob}>
      {parts.map((g, i) => (
        <mesh key={`p${i}`} ref={m => void (meshes.current[i] = m)} matrixAutoUpdate={false} geometry={g} material={body} {...order(i)} {...(i === 0 ? first : {})} />
      ))}
      {shells.map((g, i) => (
        <mesh key={`h${i}`} ref={m => void (shellMeshes.current[i] = m)} matrixAutoUpdate={false} geometry={g} material={halo} {...order(i + 1)} />
      ))}
    </group>
  );
}
