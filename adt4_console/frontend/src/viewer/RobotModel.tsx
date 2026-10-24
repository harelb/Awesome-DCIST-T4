// A robot's platform model (lib/robotModel) in its pose frame, optionally
// highlighted in the robot's colour: an outline (each URDF part's convex hull,
// grown by OUTLINE_M and drawn back faces only, so it shows as a halo around the
// part) or an emissive tint. Hulls, not the parts themselves: the decimated parts
// are many small disconnected pieces, and a shell grown from them tears apart.
import {Component, useEffect, useMemo, type ReactNode} from "react";
import * as THREE from "three";
import {useLoader} from "@react-three/fiber";
import {GLTFLoader} from "three/examples/jsm/loaders/GLTFLoader.js";
import {ConvexGeometry} from "three/examples/jsm/geometries/ConvexGeometry.js";
import {OUTLINE_M, TINT_INTENSITY, type RobotHighlight} from "../lib/robotModel";

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

/** The model's parts with their transforms baked in (one-time per URL). */
function useModelParts(url: string): THREE.BufferGeometry[] {
  const gltf = useLoader(GLTFLoader, url);
  return useMemo(() => {
    const out: THREE.BufferGeometry[] = [];
    gltf.scene.updateMatrixWorld(true);
    gltf.scene.traverse(o => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      const g = m.geometry.clone();
      g.applyMatrix4(m.matrixWorld);
      if (!g.getAttribute("normal")) g.computeVertexNormals();
      out.push(g);
    });
    return out;
  }, [gltf]);
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

export function RobotModel({url, color, highlight, opacity}: {url: string; color: string; highlight: RobotHighlight; opacity: number}) {
  const parts = useModelParts(url);
  const shells = useMemo(() => (highlight === "outline" ? parts.map(p => hullShell(p, OUTLINE_M)) : []), [parts, highlight]);
  useEffect(() => () => shells.forEach(g => g.dispose()), [shells]);

  const body = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.7,
        metalness: 0.05,
        transparent: opacity < 1,
        opacity,
        emissive: highlight === "tint" ? new THREE.Color(color) : new THREE.Color(0, 0, 0),
        emissiveIntensity: highlight === "tint" ? TINT_INTENSITY : 0,
      }),
    [color, highlight, opacity],
  );
  const halo = useMemo(() => new THREE.MeshBasicMaterial({color, side: THREE.BackSide, transparent: opacity < 1, opacity}), [color, opacity]);
  useEffect(() => () => body.dispose(), [body]);
  useEffect(() => () => halo.dispose(), [halo]);

  return (
    <group>
      {parts.map((g, i) => (
        <mesh key={`p${i}`} geometry={g} material={body} />
      ))}
      {shells.map((g, i) => (
        <mesh key={`h${i}`} geometry={g} material={halo} />
      ))}
    </group>
  );
}
