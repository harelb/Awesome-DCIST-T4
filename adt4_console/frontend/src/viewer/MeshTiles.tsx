// Progressive mesh-tile streaming: one BufferGeometry per tile, fetched nearest
// the camera first with a concurrency limit; tiles whose hash changed are refetched.
//
// Ceilings: RViz (kimera_pgmo_rviz MeshDisplay, `Cull Backfaces: true`) hides them
// because ceiling triangles face down into the room, so from above only their
// back faces show. Same here: FrontSide material (three.js and kimera_pgmo both
// wind front faces counter-clockwise). An optional clipping plane cuts the mesh
// above a height over the floor for views from inside or at grazing angles.
import {useEffect, useRef, useState} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {decodeTile} from "../codec";
import {getMsgpack} from "../lib/api";
import type {TileManifest} from "../types";
import {srcPath} from "../stores/viewer";
import type {MeshColorMode} from "./style";

/** GLSL turbo-like colormap (polynomial fit, Google "Turbo"), t in 0..1 -> sRGB. */
const TURBO_GLSL = `
vec3 adt4Turbo(float t) {
  t = clamp(t, 0.0, 1.0);
  const vec4 kr = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const vec4 kg = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const vec4 kb = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const vec2 kr2 = vec2(-152.94239396, 59.28637943);
  const vec2 kg2 = vec2(4.27729857, 2.82956604);
  const vec2 kb2 = vec2(-89.90310912, 27.34824973);
  vec4 v4 = vec4(1.0, t, t * t, t * t * t);
  vec2 v2 = v4.zw * v4.z;
  return vec3(dot(v4, kr) + dot(v2, kr2), dot(v4, kg) + dot(v2, kg2), dot(v4, kb) + dot(v2, kb2));
}
`;
const MESH_MODE: Record<MeshColorMode, number> = {rgb: 0, uniform: 1, height: 2};

const CONCURRENCY = 6;

export interface TileStats {
  loaded: number;
  total: number;
  triangles: number;
}

class TileLoader {
  group = new THREE.Group();
  private have = new Map<string, {hash: string; mesh: THREE.Mesh}>();
  private queue = new Map<string, {hash: string; center: THREE.Vector3}>();
  private inflight = new Set<string>();
  private disposed = false;
  material: THREE.MeshLambertMaterial;
  /** keeps z <= floor + height when clipping is on, in the tiles' own frame */
  localClip = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
  /** `localClip` in world space (three.js clipping planes are world space; a
   *  live source's group carries its robot's global-frame alignment) */
  clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0);
  clipHeight: number | null = null;
  private tileMinZ = new Map<string, number>();
  private tileMaxZ = new Map<string, number>();
  /** shader uniforms for the colour mode (rgb / uniform / height colormap) */
  uniforms = {
    uMeshMode: {value: 0},
    uMeshColor: {value: new THREE.Color("#b0b0b0")},
    uZMin: {value: 0},
    uZMax: {value: 3},
  };
  triangles = 0;
  total = 0;

  constructor(
    private src: string,
    private camTarget: () => THREE.Vector3,
    private onChange: () => void,
  ) {
    this.material = new THREE.MeshLambertMaterial({vertexColors: true, side: THREE.FrontSide});
    // tile colours are sRGB bytes; three treats vertex colours as linear, so decode here
    // colour mode: 0 vertex RGB, 1 uniform colour, 2 height colormap (world z)
    this.material.onBeforeCompile = shader => {
      Object.assign(shader.uniforms, this.uniforms);
      shader.vertexShader =
        "uniform int uMeshMode;\nuniform vec3 uMeshColor;\nuniform float uZMin;\nuniform float uZMax;\n" +
        TURBO_GLSL +
        shader.vertexShader.replace(
          "#include <color_vertex>",
          [
            "#include <color_vertex>",
            "#ifdef USE_COLOR",
            "  if (uMeshMode == 1) vColor.rgb = uMeshColor;",
            "  else if (uMeshMode == 2) vColor.rgb = pow(adt4Turbo(((modelMatrix * vec4(position, 1.0)).z - uZMin) / max(uZMax - uZMin, 1e-3)), vec3(2.2));",
            "  else vColor.rgb = pow(vColor.rgb, vec3(2.2));",
            "#endif",
          ].join("\n"),
        );
    };
    this.group.name = `mesh:${src}`;
  }

  setManifest(m: TileManifest | null) {
    const tiles = m?.tiles ?? {};
    const size = m?.tile_size ?? 8;
    this.total = Object.keys(tiles).length;
    // removals
    for (const [k, t] of this.have) {
      if (!tiles[k]) {
        this.drop(k, t.mesh);
      }
    }
    for (const k of [...this.queue.keys()]) if (!tiles[k]) this.queue.delete(k);
    // additions / changes
    for (const [k, t] of Object.entries(tiles)) {
      const cur = this.have.get(k);
      if (cur && cur.hash === t.hash) continue;
      const o = t.origin ?? (k.split("_").map(v => Number(v) * size) as [number, number, number]);
      this.queue.set(k, {hash: t.hash, center: new THREE.Vector3(o[0] + size / 2, o[1] + size / 2, o[2] + size / 2)});
    }
    this.onChange();
    this.pump();
  }

  setColorMode(mode: MeshColorMode, color: string) {
    this.uniforms.uMeshMode.value = MESH_MODE[mode] ?? 0;
    this.uniforms.uMeshColor.value.set(color);
    this.updateZRange();
  }

  /** Height colormap range: floor .. a high percentile of tile tops (clip height when clipping). */
  private updateZRange() {
    const lo = this.floorZ();
    const tops = [...this.tileMaxZ.values()].sort((a, b) => a - b);
    let hi = tops.length ? tops[Math.floor(tops.length * 0.9)] : lo + 3;
    if (this.clipHeight !== null) hi = Math.min(hi, lo + this.clipHeight);
    this.uniforms.uZMin.value = lo;
    this.uniforms.uZMax.value = Math.max(hi, lo + 0.5);
  }

  /** Floor height: low percentile of the tiles' lowest vertices (robust to pits). */
  floorZ(): number {
    const zs = [...this.tileMinZ.values()].sort((a, b) => a - b);
    return zs.length ? zs[Math.floor(zs.length * 0.1)] : 0;
  }

  /** Re-derive the world clip plane from the group's current world matrix. */
  syncClip() {
    this.group.updateWorldMatrix(true, false);
    this.clipPlane.copy(this.localClip).applyMatrix4(this.group.matrixWorld);
  }

  setCull(on: boolean) {
    const side = on ? THREE.FrontSide : THREE.DoubleSide;
    if (this.material.side !== side) {
      this.material.side = side;
      this.material.needsUpdate = true;
    }
  }

  setClip(height: number | null) {
    this.clipHeight = height;
    this.updateClip();
    this.updateZRange();
  }

  private updateClip() {
    if (this.clipHeight === null) {
      if (this.material.clippingPlanes?.length) {
        this.material.clippingPlanes = [];
        this.material.needsUpdate = true;
      }
      return;
    }
    this.localClip.constant = this.floorZ() + this.clipHeight;
    this.syncClip();
    if (!this.material.clippingPlanes?.length) {
      this.material.clippingPlanes = [this.clipPlane];
      this.material.needsUpdate = true;
    }
  }

  private drop(k: string, mesh: THREE.Mesh) {
    this.tileMinZ.delete(k);
    this.tileMaxZ.delete(k);
    this.group.remove(mesh);
    this.triangles -= mesh.geometry.index ? mesh.geometry.index.count / 3 : 0;
    mesh.geometry.dispose();
    this.have.delete(k);
  }

  private pump() {
    while (!this.disposed && this.inflight.size < CONCURRENCY && this.queue.size) {
      const target = this.camTarget();
      let best: string | null = null;
      let bestD = Infinity;
      for (const [k, q] of this.queue) {
        if (this.inflight.has(k)) continue;
        const d = q.center.distanceToSquared(target);
        if (d < bestD) {
          bestD = d;
          best = k;
        }
      }
      if (!best) break;
      const {hash} = this.queue.get(best)!;
      this.queue.delete(best);
      this.inflight.add(best);
      void this.fetch(best, hash);
    }
  }

  private async fetch(key: string, hash: string) {
    try {
      const raw = await getMsgpack(`${srcPath(this.src)}/mesh/tile/${encodeURIComponent(key)}`);
      if (this.disposed) return;
      const t = decodeTile(raw);
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(t.positions, 3));
      g.setAttribute("color", new THREE.BufferAttribute(t.colors, 3, true));
      g.setIndex(new THREE.BufferAttribute(t.indices, 1));
      g.computeVertexNormals();
      g.computeBoundingSphere();
      g.computeBoundingBox();
      const mesh = new THREE.Mesh(g, this.material);
      mesh.name = `tile:${key}`;
      mesh.userData.tileKey = key;
      mesh.matrixAutoUpdate = false;
      const old = this.have.get(key);
      if (old) this.drop(key, old.mesh);
      this.group.add(mesh);
      this.have.set(key, {hash: t.hash || hash, mesh});
      if (g.boundingBox && Number.isFinite(g.boundingBox.min.z)) {
        this.tileMinZ.set(key, g.boundingBox.min.z);
        this.tileMaxZ.set(key, g.boundingBox.max.z);
      }
      this.updateZRange();
      if (this.clipHeight !== null) this.updateClip();
      this.triangles += t.indices.length / 3;
      this.onChange();
    } catch {
      // leave it: a later manifest refresh re-queues it
    } finally {
      this.inflight.delete(key);
      if (!this.disposed) this.pump();
    }
  }

  stats(): TileStats {
    return {loaded: this.have.size, total: this.total, triangles: this.triangles};
  }

  dispose() {
    this.disposed = true;
    for (const [k, t] of this.have) this.drop(k, t.mesh);
    this.material.dispose();
  }
}

export function MeshTiles({src, manifest, manifestRev, opacity, cullBackfaces = true, clipHeight = null, colorMode = "rgb", uniformColor = "#b0b0b0", onStats}: {
  src: string;
  manifest: TileManifest | null;
  manifestRev: number;
  opacity: number;
  /** RViz MeshDisplay `Cull Backfaces` */
  cullBackfaces?: boolean;
  /** metres above the floor; null = no clipping */
  clipHeight?: number | null;
  colorMode?: MeshColorMode;
  uniformColor?: string;
  onStats?: (s: TileStats) => void;
}) {
  const {controls, invalidate, camera} = useThree() as unknown as {
    controls: {getTarget?: (v: THREE.Vector3) => THREE.Vector3} | null;
    invalidate: () => void;
    camera: THREE.Camera;
  };
  const ctlRef = useRef(controls);
  ctlRef.current = controls;
  const statsRef = useRef(onStats);
  statsRef.current = onStats;
  // Created in an effect (not useMemo) so StrictMode's mount/unmount/mount cycle
  // disposes the first loader and builds a fresh one instead of reusing a dead one.
  const [loader, setLoader] = useState<TileLoader | null>(null);
  useEffect(() => {
    const l: TileLoader = new TileLoader(
      src,
      () => {
        const c = ctlRef.current;
        const t = c?.getTarget ? c.getTarget(new THREE.Vector3()) : camera.position.clone();
        // tiles are in the source's own frame (live: the robot's odom frame)
        l.group.updateWorldMatrix(true, false);
        return l.group.worldToLocal(t);
      },
      () => {
        invalidate();
        statsRef.current?.(l.stats());
      },
    );
    l.group.userData.meshGroup = true;
    setLoader(l);
    return () => {
      l.dispose();
      invalidate();
    };
  }, [src, camera, invalidate]);
  useEffect(() => {
    loader?.setManifest(manifest);
  }, [loader, manifest, manifestRev]);
  useEffect(() => {
    if (!loader) return;
    loader.material.transparent = opacity < 1;
    loader.material.opacity = opacity;
    loader.material.depthWrite = opacity >= 0.5;
    loader.material.needsUpdate = true;
    invalidate();
  }, [loader, opacity, invalidate]);
  useEffect(() => {
    if (!loader) return;
    loader.setCull(cullBackfaces);
    loader.setClip(clipHeight);
    invalidate();
  }, [loader, cullBackfaces, clipHeight, invalidate]);
  useEffect(() => {
    if (!loader) return;
    loader.setColorMode(colorMode, uniformColor);
    invalidate();
  }, [loader, colorMode, uniformColor, invalidate]);
  // follow the alignment group (it glides on loop closures) with the clip plane
  useFrame(() => {
    if (loader && loader.clipHeight !== null) loader.syncClip();
  });
  if (!loader) return null;
  // picking is done by viewer/Scene.tsx Picker (on click only), so no R3F handlers here:
  // per-pointermove raycasts against ~1.6M triangles would stall the frame.
  return <primitive object={loader.group} />;
}
