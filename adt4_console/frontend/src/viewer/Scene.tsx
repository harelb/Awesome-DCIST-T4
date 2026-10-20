import {useEffect, useMemo, useRef, type CSSProperties} from "react";
import * as THREE from "three";
import {Canvas, useThree} from "@react-three/fiber";
import {CameraControls, Html, Line} from "@react-three/drei";
import CameraControlsImpl from "camera-controls";
import {useViewer, type SourceView} from "../stores/viewer";
import {frameAlignOf, sourceMatrix, useRobots} from "../stores/robots";
import {applyPoint, invertRay, poseInGlobal, robotOfSource} from "../lib/frameAlign";
import {AlignedGroup} from "./AlignedGroup";
import {layoutGraph, displayPos, type GraphLayout} from "./layout";
import {GraphLayers, pickRegistry} from "./GraphLayers";
import {pickBox, type BoxSet} from "../lib/bboxPick";
import {MeshTiles, type TileStats} from "./MeshTiles";
import {currentActionIndex, useRobotColorOf} from "./planUtil";
import {LiveOverlays} from "./LiveOverlays";
import {useViewStyle} from "../stores/viewStyle";
import {contrastFor, STYLE_BACKGROUND, type ColorOpts} from "./style";
import {captureBridge} from "./screenshot";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);


export interface SceneCallbacks {
  onNode: (src: string, sym: string) => void;
  onPoint: (p: [number, number, number]) => void;
  onMiss: () => void;
  onStats: (src: string, s: TileStats) => void;
  onLayout: (src: string, l: GraphLayout) => void;
}

/** Effective background colour (user choice or the style default). */
export function useBackground(): string {
  const style = useViewer(s => s.vizStyle);
  const bg = useViewStyle(s => s.background);
  return bg ?? STYLE_BACKGROUND[style];
}

export function useColorOpts(): ColorOpts {
  const rules = useViewStyle(s => s.colors);
  const labelColors = useViewStyle(s => s.labelColors);
  const placesByRegion = useViewStyle(s => s.regionColorPlaces);
  return useMemo(() => ({rules, labelColors, placesByRegion}), [rules, labelColors, placesByRegion]);
}

function SourceContent({src, cb}: {src: SourceView; cb: SceneCallbacks}) {
  const {hiddenLayers, edgeKinds, flat, stackHeight, showBBoxes, showTrajectory, meshOpacity, vizStyle, showText, meshCull, meshClipOn, meshClipHeight} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const regionHulls = useViewStyle(s => s.regionHulls);
  const meshColor = useViewStyle(s => s.meshColor);
  const colors = useColorOpts();
  const bg = useBackground();
  const edgeInk = contrastFor(bg, vizStyle).edge;
  const layout = useMemo(
    () => (src.model ? layoutGraph(src.model, {hiddenLayers, edgeKinds, flat, stackHeight, style: vizStyle, text: showText, heights, colors, regionHulls}) : null),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [src.model, src.rev, hiddenLayers, edgeKinds, flat, stackHeight, vizStyle, showText, heights, colors, regionHulls],
  );
  const onLayout = cb.onLayout;
  useEffect(() => {
    if (layout) onLayout(src.id, layout);
  }, [layout, src.id, onLayout]);
  const robot = robotOfSource(src.id);
  const content = (
    <group>
      {src.showMesh && src.manifest && (
        <MeshTiles
          src={src.id}
          manifest={src.manifest}
          manifestRev={src.manifestRev}
          opacity={meshOpacity}
          cullBackfaces={meshCull}
          clipHeight={meshClipOn ? meshClipHeight : null}
          colorMode={meshColor.mode}
          uniformColor={meshColor.color}
          onStats={s => cb.onStats(src.id, s)}
        />
      )}
      {src.showGraph && layout && (
        <GraphLayers
          src={src.id}
          layout={layout}
          showBBoxes={showBBoxes}
          showTrajectory={showTrajectory}
          trajectory={src.trajectory}
          edgeKinds={edgeKinds}
          style={vizStyle}
          showText={showText}
          colors={colors}
          edgeInk={edgeInk}
        />
      )}
    </group>
  );
  // a live source is in its robot's scene-graph frame: draw it in the global frame
  return robot ? <AlignedGroup robot={robot}>{content}</AlignedGroup> : content;
}

/** Click picking: nearest projected node within a few pixels, else the mesh surface. */
function Picker({cb}: {cb: SceneCallbacks}) {
  const {gl, camera, scene} = useThree();
  const cbRef = useRef(cb);
  cbRef.current = cb;
  useEffect(() => {
    const el = gl.domElement;
    let down: [number, number] | null = null;
    const pd = (e: PointerEvent) => {
      if (e.button === 0) down = [e.clientX, e.clientY];
    };
    const pu = (e: PointerEvent) => {
      if (!down || e.button !== 0) return;
      const moved = Math.hypot(e.clientX - down[0], e.clientY - down[1]);
      down = null;
      if (moved > 5) return;
      const rect = el.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const enabled = useViewer.getState().sources;
      const v = new THREE.Vector3();
      let best: {src: string; sym: string; d: number; depth: number} | null = null;
      for (const [src, layers] of pickRegistry) {
        if (!enabled[src]?.enabled || !enabled[src]?.showGraph) continue;
        // layout positions are in the source's own frame (live: robot odom)
        const m = sourceMatrix(src);
        for (const l of layers.values()) {
          const p = l.positions;
          for (let i = 0; i < l.syms.length; i++) {
            v.set(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]);
            if (m) v.applyMatrix4(m);
            v.project(camera);
            if (v.z > 1 || v.z < -1) continue;
            const sx = ((v.x + 1) / 2) * rect.width;
            const sy = ((1 - v.y) / 2) * rect.height;
            const d = Math.hypot(sx - x, sy - y);
            if (d < 10 && (!best || d < best.d - 2 || (Math.abs(d - best.d) <= 2 && v.z < best.depth))) {
              best = {src, sym: l.syms[i], d, depth: v.z};
            }
          }
        }
      }
      if (best) {
        cbRef.current.onNode(best.src, best.sym);
        return;
      }
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2((x / rect.width) * 2 - 1, -(y / rect.height) * 2 + 1), camera);
      // object bounding boxes (inspect mode, boxes shown): nodes inside a box were
      // tried first; overlapping boxes resolve to the smallest one
      const vs = useViewer.getState();
      if (vs.showBBoxes && vs.pickMode === "inspect") {
        const o = ray.ray.origin;
        const d = ray.ray.direction;
        // boxes are in each source's own frame: test the ray in that frame
        // (rigid transforms keep volumes, and ray parameters stay comparable)
        let hit: ReturnType<typeof pickBox<string>> = null;
        for (const [src, layers] of pickRegistry) {
          if (!enabled[src]?.enabled || !enabled[src]?.showGraph) continue;
          const sets: [string, BoxSet][] = [];
          for (const l of layers.values()) if (l.boxes) sets.push([src, {syms: l.syms, boxes: l.boxes}]);
          if (!sets.length) continue;
          const r = invertRay(sourceMatrix(src), [o.x, o.y, o.z], [d.x, d.y, d.z]);
          const h = pickBox(r.o, r.d, sets);
          if (h && (!hit || h.volume < hit.volume * 0.999 || (h.volume <= hit.volume * 1.001 && h.t < hit.t))) hit = h;
        }
        if (hit) {
          cbRef.current.onNode(hit.key, hit.sym);
          return;
        }
      }
      const groups: THREE.Object3D[] = [];
      scene.traverse(o => {
        if (o.userData.meshGroup && o.visible) groups.push(o);
      });
      const hit = ray.intersectObjects(groups, true)[0];
      if (hit) cbRef.current.onPoint([hit.point.x, hit.point.y, hit.point.z]);
      else {
        // no mesh: intersect the ground plane so goto picking works on graph-only sources
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const p = new THREE.Vector3();
        if (ray.ray.intersectPlane(plane, p)) cbRef.current.onPoint([p.x, p.y, p.z]);
        else cbRef.current.onMiss();
      }
    };
    el.addEventListener("pointerdown", pd);
    el.addEventListener("pointerup", pu);
    return () => {
      el.removeEventListener("pointerdown", pd);
      el.removeEventListener("pointerup", pu);
    };
  }, [gl, camera, scene]);
  return null;
}

function quatYaw(qw: number, qx: number, qy: number, qz: number) {
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

function RobotGlyphs() {
  const states = useRobots(s => s.state);
  const stateAt = useRobots(s => s.stateAt);
  const robotColor = useRobotColorOf();
  const now = Date.now();
  return (
    <>
      {Object.entries(states).map(([r, st]) => {
        const p = st.pose;
        if (!p || !Number.isFinite(p.x)) return null;
        const yaw = quatYaw(p.qw ?? 1, p.qx ?? 0, p.qy ?? 0, p.qz ?? 0);
        const color = robotColor(r);
        const stale = now - (stateAt[r] ?? 0) > 5000;
        return (
          <AlignedGroup key={r} robot={r} frame={p.frame}>
          <group position={[p.x, p.y, p.z]} rotation={[0, 0, yaw]}>
            <mesh position={[0.35, 0, 0.3]} rotation={[0, 0, -Math.PI / 2]}>
              <coneGeometry args={[0.32, 0.9, 3]} />
              <meshBasicMaterial color={color} transparent opacity={stale ? 0.4 : 1} />
            </mesh>
            <mesh position={[0, 0, 0.05]}>
              <ringGeometry args={[0.7, 0.85, 40]} />
              <meshBasicMaterial color={color} side={THREE.DoubleSide} transparent opacity={stale ? 0.3 : 0.85} />
            </mesh>
            <Html position={[0, 0, 1.4]} center zIndexRange={[20, 0]} className="robot-tag-wrap">
              <div className="robot-tag" style={{borderColor: color}}>
                {r}
                {stale ? " (stale)" : ""}
              </div>
            </Html>
          </group>
          </AlignedGroup>
        );
      })}
    </>
  );
}

// Plans come from omniplanner in the frame of the scene graph it plans on
// (relocalize / fleet: the global map), so they are drawn as is.
function PlanOverlay() {
  const plans = useRobots(s => s.plan);
  const robotColor = useRobotColorOf();
  return (
    <>
      {Object.entries(plans).map(([r, plan]) => {
        if (!plan?.plan) return null;
        const cur = currentActionIndex(plan);
        const color = robotColor(r);
        return (
          <group key={r}>
            {plan.plan.actions.map((a, i) => {
              const pts = (a.path ?? []).filter(p => p && p.length >= 3) as [number, number, number][];
              const lifted = pts.map(p => [p[0], p[1], p[2] + 0.15] as [number, number, number]);
              const state = i < cur ? "done" : i === cur ? "current" : "pending";
              return (
                <group key={i}>
                  {lifted.length > 1 && (
                    <Line
                      points={lifted}
                      color={state === "current" ? "#ffffff" : color}
                      lineWidth={state === "current" ? 5 : 2.5}
                      dashed={state === "pending"}
                      dashSize={0.4}
                      gapSize={0.25}
                      transparent
                      opacity={state === "done" ? 0.35 : 1}
                    />
                  )}
                  {state === "current" && lifted.length > 1 && (
                    <Line points={lifted} color={color} lineWidth={9} transparent opacity={0.35} />
                  )}
                  {a.point && a.point.length >= 3 && (
                    <mesh position={[a.point[0], a.point[1], a.point[2] + 0.2]}>
                      <octahedronGeometry args={[0.3]} />
                      <meshBasicMaterial color={state === "current" ? "#ffffff" : color} />
                    </mesh>
                  )}
                </group>
              );
            })}
          </group>
        );
      })}
    </>
  );
}

function Markers() {
  const {selected, highlight, sources, flat, stackHeight, gotoDraft, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  useRobots(s => s.state); // re-place markers on live sources when alignments change
  const findPos = (src: string, sym: string): [number, number, number] | null => {
    const n = sources[src]?.model?.nodes.get(sym);
    return n ? applyPoint(sourceMatrix(src), displayPos(n, flat, stackHeight, vizStyle, heights)) : null;
  };
  const sel = selected ? findPos(selected.src, selected.sym) : null;
  const selNode = selected ? sources[selected.src]?.model?.nodes.get(selected.sym) : null;
  return (
    <>
      {highlight.slice(0, 60).map(h => {
        const p = findPos(h.src, h.sym);
        if (!p) return null;
        return (
          <mesh key={h.src + h.sym} position={p}>
            <torusGeometry args={[0.55, 0.06, 6, 28]} />
            <meshBasicMaterial color="#ffe14d" depthTest={false} transparent />
          </mesh>
        );
      })}
      {sel && selNode && (
        <group position={sel}>
          <mesh>
            <torusGeometry args={[0.8, 0.09, 8, 36]} />
            <meshBasicMaterial color="#ffffff" depthTest={false} transparent />
          </mesh>
          <Html position={[0, 0, 1.2]} center zIndexRange={[30, 0]}>
            <div className="node-tag">{sources[selected!.src]?.model?.displayName(selNode)}</div>
          </Html>
        </group>
      )}
      {gotoDraft.length > 0 && (
        <>
          {gotoDraft.length > 1 && <Line points={gotoDraft.map(g => [g.pos[0], g.pos[1], g.pos[2] + 0.3] as [number, number, number])} color="#35d07f" lineWidth={2} dashed dashSize={0.3} gapSize={0.2} />}
          {gotoDraft.map((g, i) => (
            <group key={g.sym + i} position={[g.pos[0], g.pos[1], g.pos[2] + 0.3]}>
              <mesh>
                <sphereGeometry args={[0.25, 12, 8]} />
                <meshBasicMaterial color="#35d07f" />
              </mesh>
              <Html center position={[0, 0, 0.7]} zIndexRange={[25, 0]}>
                <div className="goto-tag">{i + 1}</div>
              </Html>
            </group>
          ))}
        </>
      )}
    </>
  );
}

function CameraRig({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const ref = useRef<CameraControlsImpl>(null);
  const {topDown, flyTo, follow} = useViewer();
  const fitted = useRef(false);
  // initial framing once something with bounds is loaded
  useEffect(() => {
    const c = ref.current;
    if (!c || !bounds || fitted.current) return;
    fitted.current = true;
    const cx = (bounds.min[0] + bounds.max[0]) / 2;
    const cy = (bounds.min[1] + bounds.max[1]) / 2;
    const span = Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], 10);
    const cz = bounds.min[2];
    void c.setLookAt(cx - span * 0.2, cy - span * 1.0, cz + span * 0.9, cx, cy, cz, false);
  }, [bounds]);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    if (topDown) {
      c.minPolarAngle = 0;
      c.maxPolarAngle = 0;
      c.mouseButtons.left = CameraControlsImpl.ACTION.TRUCK;
      // with the polar angle pinned straight down, orbiting only spins the map in plane
      c.mouseButtons.right = CameraControlsImpl.ACTION.ROTATE;
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_TRUCK;
      c.touches.two = CameraControlsImpl.ACTION.TOUCH_DOLLY_ROTATE;
      void c.rotatePolarTo(0, true);
      void c.rotateAzimuthTo(0, true);
    } else {
      c.minPolarAngle = 0;
      c.maxPolarAngle = Math.PI * 0.49;
      c.mouseButtons.left = CameraControlsImpl.ACTION.ROTATE;
      c.mouseButtons.right = CameraControlsImpl.ACTION.TRUCK;
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_ROTATE;
      c.touches.two = CameraControlsImpl.ACTION.TOUCH_DOLLY_TRUCK;
      void c.rotatePolarTo(Math.PI / 4, true);
    }
  }, [topDown]);
  // top view: Q / E rotate the map 90° left / right, R puts north back up
  useEffect(() => {
    if (!topDown) return;
    const onKey = (e: KeyboardEvent) => {
      const c = ref.current;
      const t = e.target as HTMLElement | null;
      if (!c || e.ctrlKey || e.metaKey || e.altKey) return;
      if (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))) return;
      const k = e.key.toLowerCase();
      if (k === "q") void c.rotate(Math.PI / 2, 0, true);
      else if (k === "e") void c.rotate(-Math.PI / 2, 0, true);
      else if (k === "r") void c.rotateAzimuthTo(0, true);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [topDown]);
  // ignore a fly-to request that predates this mount (store outlives the page)
  const flyNonce = useRef(flyTo?.nonce ?? 0);
  useEffect(() => {
    const c = ref.current;
    if (!c || !flyTo || flyTo.nonce === flyNonce.current) return;
    flyNonce.current = flyTo.nonce;
    const [x, y, z] = flyTo.target;
    const dist = Math.min(Math.max(c.distance, 6), 14);
    if (useViewer.getState().topDown) void c.setLookAt(x, y, z + dist * 1.5, x, y, z, true);
    else void c.setLookAt(x - dist * 0.3, y - dist * 0.7, z + dist * 0.65, x, y, z, true);
  }, [flyTo]);
  useEffect(() => {
    if (!follow) return;
    let last: [number, number] | null = null;
    const apply = () => {
      const raw = useRobots.getState().state[follow]?.pose;
      const c = ref.current;
      if (!raw || !c) return;
      const p = poseInGlobal(raw, frameAlignOf(follow));
      if (last && Math.hypot(p.x - last[0], p.y - last[1]) < 0.02) return;
      last = [p.x, p.y];
      void c.moveTo(p.x, p.y, p.z, true);
    };
    apply();
    return useRobots.subscribe(apply);
  }, [follow]);
  return <CameraControls ref={ref} makeDefault maxPolarAngle={Math.PI * 0.49} dollyToCursor minDistance={1} maxDistance={2000} />;
}

/** RViz look for the hydra style: grey background, light grid. */
export const BACKGROUND = STYLE_BACKGROUND;

function Background() {
  const bg = useBackground();
  const {gl, invalidate} = useThree();
  useEffect(() => {
    gl.setClearColor(bg, 1);
    invalidate();
  }, [gl, bg, invalidate]);
  return null;
}

/** Hands the renderer to the screenshot code (outside the R3F tree). */
function CaptureBridge() {
  const {gl, scene, camera, invalidate, controls} = useThree();
  useEffect(() => {
    captureBridge.current = {gl, scene, camera, invalidate, controls: controls as unknown as CameraControlsImpl | null};
    return () => {
      if (captureBridge.current?.gl === gl) captureBridge.current = null;
    };
  }, [gl, scene, camera, invalidate, controls]);
  return null;
}

function Ground({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const style = useViewer(s => s.vizStyle);
  const bg = useBackground();
  const ct = contrastFor(bg, style);
  const size = bounds ? Math.ceil(Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], 20) / 10) * 10 + 40 : 100;
  const cx = bounds ? Math.round((bounds.min[0] + bounds.max[0]) / 2) : 0;
  const cy = bounds ? Math.round((bounds.min[1] + bounds.max[1]) / 2) : 0;
  const grid = useMemo(() => {
    const g = new THREE.GridHelper(size, size / 2, ct.grid[0], ct.grid[1]);
    g.rotation.x = Math.PI / 2;
    (g.material as THREE.Material).transparent = true;
    (g.material as THREE.Material).opacity = ct.gridOpacity;
    return g;
  }, [size, ct.grid[0], ct.grid[1], ct.gridOpacity]);
  return <primitive object={grid} position={[cx, cy, -0.05]} />;
}

// Phones / tablets: tile GPUs with little memory. Cap the pixel ratio, skip
// MSAA and let the browser discard the drawing buffer (keeping it costs a copy
// per frame there). Desktop keeps it so headless screenshot tours see pixels.
const COARSE = typeof window !== "undefined" && window.matchMedia?.("(pointer: coarse)").matches === true;
const DPR: [number, number] = COARSE ? [1, 1.5] : [1, 2];
const GL = {antialias: !COARSE, powerPreference: "high-performance" as const, preserveDrawingBuffer: !COARSE};

export function Scene({sources, bounds, cb}: {sources: SourceView[]; bounds: {min: number[]; max: number[]} | null; cb: SceneCallbacks}) {
  const bg = useBackground();
  const style = useViewer(s => s.vizStyle);
  const ct = contrastFor(bg, style);
  return (
    <Canvas
      className="scene-canvas"
      style={{"--viz-text": ct.text, "--viz-halo": ct.halo} as CSSProperties}
      frameloop="demand"
      dpr={DPR}
      gl={GL}
      camera={{fov: 50, near: 0.1, far: 5000, position: [-10, -30, 30], up: [0, 0, 1]}}
      onCreated={({gl}) => {
        gl.setClearColor(useViewStyle.getState().background ?? BACKGROUND[useViewer.getState().vizStyle]);
        gl.localClippingEnabled = true; // mesh "max height" clip
      }}
      data-testid="viewer-canvas"
    >
      <ambientLight intensity={0.85} />
      <directionalLight position={[30, -20, 60]} intensity={1.0} />
      <directionalLight position={[-40, 30, 20]} intensity={0.35} />
      <Background />
      <CaptureBridge />
      <Ground bounds={bounds} />
      {sources.map(s => (
        <SourceContent key={s.id} src={s} cb={cb} />
      ))}
      <LiveOverlays />
      <RobotGlyphs />
      <PlanOverlay />
      <Markers />
      <Picker cb={cb} />
      <CameraRig bounds={bounds} />
    </Canvas>
  );
}
