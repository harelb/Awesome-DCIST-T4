import {useEffect, useMemo, useRef} from "react";
import * as THREE from "three";
import {Canvas, useThree} from "@react-three/fiber";
import {CameraControls, Html, Line} from "@react-three/drei";
import CameraControlsImpl from "camera-controls";
import {useViewer, type SourceView} from "../stores/viewer";
import {useRobots} from "../stores/robots";
import {layoutGraph, displayPos, type GraphLayout} from "./layout";
import {GraphLayers, pickRegistry} from "./GraphLayers";
import {MeshTiles, type TileStats} from "./MeshTiles";
import {currentActionIndex, robotColor} from "./planUtil";
import {LiveOverlays} from "./LiveOverlays";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);


export interface SceneCallbacks {
  onNode: (src: string, sym: string) => void;
  onPoint: (p: [number, number, number]) => void;
  onMiss: () => void;
  onStats: (src: string, s: TileStats) => void;
  onLayout: (src: string, l: GraphLayout) => void;
}

function SourceContent({src, cb}: {src: SourceView; cb: SceneCallbacks}) {
  const {hiddenLayers, edgeKinds, flat, stackHeight, showBBoxes, showTrajectory, meshOpacity} = useViewer();
  const layout = useMemo(
    () => (src.model ? layoutGraph(src.model, {hiddenLayers, edgeKinds, flat, stackHeight}) : null),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [src.model, src.rev, hiddenLayers, edgeKinds, flat, stackHeight],
  );
  const onLayout = cb.onLayout;
  useEffect(() => {
    if (layout) onLayout(src.id, layout);
  }, [layout, src.id, onLayout]);
  return (
    <group>
      {src.showMesh && src.manifest && (
        <MeshTiles src={src.id} manifest={src.manifest} manifestRev={src.manifestRev} opacity={meshOpacity} onStats={s => cb.onStats(src.id, s)} />
      )}
      {src.showGraph && layout && (
        <GraphLayers src={src.id} layout={layout} showBBoxes={showBBoxes} showTrajectory={showTrajectory} trajectory={src.trajectory} edgeKinds={edgeKinds} />
      )}
    </group>
  );
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
        for (const l of layers.values()) {
          const p = l.positions;
          for (let i = 0; i < l.syms.length; i++) {
            v.set(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]).project(camera);
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
          <group key={r} position={[p.x, p.y, p.z]} rotation={[0, 0, yaw]}>
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
        );
      })}
    </>
  );
}

function PlanOverlay() {
  const plans = useRobots(s => s.plan);
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
  const {selected, highlight, sources, flat, stackHeight, gotoDraft} = useViewer();
  const findPos = (src: string, sym: string): [number, number, number] | null => {
    const n = sources[src]?.model?.nodes.get(sym);
    return n ? displayPos(n, flat, stackHeight) : null;
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
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_TRUCK;
      void c.rotatePolarTo(0, true);
      void c.rotateAzimuthTo(0, true);
    } else {
      c.minPolarAngle = 0;
      c.maxPolarAngle = Math.PI * 0.49;
      c.mouseButtons.left = CameraControlsImpl.ACTION.ROTATE;
      c.touches.one = CameraControlsImpl.ACTION.TOUCH_ROTATE;
      void c.rotatePolarTo(Math.PI / 4, true);
    }
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
      const p = useRobots.getState().state[follow]?.pose;
      const c = ref.current;
      if (!p || !c) return;
      if (last && Math.hypot(p.x - last[0], p.y - last[1]) < 0.02) return;
      last = [p.x, p.y];
      void c.moveTo(p.x, p.y, p.z, true);
    };
    apply();
    return useRobots.subscribe(apply);
  }, [follow]);
  return <CameraControls ref={ref} makeDefault maxPolarAngle={Math.PI * 0.49} dollyToCursor minDistance={1} maxDistance={2000} />;
}

function Ground({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const size = bounds ? Math.ceil(Math.max(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], 20) / 10) * 10 + 40 : 100;
  const cx = bounds ? Math.round((bounds.min[0] + bounds.max[0]) / 2) : 0;
  const cy = bounds ? Math.round((bounds.min[1] + bounds.max[1]) / 2) : 0;
  const grid = useMemo(() => {
    const g = new THREE.GridHelper(size, size / 2, "#2a3440", "#1a222b");
    g.rotation.x = Math.PI / 2;
    (g.material as THREE.Material).transparent = true;
    (g.material as THREE.Material).opacity = 0.6;
    return g;
  }, [size]);
  return <primitive object={grid} position={[cx, cy, -0.05]} />;
}

// Phones / tablets: tile GPUs with little memory. Cap the pixel ratio, skip
// MSAA and let the browser discard the drawing buffer (keeping it costs a copy
// per frame there). Desktop keeps it so headless screenshot tours see pixels.
const COARSE = typeof window !== "undefined" && window.matchMedia?.("(pointer: coarse)").matches === true;
const DPR: [number, number] = COARSE ? [1, 1.5] : [1, 2];
const GL = {antialias: !COARSE, powerPreference: "high-performance" as const, preserveDrawingBuffer: !COARSE};

export function Scene({sources, bounds, cb}: {sources: SourceView[]; bounds: {min: number[]; max: number[]} | null; cb: SceneCallbacks}) {
  return (
    <Canvas
      className="scene-canvas"
      frameloop="demand"
      dpr={DPR}
      gl={GL}
      camera={{fov: 50, near: 0.1, far: 5000, position: [-10, -30, 30], up: [0, 0, 1]}}
      onCreated={({gl}) => gl.setClearColor("#0b0f14")}
      data-testid="viewer-canvas"
    >
      <ambientLight intensity={0.85} />
      <directionalLight position={[30, -20, 60]} intensity={1.0} />
      <directionalLight position={[-40, 30, 20]} intensity={0.35} />
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
