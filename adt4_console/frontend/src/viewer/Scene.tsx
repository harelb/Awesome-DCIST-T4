import {Suspense, useEffect, useMemo, useRef, type CSSProperties} from "react";
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
import {DETAIL_LEVEL} from "../lib/meshPlan";
import {currentActionIndex, distinctPoints} from "./planUtil";
import {LiveOverlays} from "./LiveOverlays";
import {useViewStyle} from "../stores/viewStyle";
import {contrastFor, defaultBackground, STYLE_BACKGROUND, type ColorOpts} from "./style";
import {useTheme} from "../stores/theme";
import {useBackground, useRobotColor3D} from "./background";
import {captureBridge} from "./screenshot";
import {DimGroup, useFocusLook} from "./DimGroup";
import {ModelBoundary, RobotModel} from "./RobotModel";
import {usePlatform} from "./usePlatform";
import {robotModelFor} from "../lib/robotModel";
import {PoseTrack} from "../lib/gait";
import {SatelliteGround} from "./SatelliteGround";
import {agentColors, MapHover, MissionOverlay} from "./MissionOverlay";
import {pickRadius, TapTracker} from "../lib/tap";
import {useMission} from "../stores/mission";
import {usePlanPreview} from "../stores/planPreview";
import {EditOverlay} from "./EditOverlay";
import {fitTopView} from "../lib/fitView";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);


export interface SceneCallbacks {
  onNode: (src: string, sym: string) => void;
  onPoint: (p: [number, number, number]) => void;
  onMiss: () => void;
  onStats: (src: string, s: TileStats) => void;
  onLayout: (src: string, l: GraphLayout) => void;
}

export {useBackground};

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
  const meshLevel = useViewer(s => DETAIL_LEVEL[s.meshDetail === "auto" ? s.meshAutoDetail : s.meshDetail]);
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
          detail={meshLevel}
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
          trajectoryRobots={src.trajectoryRobots}
          edgeKinds={edgeKinds}
          style={vizStyle}
          showText={showText}
          colors={colors}
          edgeInk={edgeInk}
        />
      )}
    </group>
  );
  // a live source is in its robot's scene-graph frame: draw it in the global
  // frame; dimmed (or dropped) while another robot is in focus
  return robot ? (
    <DimGroup robot={robot}>
      <AlignedGroup robot={robot}>{content}</AlignedGroup>
    </DimGroup>
  ) : (
    content
  );
}

/** Click picking: nearest projected node within a few pixels, else the mesh surface. */
function Picker({cb}: {cb: SceneCallbacks}) {
  const {gl, camera, scene} = useThree();
  const cbRef = useRef(cb);
  cbRef.current = cb;
  useEffect(() => {
    const el = gl.domElement;
    // tap vs drag / pinch (lib/tap.ts): only a tap picks or places a corner
    const taps = new TapTracker();
    const pd = (e: PointerEvent) => taps.onDown(e);
    const pc = (e: PointerEvent) => taps.onCancel(e);
    const pu = (e: PointerEvent) => {
      if (!taps.onUp(e)) return;
      const radius = pickRadius(e.pointerType);
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
            if (d < radius && (!best || d < best.d - 2 || (Math.abs(d - best.d) <= 2 && v.z < best.depth))) {
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
      if (vs.showBBoxes && (vs.pickMode === "inspect" || vs.pickMode === "edit")) {
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
    el.addEventListener("pointercancel", pc);
    return () => {
      el.removeEventListener("pointerdown", pd);
      el.removeEventListener("pointerup", pu);
      el.removeEventListener("pointercancel", pc);
    };
  }, [gl, camera, scene]);
  return null;
}

function quatYaw(qw: number, qx: number, qy: number, qz: number) {
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

function RobotGlyph({robot, children}: {robot: string; children: React.ReactNode}) {
  return <DimGroup robot={robot}>{children}</DimGroup>;
}

function GlyphTag({robot, color, stale}: {robot: string; color: string; stale: boolean}) {
  const look = useFocusLook(robot);
  const focused = useViewer(s => s.focus) === robot;
  return (
    <div className={`robot-tag ${look === "dim" ? "dim" : ""} ${focused ? "focused" : ""}`} style={{borderColor: color}}>
      {robot}
      {stale ? " (stale)" : ""}
    </div>
  );
}

function ArrowGlyph({color, stale}: {color: string; stale: boolean}) {
  return (
    <mesh position={[0.35, 0, 0.3]} rotation={[0, 0, -Math.PI / 2]}>
      <coneGeometry args={[0.32, 0.9, 3]} />
      <meshBasicMaterial color={color} transparent opacity={stale ? 0.4 : 1} />
    </mesh>
  );
}

/** The robot in its pose frame: its platform's model (the arrow while the model
 *  loads, or when there is none / the view asks for arrows), a ground ring and its
 *  name tag, all in the robot's colour. The model walks at the ground speed of its
 *  pose updates (``x``, ``y``, ``yaw`` at ``at`` ms; the drawn pose is not interpolated). */
function RobotBody({robot, color, stale, x, y, yaw, at}: {robot: string; color: string; stale: boolean; x: number; y: number; yaw: number; at: number}) {
  const platform = usePlatform(robot);
  const motion = useMemo(() => new PoseTrack(), []);
  const invalidate = useThree(s => s.invalidate);
  useEffect(() => {
    // fleetlink re-sends the state at 4 Hz: an unchanged pose is a standing sample
    if (stale) motion.reset();
    else motion.push(performance.now(), x, y, yaw);
    invalidate();
  }, [motion, x, y, yaw, at, stale, invalidate]);
  const mode = useViewStyle(s => s.robotModel);
  const highlight = useViewStyle(s => s.robotHighlight);
  const spec = mode === "model" ? robotModelFor(platform) : null;
  const groundZ = spec ? spec.groundZ + 0.02 : 0.05;
  const ring = spec?.ring ?? 0.78;
  return (
    <>
      {spec ? (
        <ModelBoundary fallback={<ArrowGlyph color={color} stale={stale} />}>
          <Suspense fallback={<ArrowGlyph color={color} stale={stale} />}>
            <RobotModel url={spec.url} color={color} highlight={highlight} opacity={stale ? 0.45 : 1} motion={motion} />
          </Suspense>
        </ModelBoundary>
      ) : (
        <ArrowGlyph color={color} stale={stale} />
      )}
      <mesh position={[0, 0, groundZ]}>
        <ringGeometry args={[ring - 0.08, ring + 0.07, 40]} />
        <meshBasicMaterial color={color} side={THREE.DoubleSide} transparent opacity={stale ? 0.3 : 0.85} />
      </mesh>
      <Html position={[0, 0, (spec?.topZ ?? 0.6) + 0.8]} center zIndexRange={[20, 0]} className="robot-tag-wrap">
        <GlyphTag robot={robot} color={color} stale={stale} />
      </Html>
    </>
  );
}

function RobotGlyphs() {
  const states = useRobots(s => s.state);
  const stateAt = useRobots(s => s.stateAt);
  const robotColor = useRobotColor3D();
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
          <RobotGlyph key={r} robot={r}>
          <AlignedGroup robot={r} frame={p.frame}>
          <group position={[p.x, p.y, p.z]} rotation={[0, 0, yaw]}>
            <RobotBody robot={r} color={color} stale={stale} x={p.x} y={p.y} yaw={yaw} at={stateAt[r] ?? 0} />
          </group>
          </AlignedGroup>
          </RobotGlyph>
        );
      })}
    </>
  );
}

// Plans come from omniplanner in the frame of the scene graph it plans on
// (relocalize / fleet: the global map), so they are drawn as is. Every part is
// in the robot's colour (agent trails use it too, so a plan is wider, outlined
// in black and drawn over the mesh / graph): done steps faded, the current one
// solid and widest, pending ones dashed; a ring marks where the plan ends.
const PLAN_Z = 0.25;
const PLAN_ORDER = 20;
// above the mission overlay (30..): a preview is looked at while it is shown
const PREVIEW_ORDER = 45;
function PlanOverlay() {
  const plans = useRobots(s => s.plan);
  const robotColor = useRobotColor3D();
  return (
    <>
      {Object.entries(plans).map(([r, plan]) => {
        if (!plan?.plan) return null;
        const cur = currentActionIndex(plan);
        const color = robotColor(r);
        const paths = plan.plan.actions.map(a =>
          distinctPoints(((a.path ?? []).filter(p => p && p.length >= 3) as [number, number, number][]).map(p => [p[0], p[1], p[2] + PLAN_Z] as [number, number, number])),
        );
        const lastPath = [...paths].reverse().find(p => p.length > 0);
        const end = lastPath?.[lastPath.length - 1];
        return (
          <DimGroup key={r} robot={r}>
            {plan.plan.actions.map((a, i) => {
              const lifted = paths[i];
              const state = i < cur ? "done" : i === cur ? "current" : "pending";
              const width = state === "current" ? 7 : state === "pending" ? 4 : 3;
              return (
                <group key={i}>
                  {lifted.length > 1 && (
                    <>
                      <Line points={lifted} color="#000000" lineWidth={width + 4} transparent opacity={state === "done" ? 0.2 : 0.55} depthTest={false} renderOrder={PLAN_ORDER} />
                      <Line
                        points={lifted}
                        color={color}
                        lineWidth={width}
                        dashed={state === "pending"}
                        dashSize={0.6}
                        gapSize={0.3}
                        transparent
                        opacity={state === "done" ? 0.35 : 1}
                        depthTest={false}
                        renderOrder={PLAN_ORDER + 1}
                      />
                    </>
                  )}
                  {a.point && a.point.length >= 3 && (
                    <mesh position={[a.point[0], a.point[1], a.point[2] + PLAN_Z]} renderOrder={PLAN_ORDER + 2}>
                      <octahedronGeometry args={[state === "current" ? 0.45 : 0.3]} />
                      <meshBasicMaterial color={color} depthTest={false} transparent opacity={state === "done" ? 0.35 : 1} />
                    </mesh>
                  )}
                </group>
              );
            })}
            {end && cur < plan.plan.actions.length && (
              <mesh position={end} renderOrder={PLAN_ORDER + 2}>
                <ringGeometry args={[0.45, 0.75, 32]} />
                <meshBasicMaterial color={color} depthTest={false} transparent opacity={0.9} side={THREE.DoubleSide} />
              </mesh>
            )}
          </DimGroup>
        );
      })}
    </>
  );
}

// A previewed PDDL plan (not sent yet): each robot's route dashed and thinner
// than a live plan, a hollow ring where it ends; flat on the ground like the
// mission overlay's routes.
function PreviewOverlay({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const preview = usePlanPreview(s => s.preview);
  const robotColor = useRobotColor3D();
  const roster = useMission(s => s.roster);
  const live = useRobots(s => s.state);
  const z0 = useMission(s => s.geo?.anchor.z0);
  // the mission overlay's route height
  const z = (z0 ?? (bounds ? bounds.min[2] : 0)) + 0.35;
  const invalidate = useThree(s => s.invalidate);
  useEffect(() => invalidate(), [preview, invalidate]); // the canvas renders on demand
  // stable point arrays: drei's Line rebuilds (and disposes) its geometry and
  // material whenever `points` is a new array
  const routes = useMemo(
    () =>
      Object.entries(preview?.routes ?? {})
        .map(([r, route]) => [r, distinctPoints(route.filter(p => p.length >= 2).map(p => [p[0], p[1], z] as [number, number, number]))] as const)
        .filter(([, pts]) => pts.length >= 2),
    [preview, z],
  );
  // the robot's colour when it is connected, else its mission agent's (as the
  // mission overlay draws it), else one of the overlay's palette
  const colors = useMemo(
    () =>
      // the roster first: the palette goes by position, as the mission overlay's does
      agentColors(
        [
          ...roster,
          ...routes
            .filter(([r]) => !roster.some(a => a.id === r))
            .map(([r]) => ({id: r, kind: "spot" as const, binding: live[r] ? r : null})),
        ],
        robotColor,
      ),
    [routes, roster, live, robotColor],
  );
  if (!preview) return null;
  return (
    <>
      {routes.map(([r, pts]) => {
        const color = colors[r];
        return (
          <group key={r}>
            {/* a light halo: the routes cross dense, dark graph edges */}
            <Line points={pts} color="#ffffff" lineWidth={11} transparent opacity={0.8} depthTest={false} renderOrder={PREVIEW_ORDER} />
            {/* transparent too, so it is drawn after the halo (same pass, higher order) */}
            <Line points={pts} color={color} lineWidth={6} dashed dashSize={1.2} gapSize={0.6} transparent opacity={1} depthTest={false} renderOrder={PREVIEW_ORDER + 1} />
            <mesh position={pts[pts.length - 1]} renderOrder={PREVIEW_ORDER + 2}>
              <ringGeometry args={[0.8, 1.3, 32]} />
              <meshBasicMaterial color={color} depthTest={false} transparent opacity={0.9} side={THREE.DoubleSide} />
            </mesh>
          </group>
        );
      })}
    </>
  );
}

/** Marker colours that stand out on the current background: white / yellow on
 *  dark (the historical look), near-black / amber on a light background. */
function useMarkerInk(): {hi: string; search: string} {
  const light = contrastFor(useBackground()).light;
  return light ? {hi: "#111111", search: "#e08a00"} : {hi: "#ffffff", search: "#ffe14d"};
}

function Markers() {
  const {selected, highlight, sources, flat, stackHeight, gotoDraft, vizStyle} = useViewer();
  const ink = useMarkerInk();
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
            <meshBasicMaterial color={ink.search} depthTest={false} transparent />
          </mesh>
        );
      })}
      {sel && selNode && (
        <group position={sel}>
          <mesh>
            <torusGeometry args={[0.8, 0.09, 8, 36]} />
            <meshBasicMaterial color={ink.hi} depthTest={false} transparent />
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
    // already in the top view (the mission planner opens in it): frame from straight above
    if (useViewer.getState().topDown) void c.setLookAt(cx, cy, cz + span * 1.1, cx, cy, cz, false);
    else void c.setLookAt(cx - span * 0.2, cy - span * 1.0, cz + span * 0.9, cx, cy, cz, false);
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
  // "Fit mission": top view keeps its rotation and frames the points in the part
  // of the canvas the overlays leave free; 3D keeps its angle and fits a sphere
  const fitTo = useViewer(s => s.fitTo);
  const fitNonce = useRef(fitTo?.nonce ?? 0);
  const {camera, size} = useThree();
  useEffect(() => {
    const c = ref.current;
    if (!c || !fitTo || fitTo.nonce === fitNonce.current || !fitTo.pts.length) return;
    fitNonce.current = fitTo.nonce;
    fitted.current = true; // the first map framing must not undo it
    const z = c.getTarget(new THREE.Vector3()).z;
    if (useViewer.getState().topDown && camera instanceof THREE.PerspectiveCamera) {
      camera.updateMatrixWorld();
      const e = camera.matrixWorld.elements;
      const axis = (x: number, y: number) => {
        const n = Math.hypot(x, y) || 1;
        return [x / n, y / n];
      };
      const f = fitTopView(fitTo.pts, axis(e[0], e[1]), axis(e[4], e[5]), size.width, size.height, (camera.fov * Math.PI) / 180, fitTo.insets);
      if (!f) return;
      void c.moveTo(f.target[0], f.target[1], z, true);
      void c.dollyTo(Math.min(Math.max(f.height, 8), 1900), true);
      return;
    }
    const box = new THREE.Box3();
    for (const p of fitTo.pts) box.expandByPoint(new THREE.Vector3(p[0], p[1], z));
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    sphere.radius = Math.max(sphere.radius * 1.15, 10);
    void c.fitToSphere(sphere, true);
  }, [fitTo, camera, size]);
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
  const show = useViewStyle(s => s.grid);
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
  return show ? <primitive object={grid} position={[cx, cy, -0.05]} /> : null;
}

// Phones / tablets: tile GPUs with little memory. Cap the pixel ratio, skip
// MSAA and let the browser discard the drawing buffer (keeping it costs a copy
// per frame there). Desktop keeps it so headless screenshot tours see pixels.
const COARSE = typeof window !== "undefined" && window.matchMedia?.("(pointer: coarse)").matches === true;
const DPR: [number, number] = COARSE ? [1, 1.5] : [1, 2];
const GL = {antialias: !COARSE, powerPreference: "high-performance" as const, preserveDrawingBuffer: !COARSE};

/** Missions are drawn at ground height: the GPS anchor's, else the floor of the map. */
function MissionLayer({bounds}: {bounds: {min: number[]; max: number[]} | null}) {
  const z0 = useMission(s => s.geo?.anchor.z0);
  const z = z0 ?? (bounds ? bounds.min[2] : 0);
  return (
    <>
      <MissionOverlay groundZ={z} />
      <MapHover z={z} />
    </>
  );
}

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
        gl.setClearColor(useViewStyle.getState().background ?? defaultBackground(useViewer.getState().vizStyle, useTheme.getState().theme));
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
      <SatelliteGround floorZ={bounds ? bounds.min[2] : null} />
      {sources.map(s => (
        <SourceContent key={s.id} src={s} cb={cb} />
      ))}
      <LiveOverlays />
      <RobotGlyphs />
      <PlanOverlay />
      <PreviewOverlay bounds={bounds} />
      <MissionLayer bounds={bounds} />
      <EditOverlay />
      <Markers />
      <Picker cb={cb} />
      <CameraRig bounds={bounds} />
    </Canvas>
  );
}
