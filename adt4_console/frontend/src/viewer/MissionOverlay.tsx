// The mission on the map: planned legs per task (state-styled while a run
// plays), search sweeps, the mission's own areas / no-go zones (map areas are
// EditOverlay's; a mission area named like one is not drawn twice) / planted mock objects,
// detection candidates, the agents (with the quad's camera footprint), and the
// polygon being drawn. Everything is 2-D, drawn at ground height over the
// scene (no depth test), like the robot plan overlay; ground agents are their
// platform's model, drawn last over the scene (RobotModel onTop), moving smoothly
// between run-state updates (a PoseTrack) and walking at their ground speed.
// An agent's start can be dragged on the map (StartDragger): grabbing its glyph
// holds the camera for that gesture, a ghost shows the new start and its
// coordinates, release commits it (stores/mission.ts moveStart), Esc cancels.
import {Suspense, useEffect, useMemo, useRef} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {Html, Line} from "@react-three/drei";
import {agentModelFor} from "../lib/robotModel";
import {PoseTrack} from "../lib/gait";
import {moveStart, useMission, type Agent, type Mission, type Preview, type RunState} from "../stores/mission";
import {dragTo, grab, nearestPlace, release, startLockReason, type PlacePoint, type StartDrag} from "../lib/agentStart";
import {fmtLonLat, mapToLonLat} from "../lib/geo";
import {pickRadius, TAP_SLOP_PX} from "../lib/tap";
import {activeTool} from "../stores/tools";
import {useViewer} from "../stores/viewer";
import {sourceMatrix} from "../stores/robots";
import {pickRegistry} from "./GraphLayers";
import {useMapEdit} from "../stores/mapEdit";
import {useViewStyle} from "../stores/viewStyle";
import {PALETTE} from "../lib/palette";
import {useRobotColor3D} from "./background";
import {ModelBoundary, RobotModel} from "./RobotModel";

const ORDER = 30;
const CAM_HFOV = (70 * Math.PI) / 180;

type V3 = [number, number, number];

export function agentColors(agents: Agent[], robotColor: (r: string) => string): Record<string, string> {
  const out: Record<string, string> = {};
  agents.forEach((a, i) => {
    if (a.color) out[a.id] = a.color;
    else if (a.binding) out[a.id] = robotColor(a.binding);
    else {
      const [r, g, b] = PALETTE[(i * 3 + 6) % PALETTE.length];
      out[a.id] = `rgb(${r},${g},${b})`;
    }
  });
  return out;
}

function Poly({pts, z, color, fill, width = 2, dashed = false, label}: {pts: number[][]; z: number; color: string; fill?: number; width?: number; dashed?: boolean; label?: string}) {
  const line = useMemo(() => [...pts, pts[0]].map(p => [p[0], p[1], z] as V3), [pts, z]);
  const shape = useMemo(() => {
    if (!fill || pts.length < 3) return null;
    const s = new THREE.Shape(pts.map(p => new THREE.Vector2(p[0], p[1])));
    return new THREE.ShapeGeometry(s);
  }, [pts, fill]);
  useEffect(() => () => shape?.dispose(), [shape]);
  const c = useMemo(() => {
    let x = 0;
    let y = 0;
    for (const p of pts) {
      x += p[0];
      y += p[1];
    }
    return [x / pts.length, y / pts.length, z] as V3;
  }, [pts, z]);
  if (pts.length < 2) return null;
  return (
    <group>
      {shape && (
        <mesh geometry={shape} position={[0, 0, z]} renderOrder={ORDER}>
          <meshBasicMaterial color={color} transparent opacity={fill} depthTest={false} depthWrite={false} side={THREE.DoubleSide} />
        </mesh>
      )}
      <Line points={line} color={color} lineWidth={width} dashed={dashed} dashSize={1} gapSize={0.6} depthTest={false} renderOrder={ORDER + 1} />
      {label && (
        <Html position={c} center className="mission-label" zIndexRange={[20, 0]}>
          {label}
        </Html>
      )}
    </group>
  );
}

function AgentCone({color}: {color: string}) {
  return (
    <mesh position={[0, 0, 0.4]} rotation={[0, 0, -Math.PI / 2]} renderOrder={ORDER + 5}>
      <coneGeometry args={[0.7, 1.8, 3]} />
      <meshBasicMaterial color={color} depthTest={false} />
    </mesh>
  );
}

/** A ground agent: its platform's model standing at the overlay height (the cone
 *  while it loads, when it fails, or when the view asks for arrows), styled like
 *  the live robots (Display > Robots / Highlight). */
function GroundAgent({kind, color, motion}: {kind: Agent["kind"]; color: string; motion: PoseTrack}) {
  const mode = useViewStyle(s => s.robotModel);
  const highlight = useViewStyle(s => s.robotHighlight);
  const spec = mode === "model" ? agentModelFor(kind) : null;
  if (!spec) return <AgentCone color={color} />;
  return (
    <ModelBoundary fallback={<AgentCone color={color} />}>
      <Suspense fallback={<AgentCone color={color} />}>
        <group position={[0, 0, -spec.groundZ]}>
          <RobotModel url={spec.url} color={color} highlight={highlight} opacity={1} onTop={ORDER + 5} motion={motion} />
        </group>
      </Suspense>
    </ModelBoundary>
  );
}

/** A ground agent at its pose, replayed one update late and interpolated between
 *  run-state updates (``stamp``: the run's sim time, so a re-sent pose while the
 *  agent dwells still counts as a sample); outside a run it is placed as is. */
function TrackedGroundAgent({agent, pos, yaw, color, z, status, stamp}: {agent: Agent; pos: number[]; yaw: number; color: string; z: number; status?: string; stamp: number | null}) {
  const track = useMemo(() => new PoseTrack(), []);
  const outer = useRef<THREE.Group>(null);
  const inner = useRef<THREE.Group>(null);
  const invalidate = useThree(s => s.invalidate);
  const [x, y] = pos;
  useEffect(() => {
    if (stamp === null) track.reset();
    track.push(performance.now(), x, y, yaw);
    invalidate();
  }, [track, x, y, yaw, stamp, invalidate]);
  useEffect(() => invalidate(), [z, invalidate]);
  useFrame(() => {
    const s = track.sample(performance.now());
    if (!s || !outer.current || !inner.current) return;
    outer.current.position.set(s.x, s.y, z);
    inner.current.rotation.set(0, 0, s.yaw);
    if (s.moving) invalidate();
  });
  // placed by useFrame only (props would snap it to the newest pose on each update)
  return (
    <group ref={outer}>
      <group ref={inner}>
        <GroundAgent kind={agent.kind} color={color} motion={track} />
      </group>
      <Html position={[0, 0, 2]} center className="mission-agent-tag" zIndexRange={[25, 0]}>
        <span style={{borderColor: color}}>
          {agent.id}
          {status && status !== "idle" ? <em> · {status}</em> : null}
        </span>
      </Html>
    </group>
  );
}

function AgentGlyph({agent, pos, yaw, color, z, status, stamp}: {agent: Agent; pos: number[]; yaw: number; color: string; z: number; status?: string; stamp: number | null}) {
  const quad = agent.kind === "quad";
  const h = agent.altitude ?? 20;
  const foot = useMemo(() => {
    if (!quad) return null;
    const w = h * Math.tan(CAM_HFOV / 2);
    const l = w * 0.75;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return [
      [l, w],
      [l, -w],
      [-l, -w],
      [-l, w],
    ].map(([f, lft]) => [pos[0] + c * f - s * lft, pos[1] + s * f + c * lft]);
  }, [quad, h, pos, yaw]);
  if (!quad) return <TrackedGroundAgent agent={agent} pos={pos} yaw={yaw} color={color} z={z} status={status} stamp={stamp} />;
  return (
    <group>
      {foot && <Poly pts={foot} z={z} color={color} fill={0.08} width={1} dashed />}
      <group position={[pos[0], pos[1], z + 1.5]} rotation={[0, 0, yaw]}>
        {[0, Math.PI / 2].map(r => (
          <mesh key={r} rotation={[0, 0, r + Math.PI / 4]} renderOrder={ORDER + 5}>
            <boxGeometry args={[3.2, 0.35, 0.2]} />
            <meshBasicMaterial color={color} depthTest={false} />
          </mesh>
        ))}
        {[0, 1, 2, 3].map(k => (
          <mesh key={k} position={[1.15 * Math.cos(Math.PI / 4 + (k * Math.PI) / 2), 1.15 * Math.sin(Math.PI / 4 + (k * Math.PI) / 2), 0.1]} rotation={[Math.PI / 2, 0, 0]} renderOrder={ORDER + 6}>
            <torusGeometry args={[0.55, 0.08, 6, 20]} />
            <meshBasicMaterial color="#ffffff" depthTest={false} />
          </mesh>
        ))}
      </group>
      <Html position={[pos[0], pos[1], z + 2]} center className="mission-agent-tag" zIndexRange={[25, 0]}>
        <span style={{borderColor: color}}>
          {agent.id}
          {status && status !== "idle" ? <em> · {status}</em> : null}
        </span>
      </Html>
    </group>
  );
}

export function MissionOverlay({groundZ}: {groundZ: number}) {
  const invalidate = useThree(s => s.invalidate);
  const m = useMission();
  const robotColor = useRobotColor3D();
  const {current, preview, run, areas, fences, planted, draft, pendingArea, roster} = m;
  const regions = useMapEdit(s => s.regions);
  const mapNames = useMemo(() => new Set(regions.map(r => (r.name ?? "").trim().toLowerCase()).filter(Boolean)), [regions]);
  useEffect(() => invalidate(), [current, preview, run, areas, fences, planted, draft, pendingArea, roster, regions, invalidate]);
  const z = groundZ + 0.3;
  const agents = current?.agents ?? roster;
  const colors = useMemo(() => agentColors(agents, robotColor), [agents, robotColor]);

  return (
    <group>
      {areas
        .filter(a => !mapNames.has(a.name.trim().toLowerCase()))
        .map(a => (
          <Poly key={`area-${a.name}`} pts={a.polygon} z={z} color="#4ea8ff" fill={0.06} width={2} label={a.name} />
        ))}
      {(current?.geofences.length ? current.geofences : fences).map(g => (
        <Poly key={`fence-${g.id}`} pts={g.polygon} z={z + 0.01} color="#ff4040" fill={0.18} width={2.5} label={`⛔ ${g.id}`} />
      ))}
      {pendingArea && <Poly pts={pendingArea} z={z + 0.3} color="#ffffff" fill={0.12} width={2.5} dashed />}
      {current?.tasks
        .filter(t => t.type === "explore_area" && t.area && t.area.length >= 3)
        .map(t => (
          <Poly key={`explore-${t.id}`} pts={t.area!} z={z + 0.02} color={colors[t.agent] ?? "#fff"} fill={0.1} width={1.5} dashed />
        ))}
      {preview &&
        Object.entries(preview.tasks).map(([tid, tp]) => {
          if (tp.path.length < 2) return null;
          const state = run?.tasks[tid]?.state ?? "planned";
          const color = state === "failed" ? "#ff4040" : (colors[tp.agent] ?? "#ffffff");
          const pts = tp.path.map(p => [p[0], p[1], z + 0.05] as V3);
          const width = state === "active" ? 6 : state === "done" ? 2.5 : 4;
          return (
            <group key={`leg-${tid}`}>
              <Line points={pts} color="#000000" lineWidth={width + 3} transparent opacity={state === "done" ? 0.15 : 0.45} depthTest={false} renderOrder={ORDER + 2} />
              <Line
                points={pts}
                color={color}
                lineWidth={width}
                dashed={tp.tentative || state === "pending" || state === "failed"}
                dashSize={tp.tentative ? 0.4 : 1.2}
                gapSize={tp.tentative ? 0.8 : 0.5}
                transparent
                opacity={state === "done" ? 0.4 : 1}
                depthTest={false}
                renderOrder={ORDER + 3}
              />
              {tp.target && (
                <mesh position={[tp.target[0], tp.target[1], z + 0.06]} renderOrder={ORDER + 4}>
                  <ringGeometry args={[0.6, 1.0, 24]} />
                  <meshBasicMaterial color={color} depthTest={false} transparent opacity={state === "done" ? 0.4 : 0.95} side={THREE.DoubleSide} />
                </mesh>
              )}
            </group>
          );
        })}
      {planted.map((p, i) => (
        <group key={`plant-${i}`} position={[p.pos[0], p.pos[1], z + 0.1]}>
          <mesh renderOrder={ORDER + 4}>
            <boxGeometry args={[p.size_m ?? 1.5, p.size_m ?? 1.5, 0.3]} />
            <meshBasicMaterial color="#ff9f1a" depthTest={false} transparent opacity={0.85} />
          </mesh>
          <Html position={[0, 0, 1]} center className="mission-plant-tag" zIndexRange={[20, 0]}>
            mock {p.label}
          </Html>
        </group>
      ))}
      {run?.candidates.map(c => (
        <group key={`cand-${c.id}`} position={[c.pos[0], c.pos[1], z + 0.15]}>
          <mesh renderOrder={ORDER + 6}>
            <ringGeometry args={[1.4, 2.0, 32]} />
            <meshBasicMaterial color={c.status === "approved" ? "#34d058" : c.status === "rejected" ? "#888888" : "#ffd33d"} depthTest={false} side={THREE.DoubleSide} />
          </mesh>
          <Html position={[0, 0, 1.5]} center className={`mission-cand-tag ${c.status}`} zIndexRange={[30, 0]}>
            {c.sym && !c.sym.startsWith("sim:") ? `${c.label} ${c.sym}` : `${c.label}?`}
          </Html>
        </group>
      ))}
      {preview?.issues
        .filter(i => i.pos)
        .map((i, k) => (
          <mesh key={`issue-${k}`} position={[i.pos![0], i.pos![1], z + 0.2]} rotation={[0, 0, Math.PI / 4]} renderOrder={ORDER + 6}>
            <ringGeometry args={[1.0, 1.4, 4]} />
            <meshBasicMaterial color="#ff4040" depthTest={false} side={THREE.DoubleSide} />
          </mesh>
        ))}
      {agents.map(a => {
        const r = run?.agents[a.id];
        const start = shownPos(a, run, preview);
        if (!start) return null;
        return <AgentGlyph key={`agent-${a.id}`} agent={a} pos={start} yaw={r?.yaw ?? 0} color={colors[a.id]} z={z} status={r?.status} stamp={r ? (run?.t ?? null) : null} />;
      })}
      {m.drag?.moved && <DragGhost drag={m.drag} color={colors[m.drag.agent] ?? "#ffffff"} z={z} />}
      <StartDragger z={z} />
      {draft.length > 0 && (
        <group>
          <Line points={draft.map(p => [p[0], p[1], z + 0.3] as V3)} color="#4ea8ff" lineWidth={3} depthTest={false} renderOrder={ORDER + 8} />
          {draft.map((p, i) => (
            <mesh key={i} position={[p[0], p[1], z + 0.3]} renderOrder={ORDER + 9}>
              <circleGeometry args={[0.5, 12]} />
              <meshBasicMaterial color="#ffffff" depthTest={false} />
            </mesh>
          ))}
        </group>
      )}
    </group>
  );
}

/** Where an agent is drawn: its run pose while a run has one, else its start
 *  (the mission's own start first, so a moved start shows before the re-preview
 *  returns), else the preview's (a bound robot's live pose). */
export function shownPos(a: Agent, run: RunState | null, preview: Preview | null): number[] | null {
  return run?.agents[a.id]?.pos ?? a.start ?? preview?.agents[a.id]?.start ?? null;
}

/** The agents a press may grab, with where each is drawn: none while a map tool
 *  is armed, map edit or goto picking is on, or (open mission) a run locks the starts. */
export function draggableAgents(s: {current: Mission | null; roster: Agent[]; run: RunState | null; preview: Preview | null}, busy: boolean): {agent: Agent; pos: number[]}[] {
  if (busy) return [];
  if (s.current && startLockReason(s.run)) return [];
  const out: {agent: Agent; pos: number[]}[] = [];
  for (const a of s.current?.agents ?? s.roster) {
    const p = shownPos(a, s.run, s.preview);
    if (p) out.push({agent: a, pos: p});
  }
  return out;
}

/** Place nodes of the shown graph sources, map frame (to snap a dragged start to). */
function shownPlaces(): PlacePoint[] {
  const out: PlacePoint[] = [];
  const sources = useViewer.getState().sources;
  const v = new THREE.Vector3();
  for (const [src, layers] of pickRegistry) {
    if (!sources[src]?.enabled) continue;
    const m = sourceMatrix(src);
    for (const l of layers.values()) {
      if (l.info.kind !== "places") continue;
      for (let i = 0; i < l.syms.length; i++) {
        v.set(l.positions[i * 3], l.positions[i * 3 + 1], l.positions[i * 3 + 2]);
        if (m) v.applyMatrix4(m);
        out.push({sym: l.syms[i], x: v.x, y: v.y});
      }
    }
  }
  return out;
}

const SNAP_M = 4;

/** Drag an agent's glyph to move its start. Listens on the Canvas' outer element in
 *  the capture phase, so a press on a glyph never reaches the camera controls or
 *  the click picker (and the controls are off until release); a press anywhere
 *  else is left alone. Mouse, touch and pen alike (pointer events). */
function StartDragger({z}: {z: number}) {
  const {gl, camera, invalidate} = useThree();
  const controls = useThree(s => s.controls) as unknown as {enabled: boolean} | null;
  useEffect(() => {
    const canvas = gl.domElement;
    // the Canvas' outer element: it holds the canvas and the Html tags (a press on
    // an agent's name tag grabs it too)
    const host = (canvas.closest(".scene-canvas") as HTMLElement | null) ?? canvas.parentElement?.parentElement ?? canvas;
    const ray = new THREE.Raycaster();
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z);
    const hit = new THREE.Vector3();
    let places: PlacePoint[] = [];
    const ground = (cx: number, cy: number): number[] | null => {
      const r = canvas.getBoundingClientRect();
      ray.setFromCamera(new THREE.Vector2(((cx - r.left) / r.width) * 2 - 1, -((cy - r.top) / r.height) * 2 + 1), camera);
      return ray.ray.intersectPlane(plane, hit) ? [hit.x, hit.y] : null;
    };
    const screen = (p: number[], dz = 0): [number, number] | null => {
      const r = canvas.getBoundingClientRect();
      const q = new THREE.Vector3(p[0], p[1], z + dz).project(camera);
      if (q.z > 1 || q.z < -1) return null;
      return [r.left + ((q.x + 1) / 2) * r.width, r.top + ((1 - q.y) / 2) * r.height];
    };
    /** the agent under the pointer: within the pick radius of its glyph (or its ~1 m body) */
    const under = (e: PointerEvent): {agent: Agent; pos: number[]} | null => {
      const t = activeTool();
      const pm = useViewer.getState().pickMode;
      const cands = draggableAgents(useMission.getState(), t.owner !== "none" || pm === "goto" || pm === "edit");
      let best: {agent: Agent; pos: number[]} | null = null;
      let bd = Infinity;
      for (const c of cands) {
        const s = screen(c.pos);
        if (!s) continue;
        const edge = screen([c.pos[0] + 0.9, c.pos[1]]);
        const body = edge ? Math.hypot(edge[0] - s[0], edge[1] - s[1]) : 0;
        const radius = Math.max(pickRadius(e.pointerType) * 1.4, body);
        const d = Math.hypot(e.clientX - s[0], e.clientY - s[1]);
        if (d <= radius && d < bd) {
          bd = d;
          best = c;
        }
      }
      return best;
    };
    const end = () => {
      if (controls) controls.enabled = true;
      host.style.cursor = "";
      window.removeEventListener("pointermove", move, true);
      window.removeEventListener("pointerup", up, true);
      window.removeEventListener("pointercancel", cancel, true);
      window.removeEventListener("keydown", key, true);
      useMission.getState().set({drag: null});
      invalidate();
    };
    const move = (e: PointerEvent) => {
      const d = useMission.getState().drag;
      if (!d || e.pointerId !== d.pointerId) return;
      e.preventDefault();
      e.stopPropagation();
      const p = ground(e.clientX, e.clientY);
      if (!p) return;
      const snapOn = useMission.getState().snapStarts !== e.shiftKey;
      const snap = snapOn ? nearestPlace(p, places, SNAP_M) : null;
      const slop = TAP_SLOP_PX[e.pointerType === "touch" || e.pointerType === "pen" ? e.pointerType : "mouse"];
      useMission.getState().set({drag: dragTo(d, p, e.clientX, e.clientY, slop, snap)});
      invalidate();
    };
    const up = (e: PointerEvent) => {
      const d = useMission.getState().drag;
      if (!d || e.pointerId !== d.pointerId) return;
      e.stopPropagation();
      const to = release(d);
      end();
      if (to) void moveStart(d.agent, to);
    };
    const cancel = (e: PointerEvent) => {
      const d = useMission.getState().drag;
      if (d && e.pointerId === d.pointerId) end();
    };
    const key = (e: KeyboardEvent) => {
      if (e.key !== "Escape") return;
      e.preventDefault();
      e.stopPropagation();
      end();
    };
    const down = (e: PointerEvent) => {
      if (useMission.getState().drag) {
        // a second finger while dragging: not a pinch, not a pick
        e.stopPropagation();
        return;
      }
      if (e.pointerType === "mouse" && e.button !== 0) return;
      const c = under(e);
      if (!c) return;
      e.preventDefault();
      e.stopPropagation(); // the camera controls and the picker never see this press
      if (controls) controls.enabled = false;
      places = shownPlaces();
      host.style.cursor = "grabbing";
      useMission.getState().set({drag: grab(c.agent.id, e.pointerId, c.pos, e.clientX, e.clientY)});
      window.addEventListener("pointermove", move, true);
      window.addEventListener("pointerup", up, true);
      window.addEventListener("pointercancel", cancel, true);
      window.addEventListener("keydown", key, true);
    };
    // desktop: a grab hand over a draggable glyph
    let lastHover = 0;
    const hover = (e: PointerEvent) => {
      if (e.pointerType !== "mouse" || e.buttons || useMission.getState().drag) return;
      const now = performance.now();
      if (now - lastHover < 50) return;
      lastHover = now;
      host.style.cursor = under(e) ? "grab" : "";
    };
    host.addEventListener("pointerdown", down, {capture: true});
    host.addEventListener("pointermove", hover);
    // e2e scripts find a glyph on screen with this (client px; null: not drawn / behind the camera)
    const w = window as unknown as {__adt4AgentScreen?: (id: string) => [number, number] | null};
    w.__adt4AgentScreen = (id: string) => {
      const st = useMission.getState();
      const a = (st.current?.agents ?? st.roster).find(x => x.id === id);
      const p = a ? shownPos(a, st.run, st.preview) : null;
      return p ? screen(p) : null;
    };
    return () => {
      delete w.__adt4AgentScreen;
      host.removeEventListener("pointerdown", down, {capture: true});
      host.removeEventListener("pointermove", hover);
      if (useMission.getState().drag) end();
    };
  }, [gl, camera, controls, invalidate, z]);
  return null;
}

/** The dragged start: a ring where it will go (on the snapped place), a line from
 *  where the agent is, and the coordinates. */
function DragGhost({drag, color, z}: {drag: StartDrag; color: string; z: number}) {
  const geo = useMission(s => s.geo);
  const snapOn = useMission(s => s.snapStarts);
  const coarse = typeof window !== "undefined" && window.matchMedia?.("(pointer: coarse)").matches === true;
  const [x, y] = drag.at;
  return (
    <group>
      <Line points={[[drag.from[0], drag.from[1], z + 0.4], [x, y, z + 0.4]]} color={color} lineWidth={2} dashed dashSize={0.6} gapSize={0.4} depthTest={false} renderOrder={ORDER + 10} />
      <mesh position={[x, y, z + 0.4]} renderOrder={ORDER + 11}>
        <ringGeometry args={[0.9, 1.3, 32]} />
        <meshBasicMaterial color={drag.snap ? "#34d058" : color} depthTest={false} side={THREE.DoubleSide} />
      </mesh>
      <mesh position={[x, y, z + 0.4]} renderOrder={ORDER + 11}>
        <circleGeometry args={[0.25, 16]} />
        <meshBasicMaterial color="#ffffff" depthTest={false} />
      </mesh>
      <Html position={[x, y, z + 0.4]} center className="mission-drag-tag" zIndexRange={[40, 0]}>
        <div data-testid="start-drag-readout">
          <b>{drag.agent}</b> ({x.toFixed(2)}, {y.toFixed(2)}){drag.snap ? <em> · on {drag.snap}</em> : null}
          {geo && <div className="mono">{fmtLonLat(...mapToLonLat(geo, x, y))}</div>}
          <div className="hint">{coarse ? "lift to set the start" : `release to set · Shift ${snapOn ? "frees" : "snaps to a place"} · Esc cancels`}</div>
        </div>
      </Html>
    </group>
  );
}

/** Broadcasts the map point under the cursor (``adt4-map-hover``, map frame,
 *  at ground height) for the coordinate readout; at most every 50 ms. */
export function MapHover({z}: {z: number}) {
  const {gl, camera} = useThree();
  useEffect(() => {
    const el = gl.domElement;
    let last = 0;
    const ray = new THREE.Raycaster();
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z);
    const hit = new THREE.Vector3();
    const move = (e: PointerEvent) => {
      const now = performance.now();
      if (now - last < 50) return;
      last = now;
      const r = el.getBoundingClientRect();
      ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
      if (ray.ray.intersectPlane(plane, hit)) window.dispatchEvent(new CustomEvent("adt4-map-hover", {detail: [hit.x, hit.y]}));
    };
    el.addEventListener("pointermove", move);
    return () => el.removeEventListener("pointermove", move);
  }, [gl, camera, z]);
  return null;
}
