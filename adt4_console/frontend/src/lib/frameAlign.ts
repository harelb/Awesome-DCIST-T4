// Global-frame alignment of a robot's live layers.
//
// A robot streams everything in its own scene-graph frame (the DSG header
// frame, e.g. `hamilton/odom`): live graph, mesh tiles, executor paths and
// target, and (when TF allows) the pose and occupancy grid. Prior maps (file /
// Neo4j sources) and the planner live in the global `map` frame. Once the robot
// is relocalized (hydra_multi / ROMAN loop closures publish `map -> <robot>/odom`)
// its fleetlink agent sends that transform in the C1 state as `frame_align`.
//
// Policy: the transform is applied to every layer of that robot whose frame is
// the alignment's child frame whenever `ok` is true, regardless of which other
// sources are shown (the view's fixed frame is always `map`, like RViz with
// `Fixed Frame: map`). `ok` false (not relocalized, no global frame) draws the
// layers in their native frame, as before, and the legend says so in amber.
// A layer already in the parent frame (e.g. a pose the agent could only resolve
// in `map`) is drawn as is; a layer in some other frame is drawn as is too.
import * as THREE from "three";
import type {FrameAlign, Pose} from "../types";
import {nearestPlace, type GraphModel} from "./graphModel";

export type Vec3 = [number, number, number];

/** `live:<robot>` graph source -> robot name (null for prior-map sources). */
export function robotOfSource(srcId: string): string | null {
  return srcId.startsWith("live:") ? srcId.slice(5) || null : null;
}

function validT(T: unknown): T is number[] {
  return Array.isArray(T) && T.length === 7 && T.every(v => Number.isFinite(v));
}

/** parent <- child as a Matrix4, or null when there is no usable alignment. */
export function alignMatrix(fa: FrameAlign | null | undefined): THREE.Matrix4 | null {
  if (!fa || !fa.ok || !validT(fa.T)) return null;
  const [x, y, z, qw, qx, qy, qz] = fa.T;
  const q = new THREE.Quaternion(qx, qy, qz, qw);
  if (q.lengthSq() < 1e-12) return null;
  q.normalize();
  return new THREE.Matrix4().compose(new THREE.Vector3(x, y, z), q, new THREE.Vector3(1, 1, 1));
}

/** The matrix to draw a layer expressed in `frame` with (null = draw as is).
 *  An empty / missing frame is taken to be the scene-graph (child) frame. */
export function layerMatrix(fa: FrameAlign | null | undefined, frame?: string | null): THREE.Matrix4 | null {
  const m = alignMatrix(fa);
  if (!m || !fa) return null;
  if (frame && frame !== fa.child) return null; // parent frame (already global) or unknown
  return m;
}

/** Stable key of what `layerMatrix` would return (for memo / effect deps). */
export function layerKey(fa: FrameAlign | null | undefined, frame?: string | null): string {
  const m = layerMatrix(fa, frame);
  return m ? m.elements.map(v => v.toFixed(5)).join(",") : "";
}

export function applyPoint(m: THREE.Matrix4 | null, p: Vec3): Vec3 {
  if (!m) return p;
  const v = new THREE.Vector3(p[0], p[1], p[2]).applyMatrix4(m);
  return [v.x, v.y, v.z];
}

/** World (global frame) point -> the layer's own frame. */
export function invertPoint(m: THREE.Matrix4 | null, p: Vec3): Vec3 {
  if (!m) return p;
  const v = new THREE.Vector3(p[0], p[1], p[2]).applyMatrix4(m.clone().invert());
  return [v.x, v.y, v.z];
}

/** World ray -> the layer's own frame (origin transformed, direction rotated). */
export function invertRay(m: THREE.Matrix4 | null, o: Vec3, d: Vec3): {o: Vec3; d: Vec3} {
  if (!m) return {o, d};
  const inv = m.clone().invert();
  const vo = new THREE.Vector3(...o).applyMatrix4(inv);
  const vd = new THREE.Vector3(...d).transformDirection(inv);
  return {o: [vo.x, vo.y, vo.z], d: [vd.x, vd.y, vd.z]};
}

/** A robot pose in the global frame (position and orientation), when aligned. */
export function poseInGlobal(pose: Pose, fa: FrameAlign | null | undefined): Pose {
  const m = layerMatrix(fa, pose.frame);
  if (!m) return pose;
  const local = new THREE.Matrix4().compose(
    new THREE.Vector3(pose.x, pose.y, pose.z),
    new THREE.Quaternion(pose.qx ?? 0, pose.qy ?? 0, pose.qz ?? 0, pose.qw ?? 1).normalize(),
    new THREE.Vector3(1, 1, 1),
  );
  const world = m.clone().multiply(local);
  const p = new THREE.Vector3();
  const q = new THREE.Quaternion();
  world.decompose(p, q, new THREE.Vector3());
  return {...pose, x: p.x, y: p.y, z: p.z, qw: q.w, qx: q.x, qy: q.y, qz: q.z, frame: fa!.parent};
}

export interface AlignStatus {
  tone: "ok" | "warn" | "none";
  text: string;
  title: string;
}

function fmtAge(s: number): string {
  return s < 90 ? `${s.toFixed(s < 10 ? 1 : 0)} s` : s < 5400 ? `${Math.round(s / 60)} min` : `${(s / 3600).toFixed(1)} h`;
}

/** Legend / Live-panel status. `sinceStateS` = seconds since that state arrived
 *  (added to the robot-side `age_s`). */
export function alignStatus(fa: FrameAlign | null | undefined, sinceStateS = 0): AlignStatus {
  if (!fa) return {tone: "none", text: "", title: "this robot's agent does not report a global-frame alignment"};
  if (!fa.ok || !validT(fa.T)) {
    return {
      tone: "warn",
      text: `${fa.child || "odom"} frame — not relocalized`,
      title: `no ${fa.parent} -> ${fa.child} transform: the live layers are drawn in the robot's own frame${fa.reason ? `\n${fa.reason}` : ""}`,
    };
  }
  if (fa.parent === fa.child) return {tone: "ok", text: `in ${fa.parent} frame`, title: `the robot's scene graph is published in ${fa.parent}`};
  const [x, y, z] = fa.T;
  const age = fa.age_s === null || fa.age_s === undefined || !fa.stamp ? null : fa.age_s + Math.max(0, sinceStateS);
  return {
    tone: "ok",
    text: `aligned to ${fa.parent} (relocalized)${age === null ? " · static" : ` · ${fmtAge(age)}`}`,
    title: `${fa.parent} -> ${fa.child}: t = (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) m, yaw ${((quatYaw(fa.T) * 180) / Math.PI).toFixed(1)}°${age === null ? " (static transform)" : `, transform ${fmtAge(age)} old`}`,
  };
}

function quatYaw(T: number[]): number {
  const [, , , qw, qx, qy, qz] = T;
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

/** Goto picking: the place nearest a clicked global-frame point across the
 *  shown sources, each searched in its own frame (`matrixOf(id)` = that source's
 *  drawing matrix; live sources: the robot alignment). omniplanner's goto / TSP
 *  goals are node symbols resolved in the planner's own scene graph, so no
 *  coordinates go to the planner; only the snapping needs the conversion. */
export function nearestGotoPlace(
  models: {id: string; model: GraphModel}[],
  p: Vec3,
  matrixOf: (id: string) => THREE.Matrix4 | null,
  maxDist = 15,
): {src: string; sym: string; d: number; pos: Vec3} | null {
  let best: {src: string; sym: string; d: number; pos: Vec3} | null = null;
  for (const {id, model} of models) {
    const m = matrixOf(id);
    const local = invertPoint(m, p);
    const n = nearestPlace(model, local, {maxDist});
    if (!n) continue;
    const d = Math.hypot(n.pos[0] - local[0], n.pos[1] - local[1]);
    if (!best || d < best.d) best = {src: id, sym: n.sym, d, pos: applyPoint(m, n.pos)};
  }
  return best;
}

/** Snap a goto pick (global frame) to a node of the planner's own map.
 *  The planner refuses a symbol its map lacks ("not in scene graph"), and a
 *  symbol picked on another source (a live mirror's agent pose, a prior file)
 *  can be missing there. `object` (the pick was an object) first tries a
 *  planner object within `objectDist`, else the nearest planner place. */
export function snapToPlannerMap(
  planner: {id: string; model: GraphModel},
  p: Vec3,
  matrixOf: (id: string) => THREE.Matrix4 | null,
  opts: {object?: boolean; objectDist?: number; maxDist?: number} = {},
): {src: string; sym: string; d: number; pos: Vec3} | null {
  const m = matrixOf(planner.id);
  const local = invertPoint(m, p);
  const n =
    (opts.object ? nearestPlace(planner.model, local, {kinds: ["objects"], maxDist: opts.objectDist ?? 1}) : null) ??
    nearestPlace(planner.model, local, {maxDist: opts.maxDist ?? 15});
  if (!n) return null;
  return {src: planner.id, sym: n.sym, d: Math.hypot(n.pos[0] - local[0], n.pos[1] - local[1]), pos: applyPoint(m, n.pos)};
}
