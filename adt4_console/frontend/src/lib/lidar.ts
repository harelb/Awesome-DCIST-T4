// Live lidar frames (GET /api/live/<r>/lidar, msgpack): the robot agent's
// cropped, voxel-downsampled point cloud, xyz as int16 little-endian in the
// sensor frame (units of ``scale`` m) and ``origin`` = the sensor pose
// [x, y, z, qx, qy, qz, qw] in the frame the agent's pose is expressed in.
// Pure decoding + colouring, unit tested; LiveOverlays draws it.
import type {FrameAlign} from "../types";

export interface LidarFrame {
  frame: string;
  /** sensor pose: x, y, z, qx, qy, qz, qw */
  origin: [number, number, number, number, number, number, number];
  n: number;
  /** xyz in metres, sensor frame */
  positions: Float32Array;
  /** rgb 0..1, coloured by height in the origin's frame */
  colors: Float32Array;
  stamp: number | null;
  seq: number | null;
}

const toBytes = (v: unknown): Uint8Array | null => {
  if (v instanceof Uint8Array) return v;
  if (v instanceof ArrayBuffer) return new Uint8Array(v);
  if (ArrayBuffer.isView(v)) return new Uint8Array(v.buffer, v.byteOffset, v.byteLength);
  return null;
};

const num = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);

/** Height (0 low .. 1 high) -> rgb: blue, cyan, green, yellow, red. */
export function heightColor(t: number): [number, number, number] {
  const x = Math.max(0, Math.min(1, Number.isFinite(t) ? t : 0));
  const r = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * x - 3)));
  const g = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * x - 2)));
  const b = Math.max(0, Math.min(1, 1.5 - Math.abs(4 * x - 1)));
  return [r, g, b];
}

/** Decode one lidar message; throws on a malformed one. */
export function decodeLidar(raw: unknown): LidarFrame {
  if (!raw || typeof raw !== "object") throw new Error("lidar frame is not a dict");
  const d = raw as Record<string, unknown>;
  const o = d.origin;
  if (!Array.isArray(o) || o.length !== 7 || !o.every(v => typeof v === "number" && Number.isFinite(v))) throw new Error("lidar frame has no origin");
  const origin = o as LidarFrame["origin"];
  const scale = num(d.scale) ?? 0.01;
  const xyz = toBytes(d.xyz);
  if (!xyz) throw new Error("lidar frame has no xyz");
  const n = Math.min(num(d.n) ?? Infinity, Math.floor(xyz.byteLength / 6));
  const dv = new DataView(xyz.buffer, xyz.byteOffset, xyz.byteLength);
  const positions = new Float32Array(n * 3);
  for (let i = 0; i < n * 3; i++) positions[i] = dv.getInt16(i * 2, true) * scale;

  // height in the origin's frame: oz + (third row of R) . p
  const [, , oz, qx, qy, qz, qw] = origin;
  const ql = Math.hypot(qx, qy, qz, qw) || 1;
  const [x, y, z, w] = [qx / ql, qy / ql, qz / ql, qw / ql];
  const r20 = 2 * (x * z - w * y);
  const r21 = 2 * (y * z + w * x);
  const r22 = 1 - 2 * (x * x + y * y);
  const h = new Float32Array(n);
  for (let i = 0; i < n; i++) h[i] = oz + r20 * positions[i * 3] + r21 * positions[i * 3 + 1] + r22 * positions[i * 3 + 2];
  // colour range: 2nd..98th percentile, so a few stray points do not flatten it
  let lo = 0;
  let hi = 1;
  if (n) {
    const sorted = Float32Array.from(h).sort();
    lo = sorted[Math.floor((n - 1) * 0.02)];
    hi = sorted[Math.ceil((n - 1) * 0.98)];
  }
  const span = hi - lo > 0.3 ? hi - lo : 0.3;
  const colors = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const c = heightColor((h[i] - lo) / span);
    colors[i * 3] = c[0];
    colors[i * 3 + 1] = c[1];
    colors[i * 3 + 2] = c[2];
  }
  return {frame: typeof d.frame === "string" ? d.frame : "", origin, n, positions, colors, stamp: num(d.stamp), seq: num(d.seq)};
}

/** The frame to place a lidar frame's ``origin`` in: the message's frame when
 *  it is one the alignment knows (the robot's scene-graph or the global frame),
 *  else the robot pose's frame (the contract: origin is in the pose's frame). */
export function lidarLayerFrame(msgFrame: string | null | undefined, poseFrame: string | null | undefined, fa: FrameAlign | null | undefined): string | null {
  if (msgFrame && (msgFrame === poseFrame || (fa && (msgFrame === fa.child || msgFrame === fa.parent)))) return msgFrame;
  return poseFrame || msgFrame || null;
}

/** How often a robot is asked again to stream while the console wants lidar
 *  but the robot reports it off (an agent restart, a reload). */
export const LIDAR_REENABLE_S = 10;
export const LIDAR_POLL_MS = 1000;
