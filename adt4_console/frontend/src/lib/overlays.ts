// Decoders for the live RViz-replacement overlays (API.md §11): occupancy grid
// (msgpack dict with a zlib int8 payload) and executor paths.

export type Vec3 = [number, number, number];

export interface GridMeta {
  frame: string;
  resolution: number;
  /** x, y, z, yaw of cell (0, 0)'s corner */
  origin: [number, number, number, number];
  width: number;
  height: number;
  hash: string;
  t?: number | null;
}

export interface DecodedGrid extends GridMeta {
  /** RGBA, row 0 = lowest y (matches a DataTexture on a PlaneGeometry) */
  rgba: Uint8Array;
}

export interface RobotPaths {
  smooth: Vec3[];
  mlp: Vec3[];
  target: Vec3 | null;
  frame: string;
  t: number | null;
  hash?: string;
}

const toBytes = (v: unknown): Uint8Array => {
  if (v instanceof Uint8Array) return v;
  if (v instanceof ArrayBuffer) return new Uint8Array(v);
  if (ArrayBuffer.isView(v)) return new Uint8Array(v.buffer, v.byteOffset, v.byteLength);
  throw new Error("grid data is not binary");
};

/** zlib (RFC 1950) inflate via the platform DecompressionStream ("deflate" = zlib framing). */
export async function inflateZlib(data: Uint8Array): Promise<Uint8Array> {
  const ds = new DecompressionStream("deflate");
  const writer = ds.writable.getWriter();
  void writer.write(new Uint8Array(data)); // copy: may be a view into the msgpack buffer
  void writer.close();
  return new Uint8Array(await new Response(ds.readable).arrayBuffer());
}

// free -> transparent, occupied -> dark red, unknown -> faint grey
export const OCCUPIED: [number, number, number, number] = [150, 20, 20, 235];
export const UNKNOWN: [number, number, number, number] = [150, 150, 150, 55];
export const OCCUPIED_THRESHOLD = 50;

/** OccupancyGrid cells (int8: -1 unknown, 0..100) -> RGBA bytes. */
export function gridToRgba(values: Int8Array): Uint8Array {
  const out = new Uint8Array(values.length * 4);
  for (let i = 0; i < values.length; i++) {
    const v = values[i];
    const c = v < 0 ? UNKNOWN : v >= OCCUPIED_THRESHOLD ? OCCUPIED : null;
    if (!c) continue; // free: all zero (transparent)
    const o = i * 4;
    out[o] = c[0];
    out[o + 1] = c[1];
    out[o + 2] = c[2];
    out[o + 3] = c[3];
  }
  return out;
}

/** `GET /api/live/{r}/grid` msgpack body -> metadata + RGBA texture bytes. */
export async function decodeGrid(raw: unknown): Promise<DecodedGrid> {
  const g = raw as Record<string, unknown>;
  const width = Number(g.width);
  const height = Number(g.height);
  if (!(width > 0 && height > 0)) throw new Error("empty grid");
  const cells = await inflateZlib(toBytes(g.data));
  if (cells.length !== width * height) throw new Error(`grid has ${cells.length} cells, expected ${width}x${height}`);
  const o = (g.origin as number[]) ?? [0, 0, 0, 0];
  return {
    frame: String(g.frame ?? ""),
    resolution: Number(g.resolution),
    origin: [Number(o[0] ?? 0), Number(o[1] ?? 0), Number(o[2] ?? 0), Number(o[3] ?? 0)],
    width,
    height,
    hash: String(g.hash ?? ""),
    t: (g.t as number | null) ?? null,
    rgba: gridToRgba(new Int8Array(cells.buffer, cells.byteOffset, cells.length)),
  };
}

const isVec3 = (p: unknown): p is number[] => Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every(v => Number.isFinite(Number(v)));
const vec3 = (p: number[]): Vec3 => [Number(p[0]), Number(p[1]), Number(p[2])];

/** Validate a `robot/<r>/paths` payload; malformed points are dropped. */
export function parsePaths(raw: unknown): RobotPaths | null {
  if (!raw || typeof raw !== "object") return null;
  const r = raw as Record<string, unknown>;
  const pts = (v: unknown): Vec3[] => (Array.isArray(v) ? v.filter(isVec3).map(vec3) : []);
  return {
    smooth: pts(r.smooth),
    mlp: pts(r.mlp),
    target: isVec3(r.target) ? vec3(r.target) : null,
    frame: String(r.frame ?? ""),
    t: typeof r.t === "number" ? r.t : null,
    hash: typeof r.hash === "string" ? r.hash : undefined,
  };
}

/** Lift points slightly above the floor so they are not hidden by the mesh. */
export const lift = (pts: Vec3[], dz = 0.12): Vec3[] => pts.map(p => [p[0], p[1], p[2] + dz]);
