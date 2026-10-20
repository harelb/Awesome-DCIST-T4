// Pure scene-graph -> render-buffer layout (hydra_visualizer-style layer stacking).
import type {NodeRecord} from "../codec";
import {GraphModel, layerInfo, type LayerInfo} from "../lib/graphModel";
import {LAYER_COLORS, nodeColor} from "../lib/palette";

/** "hydra": mimic the ADT4 hydra_visualizer default config
 *  (dcist_launch_system/config_generation/base_params/hydra_visualizer.yaml);
 *  "classic": the console's original look. */
export type VizStyle = "hydra" | "classic";

export interface LayoutOpts {
  hiddenLayers: Record<string, boolean>;
  edgeKinds: Record<string, boolean>;
  flat: boolean;
  /** layer spacing; in hydra style this is `layer_z_step` */
  stackHeight: number;
  labelFilter?: Set<number> | null;
  style?: VizStyle; // default "classic"
  /** collect text labels (hydra text adapters) */
  text?: boolean;
}

export interface TextLabel {
  pos: [number, number, number];
  text: string;
  kind: LayerInfo["kind"];
}

/** Per-kind label caps: DOM labels get expensive past a few hundred. */
export const TEXT_LIMITS: Partial<Record<LayerInfo["kind"], number>> = {objects: 400, places: 300, rooms: 200, buildings: 50};
/** hydra style: kinds with per-node text, and kinds with one layer label (`draw_layer`). */
export const HYDRA_NODE_TEXT = new Set<LayerInfo["kind"]>(["objects", "places", "rooms"]);
export const HYDRA_LAYER_TEXT = new Set<LayerInfo["kind"]>(["agents"]);
/** hydra text `height` (metres above the node) */
const TEXT_HEIGHT: Partial<Record<LayerInfo["kind"], number>> = {objects: 0.5, places: 0.5, rooms: 1.25, buildings: 1.5};

export interface LayerBuffers {
  key: string;
  info: LayerInfo;
  syms: string[];
  positions: Float32Array; // displayed (offset) positions
  colors: Float32Array; // 0..1
  size: number;
  /** node alpha (hydra `alpha`) */
  opacity: number;
  /** spheres or cubes (hydra `use_sphere`) */
  shape: "sphere" | "box";
  /** object layers: displayed bounding boxes, 6 floats per sym (min xyz, max xyz),
   *  NaN where a node has none — used to pick objects by their wireframe box */
  boxes?: Float32Array;
}

export interface GraphLayout {
  layers: LayerBuffers[];
  bboxes: {positions: Float32Array; colors: Float32Array};
  edges: Record<string, Float32Array>;
  /** per-vertex colours for edge kinds drawn in their child's colour (hydra use_child_color) */
  edgeColors: Record<string, Float32Array>;
  /** hydra RegionGrowingBoundaryPlugin fill: place polygons (triangle list, node colour) */
  placeFill: {positions: Float32Array; colors: Float32Array} | null;
  /** hydra RegionGrowingBoundaryPlugin boundary: state-coloured line list (segment pairs) */
  placeBoundary: {positions: Float32Array; colors: Float32Array} | null;
  texts: TextLabel[];
  edgeCounts: Record<string, number>;
  agentPath: Float32Array | null;
  bounds: {min: [number, number, number]; max: [number, number, number]} | null;
}

export const NODE_SIZE: Record<LayerInfo["kind"], number> = {
  objects: 0.28,
  agents: 0.12,
  subkeyframes: 0.06,
  places: 0.16,
  rooms: 0.55,
  buildings: 0.9,
  segments: 0.15,
  other: 0.2,
};

/** hydra_visualizer.yaml, per layer: z_offset_scale, node scale (sphere
 *  diameter / cube edge), alpha, use_sphere. Layers the config does not list
 *  get neighbouring values. */
export const HYDRA_LAYERS: Record<LayerInfo["kind"], {z: number; scale: number; alpha: number; sphere: boolean}> = {
  agents: {z: 0, scale: 0.1, alpha: 1, sphere: true}, // 2p*
  subkeyframes: {z: 0, scale: 0.06, alpha: 1, sphere: true}, // 2p*
  objects: {z: 1, scale: 0.4, alpha: 0.8, sphere: false}, // 2
  segments: {z: 1, scale: 0.15, alpha: 0.8, sphere: true},
  places: {z: 2, scale: 0.2, alpha: 0.9, sphere: true}, // 3p1
  rooms: {z: 3, scale: 0.6, alpha: 0.8, sphere: true}, // 4
  buildings: {z: 4, scale: 0.9, alpha: 0.8, sphere: true},
  other: {z: 3, scale: 0.2, alpha: 0.8, sphere: true},
};
export const HYDRA_LAYER_Z_STEP = 2.0;

/** RegionGrowingBoundaryPlugin as configured in hydra_visualizer.yaml
 *  ({fill_boundaries: true, fill_alpha: 0.3}, defaults otherwise): boundary
 *  lines 0.07 m wide at the node alpha (3p1: 0.9), coloured per
 *  TraversabilityState with spark_dsg Color::gray/blue/red/green. */
export const PLACE_BOUNDARY = {
  lineWidth: 0.07,
  lineAlpha: HYDRA_LAYERS.places.alpha,
  fillAlpha: 0.3,
  /** indexed by TraversabilityState: UNKNOWN, TRAVERSABLE, INTRAVERSABLE, TRAVERSED (0..255 sRGB) */
  stateColors: [
    [128, 128, 128],
    [0, 0, 255],
    [255, 0, 0],
    [0, 255, 0],
  ] as [number, number, number][],
};

/** spark_dsg colormaps::colorbrewerPalette (IdColorAdapter / PartitionColorAdapter). */
export const COLORBREWER: [number, number, number][] = [
  [166, 206, 227], [31, 120, 180], [178, 223, 138], [51, 160, 44], [251, 154, 153], [227, 26, 28],
  [253, 191, 111], [255, 127, 0], [202, 178, 214], [106, 61, 154], [255, 255, 153], [177, 89, 40],
];

export function zOffset(info: LayerInfo, flat: boolean, stack: number, style: VizStyle = "classic"): number {
  if (flat) return 0;
  return (style === "hydra" ? HYDRA_LAYERS[info.kind].z : info.order) * stack;
}

/** Displayed position of a node (true position + layer stacking offset). */
export function displayPos(n: NodeRecord, flat: boolean, stack: number, style: VizStyle = "classic"): [number, number, number] {
  const info = layerInfo(n.layer, n.part);
  return [n.pos[0], n.pos[1], n.pos[2] + zOffset(info, flat, stack, style)];
}

/** Node colour per style (0..255 sRGB). */
export function styleColor(n: NodeRecord, info: LayerInfo, style: VizStyle): [number, number, number] {
  if (style === "hydra") {
    switch (info.kind) {
      case "objects":
      case "segments":
        return nodeColor(n.label, n.color); // LabelColorAdapter
      case "places":
        return [0, 0, 0]; // UniformColorAdapter [0, 0, 0]
      case "rooms":
        return COLORBREWER[symIndex(n.sym) % COLORBREWER.length]; // IdColorAdapter colorbrewer
      case "agents":
      case "subkeyframes":
        return COLORBREWER[n.part % COLORBREWER.length]; // PartitionColorAdapter
      default:
        return LAYER_COLORS[info.kind] ?? LAYER_COLORS.other;
    }
  }
  return info.kind === "objects" || info.kind === "segments" ? nodeColor(n.label, n.color) : LAYER_COLORS[info.kind] ?? LAYER_COLORS.other;
}

export const edgeKind = (a: NodeRecord, b: NodeRecord) => {
  const ka = layerInfo(a.layer, a.part).kind;
  const kb = layerInfo(b.layer, b.part).kind;
  return ka < kb ? `${ka}-${kb}` : `${kb}-${ka}`;
};

/** sRGB -> linear (vertex colours are linear in three.js). */
const lin = (c: number) => Math.pow(c, 2.2);

/** Append one place's RegionGrowingBoundaryPlugin geometry to the merged
 *  buffers (linear-RGB vertex colours).  Mirrors the C++ plugin: boundary point
 *  i = pos + radii[i]·(cos θ, sin θ, 0) with θ = i/N·2π, lifted by the layer
 *  z offset; segment i → (i+1)%N is coloured states[i] → states[(i+1)%N]
 *  (a gradient on mixed segments); the fill is the boundary polygon in the
 *  node colour.  The plugin ear-clips that polygon; the rays make it
 *  star-shaped around pos, so the fan (pos, p_i, p_i+1) covers exactly the
 *  same area with the same number of triangles and no overlaps.
 *  Without radii (peers / sources that carry none) the bbox footprint stands
 *  in, outlined in the UNKNOWN colour. Returns false when nothing was drawn. */
export function appendPlaceBoundary(
  n: Pick<NodeRecord, "pos" | "bbox" | "radii" | "states">,
  z: number,
  nodeRgb: [number, number, number],
  linePos: number[],
  lineCol: number[],
  fillPos: number[],
  fillCol: number[],
): boolean {
  const fr = lin(nodeRgb[0] / 255);
  const fg = lin(nodeRgb[1] / 255);
  const fb = lin(nodeRgb[2] / 255);
  const pz = n.pos[2] + z;
  const sc = (s: number | undefined) => {
    const c = PLACE_BOUNDARY.stateColors[s ?? 0] ?? PLACE_BOUNDARY.stateColors[0];
    return [lin(c[0] / 255), lin(c[1] / 255), lin(c[2] / 255)];
  };
  let pts: number[] = [];
  let states: (number | undefined)[] = [];
  const radii = n.radii;
  if (radii && radii.length > 0) {
    const N = radii.length;
    for (let i = 0; i < N; i++) {
      const a = (i / N) * 2 * Math.PI;
      pts.push(n.pos[0] + radii[i] * Math.cos(a), n.pos[1] + radii[i] * Math.sin(a));
      states.push(n.states?.[i]);
    }
  } else if (n.bbox) {
    const b = n.bbox;
    pts = [b[0], b[1], b[3], b[1], b[3], b[4], b[0], b[4]];
    states = [0, 0, 0, 0];
  } else return false;
  const N = pts.length / 2;
  for (let i = 0; i < N; i++) {
    const j = (i + 1) % N;
    linePos.push(pts[i * 2], pts[i * 2 + 1], pz, pts[j * 2], pts[j * 2 + 1], pz);
    lineCol.push(...sc(states[i]), ...sc(states[j]));
    fillPos.push(n.pos[0], n.pos[1], pz, pts[i * 2], pts[i * 2 + 1], pz, pts[j * 2], pts[j * 2 + 1], pz);
    for (let k = 0; k < 3; k++) fillCol.push(fr, fg, fb);
  }
  return true;
}

const BOX_EDGES = [
  [0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4], [0, 4], [1, 5], [2, 6], [3, 7],
];

export function layoutGraph(model: GraphModel, opts: LayoutOpts): GraphLayout {
  const groups = new Map<string, NodeRecord[]>();
  for (const n of model.nodes.values()) {
    const key = `${n.layer}/${n.part}`;
    let g = groups.get(key);
    if (!g) groups.set(key, (g = []));
    g.push(n);
  }
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  const layers: LayerBuffers[] = [];
  const bbPos: number[] = [];
  const bbCol: number[] = [];
  const fillPos: number[] = [];
  const fillCol: number[] = [];
  const lineBPos: number[] = [];
  const lineBCol: number[] = [];
  const texts: TextLabel[] = [];
  const style: VizStyle = opts.style ?? "classic";
  for (const [key, nodes] of groups) {
    const info = layerInfo(nodes[0].layer, nodes[0].part);
    for (const n of nodes)
      for (let k = 0; k < 3; k++) {
        if (Number.isFinite(n.pos[k])) {
          min[k] = Math.min(min[k], n.pos[k]);
          max[k] = Math.max(max[k], n.pos[k]);
        }
      }
    if (opts.hiddenLayers[key]) continue;
    const z = zOffset(info, opts.flat, opts.stackHeight, style);
    const shown = opts.labelFilter && info.kind === "objects" ? nodes.filter(n => opts.labelFilter!.has(n.label)) : nodes;
    const positions = new Float32Array(shown.length * 3);
    const colors = new Float32Array(shown.length * 3);
    const syms: string[] = new Array(shown.length);
    const boxes = info.kind === "objects" ? new Float32Array(shown.length * 6).fill(NaN) : undefined;
    shown.forEach((n, i) => {
      positions[i * 3] = n.pos[0];
      positions[i * 3 + 1] = n.pos[1];
      positions[i * 3 + 2] = n.pos[2] + z;
      const c = styleColor(n, info, style);
      colors[i * 3] = c[0] / 255;
      colors[i * 3 + 1] = c[1] / 255;
      colors[i * 3 + 2] = c[2] / 255;
      syms[i] = n.sym;
      if (info.kind === "objects" && n.bbox) {
        const b = n.bbox;
        boxes?.set([b[0], b[1], b[2] + z, b[3], b[4], b[5] + z], i * 6);
        const corners: number[][] = [];
        for (let cz = 0; cz < 2; cz++)
          for (let cy = 0; cy < 2; cy++)
            for (let cx = 0; cx < 2; cx++) corners.push([cx ? b[3] : b[0], cy ? b[4] : b[1], (cz ? b[5] : b[2]) + z]);
        for (const [a, bb] of BOX_EDGES) {
          bbPos.push(...corners[a], ...corners[bb]);
          const r = lin(colors[i * 3]);
          const g = lin(colors[i * 3 + 1]);
          const bl = lin(colors[i * 3 + 2]);
          bbCol.push(r, g, bl, r, g, bl);
        }
      }
      // hydra 3p1 RegionGrowingBoundaryPlugin: state-coloured ray boundary + filled polygon
      if (style === "hydra" && info.kind === "places") appendPlaceBoundary(n, z, c, lineBPos, lineBCol, fillPos, fillCol);
    });
    const cap = TEXT_LIMITS[info.kind];
    // hydra_visualizer.yaml sets per-node `text.draw: true` only for 2 (objects),
    // 3p1 (places) and 4 (rooms); 2p* has `draw_layer: true` = one layer label
    const perNode = style !== "hydra" || HYDRA_NODE_TEXT.has(info.kind);
    if (opts.text && style === "hydra" && HYDRA_LAYER_TEXT.has(info.kind) && shown.length) {
      // the layer label sits above the partition's newest node (the robot's latest keyframe)
      const last = shown.reduce((a, b) => (symIndex(b.sym) > symIndex(a.sym) ? b : a));
      const chr = String.fromCharCode(info.key.split("/").map(Number)[1] || 32).trim();
      texts.push({pos: [last.pos[0], last.pos[1], last.pos[2] + z + 0.9], text: `${info.name}${chr ? ` ${chr}` : ""}`, kind: "agents"});
    }
    if (opts.text && perNode && cap && shown.length <= cap) {
      const dz = TEXT_HEIGHT[info.kind] ?? 0.5;
      for (const n of shown) {
        let text = n.sym; // IdTextAdapter
        if (info.kind === "objects") text = `${model.labelName(n) || n.name || "?"} ${n.sym}`; // LabelIdTextAdapter
        else if (info.kind === "rooms" || info.kind === "buildings") text = n.name ? `${n.name} ${n.sym}` : n.sym; // NameIdTextAdapter
        texts.push({pos: [n.pos[0], n.pos[1], n.pos[2] + z + dz], text, kind: info.kind});
      }
    }
    const h = HYDRA_LAYERS[info.kind];
    layers.push(
      style === "hydra"
        ? {key, info, syms, positions, colors, size: h.sphere ? h.scale / 2 : h.scale / 1.4, opacity: h.alpha, shape: h.sphere ? "sphere" : "box", boxes}
        : {key, info, syms, positions, colors, size: NODE_SIZE[info.kind], opacity: 1, shape: info.kind === "objects" ? "box" : "sphere", boxes},
    );
  }
  layers.sort((a, b) => a.info.order - b.info.order || a.key.localeCompare(b.key));

  // edges, grouped by kind
  const buckets = new Map<string, number[]>();
  const colBuckets = new Map<string, number[]>();
  const counts: Record<string, number> = {};
  for (const [s, d] of model.edges.values()) {
    const a = model.nodes.get(s);
    const b = model.nodes.get(d);
    if (!a || !b) continue;
    const kind = edgeKind(a, b);
    counts[kind] = (counts[kind] ?? 0) + 1;
    if (opts.edgeKinds[kind] === false || (opts.edgeKinds[kind] === undefined && !defaultEdgeOn(kind, style))) continue;
    if (opts.hiddenLayers[`${a.layer}/${a.part}`] || opts.hiddenLayers[`${b.layer}/${b.part}`]) continue;
    const pa = displayPos(a, opts.flat, opts.stackHeight, style);
    const pb = displayPos(b, opts.flat, opts.stackHeight, style);
    let arr = buckets.get(kind);
    if (!arr) buckets.set(kind, (arr = []));
    arr.push(...pa, ...pb);
    if (style === "hydra" && kind === "objects-places") {
      // interlayer 3 -> 2: use_child_color (the object's colour)
      const child = layerInfo(a.layer, a.part).kind === "objects" ? a : b;
      const c = styleColor(child, layerInfo(child.layer, child.part), style);
      let ca = colBuckets.get(kind);
      if (!ca) colBuckets.set(kind, (ca = []));
      const r = lin(c[0] / 255);
      const g = lin(c[1] / 255);
      const bl = lin(c[2] / 255);
      ca.push(r, g, bl, r, g, bl);
    }
  }
  const edges: Record<string, Float32Array> = {};
  for (const [k, v] of buckets) edges[k] = new Float32Array(v);
  const edgeColors: Record<string, Float32Array> = {};
  for (const [k, v] of colBuckets) edgeColors[k] = new Float32Array(v);

  // agent trajectory from agent nodes ordered by symbol index
  const agents = [...model.nodes.values()].filter(n => layerInfo(n.layer, n.part).kind === "agents");
  let agentPath: Float32Array | null = null;
  if (agents.length > 1) {
    agents.sort((a, b) => symIndex(a.sym) - symIndex(b.sym));
    agentPath = new Float32Array(agents.length * 3);
    agents.forEach((n, i) => agentPath!.set(displayPos(n, opts.flat, opts.stackHeight, style), i * 3));
  }

  return {
    layers,
    bboxes: {positions: new Float32Array(bbPos), colors: new Float32Array(bbCol)},
    edges,
    edgeColors,
    texts,
    placeFill: fillPos.length ? {positions: new Float32Array(fillPos), colors: new Float32Array(fillCol)} : null,
    placeBoundary: lineBPos.length ? {positions: new Float32Array(lineBPos), colors: new Float32Array(lineBCol)} : null,
    edgeCounts: counts,
    agentPath,
    bounds: Number.isFinite(min[0]) ? robustBounds(model) : null,
  };
}

/** 2nd-98th percentile box of node positions, so a few outliers don't wreck framing. */
function robustBounds(model: GraphModel): {min: [number, number, number]; max: [number, number, number]} {
  const axes: number[][] = [[], [], []];
  for (const n of model.nodes.values()) for (let k = 0; k < 3; k++) if (Number.isFinite(n.pos[k])) axes[k].push(n.pos[k]);
  const q = (a: number[], f: number) => a[Math.min(a.length - 1, Math.max(0, Math.round(f * (a.length - 1))))];
  const min = [0, 0, 0] as [number, number, number];
  const max = [0, 0, 0] as [number, number, number];
  axes.forEach((a, k) => {
    a.sort((x, y) => x - y);
    min[k] = q(a, 0.02);
    max[k] = q(a, 0.98);
  });
  return {min, max};
}

export const DEFAULT_EDGE_ON = (kind: string) => !kind.includes("buildings") && !kind.includes("subkeyframes") && kind !== "places-rooms" && kind !== "agents-places" && kind !== "agents-objects";

/** Edge kinds drawn by default. hydra_visualizer draws 4 -> 3* (rooms-places) and
 *  skips 3* -> 2p* (agents-places). */
export const defaultEdgeOn = (kind: string, style: VizStyle = "classic") =>
  style === "hydra" ? kind === "places-rooms" || DEFAULT_EDGE_ON(kind) : DEFAULT_EDGE_ON(kind);

/** hydra edge styling (colour, alpha) per kind; classic uses EDGE_COLORS. */
export const HYDRA_EDGES: Record<string, {color: string; alpha: number}> = {
  "places-places": {color: "#000000", alpha: 0.5}, // 3p1 edges 0.06 / 0.5
  "rooms-rooms": {color: "#000000", alpha: 0.2}, // 4 edges 0.1 / 0.2
  "objects-places": {color: "#000000", alpha: 0.5}, // 3 -> 2 child colour, 0.08 / 0.5
  "places-rooms": {color: "#000000", alpha: 0.5}, // 4 -> 3, 0.08 / 0.5
  "agents-agents": {color: "#1f78b4", alpha: 1}, // 2p* edges, partition colour
};

export function symIndex(sym: string): number {
  const m = /\((\d+)\)$/.exec(sym);
  return m ? Number(m[1]) : 0;
}

export const EDGE_COLORS: Record<string, string> = {
  "agents-agents": "#ff7a1a",
  "places-places": "#3fb8e8",
  "objects-places": "#8a96a3",
  "places-rooms": "#e05ad0",
  "rooms-rooms": "#e05ad0",
  "buildings-rooms": "#ffffff",
};
