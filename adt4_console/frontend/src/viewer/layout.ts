// Pure scene-graph -> render-buffer layout (hydra_visualizer-style layer stacking).
import type {NodeRecord} from "../codec";
import {GraphModel, layerInfo, type LayerInfo} from "../lib/graphModel";
import {LAYER_COLORS, nodeColor} from "../lib/palette";

export interface LayoutOpts {
  hiddenLayers: Record<string, boolean>;
  edgeKinds: Record<string, boolean>;
  flat: boolean;
  stackHeight: number;
  labelFilter?: Set<number> | null;
}

export interface LayerBuffers {
  key: string;
  info: LayerInfo;
  syms: string[];
  positions: Float32Array; // displayed (offset) positions
  colors: Float32Array; // 0..1
  size: number;
}

export interface GraphLayout {
  layers: LayerBuffers[];
  bboxes: {positions: Float32Array; colors: Float32Array};
  edges: Record<string, Float32Array>;
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

export function zOffset(info: LayerInfo, flat: boolean, stack: number): number {
  return flat ? 0 : info.order * stack;
}

/** Displayed position of a node (true position + layer stacking offset). */
export function displayPos(n: NodeRecord, flat: boolean, stack: number): [number, number, number] {
  const info = layerInfo(n.layer, n.part);
  return [n.pos[0], n.pos[1], n.pos[2] + zOffset(info, flat, stack)];
}

export const edgeKind = (a: NodeRecord, b: NodeRecord) => {
  const ka = layerInfo(a.layer, a.part).kind;
  const kb = layerInfo(b.layer, b.part).kind;
  return ka < kb ? `${ka}-${kb}` : `${kb}-${ka}`;
};

/** sRGB -> linear (vertex colours are linear in three.js). */
const lin = (c: number) => Math.pow(c, 2.2);

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
    const z = zOffset(info, opts.flat, opts.stackHeight);
    const shown = opts.labelFilter && info.kind === "objects" ? nodes.filter(n => opts.labelFilter!.has(n.label)) : nodes;
    const positions = new Float32Array(shown.length * 3);
    const colors = new Float32Array(shown.length * 3);
    const syms: string[] = new Array(shown.length);
    shown.forEach((n, i) => {
      positions[i * 3] = n.pos[0];
      positions[i * 3 + 1] = n.pos[1];
      positions[i * 3 + 2] = n.pos[2] + z;
      const c = info.kind === "objects" || info.kind === "segments" ? nodeColor(n.label, n.color) : LAYER_COLORS[info.kind] ?? LAYER_COLORS.other;
      colors[i * 3] = c[0] / 255;
      colors[i * 3 + 1] = c[1] / 255;
      colors[i * 3 + 2] = c[2] / 255;
      syms[i] = n.sym;
      if (info.kind === "objects" && n.bbox) {
        const b = n.bbox;
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
    });
    layers.push({key, info, syms, positions, colors, size: NODE_SIZE[info.kind]});
  }
  layers.sort((a, b) => a.info.order - b.info.order || a.key.localeCompare(b.key));

  // edges, grouped by kind
  const buckets = new Map<string, number[]>();
  const counts: Record<string, number> = {};
  for (const [s, d] of model.edges.values()) {
    const a = model.nodes.get(s);
    const b = model.nodes.get(d);
    if (!a || !b) continue;
    const kind = edgeKind(a, b);
    counts[kind] = (counts[kind] ?? 0) + 1;
    if (opts.edgeKinds[kind] === false || (opts.edgeKinds[kind] === undefined && !DEFAULT_EDGE_ON(kind))) continue;
    if (opts.hiddenLayers[`${a.layer}/${a.part}`] || opts.hiddenLayers[`${b.layer}/${b.part}`]) continue;
    const pa = displayPos(a, opts.flat, opts.stackHeight);
    const pb = displayPos(b, opts.flat, opts.stackHeight);
    let arr = buckets.get(kind);
    if (!arr) buckets.set(kind, (arr = []));
    arr.push(...pa, ...pb);
  }
  const edges: Record<string, Float32Array> = {};
  for (const [k, v] of buckets) edges[k] = new Float32Array(v);

  // agent trajectory from agent nodes ordered by symbol index
  const agents = [...model.nodes.values()].filter(n => layerInfo(n.layer, n.part).kind === "agents");
  let agentPath: Float32Array | null = null;
  if (agents.length > 1) {
    agents.sort((a, b) => symIndex(a.sym) - symIndex(b.sym));
    agentPath = new Float32Array(agents.length * 3);
    agents.forEach((n, i) => agentPath!.set(displayPos(n, opts.flat, opts.stackHeight), i * 3));
  }

  return {
    layers,
    bboxes: {positions: new Float32Array(bbPos), colors: new Float32Array(bbCol)},
    edges,
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
