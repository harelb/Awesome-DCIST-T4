// Map editing (API.md §17): REST wrappers, op builders and the pure helpers the
// edit mode uses (selection summary, lasso membership, subtree, symbol forms).
import {deleteJSON, getJSON, postJSON, sendJSON} from "./api";
import {layerInfo, type GraphModel} from "./graphModel";
import type {NodeRecord} from "../codec";

export type EditLayer = "Object" | "Place" | "MeshPlace" | "Room" | "Building";

export interface EditOp {
  op: string;
  [k: string]: unknown;
}

export interface EditChange {
  kind: string;
  symbol?: string;
  layer?: string;
  field?: string;
  old?: unknown;
  new?: unknown;
  [k: string]: unknown;
}

export interface EditResult {
  mutation_id: string | null;
  revision: number;
  changes: EditChange[];
  summary: string[];
  affected: string[];
  map_version: number | null;
  staged: boolean;
  dry_run?: boolean;
  /** staged answers */
  count?: number;
  staged_summary?: string[];
}

export interface StagedEdit {
  op: EditOp;
  t: number;
  author: string | null;
  summary: string[];
}

export interface EditState {
  src: string;
  mode: "immediate" | "staged";
  live: boolean;
  launch: {experiment?: string; robots?: string[]; started_at?: number} | null;
  revision: number;
  state_version: number;
  map_version: number;
  staged: StagedEdit[];
  can_undo: boolean;
}

export interface HistoryItem {
  mutation_id: string;
  revision: number;
  kind: string;
  committed_at: string | null;
  summary: string;
  provenance: Record<string, unknown>;
  undo_of: string | null;
  undone_by: string | null;
  undoable: boolean;
  n_ops: number;
  affected: string[];
  map_version: number | null;
}

export interface Region {
  symbol: string;
  name: string | null;
  class: string | null;
  center: number[] | null;
  polygon: number[][] | null;
  /** "drawn": the area's own boundary; "members": the ground its places cover (drawn by the graph layer) */
  polygon_source?: "drawn" | "members" | null;
  source: string | null;
  members: Record<string, number>;
}

export interface SnapshotInfo {
  id: string;
  name: string;
  created_at: number;
  bytes: number;
  author: string | null;
  mission: string | null;
  counts: Record<string, number>;
  map_version: number;
  revision: number;
}

const base = (src: string) => `api/edit/${encodeURIComponent(src)}`;

export const editApi = {
  state: (src: string) => getJSON<EditState>(`${base(src)}/state`),
  batch: (src: string, ops: EditOp[], summary?: string, mission?: string | null) => postJSON<EditResult>(`${base(src)}/batch`, {ops, summary, mission: mission ?? undefined}),
  preview: (src: string, ops: EditOp[]) => postJSON<EditResult>(`${base(src)}/preview`, {ops}),
  patch: (src: string, sym: string, set: Record<string, unknown>) => sendJSON<EditResult>("PATCH", `${base(src)}/nodes/${encodeURIComponent(sym)}`, {set}),
  retire: (src: string, sym: string) => deleteJSON<EditResult>(`${base(src)}/nodes/${encodeURIComponent(sym)}`),
  hardDelete: (src: string, sym: string) => deleteJSON<EditResult>(`${base(src)}/nodes/${encodeURIComponent(sym)}?hard=true&confirm=delete`),
  group: (src: string, body: {name: string; class?: string; polygon?: number[][]; members?: string[]; member_layers?: string[]}) => postJSON<EditResult>(`${base(src)}/group`, {class: "region", ...body}),
  history: (src: string, limit = 50) => getJSON<{history: HistoryItem[]}>(`${base(src)}/history?limit=${limit}`),
  undo: (src: string, mutationId?: string | null, force = false) => postJSON<EditResult>(`${base(src)}/undo`, {mutation_id: mutationId ?? undefined, force}),
  regions: (src: string) => getJSON<{regions: Region[]}>(`${base(src)}/regions`),
  staged: (src: string) => getJSON<{staged: StagedEdit[]; count: number; summary: string[]}>(`${base(src)}/staged`),
  publish: (src: string, confirm: string) => postJSON<EditResult & {published: number}>(`${base(src)}/staged/publish`, {confirm}),
  discard: (src: string, index?: number) => deleteJSON<{count: number}>(`${base(src)}/staged${index === undefined ? "" : `/${index}`}`),
  snapshots: (src: string) => getJSON<{snapshots: SnapshotInfo[]}>(`${base(src)}/snapshots`),
  snapshot: (src: string, name: string) => postJSON<SnapshotInfo>(`${base(src)}/snapshots`, {name}),
  restore: (src: string, id: string) => postJSON<{job_id: string}>(`${base(src)}/snapshots/${encodeURIComponent(id)}/restore`),
  downloadUrl: (src: string, id: string) => `${base(src)}/snapshots/${encodeURIComponent(id)}/download`,
  deleteSnapshot: (src: string, id: string) => deleteJSON<{ok: boolean}>(`${base(src)}/snapshots/${encodeURIComponent(id)}`),
};

// ------------------------------------------------------------------ helpers

/** The heracles label of a viewer node (null for agents, segments, ...). */
export function editLayerOf(n: Pick<NodeRecord, "layer" | "part">): EditLayer | null {
  if (n.layer === 2 && n.part === 0) return "Object";
  if (n.layer === 3 && n.part === 0) return "Place";
  if ((n.layer === 3 && n.part === 1) || n.layer === 20) return "MeshPlace";
  if (n.layer === 4) return "Room";
  if (n.layer === 5) return "Building";
  return null;
}

const PLURAL: Record<EditLayer, [string, string]> = {
  Object: ["Object", "Objects"],
  Place: ["Place", "Places"],
  MeshPlace: ["Place", "Places"],
  Room: ["Area", "Areas"], // Room nodes are "areas" in the UI
  Building: ["Building", "Buildings"],
};

/** "3 Objects, 1 Area" (SGET's status line). */
export function selectionSummary(syms: readonly string[], model: GraphModel | null | undefined): string {
  if (!syms.length) return "nothing selected";
  const counts = new Map<string, number>();
  for (const s of syms) {
    const n = model?.nodes.get(s);
    const l = n ? editLayerOf(n) : null;
    const word = l ? PLURAL[l] : ["node", "nodes"];
    const key = word.join("|");
    counts.set(key, (counts.get(key) ?? 0) + 1);
  }
  return [...counts.entries()].map(([k, c]) => `${c} ${k.split("|")[c === 1 ? 0 : 1]}`).join(", ");
}

/** Even-odd point in polygon ([x, y] vertices). */
export function pointInPolygon(x: number, y: number, poly: readonly (readonly number[])[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i];
    const [xj, yj] = poly[j];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi || 1e-300) + xi) inside = !inside;
  }
  return inside;
}

/** Editable nodes of `model` whose position lies in `poly` (map frame = the
 *  model's frame for a Neo4j source), optionally only some layers. */
export function nodesInPolygon(model: GraphModel, poly: readonly (readonly number[])[], layers?: readonly EditLayer[]): string[] {
  if (poly.length < 3) return [];
  const out: string[] = [];
  for (const n of model.nodes.values()) {
    const l = editLayerOf(n);
    if (!l || (layers && !layers.includes(l))) continue;
    if (pointInPolygon(n.pos[0], n.pos[1], poly)) out.push(n.sym);
  }
  return out;
}

/** Descendants of `root` through parent -> child edges (a child is on a lower layer). */
export function descendants(model: GraphModel, root: string): Set<string> {
  const rank = (s: string) => {
    const n = model.nodes.get(s);
    if (!n) return -1;
    if (n.layer === 20) return 3;
    return layerInfo(n.layer, n.part).kind === "agents" ? -1 : n.layer;
  };
  const kids = new Map<string, string[]>();
  for (const [a, b] of model.edges.values()) {
    const ra = rank(a);
    const rb = rank(b);
    if (ra < 0 || rb < 0 || ra === rb) continue;
    const [p, c] = ra > rb ? [a, b] : [b, a];
    const list = kids.get(p);
    if (list) list.push(c);
    else kids.set(p, [c]);
  }
  const out = new Set<string>();
  const stack = [...(kids.get(root) ?? [])];
  while (stack.length) {
    const s = stack.pop()!;
    if (out.has(s)) continue;
    out.add(s);
    for (const c of kids.get(s) ?? []) stack.push(c);
  }
  return out;
}

/** An edit summary line in the UI's words: Room nodes are "areas" (the
 *  server's diff says "new region R1 'yard' containing ..."). */
export function uiSummary(line: string): string {
  return line.replace(/\bnew region [A-Za-z]\(?\d+\)? /g, "new area ").replace(/\bregions?\b/g, m => (m === "region" ? "area" : "areas"));
}

/** The viewer's symbol form ("O(12)") for a stored one ("O12"). */
export function uiSym(s: string): string {
  const m = /^([A-Za-z])\(?(\d+)\)?$/.exec(s);
  return m ? `${m[1]}(${m[2]})` : s;
}

/** The common value of `get` over `items`, or null when they differ. */
export function common<T>(items: readonly T[], get: (t: T) => string): string | null {
  if (!items.length) return null;
  const v = get(items[0]);
  return items.every(i => get(i) === v) ? v : null;
}

/** Ops for the batch editor: set `fields` on every selected node. */
export function batchUpdateOps(syms: readonly string[], fields: Record<string, unknown>): EditOp[] {
  const set = Object.fromEntries(Object.entries(fields).filter(([, v]) => v !== undefined && v !== ""));
  if (!Object.keys(set).length) return [];
  return syms.map(symbol => ({op: "update_node", symbol, set}));
}

/** Connect (or disconnect) two nodes; the backend derives the edge type. */
export function edgeOp(kind: "create_edge" | "delete_edge", a: string, b: string): EditOp {
  return {op: kind, from: a, to: b};
}
