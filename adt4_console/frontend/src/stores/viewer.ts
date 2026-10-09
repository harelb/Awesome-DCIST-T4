// Map-viewer state: overlaid graph sources (heavy data kept in mutable GraphModel
// objects, React re-renders keyed on `rev`) plus all viewer UI toggles.
import {create} from "zustand";
import {decodeDelta, decodeGraph} from "../codec";
import {GraphModel} from "../lib/graphModel";
import {errorText, getJSON, getMsgpack} from "../lib/api";
import type {GraphSource, TileManifest} from "../types";
import {ws} from "../ws";

export interface SourceView {
  id: string;
  kind: GraphSource["kind"];
  label: string;
  enabled: boolean;
  status: "idle" | "loading" | "ready" | "error";
  error: string | null;
  model: GraphModel | null;
  rev: number;
  version: number | null;
  manifest: TileManifest | null;
  manifestRev: number;
  trajectory: [number, number, number][] | null;
  showMesh: boolean;
  showGraph: boolean;
}

export type PickMode = "inspect" | "goto";

export interface Selection {
  src: string;
  sym: string;
}

interface ViewerStore {
  sources: Record<string, SourceView>;
  order: string[];
  catalog: GraphSource[] | null;
  catalogError: string | null;
  // display
  hiddenLayers: Record<string, boolean>; // "layer/part" -> hidden
  edgeKinds: Record<string, boolean>; // kind -> visible
  flat: boolean;
  stackHeight: number;
  showBBoxes: boolean;
  showTrajectory: boolean;
  topDown: boolean;
  follow: string | null; // robot name
  meshOpacity: number;
  planRobot: string | null;
  pddlDraft: string;
  // interaction
  selected: Selection | null;
  query: string;
  highlight: Selection[];
  flyTo: {target: [number, number, number]; nonce: number} | null;
  pickMode: PickMode;
  gotoDraft: {sym: string; src: string; pos: [number, number, number]; label: string}[];
  // actions
  loadCatalog: () => Promise<void>;
  enableSource: (id: string, on: boolean) => void;
  reloadSource: (id: string) => Promise<void>;
  patchSource: (id: string, patch: Partial<SourceView>) => void;
  set: (patch: Partial<ViewerStore>) => void;
  select: (sel: Selection | null) => void;
  fly: (target: [number, number, number]) => void;
}

const subs = new Map<string, () => void>();

function blankSource(s: GraphSource): SourceView {
  return {
    id: s.id,
    kind: s.kind,
    label: s.label || s.id,
    enabled: false,
    status: "idle",
    error: null,
    model: null,
    rev: 0,
    version: s.version ?? null,
    manifest: null,
    manifestRev: 0,
    trajectory: null,
    showMesh: true,
    showGraph: true,
  };
}

export const srcPath = (id: string) => `api/graph/${encodeURIComponent(id)}`;

export const useViewer = create<ViewerStore>((set, get) => ({
  sources: {},
  order: [],
  catalog: null,
  catalogError: null,
  // sub-keyframes are dense camera anchors; off by default
  hiddenLayers: {"2/115": true},
  edgeKinds: {},
  flat: false,
  stackHeight: 3,
  showBBoxes: true,
  showTrajectory: true,
  topDown: false,
  follow: null,
  meshOpacity: 1,
  planRobot: null,
  pddlDraft: "",
  selected: null,
  query: "",
  highlight: [],
  flyTo: null,
  pickMode: "inspect",
  gotoDraft: [],

  loadCatalog: async () => {
    try {
      const raw = await getJSON<GraphSource[] | {sources: GraphSource[]}>("api/graph/sources");
      const list = Array.isArray(raw) ? raw : raw.sources ?? [];
      set(s => {
        const sources = {...s.sources};
        for (const g of list) {
          sources[g.id] = sources[g.id]
            ? {...sources[g.id], label: g.label || g.id, kind: g.kind}
            : blankSource(g);
        }
        const order = [...new Set([...s.order.filter(id => sources[id]), ...list.map(g => g.id)])];
        return {catalog: list, catalogError: null, sources, order};
      });
    } catch (e) {
      set({catalogError: errorText(e)});
    }
  },

  patchSource: (id, patch) =>
    set(s => (s.sources[id] ? {sources: {...s.sources, [id]: {...s.sources[id], ...patch}}} : {})),

  enableSource: (id, on) => {
    const src = get().sources[id];
    if (!src) return;
    get().patchSource(id, {enabled: on});
    if (on) {
      if (!subs.has(id)) subs.set(id, subscribeSource(id));
      if (src.status === "idle" || src.status === "error") void get().reloadSource(id);
    } else {
      subs.get(id)?.();
      subs.delete(id);
    }
  },

  reloadSource: async id => {
    const {patchSource} = get();
    patchSource(id, {status: "loading", error: null});
    try {
      const [snap, manifest, traj] = await Promise.all([
        getMsgpack(`${srcPath(id)}/snapshot`),
        getJSON<TileManifest>(`${srcPath(id)}/mesh/manifest`).catch(() => null),
        getJSON<{points: number[][]}>(`${srcPath(id)}/trajectory`).catch(() => null),
      ]);
      const g = decodeGraph(snap);
      const cur = get().sources[id];
      const model = cur?.model ?? new GraphModel();
      model.loadSnapshot(g);
      model.lastSeq = g.seq ?? null;
      patchSource(id, {
        status: "ready",
        model,
        rev: model.rev,
        version: g.version ?? cur?.version ?? null,
        // a failed manifest fetch keeps the tiles we already show
        manifest: manifest ?? cur?.manifest ?? {tile_size: 8, tiles: {}},
        manifestRev: (cur?.manifestRev ?? 0) + 1,
        trajectory: traj?.points?.length ? traj.points.map(p => [p[1], p[2], p[3]] as [number, number, number]) : null,
      });
    } catch (e) {
      patchSource(id, {status: "error", error: errorText(e)});
    }
  },

  set: patch => set(patch as Partial<ViewerStore>),
  select: sel => set({selected: sel}),
  fly: target => set(s => ({flyTo: {target, nonce: (s.flyTo?.nonce ?? 0) + 1}})),
}));

async function refetchManifest(id: string) {
  try {
    const manifest = await getJSON<TileManifest>(`${srcPath(id)}/mesh/manifest`);
    const cur = useViewer.getState().sources[id];
    useViewer.getState().patchSource(id, {manifest, manifestRev: (cur?.manifestRev ?? 0) + 1});
  } catch {
    /* keep old */
  }
}

/** WS subscription for one source: version bumps, live deltas, tile changes. */
function subscribeSource(id: string): () => void {
  const prefix = `graph/${id}/`;
  return ws.on(prefix, (data, frame) => {
    const kind = frame.topic.slice(prefix.length);
    const st = useViewer.getState();
    const src = st.sources[id];
    if (!src?.enabled) return;
    if (kind === "version") {
      const v = (data as {version?: number})?.version ?? null;
      if (v !== src.version) {
        st.patchSource(id, {version: v});
        void st.reloadSource(id);
      }
    } else if (kind === "delta") {
      if (!src.model || src.status !== "ready") return;
      const d = decodeDelta(data);
      const last = src.model.lastSeq;
      // already folded into the snapshot we hold
      if (d.seq !== undefined && last !== null && d.seq <= last && d.seq !== 1) return;
      if (d.seq !== undefined && last !== null && d.seq > last + 1) {
        // missed frames (queue dropped them): resync from a fresh snapshot
        void st.reloadSource(id);
        return;
      }
      src.model.applyDelta(d);
      st.patchSource(id, {rev: src.model.rev});
    } else if (kind === "tiles") {
      void refetchManifest(id);
    }
  });
}

/** All enabled + ready models (for picking / nearest-place). */
export function readyModels(): {id: string; model: GraphModel}[] {
  const s = useViewer.getState();
  return s.order
    .map(id => s.sources[id])
    .filter(v => v?.enabled && v.model)
    .map(v => ({id: v.id, model: v.model!}));
}
