import {useEffect, useMemo, useState} from "react";
import {useViewer, readyModels, srcPath} from "../stores/viewer";
import {layerInfo, searchNodes} from "../lib/graphModel";
import {nodeColor, LAYER_COLORS, rgbCss} from "../lib/palette";
import {errorText, getJSON, postJSON} from "../lib/api";
import {Badge, Dot, Empty, Spinner, Toggle} from "../components/ui";
import type {NodeDetail} from "../types";
import {displayPos, defaultEdgeOn, EDGE_COLORS, HYDRA_EDGES} from "./layout";
import type {TileStats} from "./MeshTiles";
import {fmtBytes} from "../lib/fuzzy";
import {StatsPanel} from "./StatsPanel";
import {applyPoint, robotOfSource} from "../lib/frameAlign";
import {sourceMatrix, useRobots} from "../stores/robots";
import {ObjectImages, SearchThumb, useImaged, type ImageInfo} from "./objectImages";
import {useViewStyle} from "../stores/viewStyle";
import {BackgroundControl, HeightsControl, RegionControls, RobotLookControl, ThemeControl} from "./StylePanels";

const KIND_LABEL: Record<string, string> = {neo4j: "Neo4j", file: "file", live: "live"};

export function SourcesPanel({stats}: {stats: Record<string, TileStats>}) {
  const {sources, order, catalogError, loadCatalog, enableSource, patchSource, reloadSource} = useViewer();
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Sources</h4>
        <button className="btn btn-xs" onClick={() => void loadCatalog()}>
          Refresh
        </button>
      </header>
      {catalogError && <div className="inline-err small">{catalogError}</div>}
      {!catalogError && order.length === 0 && <div className="muted small">No graph sources. Load a map from the Maps page.</div>}
      <ul className="src-list">
        {order.map(id => {
          const s = sources[id];
          if (!s) return null;
          const st = stats[id];
          return (
            <li key={id} className={s.enabled ? "on" : ""}>
              <div className="row gap between">
                <Toggle checked={s.enabled} onChange={v => enableSource(id, v)} label={<span className="src-label">{s.label}</span>} />
                <Badge tone={s.kind === "live" ? "ok" : s.kind === "neo4j" ? "info" : "off"}>{KIND_LABEL[s.kind] ?? s.kind}</Badge>
              </div>
              <div className="src-meta mono small">{id}</div>
              {s.enabled && (
                <div className="src-sub">
                  {s.status === "loading" && <Spinner label="loading" />}
                  {s.status === "error" && (
                    <div className="inline-err small">
                      {s.error}{" "}
                      <button className="btn btn-xs" onClick={() => void reloadSource(id)}>
                        Retry
                      </button>
                    </div>
                  )}
                  {s.status === "ready" && (
                    <>
                      <div className="row gap small">
                        <label className="mini-check">
                          <input type="checkbox" checked={s.showMesh} onChange={e => patchSource(id, {showMesh: e.target.checked})} /> mesh
                        </label>
                        <label className="mini-check">
                          <input type="checkbox" checked={s.showGraph} onChange={e => patchSource(id, {showGraph: e.target.checked})} /> graph
                        </label>
                        <span className="muted">{s.model?.nodes.size ?? 0} nodes</span>
                      </div>
                      {s.kind === "live" && robotOfSource(id) && <MeshLodControl robot={robotOfSource(id)!} />}
                      {st && st.total > 0 && (
                        <div className="tile-bar" title={`${st.loaded}/${st.total} tiles`}>
                          <div className="tile-fill" style={{width: `${(100 * st.loaded) / st.total}%`}} />
                          <span className="tile-text mono">
                            {st.loaded}/{st.total} tiles · {fmtBytes(st.triangles).replace("B", "").trim()} tris
                          </span>
                        </div>
                      )}
                    </>
                  )}
                </div>
              )}
            </li>
          );
        })}
      </ul>
    </section>
  );
}

/** Layer swatches in hydra style (places black, rooms/agents colorbrewer). */
const HYDRA_SWATCH: Record<string, string> = {places: "#000000", rooms: "#a6cee3", agents: "#1f78b4", subkeyframes: "#ff7f00"};

export function LayersPanel() {
  const {sources, order, hiddenLayers, edgeKinds, set, vizStyle} = useViewer();
  const hydra = vizStyle === "hydra";
  const revs = order.map(id => sources[id]?.rev).join(",");
  const {layers, edgeCounts} = useMemo(() => {
    const layers = new Map<string, {name: string; kind: string; count: number; order: number}>();
    const edgeCounts: Record<string, number> = {};
    for (const {model} of readyModels()) {
      for (const [k, v] of model.layers()) {
        const cur = layers.get(k);
        if (cur) cur.count += v.count;
        else layers.set(k, {name: v.info.name, kind: v.info.kind, count: v.count, order: v.info.order});
      }
      for (const [a, b] of model.edges.values()) {
        const na = model.nodes.get(a);
        const nb = model.nodes.get(b);
        if (!na || !nb) continue;
        const ka = layerInfo(na.layer, na.part).kind;
        const kb = layerInfo(nb.layer, nb.part).kind;
        const k = ka < kb ? `${ka}-${kb}` : `${kb}-${ka}`;
        edgeCounts[k] = (edgeCounts[k] ?? 0) + 1;
      }
    }
    return {layers: [...layers.entries()].sort((a, b) => a[1].order - b[1].order || a[0].localeCompare(b[0])), edgeCounts};
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [revs, order, sources]);
  if (!layers.length) return null;
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Layers</h4>
      </header>
      {layers.some(([, l]) => l.kind === "rooms") && <RegionControls />}
      <ul className="layer-list">
        {layers.map(([k, l]) => (
          <li key={k}>
            <label className="layer-row">
              <input type="checkbox" checked={!hiddenLayers[k]} onChange={e => set({hiddenLayers: {...hiddenLayers, [k]: !e.target.checked}})} />
              <span
                className="swatch"
                style={{
                  background:
                    hydra && HYDRA_SWATCH[l.kind] ? HYDRA_SWATCH[l.kind] : rgbCss((LAYER_COLORS as Record<string, [number, number, number]>)[l.kind] ?? [230, 25, 75]),
                }}
              />
              <span className="layer-name">{l.kind === "rooms" ? "rooms / regions" : l.name}</span>
              <span className="mono small muted count-r">{l.count.toLocaleString()}</span>
            </label>
          </li>
        ))}
      </ul>
      <header className="vsec-head">
        <h4>Edges</h4>
      </header>
      <ul className="layer-list">
        {Object.entries(edgeCounts)
          .sort()
          .map(([k, n]) => (
            <li key={k}>
              <label className="layer-row">
                <input type="checkbox" checked={edgeKinds[k] ?? defaultEdgeOn(k, vizStyle)} onChange={e => set({edgeKinds: {...edgeKinds, [k]: e.target.checked}})} />
                <span className="swatch line" style={{background: (hydra ? HYDRA_EDGES[k]?.color : EDGE_COLORS[k]) ?? "#7f8b96"}} />
                <span className="layer-name">{k === "agents-agents" ? "trajectory" : k.replace("-", " / ")}</span>
                <span className="mono small muted count-r">{n.toLocaleString()}</span>
              </label>
            </li>
          ))}
      </ul>
    </section>
  );
}

export function DisplayPanel() {
  const {showBBoxes, showTrajectory, meshOpacity, vizStyle, showText, meshCull, meshClipOn, meshClipHeight, focusOthers, set} = useViewer();
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Display</h4>
      </header>
      <label className="slider-row">
        <span>Style</span>
        <select
          className="input input-sm"
          value={vizStyle}
          onChange={e => set({vizStyle: e.target.value as typeof vizStyle})}
          aria-label="Graph style"
          title="hydra_visualizer: the ADT4 RViz look (base_params/hydra_visualizer.yaml)"
        >
          <option value="hydra">hydra_visualizer</option>
          <option value="classic">Classic console</option>
        </select>
      </label>
      <ThemeControl />
      <BackgroundControl />
      <RobotLookControl />
      <HeightsControl />
      <label className="slider-row">
        <span>Mesh opacity</span>
        <input type="range" min={0} max={1} step={0.05} value={meshOpacity} onChange={e => set({meshOpacity: Number(e.target.value)})} />
        <span className="mono small">{Math.round(meshOpacity * 100)}%</span>
      </label>
      <Toggle checked={meshCull} onChange={v => set({meshCull: v})} label="Cull mesh back faces (hide ceilings)" />
      <Toggle checked={meshClipOn} onChange={v => set({meshClipOn: v})} label="Clip mesh above a height" />
      {meshClipOn && (
        <label className="slider-row">
          <span>Max height</span>
          <input type="range" min={0.2} max={6} step={0.1} value={meshClipHeight} onChange={e => set({meshClipHeight: Number(e.target.value)})} aria-label="Mesh max height above floor" />
          <span className="mono small">{meshClipHeight.toFixed(1)} m</span>
        </label>
      )}
      <Toggle checked={showBBoxes} onChange={v => set({showBBoxes: v})} label="Object bounding boxes" />
      <Toggle checked={showText} onChange={v => set({showText: v})} label="Text labels" />
      <Toggle checked={showTrajectory} onChange={v => set({showTrajectory: v})} label="Agent trajectory" />
      <label className="slider-row">
        <span>Other robots</span>
        <select
          className="input input-sm"
          value={focusOthers}
          onChange={e => set({focusOthers: e.target.value as typeof focusOthers})}
          aria-label="Robots not in focus"
          title="How the robots that are not in focus are drawn (live map, paths, plan, glyph) while one robot is focused"
        >
          <option value="dim">Dim when focused</option>
          <option value="hide">Hide when focused</option>
          <option value="show">Always full</option>
        </select>
      </label>
    </section>
  );
}

export function SearchBox({autoFocus, onPicked}: {autoFocus?: boolean; onPicked?: (src: string, sym: string) => void} = {}) {
  const {query, set, select, fly, sources, flat, stackHeight, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const [open, setOpen] = useState(false);
  const revs = Object.values(sources).map(s => s.rev).join(",");
  const imaged = useImaged(readyModels().map(({id}) => ({id, rev: sources[id]?.rev ?? 0})));
  const results = useMemo(() => {
    const out: {src: string; sym: string; text: string; kind: string; label: number; color: [number, number, number]}[] = [];
    for (const {id, model} of readyModels()) {
      for (const n of searchNodes(model, query, 200)) {
        out.push({src: id, sym: n.sym, text: model.displayName(n), kind: layerInfo(n.layer, n.part).kind, label: n.label, color: n.color});
      }
    }
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [query, revs]);
  useEffect(() => {
    set({highlight: query.trim() ? results.map(r => ({src: r.src, sym: r.sym})) : []});
  }, [results, query, set]);
  const go = (r: (typeof results)[number]) => {
    const n = sources[r.src]?.model?.nodes.get(r.sym);
    if (!n) return;
    select({src: r.src, sym: r.sym});
    fly(applyPoint(sourceMatrix(r.src), displayPos(n, flat, stackHeight, vizStyle, heights)));
    setOpen(false);
    onPicked?.(r.src, r.sym);
  };
  return (
    <div className="searchbox">
      <input
        className="input"
        placeholder="Search objects, labels, symbols (e.g. sign, O(12))"
        value={query}
        autoFocus={autoFocus}
        enterKeyHint="search"
        aria-label="Search graph"
        onChange={e => {
          set({query: e.target.value});
          setOpen(true);
        }}
        onFocus={() => setOpen(true)}
        onBlur={() => setTimeout(() => setOpen(false), 150)}
        onKeyDown={e => {
          if (e.key === "Enter" && results[0]) go(results[0]);
          if (e.key === "Escape") {
            set({query: ""});
            setOpen(false);
          }
        }}
      />
      {query && <span className="search-count mono">{results.length >= 200 ? "200+" : results.length} hits</span>}
      {open && query.trim() && results.length > 0 && (
        <ul className="search-results">
          {results.slice(0, 40).map(r => {
            const color = rgbCss(r.kind === "objects" ? nodeColor(r.label, r.color) : (LAYER_COLORS as Record<string, [number, number, number]>)[r.kind] ?? [200, 200, 200]);
            if (r.kind !== "objects")
              return (
                <li key={r.src + r.sym} onMouseDown={e => e.preventDefault()} onClick={() => go(r)}>
                  <span className="swatch" style={{background: color}} />
                  <span className="sr-text">{r.text}</span>
                  <span className="mono small muted">{r.sym}</span>
                </li>
              );
            const set_ = imaged[r.src];
            const has = set_ ? set_.has(r.sym) : null;
            return (
              <li key={r.src + r.sym} className="has-thumb" onMouseDown={e => e.preventDefault()} onClick={() => go(r)}>
                <SearchThumb src={r.src} sym={r.sym} has={has} color={color} />
                <span className="sr-main">
                  <span className="sr-text">{r.text}</span>
                  <span className="sr-sub small">
                    <span className="mono muted">{r.sym}</span>
                    {has === false && <span className="muted">no image</span>}
                  </span>
                </span>
              </li>
            );
          })}
        </ul>
      )}
    </div>
  );
}

export function Inspector({onVisit, onGoto}: {onVisit: (src: string, sym: string) => void; onGoto: (src: string, sym: string) => void}) {
  const {selected, sources, fly, flat, stackHeight, select, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const [detail, setDetail] = useState<(NodeDetail & {masks?: boolean[]; image_info?: ImageInfo[]}) | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const node = selected ? sources[selected.src]?.model?.nodes.get(selected.sym) : null;
  const model = selected ? sources[selected.src]?.model : null;
  useEffect(() => {
    setDetail(null);
    setErr(null);
    if (!selected) return;
    const ctl = new AbortController();
    setLoading(true);
    getJSON<NodeDetail & {masks?: boolean[]; image_info?: ImageInfo[]}>(`${srcPath(selected.src)}/node/${encodeURIComponent(selected.sym)}`, ctl.signal)
      .then(setDetail)
      .catch(e => !ctl.signal.aborted && setErr(errorText(e)))
      .finally(() => !ctl.signal.aborted && setLoading(false));
    return () => ctl.abort();
  }, [selected]);
  if (!selected || !node || !model)
    return (
      <>
        <Empty title="Nothing selected">Click a node in the 3D view, or search above.</Empty>
        <StatsPanel />
      </>
    );
  const info = layerInfo(node.layer, node.part);
  const images = detail?.images ?? [];
  const isObject = info.kind === "objects";
  const isAgent = info.kind === "agents";
  const imagesEl = (
    <ObjectImages images={images} masks={detail?.masks} info={detail?.image_info} title={model.displayName(node)} sym={node.sym} isObject={isObject} />
  );
  const color = info.kind === "objects" ? nodeColor(node.label, node.color) : (LAYER_COLORS as Record<string, [number, number, number]>)[info.kind] ?? [200, 200, 200];
  return (
    <div className="inspector">
      <div className="insp-head">
        <span className="swatch big" style={{background: rgbCss(color)}} />
        <div>
          <div className="insp-title">{model.displayName(node)}</div>
          <div className="mono small muted">
            {node.sym} · {info.name} · {sources[selected.src]?.label}
          </div>
        </div>
        <button className="icon-btn" aria-label="Clear selection" onClick={() => select(null)}>
          ×
        </button>
      </div>
      <div className="row gap-sm wrap">
        <button className="btn btn-sm" onClick={() => fly(applyPoint(sourceMatrix(selected.src), displayPos(node, flat, stackHeight, vizStyle, heights)))}>
          Fly to
        </button>
        <button className="btn btn-sm" onClick={() => onGoto(selected.src, selected.sym)}>
          Add goto point
        </button>
        {info.kind === "objects" && (
          <button className="btn btn-sm btn-primary" onClick={() => onVisit(selected.src, selected.sym)}>
            Visit (PDDL)
          </button>
        )}
      </div>
      {(isObject || isAgent) && imagesEl}
      {(isObject || isAgent) && loading && !detail && <div className="obj-hero-wait muted small">Loading images…</div>}
      <table className="table compact kv-table">
        <tbody>
          <tr>
            <th>position</th>
            <td className="mono">{node.pos.map(v => v.toFixed(2)).join(", ")}</td>
          </tr>
          {node.label >= 0 && (
            <tr>
              <th>label</th>
              <td>
                {detail?.label_name ?? model.labelName(node)} <span className="muted mono">({node.label})</span>
              </td>
            </tr>
          )}
          {node.bbox && (
            <tr>
              <th>bbox size</th>
              <td className="mono">
                {(node.bbox[3] - node.bbox[0]).toFixed(2)} x {(node.bbox[4] - node.bbox[1]).toFixed(2)} x {(node.bbox[5] - node.bbox[2]).toFixed(2)} m
              </td>
            </tr>
          )}
          <tr>
            <th>active</th>
            <td>{node.active ? "yes" : "no"}</td>
          </tr>
          {Object.entries(detail?.attributes ?? {}).map(([k, v]) => (
            <tr key={k}>
              <th>{k}</th>
              <td className="mono small attr-v">{typeof v === "object" ? JSON.stringify(v) : String(v)}</td>
            </tr>
          ))}
        </tbody>
      </table>
      {loading && <Spinner label="Loading attributes" />}
      {err && <div className="inline-err small">{err}</div>}
      {!isObject && !isAgent && imagesEl}
    </div>
  );
}

export function SourceDot({status}: {status: string}) {
  return <Dot tone={status === "ready" ? "ok" : status === "error" ? "err" : status === "loading" ? "warn" : "off"} />;
}

/** Live robot mesh level of detail: decimated by default (~8x fewer faces over
 *  the radio), "Full mesh" asks the robot for Hydra's full tiles. */
export function meshLodText(m: {lod: string | null; wanted: string | null; pending: number; bytes: number}): string {
  const lod = m.lod ?? "full (robot has no LOD support)";
  const pend = m.pending > 0 ? ` · ${m.pending} tile${m.pending === 1 ? "" : "s"} to go` : "";
  return `${lod}${pend} · ${fmtBytes(m.bytes)}`;
}

function MeshLodControl({robot}: {robot: string}) {
  const mesh = useRobots(s => s.link[robot]?.mesh);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  if (!mesh) return null;
  const full = mesh.wanted === "full";
  const set = async (on: boolean) => {
    setBusy(true);
    setErr(null);
    try {
      await postJSON(`api/live/${encodeURIComponent(robot)}/mesh_lod`, {level: on ? "full" : "decimated"});
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  return (
    <div className="mesh-lod small">
      <label className="mini-check" title="Full: Hydra's mesh as is (more radio traffic). Off: decimated, ~8x fewer faces.">
        <input type="checkbox" checked={full} disabled={busy || mesh.lod === null} onChange={e => void set(e.target.checked)} aria-label={`Full mesh for ${robot}`} /> full mesh
      </label>
      <span className="muted mono">{meshLodText(mesh)}</span>
      {err && <div className="inline-err small">{err}</div>}
    </div>
  );
}
