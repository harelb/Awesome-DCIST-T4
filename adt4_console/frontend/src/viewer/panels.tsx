import {useEffect, useMemo, useState} from "react";
import {useViewer, readyModels, srcPath} from "../stores/viewer";
import {layerInfo, searchNodes} from "../lib/graphModel";
import {nodeColor, LAYER_COLORS, rgbCss} from "../lib/palette";
import {errorText, getJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import {Badge, Dot, Empty, Modal, Spinner, Toggle} from "../components/ui";
import type {NodeDetail} from "../types";
import {displayPos, DEFAULT_EDGE_ON, EDGE_COLORS} from "./layout";
import type {TileStats} from "./MeshTiles";
import {fmtBytes} from "../lib/fuzzy";
import {StatsPanel} from "./StatsPanel";

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

export function LayersPanel() {
  const {sources, order, hiddenLayers, edgeKinds, set} = useViewer();
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
      <ul className="layer-list">
        {layers.map(([k, l]) => (
          <li key={k}>
            <label className="layer-row">
              <input type="checkbox" checked={!hiddenLayers[k]} onChange={e => set({hiddenLayers: {...hiddenLayers, [k]: !e.target.checked}})} />
              <span className="swatch" style={{background: rgbCss((LAYER_COLORS as Record<string, [number, number, number]>)[l.kind] ?? [230, 25, 75])}} />
              <span className="layer-name">{l.name}</span>
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
                <input type="checkbox" checked={edgeKinds[k] ?? DEFAULT_EDGE_ON(k)} onChange={e => set({edgeKinds: {...edgeKinds, [k]: e.target.checked}})} />
                <span className="swatch line" style={{background: EDGE_COLORS[k] ?? "#7f8b96"}} />
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
  const {flat, stackHeight, showBBoxes, showTrajectory, meshOpacity, set} = useViewer();
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Display</h4>
      </header>
      <Toggle checked={flat} onChange={v => set({flat: v})} label="Flat (no layer stacking)" />
      {!flat && (
        <label className="slider-row">
          <span>Layer spacing</span>
          <input type="range" min={0.5} max={10} step={0.5} value={stackHeight} onChange={e => set({stackHeight: Number(e.target.value)})} />
          <span className="mono small">{stackHeight} m</span>
        </label>
      )}
      <label className="slider-row">
        <span>Mesh opacity</span>
        <input type="range" min={0} max={1} step={0.05} value={meshOpacity} onChange={e => set({meshOpacity: Number(e.target.value)})} />
        <span className="mono small">{Math.round(meshOpacity * 100)}%</span>
      </label>
      <Toggle checked={showBBoxes} onChange={v => set({showBBoxes: v})} label="Object bounding boxes" />
      <Toggle checked={showTrajectory} onChange={v => set({showTrajectory: v})} label="Agent trajectory" />
    </section>
  );
}

export function SearchBox() {
  const {query, set, select, fly, sources, flat, stackHeight} = useViewer();
  const [open, setOpen] = useState(false);
  const revs = Object.values(sources).map(s => s.rev).join(",");
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
    fly(displayPos(n, flat, stackHeight));
    setOpen(false);
  };
  return (
    <div className="searchbox">
      <input
        className="input"
        placeholder="Search objects, labels, symbols (e.g. sign, O(12))"
        value={query}
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
          {results.slice(0, 40).map(r => (
            <li key={r.src + r.sym} onMouseDown={e => (e.preventDefault(), go(r))}>
              <span className="swatch" style={{background: rgbCss(r.kind === "objects" ? nodeColor(r.label, r.color) : (LAYER_COLORS as Record<string, [number, number, number]>)[r.kind] ?? [200, 200, 200])}} />
              <span className="sr-text">{r.text}</span>
              <span className="mono small muted">{r.sym}</span>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export function Inspector({onVisit, onGoto}: {onVisit: (src: string, sym: string) => void; onGoto: (src: string, sym: string) => void}) {
  const {selected, sources, fly, flat, stackHeight, select} = useViewer();
  const [detail, setDetail] = useState<NodeDetail | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [lightbox, setLightbox] = useState<number | null>(null);
  const node = selected ? sources[selected.src]?.model?.nodes.get(selected.sym) : null;
  const model = selected ? sources[selected.src]?.model : null;
  useEffect(() => {
    setDetail(null);
    setErr(null);
    if (!selected) return;
    const ctl = new AbortController();
    setLoading(true);
    getJSON<NodeDetail>(`${srcPath(selected.src)}/node/${encodeURIComponent(selected.sym)}`, ctl.signal)
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
        <button className="btn btn-sm" onClick={() => fly(displayPos(node, flat, stackHeight))}>
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
      {images.length > 0 && (
        <>
          <h4 className="sub-title">Images ({images.length})</h4>
          <div className="image-strip">
            {images.map((u, i) => (
              <button key={u} className="thumb" onClick={() => setLightbox(i)} aria-label={`Image ${i + 1}`}>
                <img src={withToken(u)} loading="lazy" alt={`${node.sym} frame ${i + 1}`} />
              </button>
            ))}
          </div>
        </>
      )}
      {lightbox !== null && images[lightbox] && (
        <Modal title={`${model.displayName(node)}: frame ${lightbox + 1}/${images.length}`} onClose={() => setLightbox(null)} wide>
          <img className="lightbox-img" src={withToken(images[lightbox])} alt="" />
          <div className="row gap center">
            <button className="btn btn-sm" disabled={lightbox === 0} onClick={() => setLightbox(lightbox - 1)}>
              Previous
            </button>
            <button className="btn btn-sm" disabled={lightbox >= images.length - 1} onClick={() => setLightbox(lightbox + 1)}>
              Next
            </button>
          </div>
        </Modal>
      )}
    </div>
  );
}

export function SourceDot({status}: {status: string}) {
  return <Dot tone={status === "ready" ? "ok" : status === "error" ? "err" : status === "loading" ? "warn" : "off"} />;
}
