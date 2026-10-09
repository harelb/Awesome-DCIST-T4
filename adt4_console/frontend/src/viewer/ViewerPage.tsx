import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useSearchParams} from "react-router";
import {useViewer} from "../stores/viewer";
import {useRobots} from "../stores/robots";
import {layerInfo, nearestPlace} from "../lib/graphModel";
import {Scene, type SceneCallbacks} from "./Scene";
import type {GraphLayout} from "./layout";
import type {TileStats} from "./MeshTiles";
import {DisplayPanel, Inspector, LayersPanel, SearchBox, SourcesPanel} from "./panels";
import {PlanPanel, visitGoal} from "./PlanPanel";
import {ChatPanel} from "./ChatPanel";
import {LivePanel} from "./LivePanel";
import {LiveOverlaysPanel} from "./LiveOverlays";
import {CameraPanel} from "./CameraPanel";
import {Segmented} from "../components/ui";
import {readyModels} from "../stores/viewer";
import {useWsStatus} from "../ws";

type Tab = "inspect" | "plan" | "chat" | "live" | "camera";

export default function ViewerPage() {
  const v = useViewer();
  const [params] = useSearchParams();
  const [tab, setTab] = useState<Tab>("inspect");
  const [leftOpen, setLeftOpen] = useState(() => window.innerWidth > 1100);
  const [rightOpen, setRightOpen] = useState(() => window.innerWidth > 900);
  const [stats, setStats] = useState<Record<string, TileStats>>({});
  const [bounds, setBounds] = useState<{min: number[]; max: number[]} | null>(null);
  const [toast, setToast] = useState<string | null>(null);
  const robotStates = useRobots(s => s.state);
  const wsUp = useWsStatus(s => s.connected);
  const initDone = useRef(false);

  // catalog + initial source selection (?src=... or the first prior/neo4j source)
  useEffect(() => {
    void (async () => {
      await useViewer.getState().loadCatalog();
      if (initDone.current) return;
      initDone.current = true;
      const st = useViewer.getState();
      const wanted = params.get("src");
      if (wanted) {
        if (!st.sources[wanted]) await st.loadCatalog();
        if (useViewer.getState().sources[wanted]) st.enableSource(wanted, true);
        return;
      }
      if (st.order.some(id => st.sources[id].enabled)) return;
      const first = st.order.find(id => st.sources[id].kind === "neo4j") ?? st.order.find(id => st.sources[id].kind === "file") ?? st.order[0];
      if (first) st.enableSource(first, true);
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  // refresh the catalog when sources announce new versions / reconnects
  useEffect(() => {
    if (wsUp) void useViewer.getState().loadCatalog();
  }, [wsUp]);

  useEffect(() => {
    if (!toast) return;
    const id = window.setTimeout(() => setToast(null), 3500);
    return () => window.clearTimeout(id);
  }, [toast]);

  const enabled = useMemo(() => v.order.map(id => v.sources[id]).filter(s => s?.enabled), [v.order, v.sources]);

  const addGoto = useCallback((src: string, sym: string) => {
    const st = useViewer.getState();
    const model = st.sources[src]?.model;
    const n = model?.nodes.get(sym);
    if (!n || !model) return;
    st.set({gotoDraft: [...st.gotoDraft, {src, sym, pos: n.pos, label: model.displayName(n)}]});
    setTab("plan");
  }, []);

  const cb: SceneCallbacks = useMemo(
    () => ({
      onNode: (src, sym) => {
        const st = useViewer.getState();
        if (st.pickMode === "goto") {
          const n = st.sources[src]?.model?.nodes.get(sym);
          const kind = n ? layerInfo(n.layer, n.part).kind : null;
          if (kind === "places" || kind === "objects") addGoto(src, sym);
          else if (n) {
            const near = nearestPlace(st.sources[src]?.model, n.pos);
            if (near) addGoto(src, near.sym);
          }
          return;
        }
        st.select({src, sym});
        setTab("inspect");
        setRightOpen(true);
      },
      onPoint: p => {
        const st = useViewer.getState();
        if (st.pickMode !== "goto") return;
        let best: {src: string; sym: string; d: number} | null = null;
        for (const {id, model} of readyModels()) {
          const n = nearestPlace(model, p, {maxDist: 15});
          if (!n) continue;
          const d = Math.hypot(n.pos[0] - p[0], n.pos[1] - p[1]);
          if (!best || d < best.d) best = {src: id, sym: n.sym, d};
        }
        if (best) addGoto(best.src, best.sym);
        else setToast("No place within 15 m of that point");
      },
      onMiss: () => {
        if (useViewer.getState().pickMode === "inspect") useViewer.getState().select(null);
      },
      onStats: (src, s) => setStats(cur => ({...cur, [src]: s})),
      onLayout: (_src: string, l: GraphLayout) => {
        if (l.bounds) setBounds(b => b ?? l.bounds);
      },
    }),
    [addGoto],
  );

  const onVisit = (src: string, sym: string) => {
    void src;
    useViewer.getState().set({pddlDraft: visitGoal([sym])});
    setTab("plan");
  };

  const robots = Object.keys(robotStates);
  const totalNodes = enabled.reduce((n, s) => n + (s.model?.nodes.size ?? 0), 0);
  const totalTris = Object.entries(stats).reduce((n, [id, s]) => n + (v.sources[id]?.enabled ? s.triangles : 0), 0);
  const tileText = Object.entries(stats)
    .filter(([id]) => v.sources[id]?.enabled)
    .map(([, s]) => `${s.loaded}/${s.total}`)
    .join(" + ");

  return (
    <div className={`viewer ${leftOpen ? "left-open" : ""} ${rightOpen ? "right-open" : ""}`}>
      {leftOpen && (
        <aside className="vpanel vleft" aria-label="Sources and layers">
          <SourcesPanel stats={stats} />
          <LayersPanel />
          <LiveOverlaysPanel />
          <DisplayPanel />
        </aside>
      )}
      <div className="vcanvas">
        {enabled.length === 0 && (
          <div className="canvas-empty">
            <div className="state-title">No source shown</div>
            <div className="muted">Enable a graph source in the left panel.</div>
          </div>
        )}
        <Scene sources={enabled} bounds={bounds} cb={cb} />
        <div className="vtoolbar top-left">
          <button className="btn btn-sm panel-btn" onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel">
            {leftOpen ? "Hide panel" : "Layers"}
          </button>
          <SearchBox />
        </div>
        <div className="vtoolbar top-right">
          <Segmented
            ariaLabel="Camera"
            value={v.topDown ? "top" : "3d"}
            onChange={x => v.set({topDown: x === "top"})}
            options={[
              {value: "3d", label: "3D"},
              {value: "top", label: "Top"},
            ]}
          />
          <select className="input input-sm" value={v.follow ?? ""} onChange={e => v.set({follow: e.target.value || null})} aria-label="Follow robot">
            <option value="">Follow: off</option>
            {robots.map(r => (
              <option key={r} value={r}>
                Follow {r}
              </option>
            ))}
          </select>
          <button className="btn btn-sm panel-btn" onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel">
            {rightOpen ? "Hide" : "Panel"}
          </button>
        </div>
        {v.pickMode === "goto" && (
          <div className="pick-banner">
            Goto picking: click places, objects or the mesh.{" "}
            <button className="btn btn-xs" onClick={() => v.set({pickMode: "inspect"})}>
              Done
            </button>
          </div>
        )}
        {toast && <div className="toast">{toast}</div>}
        <div className="vstatus mono">
          {totalNodes.toLocaleString()} nodes{tileText ? ` · tiles ${tileText}` : ""}
          {totalTris ? ` · ${(totalTris / 1e6).toFixed(2)}M tris` : ""}
          {v.flat ? " · flat" : ""}
        </div>
      </div>
      {rightOpen && (
        <aside className="vpanel vright" aria-label="Inspector and planning">
          <Segmented<Tab>
            ariaLabel="Side panel"
            value={tab}
            onChange={setTab}
            options={[
              {value: "inspect", label: "Inspect"},
              {value: "plan", label: "Plan"},
              {value: "chat", label: "Chat"},
              {value: "live", label: "Live link"},
              {value: "camera", label: "Camera"},
            ]}
          />
          <div className="vright-body">
            {tab === "inspect" && <Inspector onVisit={onVisit} onGoto={addGoto} />}
            {tab === "plan" && <PlanPanel />}
            {tab === "chat" && <ChatPanel />}
            {tab === "live" && <LivePanel />}
            {tab === "camera" && <CameraPanel />}
          </div>
        </aside>
      )}
    </div>
  );
}
