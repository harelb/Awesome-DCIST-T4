import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useSearchParams} from "react-router";
import {useViewer} from "../stores/viewer";
import {getJSON} from "../lib/api";
import {pickDefaultSources} from "../lib/runCopies";
import type {ActiveLaunch} from "../types";
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
import {RobotLegend} from "./RobotLegend";
import {CameraPanel} from "./CameraPanel";
import {Segmented} from "../components/ui";
import {readyModels} from "../stores/viewer";
import {useWsStatus} from "../ws";
import {useIsPhone, useMedia} from "../lib/media";
import {Icon} from "../components/Shell";
import {BottomSheet} from "./BottomSheet";
import {ControlsHelpButton, TopViewHint} from "./ControlsHelp";
import {FloatingWindow, Splitter, TabPortal, TabSlot, useElementSize, usePaneLayout, useTabHosts, useTabTearOff} from "./Panes";
import {FLOAT_MQ, LEFT, MIN_CANVAS, PANE_TABS, RIGHT, clampWidths, dock, popOut, raise, updateWin, type PaneTab} from "./paneLayout";

type Tab = PaneTab;
const TAB_TITLE: Record<Tab, string> = {inspect: "Inspect", plan: "Plan", chat: "Chat", live: "Live link", camera: "Camera"};

export default function ViewerPage() {
  const v = useViewer();
  const [params] = useSearchParams();
  const [tab, setTabRaw] = useState<Tab>("inspect");
  const isPhone = useIsPhone();
  // pane layout: splitter widths + popped-out tabs (floating windows only on wide screens)
  const [layout, setLayout, resetLayout] = usePaneLayout();
  const wide = useMedia(FLOAT_MQ) && !isPhone;
  const floats = useCallback((t: Tab) => wide && !!layout.windows[t], [wide, layout.windows]);
  const floatsRef = useRef(floats);
  floatsRef.current = floats;
  const [leftOpen, setLeftOpenRaw] = useState(() => window.innerWidth > 1100);
  const [rightOpen, setRightOpenRaw] = useState(() => window.innerWidth > 900);
  const [searchOpen, setSearchOpen] = useState(false);
  const viewerRef = useRef<HTMLDivElement>(null);
  const area = useElementSize(viewerRef);
  const leftRef = useRef<HTMLElement>(null);
  const rightRef = useRef<HTMLElement>(null);
  const stripRef = useRef<HTMLDivElement>(null);
  const hosts = useTabHosts(PANE_TABS);
  // phones show one panel at a time, as a bottom sheet
  const phoneRef = useRef(isPhone);
  phoneRef.current = isPhone;
  const setLeftOpen = useCallback((f: boolean | ((o: boolean) => boolean)) => {
    setLeftOpenRaw(o => {
      const next = typeof f === "function" ? f(o) : f;
      if (next && phoneRef.current) setRightOpenRaw(false);
      return next;
    });
  }, []);
  const setRightOpen = useCallback((f: boolean | ((o: boolean) => boolean)) => {
    setRightOpenRaw(o => {
      const next = typeof f === "function" ? f(o) : f;
      if (next && phoneRef.current) setLeftOpenRaw(false);
      return next;
    });
  }, []);
  /** Bring a tab forward: raise (and restore) its floating window, else select it in the side panel. */
  const setTab = useCallback(
    (t: Tab) => {
      if (floatsRef.current(t)) setLayout(l => updateWin(raise(l, t), t, {min: false}));
      else setTabRaw(t);
    },
    [setLayout],
  );
  const showTab = useCallback(
    (t: Tab) => {
      setTab(t);
      if (!floatsRef.current(t)) setRightOpen(true);
    },
    [setTab, setRightOpen],
  );
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
      // an active launch with a prior map: prior + live:<robots> by default
      const active = await getJSON<ActiveLaunch>("api/launch/active").catch(() => null);
      if (active?.default_sources?.length) {
        const {enable, pending} = pickDefaultSources(useViewer.getState().order, active.default_sources);
        useViewer.getState().set({pendingEnable: pending});
        for (const id of enable) useViewer.getState().enableSource(id, true);
        if (enable.length) return;
      }
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
  }, [setTab]);

  // a picked search result: the Inspect tab (object images) comes to the front
  const showInspect = useCallback(() => {
    setTab("inspect");
    setRightOpen(true);
  }, [setRightOpen]);

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
        showTab("inspect");
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
    [addGoto, showTab],
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

  const leftPanel = (
    <>
      <SourcesPanel stats={stats} />
      <LayersPanel />
      <LiveOverlaysPanel />
      <DisplayPanel />
      {!isPhone && (
        <section className="vsec">
          <header className="vsec-head">
            <h4>Layout</h4>
          </header>
          <div className="layout-row">
            <span className="muted small">{wide ? "Drag pane edges to resize; pop side-panel tabs out with ⧉." : "Pane widths are fixed on narrow screens."}</span>
            <button className="btn btn-xs" onClick={resetLayout} title="Default pane widths, dock all floating windows">
              Reset layout
            </button>
          </div>
        </section>
      )}
    </>
  );

  // the docked side panel shows the selected tab, or the first tab that is not floating
  const dockTab: Tab | null = !floats(tab) ? tab : (PANE_TABS.find(t => !floats(t)) ?? null);
  const mounted = (t: Tab) => floats(t) || (rightOpen && t === dockTab);
  const pop = useCallback(
    (t: Tab, at?: {x: number; y: number}) => {
      const r = viewerRef.current?.getBoundingClientRect();
      if (!r) return;
      setLayout(l => popOut(l, t, r.width, r.height, at && {x: at.x - r.left, y: at.y - r.top}));
    },
    [setLayout],
  );
  const dockWin = (t: Tab) => {
    setLayout(l => dock(l, t));
    setTabRaw(t);
    setRightOpen(true);
  };
  const canTear = useCallback((t: Tab) => !floats(t), [floats]);
  const onStripDown = useTabTearOff(stripRef, PANE_TABS, canTear, useCallback((t: Tab, x: number, y: number) => pop(t, {x, y}), [pop]));
  const tabContent = (t: Tab) => {
    switch (t) {
      case "inspect":
        return <Inspector onVisit={onVisit} onGoto={addGoto} />;
      case "plan":
        return <PlanPanel />;
      case "chat":
        return <ChatPanel />;
      case "live":
        return <LivePanel />;
      case "camera":
        return <CameraPanel />;
    }
  };
  const tabs = (
    <div className="vright-head" ref={stripRef} onPointerDown={wide ? onStripDown : undefined}>
      <Segmented<Tab>
        ariaLabel="Side panel"
        value={dockTab ?? tab}
        onChange={setTab}
        options={PANE_TABS.map(t => ({
          value: t,
          label: floats(t) ? (
            <>
              {TAB_TITLE[t]}
              <span className="tab-float-mark" title="In a floating window" aria-label="(floating)">
                ⧉
              </span>
            </>
          ) : t === "live" && isPhone ? (
            "Link"
          ) : (
            TAB_TITLE[t]
          ),
        }))}
      />
      {wide && (
        <button className="pop-btn" disabled={!dockTab} onClick={() => dockTab && pop(dockTab)} aria-label={dockTab ? `Pop out ${TAB_TITLE[dockTab]}` : "Pop out"} title="Pop out into a floating window (or drag a tab out)">
          ⧉
        </button>
      )}
    </div>
  );
  const rightBody = (
    <div className="vright-body">
      {dockTab ? <TabSlot key={dockTab} host={hosts[dockTab]} /> : <div className="dock-empty muted">Every tab is in a floating window. Dock one with ⇥ or drag its title bar onto the tab strip.</div>}
    </div>
  );
  const widths = clampWidths(layout.leftW, layout.rightW, area.w || window.innerWidth, leftOpen, rightOpen, "right");
  const splitMax = (side: "left" | "right") => Math.min(side === "left" ? LEFT.max : RIGHT.max, (area.w || window.innerWidth) - MIN_CANVAS - (side === "left" ? (rightOpen ? widths.rightW : 0) : leftOpen ? widths.leftW : 0));
  const follow = (
    <select className="input input-sm follow-select" value={v.follow ?? ""} onChange={e => v.set({follow: e.target.value || null})} aria-label="Follow robot">
      <option value="">{isPhone ? "Follow" : "Follow: off"}</option>
      {robots.map(r => (
        <option key={r} value={r}>
          {isPhone ? r : `Follow ${r}`}
        </option>
      ))}
    </select>
  );
  const camera = (
    <Segmented
      ariaLabel="Camera"
      value={v.topDown ? "top" : "3d"}
      onChange={x => v.set({topDown: x === "top"})}
      options={[
        {value: "3d", label: "3D"},
        {value: "top", label: "Top"},
      ]}
    />
  );

  return (
    <div ref={viewerRef} className={`viewer ${leftOpen ? "left-open" : ""} ${rightOpen ? "right-open" : ""} ${isPhone ? "phone" : ""}`}>
      {PANE_TABS.filter(mounted).map(t => (
        <TabPortal key={t} host={hosts[t]}>
          {tabContent(t)}
        </TabPortal>
      ))}
      {leftOpen && !isPhone && (
        <aside ref={leftRef} className="vpanel vleft" aria-label="Sources and layers" style={wide ? {width: widths.leftW} : undefined}>
          {leftPanel}
        </aside>
      )}
      {leftOpen && wide && (
        <Splitter
          side="left"
          label="Resize layers panel"
          value={widths.leftW}
          min={LEFT.min}
          max={splitMax("left")}
          target={leftRef}
          onCommit={w => setLayout(l => ({...l, leftW: w}))}
          onReset={() => setLayout(l => ({...l, leftW: LEFT.def}))}
        />
      )}
      <div className="vcanvas">
        {enabled.length === 0 && (
          <div className="canvas-empty">
            <div className="state-title">No source shown</div>
            <div className="muted">Enable a graph source in the {isPhone ? "layers sheet" : "left panel"}.</div>
          </div>
        )}
        <Scene sources={enabled} bounds={bounds} cb={cb} />
        {isPhone ? (
          searchOpen ? (
            <div className="vtoolbar phone-bar phone-search">
              <SearchBox
                autoFocus
                onPicked={() => {
                  setSearchOpen(false);
                  showInspect();
                }}
              />
              <button className="btn icon-only" onClick={() => setSearchOpen(false)} aria-label="Close search">
                ×
              </button>
            </div>
          ) : (
            <div className="vtoolbar phone-bar">
              <button className={`btn icon-only panel-btn ${leftOpen ? "on" : ""}`} onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel" aria-pressed={leftOpen}>
                <Icon name="layers" size={20} />
              </button>
              <button className={`btn icon-only panel-btn ${v.query ? "on" : ""}`} onClick={() => setSearchOpen(true)} aria-label="Search graph">
                <Icon name="search" size={20} />
              </button>
              <span className="grow" />
              {camera}
              {follow}
              <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={false} compact />
              <button className={`btn icon-only panel-btn ${rightOpen ? "on" : ""}`} onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel" aria-pressed={rightOpen}>
                <Icon name="panel" size={20} />
              </button>
            </div>
          )
        ) : (
          <>
            <div className="vtoolbar top-left">
              <button className="btn btn-sm panel-btn" onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel">
                {leftOpen ? "Hide panel" : "Layers"}
              </button>
              <SearchBox onPicked={showInspect} />
            </div>
            <div className="vtoolbar top-right">
              {camera}
              {follow}
              <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={wide} />
              <button className="btn btn-sm panel-btn" onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel">
                {rightOpen ? "Hide" : "Panel"}
              </button>
            </div>
          </>
        )}
        {v.pickMode === "goto" && (
          <div className="pick-banner">
            {isPhone ? "Tap places, objects or the mesh." : "Goto picking: click places, objects or the mesh."}{" "}
            <button className="btn btn-xs" onClick={() => v.set({pickMode: "inspect"})}>
              Done
            </button>
          </div>
        )}
        {toast && <div className="toast">{toast}</div>}
        <TopViewHint active={v.topDown} />
        <RobotLegend />
        <div className="vstatus mono">
          {totalNodes.toLocaleString()} nodes{tileText ? ` · tiles ${tileText}` : ""}
          {totalTris ? ` · ${(totalTris / 1e6).toFixed(2)}M tris` : ""}
          {v.flat ? " · flat" : ""}
        </div>
      </div>
      {rightOpen && wide && (
        <Splitter
          side="right"
          label="Resize side panel"
          value={widths.rightW}
          min={RIGHT.min}
          max={splitMax("right")}
          target={rightRef}
          onCommit={w => setLayout(l => ({...l, rightW: w}))}
          onReset={() => setLayout(l => ({...l, rightW: RIGHT.def}))}
        />
      )}
      {rightOpen && !isPhone && (
        <aside ref={rightRef} className="vpanel vright" aria-label="Inspector and planning" style={wide ? {width: widths.rightW} : undefined}>
          {tabs}
          {rightBody}
        </aside>
      )}
      {wide &&
        // stable DOM order (moving a window's node mid-click would swallow the click); stacking via z-index
        PANE_TABS.map(t => {
          const w = layout.windows[t];
          const i = layout.order.indexOf(t);
          if (!w) return null;
          return (
            <FloatingWindow
              key={t}
              title={TAB_TITLE[t]}
              win={w}
              z={60 + i}
              area={area}
              areaRef={viewerRef}
              dockTarget={stripRef}
              onChange={patch => setLayout(l => updateWin(l, t, patch))}
              onRaise={() => setLayout(l => raise(l, t))}
              onDock={() => dockWin(t)}
              onClose={() => setLayout(l => dock(l, t))}
            >
              <TabSlot host={hosts[t]} />
            </FloatingWindow>
          );
        })}
      {isPhone && leftOpen && (
        <BottomSheet key="left" label="Sources and layers" host={viewerRef} onClose={() => setLeftOpen(false)}>
          {leftPanel}
        </BottomSheet>
      )}
      {isPhone && rightOpen && (
        <BottomSheet key="right" label="Inspector and planning" head={tabs} host={viewerRef} onClose={() => setRightOpen(false)}>
          {rightBody}
        </BottomSheet>
      )}
    </div>
  );
}
