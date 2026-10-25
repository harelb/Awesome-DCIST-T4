import {useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactNode} from "react";
import {useSearchParams} from "react-router";
import {useViewer} from "../stores/viewer";
import {getJSON} from "../lib/api";
import {pickDefaultSources} from "../lib/runCopies";
import type {ActiveLaunch} from "../types";
import {sourceMatrix, useRobots} from "../stores/robots";
import {applyPoint, nearestGotoPlace, snapToPlannerMap, type Vec3} from "../lib/frameAlign";
import {layerInfo} from "../lib/graphModel";
import {Scene, type SceneCallbacks} from "./Scene";
import type {GraphLayout} from "./layout";
import type {TileStats} from "./MeshTiles";
import {DisplayPanel, Inspector, LayersPanel, SearchBox, SourcesPanel} from "./panels";
import {ColorsPanel, ScreenshotButton} from "./StylePanels";
import {PlanPanel, visitGoal} from "./PlanPanel";
import {ChatPanel} from "./ChatPanel";
import {LivePanel} from "./LivePanel";
import {LiveOverlaysPanel} from "./LiveOverlays";
import {RobotLegend} from "./RobotLegend";
import {CameraPanel} from "./CameraPanel";
import {DriveFullscreen, requestPageFullscreen} from "./DriveFullscreen";
import {TeleopPanel} from "./TeleopPanel";
import {SearchImagesPanel} from "./SearchImagesPanel";
import {Segmented} from "../components/ui";
import {plannerSourceId, readyModels} from "../stores/viewer";
import {useWsStatus} from "../ws";
import {useIsPhone, useMedia} from "../lib/media";
import {Icon} from "../components/Shell";
import {BottomSheet} from "./BottomSheet";
import {ControlsHelpButton, TopViewHint} from "./ControlsHelp";
import {FloatingWindow, Splitter, TabPortal, TabSlot, useElementSize, usePaneLayout, useTabHosts, useTabTearOff} from "./Panes";
import {CAM_H, FLOAT_MQ, LEFT, MIN_CANVAS, PANE_TABS, RIGHT, clampCameraH, clampWidths, dock, driveOn, popOut, raise, stripTabs, toggleDrive, updateWin, type PaneTab} from "./paneLayout";
import {RobotFocusPicker} from "./RobotFocusPicker";
import {useFocusedRobot, useRobotRoster} from "./useRobotFocus";
import {cycleFocus} from "./robotFocus";
import {DetectionToast, ImageViewer, MissionPanel} from "./MissionPanel";
import {fitMission, loadGeo, onMapClick, useMission} from "../stores/mission";
import {enterMissionView, leaveMissionView} from "../stores/missionView";
import {useMapShortcuts} from "./shortcuts";
import {editLayerOf} from "../lib/mapEdit";
import {idlePickMode, onEditClick, pickIntoSelection} from "../stores/mapEdit";
import {finishDrawing, useActiveTool} from "../stores/tools";
import {useChat} from "../stores/chat";
import {EditCanvasOverlay, LiveEditStrip, MapEditPanel, useEditFrameClass} from "./MapEditPanel";
import {AreaPopover} from "./AreaTools";
import {ToolBanner} from "./ToolBanner";
import {LoadBar} from "./LoadBar";
import {autoDetail} from "../lib/meshPlan";

type Tab = PaneTab;
const TAB_TITLE: Record<Tab, string> = {inspect: "Inspect", mission: "Mission", map: "Map", plan: "Plan", teleop: "Teleop", chat: "Chat", images: "Images", live: "Live link", camera: "Camera"};
/** the mission chat's narrowest width (the map keeps the rest) */
const CHAT_MIN_W = 300;
/** below this map width the viewer toolbar folds into a "more" menu */
export const NARROW_MAP_W = 700;
/** the same on /mission, whose toolbar has the extra "Fit mission" button */
export const NARROW_MISSION_MAP_W = 1000;
/** the mission planner's side-panel tabs; the others are under "More" (desktop) */
export const MISSION_TABS: Tab[] = ["mission", "map", "inspect"];

/** The side-panel strip on /mission (desktop): Mission | Map | Inspect, the
 *  rest in the More menu; elsewhere (and on phones) every tab is on the strip. */
export function splitTabs(strip: Tab[], missionMode: boolean, phone: boolean): {shown: Tab[]; more: Tab[]} {
  if (!missionMode || phone) return {shown: strip, more: []};
  return {shown: MISSION_TABS.filter(t => strip.includes(t)), more: strip.filter(t => !MISSION_TABS.includes(t))};
}

/** ``mode="mission"``: the mission planner (route /mission) — the chat is the
 *  left pane, the Mission tab leads the side panel, top view over satellite. */
export default function ViewerPage({mode = "viewer"}: {mode?: "viewer" | "mission"}) {
  const v = useViewer();
  const [params] = useSearchParams();
  const missionMode = mode === "mission";
  const [tab, setTabRaw] = useState<Tab>(missionMode ? "mission" : "inspect");
  const [layersInMission, setLayersInMission] = useState(false);
  const isPhone = useIsPhone();
  // mesh detail "auto": coarse under the mission planner's satellite top view,
  // medium on a phone, full on a desktop viewer (lib/meshPlan autoDetail)
  useEffect(() => {
    useViewer.getState().set({meshAutoDetail: autoDetail(missionMode, isPhone)});
  }, [missionMode, isPhone]);
  // pane layout: splitter widths + popped-out tabs (floating windows only on wide screens)
  const [layout, setLayout, resetLayout] = usePaneLayout();
  const wide = useMedia(FLOAT_MQ) && !isPhone;
  const floats = useCallback((t: Tab) => wide && !!layout.windows[t], [wide, layout.windows]);
  const floatsRef = useRef(floats);
  floatsRef.current = floats;
  const [leftOpen, setLeftOpenRaw] = useState(() => window.innerWidth > (missionMode ? 900 : 1100));
  const [rightOpen, setRightOpenRaw] = useState(() => window.innerWidth > 900);
  const [searchOpen, setSearchOpen] = useState(false);
  const viewerRef = useRef<HTMLDivElement>(null);
  const area = useElementSize(viewerRef);
  const canvasRef = useRef<HTMLDivElement>(null);
  const canvasSize = useElementSize(canvasRef);
  const narrow = !isPhone && canvasSize.w > 0 && canvasSize.w < (missionMode ? NARROW_MISSION_MAP_W : NARROW_MAP_W);
  // the top-left toolbar stops short of the top-right one (--tr-w), whatever
  // buttons that one shows
  const topRightRef = useRef<HTMLDivElement>(null);
  useLayoutEffect(() => {
    const tr = topRightRef.current;
    const canvas = canvasRef.current;
    if (!tr || !canvas) return;
    const read = () => canvas.style.setProperty("--tr-w", `${Math.ceil(tr.getBoundingClientRect().width)}px`);
    read();
    const ro = new ResizeObserver(read);
    ro.observe(tr);
    return () => ro.disconnect();
  }, [isPhone, missionMode]);
  const armed = useActiveTool();
  const pendingArea = useMission(s => s.pendingArea);
  const leftRef = useRef<HTMLElement>(null);
  const rightRef = useRef<HTMLElement>(null);
  const stripRef = useRef<HTMLDivElement>(null);
  const hosts = useTabHosts(PANE_TABS);
  const camPaneRef = useRef<HTMLElement>(null);
  const focused = useFocusedRobot();
  const roster = useRobotRoster();
  const drive = driveOn(layout, focused);
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
  const showTabRef = useRef(showTab);
  showTabRef.current = showTab;
  const [stats, setStats] = useState<Record<string, TileStats>>({});
  const [bounds, setBounds] = useState<{min: number[]; max: number[]} | null>(null);
  const [toast, setToastState] = useState<{text: string; err: boolean} | null>(null);
  const setToast = useCallback((text: string | null, err = false) => setToastState(text ? {text, err} : null), []);
  const robotStates = useRobots(s => s.state);
  const plans = useRobots(s => s.plan);
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
  // mission planner: satellite anchor + map extent, top view; tab requests from the chat
  useEffect(() => {
    if (missionMode) {
      useViewer.getState().set({topDown: true});
      enterMissionView();
    }
    void loadGeo(null);
    // {tab, soft}: a soft request (a mission the chat just proposed) only
    // selects the tab on a phone, where opening it would fold the chat away
    const show = (e: Event) => {
      const d = (e as CustomEvent<Tab | {tab: Tab; soft?: boolean}>).detail;
      const t = typeof d === "string" ? d : d.tab;
      if (typeof d !== "string" && d.soft && phoneRef.current) setTabRaw(t);
      else showTabRef.current(t);
    };
    window.addEventListener("adt4-show-tab", show);
    return () => {
      if (missionMode) leaveMissionView();
      window.removeEventListener("adt4-show-tab", show);
      if (useViewer.getState().pickMode === "mission") useViewer.getState().set({pickMode: idlePickMode()});
      useMission.getState().set({tool: {kind: "none"}, draft: [], pendingArea: null});
    };
  }, [missionMode]);
  // phones: a map tool needs the map, so arming one folds the sheets away (the
  // banner keeps Finish / Cancel in reach)
  const armedOwner = useActiveTool().owner;
  useEffect(() => {
    if (!phoneRef.current || armedOwner === "none") return;
    setLeftOpenRaw(false);
    setRightOpenRaw(false);
  }, [armedOwner]);
  // F / Space / Delete / Ctrl+Z (viewer/shortcuts.ts)
  useMapShortcuts();
  const editFrame = useEditFrameClass();
  const hasMission = useMission(s => !!s.current);
  const pendingDetections = useMission(s => s.run?.candidates.filter(c => c.status === "pending").length ?? 0);
  // "Re-plan in chat": bring the chat forward (the panel takes the text from the store)
  const prefill = useChat(s => s.prefill);
  useEffect(() => {
    if (prefill === null) return;
    if (missionMode) {
      setLayersInMission(false);
      setLeftOpen(true);
    } else showTabRef.current("chat");
  }, [prefill, missionMode]);
  // refresh the catalog when sources announce new versions / reconnects
  useEffect(() => {
    if (wsUp) void useViewer.getState().loadCatalog();
  }, [wsUp]);

  useEffect(() => {
    if (!toast) return;
    const id = window.setTimeout(() => setToast(null), toast.err ? 8000 : 3500);
    return () => window.clearTimeout(id);
  }, [toast, setToast]);

  // a goal the planner refused (e.g. "Requested symbol ... not in scene graph")
  // shows as an error toast; entries already there when a robot's plan is first
  // seen are history, not news
  const seenStatus = useRef<Record<string, number>>({});
  useEffect(() => {
    for (const [robot, p] of Object.entries(plans)) {
      const list = p?.status ?? [];
      const seen = seenStatus.current[robot];
      seenStatus.current[robot] = list.reduce((m, st) => Math.max(m, st.seq ?? 0), seen ?? 0);
      if (seen === undefined) continue;
      const failed = list.filter(st => (st.seq ?? 0) > seen && !st.ok);
      if (failed.length) setToast(`${robot}: planner rejected the goal: ${failed[failed.length - 1].error || "no reason given"}`, true);
    }
  }, [plans, setToast]);

  // goto picking snaps to the planner's map: load it as soon as picking starts
  useEffect(() => {
    if (v.pickMode !== "goto") return;
    const pid = plannerSourceId();
    if (pid && !useViewer.getState().sources[pid]?.enabled) useViewer.getState().enableSource(pid, true);
  }, [v.pickMode]);

  // [ / ] cycle the robot focus (all -> each robot -> all)
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== "[" && e.key !== "]") return;
      const t = e.target as HTMLElement | null;
      if (e.ctrlKey || e.metaKey || e.altKey || (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName)))) return;
      const st = useViewer.getState();
      st.setFocus(cycleFocus(focused ?? "all", roster, e.key === "]" ? 1 : -1));
      e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [focused, roster]);

  const enabled = useMemo(() => v.order.map(id => v.sources[id]).filter(s => s?.enabled), [v.order, v.sources]);

  const addGoto = useCallback((src: string, sym: string) => {
    const st = useViewer.getState();
    const model = st.sources[src]?.model;
    const n = model?.nodes.get(sym);
    if (!n || !model) return;
    // drawn in the global frame (a live source's nodes are in the robot's frame)
    st.set({gotoDraft: [...st.gotoDraft, {src, sym, pos: applyPoint(sourceMatrix(src), n.pos), label: model.displayName(n)}]});
    setTab("plan");
  }, [setTab]);

  // a goto pick (global frame) -> a node of the planner's map, the only symbols
  // omniplanner can look up; without a Neo4j source, the nearest shown place
  const pickGoto = useCallback(
    (p: Vec3, object: boolean) => {
      const st = useViewer.getState();
      const pid = plannerSourceId();
      if (!pid) {
        const best = nearestGotoPlace(readyModels(), p, sourceMatrix);
        if (best) addGoto(best.src, best.sym);
        else setToast("No place within 15 m of that point");
        return;
      }
      const ps = st.sources[pid];
      if (!ps.enabled) st.enableSource(pid, true);
      if (!ps.model || ps.status !== "ready") {
        setToast(ps.status === "error" ? `Planner map ${ps.label} failed to load: ${ps.error}` : `Loading the planner map (${ps.label}); pick again in a moment`, ps.status === "error");
        return;
      }
      const best = snapToPlannerMap({id: pid, model: ps.model}, p, sourceMatrix, {object});
      if (best) addGoto(best.src, best.sym);
      else setToast(`No place in the planner map (${ps.label}) within 15 m of that point`);
    },
    [addGoto, setToast],
  );

  // a picked node: the planner's own places / objects go as is; anything else
  // (agent poses, live-mirror or prior-file nodes, mesh) snaps to its map
  const gotoNode = useCallback(
    (src: string, sym: string) => {
      const n = useViewer.getState().sources[src]?.model?.nodes.get(sym);
      if (!n) return;
      const kind = layerInfo(n.layer, n.part).kind;
      if (src === plannerSourceId() && (kind === "places" || kind === "objects")) addGoto(src, sym);
      else pickGoto(applyPoint(sourceMatrix(src), n.pos), kind === "objects");
    },
    [addGoto, pickGoto],
  );

  // a picked search result: the Inspect tab (object images) comes to the front;
  // while editing the map it joins the edit selection instead (the tab stays)
  const showInspect = useCallback(
    (src?: string, sym?: string) => {
      if (src && sym && pickIntoSelection(src, sym)) return;
      setTab("inspect");
      setRightOpen(true);
    },
    [setRightOpen],
  );

  const cb: SceneCallbacks = useMemo(
    () => ({
      onNode: (src, sym) => {
        const st = useViewer.getState();
        if (st.pickMode === "edit") {
          // the selection grows in the Map tab; no tab switch
          const n = st.sources[src]?.model?.nodes.get(sym);
          onEditClick(src, sym, n ? applyPoint(sourceMatrix(src), n.pos) : null);
          return;
        }
        if (st.pickMode === "mission") {
          const n = st.sources[src]?.model?.nodes.get(sym);
          // a new task target takes the node's symbol when it is the planner map's
          const own = n && st.sources[src]?.kind === "neo4j" && editLayerOf(n) ? sym : null;
          if (n) onMapClick(applyPoint(sourceMatrix(src), n.pos), own);
          return;
        }
        if (st.pickMode === "goto") {
          gotoNode(src, sym);
          return;
        }
        st.select({src, sym});
        showTab("inspect");
      },
      onPoint: p => {
        const st = useViewer.getState();
        if (st.pickMode === "mission") {
          onMapClick(p);
          return;
        }
        if (st.pickMode === "edit") {
          onEditClick(null, null, p);
          return;
        }
        if (st.pickMode !== "goto") return;
        // the click is in the global frame (goals go to the planner as node
        // symbols of its map, not points)
        pickGoto(p, false);
      },
      onMiss: () => {
        if (useViewer.getState().pickMode === "inspect") useViewer.getState().select(null);
      },
      onStats: (src, s) => setStats(cur => ({...cur, [src]: s})),
      onLayout: (_src: string, l: GraphLayout) => {
        if (l.bounds) setBounds(b => b ?? l.bounds);
      },
    }),
    [gotoNode, pickGoto, showTab],
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
      <ColorsPanel />
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

  // the mission planner's left pane (and phone sheet): chat or layers
  const leftSwitch = (
    <div className="vleft-switch">
      <Segmented
        ariaLabel="Left pane"
        value={layersInMission ? "layers" : "chat"}
        onChange={x => setLayersInMission(x === "layers")}
        options={[
          {value: "chat", label: "Mission chat"},
          {value: "layers", label: "Layers"},
        ]}
      />
    </div>
  );

  // drive layout: the camera has its own pane (desktop: above the side-panel
  // tabs; phone: picture-in-picture over the map) unless it floats
  // the mission planner shows the chat as its left pane
  const strip = stripTabs(drive, floats("camera")).filter(t => !(missionMode && t === "chat"));
  const camPinned = drive && !isPhone && rightOpen && !floats("camera");
  const camPip = drive && isPhone && !floats("camera");
  // the docked side panel shows the selected tab, or the first strip tab that is not floating
  const dockTab: Tab | null = strip.includes(tab) && !floats(tab) ? tab : (strip.find(t => !floats(t)) ?? null);
  const mounted = (t: Tab) => floats(t) || (rightOpen && t === dockTab) || (t === "camera" && (camPinned || camPip));
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
  const wasDrive = useRef<boolean | null>(null);
  useEffect(() => {
    if (wasDrive.current === drive) return;
    const first = wasDrive.current === null;
    wasDrive.current = drive;
    // entering the drive layout (or opening the page in it) brings Teleop forward
    if (drive && (!first || focused)) {
      setTab("teleop");
      if (!floatsRef.current("teleop")) setRightOpen(true);
    }
  }, [drive, focused, setTab, setRightOpen]);
  const canTear = useCallback((t: Tab) => !floats(t), [floats]);
  const {shown: stripShown, more: stripMore} = splitTabs(strip, missionMode, isPhone);
  const onStripDown = useTabTearOff(stripRef, stripShown, canTear, useCallback((t: Tab, x: number, y: number) => pop(t, {x, y}), [pop]));
  const tabLabel = (t: Tab): ReactNode => (
    <>
      {t === "live" && isPhone ? "Link" : TAB_TITLE[t]}
      {t === "mission" && pendingDetections > 0 && (
        <span className="tab-badge" title={`${pendingDetections} detection(s) to review`} data-testid="mission-tab-badge">
          {pendingDetections}
        </span>
      )}
      {floats(t) && (
        <span className="tab-float-mark" title="In a floating window" aria-label="(floating)">
          ⧉
        </span>
      )}
    </>
  );
  const tabContent = (t: Tab) => {
    switch (t) {
      case "inspect":
        return <Inspector onVisit={onVisit} onGoto={gotoNode} />;
      case "mission":
        return <MissionPanel />;
      case "map":
        return <MapEditPanel />;
      case "plan":
        return <PlanPanel />;
      case "teleop":
        return <TeleopPanel />;
      case "chat":
        return <ChatPanel />;
      case "images":
        return <SearchImagesPanel />;
      case "live":
        return <LivePanel />;
      case "camera":
        return <CameraPanel compact={camPip || camPinned} />;
    }
  };
  const tabs = (
    <div className="vright-head" ref={stripRef} onPointerDown={wide ? onStripDown : undefined}>
      <Segmented<Tab>
        ariaLabel="Side panel"
        value={dockTab ?? tab}
        onChange={setTab}
        options={stripShown.map(t => ({value: t, label: tabLabel(t)}))}
      />
      {stripMore.length > 0 && <TabMore tabs={stripMore} active={dockTab && stripMore.includes(dockTab) ? dockTab : null} label={tabLabel} onPick={setTab} />}
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
  // a focused robot: follow is an on/off toggle for that robot; all robots: pick one to follow
  const follow = focused ? (
    <button
      className={`btn btn-sm panel-btn follow-btn ${v.follow === focused ? "on" : ""}`}
      aria-pressed={v.follow === focused}
      onClick={() => v.set({follow: v.follow === focused ? null : focused})}
      title={`Keep the camera on ${focused}`}
    >
      Follow
    </button>
  ) : (
    <select className="input input-sm follow-select" value={v.follow ?? ""} onChange={e => v.set({follow: e.target.value || null})} aria-label="Follow robot">
      <option value="">{isPhone ? "Follow" : "Follow: off"}</option>
      {robots.map(r => (
        <option key={r} value={r}>
          {isPhone ? r : `Follow ${r}`}
        </option>
      ))}
    </select>
  );
  const driveBtn = (
    <button
      className={`btn btn-sm panel-btn drive-btn ${drive ? "on" : ""}`}
      aria-pressed={drive}
      data-testid="drive-layout"
      onClick={() => {
        // teleop / full screen need one robot in focus: with "all robots", take
        // the only robot that is streaming (else the only one in the roster);
        // with several, the drive layout shows them all and says how to pick
        if (!focused && !drive) {
          const live = Object.keys(robotStates);
          const only = live.length === 1 ? live[0] : roster.length === 1 ? roster[0] : null;
          if (only) {
            useViewer.getState().setFocus(only);
            setLayout(l => ({...l, drive: true}));
            return;
          }
          setToast(`To drive (and for full screen), pick one robot in the robot focus menu: ${live.join(", ") || roster.join(", ")}`);
        }
        setLayout(l => toggleDrive(l, focused));
      }}
      title={drive ? "Drive layout on: camera pinned over Teleop · click for the standard layout" : "Drive layout: map, the focused robot's camera and Teleop side by side"}
    >
      {isPhone ? "Drive" : "Drive layout"}
    </button>
  );
  const camHead = (
    <header className="drive-cam-head">
      <span className="small muted">{focused ? `Camera · ${focused}` : "Cameras · all robots"}</span>
      {wide && (
        <button className="pop-btn" onClick={() => pop("camera")} aria-label="Pop out Camera" title="Pop the camera out into a floating window">
          ⧉
        </button>
      )}
    </header>
  );
  // full-screen drive (phones, touch tablets): offered in the drive layout
  // with a robot in focus
  const coarse = useMedia("(pointer: coarse)");
  useEffect(() => {
    if (!focused && useViewer.getState().driveFull) useViewer.getState().set({driveFull: false});
  }, [focused]);
  const fullBtn =
    drive && focused && (isPhone || coarse) ? (
      <button
        className="btn btn-sm panel-btn drive-full-btn"
        data-testid="drive-full-open"
        onClick={() => {
          requestPageFullscreen();
          setRightOpen(false);
          v.set({driveFull: true});
        }}
        title={`Drive ${focused} full screen: camera with two sticks`}
      >
        ⛶ {isPhone ? "Full" : "Full screen"}
      </button>
    ) : null;
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
    <div ref={viewerRef} className={`viewer ${leftOpen ? "left-open" : ""} ${rightOpen ? "right-open" : ""} ${isPhone ? "phone" : ""} ${narrow ? "map-narrow" : ""}`}>
      {PANE_TABS.filter(mounted).map(t => (
        <TabPortal key={t} host={hosts[t]}>
          {tabContent(t)}
        </TabPortal>
      ))}
      {leftOpen && !isPhone && (
        <aside ref={leftRef} className={`vpanel vleft ${missionMode && !layersInMission ? "vleft-chat" : ""}`} aria-label={missionMode ? "Mission chat" : "Sources and layers"} style={wide ? {width: Math.max(widths.leftW, missionMode && !layersInMission ? CHAT_MIN_W : 0)} : undefined}>
          {missionMode && leftSwitch}
          {missionMode && !layersInMission ? <ChatPanel missionMode /> : leftPanel}
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
      <div className={`vcanvas ${editFrame}`} ref={canvasRef} onDoubleClick={() => void finishDrawing()}>
        <LiveEditStrip phone={isPhone} />
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
                onPicked={(src, sym) => {
                  setSearchOpen(false);
                  showInspect(src, sym);
                }}
              />
              <button className="btn icon-only" onClick={() => setSearchOpen(false)} aria-label="Close search">
                ×
              </button>
            </div>
          ) : (
            <div className="vtoolbar phone-bar">
              {missionMode ? (
                <button className={`btn btn-sm panel-btn ${leftOpen ? "on" : ""}`} onClick={() => setLeftOpen(o => !o)} aria-label="Toggle mission chat" aria-pressed={leftOpen}>
                  Chat
                </button>
              ) : (
                <button className={`btn icon-only panel-btn ${leftOpen ? "on" : ""}`} onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel" aria-pressed={leftOpen}>
                  <Icon name="layers" size={20} />
                </button>
              )}
              <button className={`btn icon-only panel-btn ${v.query ? "on" : ""}`} onClick={() => setSearchOpen(true)} aria-label="Search graph">
                <Icon name="search" size={20} />
              </button>
              <span className="grow" />
              {missionMode && hasMission && (
                <button className="btn icon-only panel-btn" onClick={() => fitMission()} aria-label="Fit mission" title="Frame the open mission" data-testid="fit-mission">
                  ⌖
                </button>
              )}
              {camera}
              <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={false} compact mission={missionMode} />
              <button className={`btn icon-only panel-btn ${rightOpen ? "on" : ""}`} onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel" aria-pressed={rightOpen}>
                <Icon name="panel" size={20} />
              </button>
            </div>
          )
        ) : (
          <>
            <div className="vtoolbar top-left">
              <button className="btn btn-sm panel-btn" onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel">
                {leftOpen ? (narrow ? "Hide" : "Hide panel") : missionMode ? "Chat" : "Layers"}
              </button>
              {!narrow && <RobotFocusPicker />}
              <SearchBox onPicked={showInspect} />
            </div>
            <div className="vtoolbar top-right" ref={topRightRef}>
              {missionMode && (
                <button className="btn btn-sm panel-btn" disabled={!hasMission} onClick={() => fitMission()} title={hasMission ? "Frame the open mission: its agents, paths and areas (F)" : "Open a mission first"} data-testid="fit-mission">
                  {narrow ? "⌖" : "⌖ Fit mission"}
                </button>
              )}
              {!narrow && follow}
              {!narrow && driveBtn}
              {!narrow && fullBtn}
              {camera}
              {!narrow && <ScreenshotButton onDone={setToast} />}
              {!narrow && <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={wide} mission={missionMode} />}
              {narrow && (
                <ToolbarMore>
                  <RobotFocusPicker />
                  {follow}
                  {driveBtn}
                  {fullBtn}
                  <ScreenshotButton onDone={setToast} />
                  <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={wide} mission={missionMode} />
                </ToolbarMore>
              )}
              <button className="btn btn-sm panel-btn" onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel">
                {rightOpen ? "Hide" : "Panel"}
              </button>
            </div>
          </>
        )}
        {/* the second phone toolbar row steps aside while a map tool needs the map */}
        {isPhone && !searchOpen && armed.owner === "none" && (
          <div className="vtoolbar phone-bar phone-bar2">
            <RobotFocusPicker compact />
            <span className="grow" />
            <ScreenshotButton onDone={setToast} compact />
            {follow}
            {driveBtn}
            {fullBtn}
          </div>
        )}
        {camPip && (
          <section className="drive-pip" aria-label="Camera">
            <TabSlot host={hosts.camera} />
          </section>
        )}
        <ToolBanner phone={isPhone} />
        <AreaPopover />
        <ImageViewer />
        {/* notices stack under the toolbar instead of on top of each other */}
        <div className="canvas-notices">
          {toast && <div className={`toast${toast.err ? " toast-err" : ""}`} role={toast.err ? "alert" : "status"}>{toast.text}</div>}
          <EditCanvasOverlay />
          <DetectionToast onReview={() => showTab("mission")} />
        </div>
        {/* no rotate hint while a banner says what the next click does */}
        <TopViewHint active={v.topDown && armed.owner === "none" && v.pickMode !== "goto" && v.pickMode !== "edit" && !pendingArea} />
        <RobotLegend />
        <LoadBar />
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
        <aside ref={rightRef} className={`vpanel vright ${camPinned ? "drive" : ""}`} aria-label="Inspector and planning" style={wide ? {width: widths.rightW} : undefined}>
          {camPinned && (
            <>
              <section ref={camPaneRef} className="drive-cam" aria-label="Camera" style={{height: clampCameraH(layout.cameraH, area.h || window.innerHeight)}}>
                {camHead}
                <TabSlot host={hosts.camera} />
              </section>
              <Splitter
                side="top"
                orientation="horizontal"
                label="Resize camera pane"
                value={clampCameraH(layout.cameraH, area.h || window.innerHeight)}
                min={CAM_H.min}
                max={clampCameraH(CAM_H.max, area.h || window.innerHeight)}
                target={camPaneRef}
                onCommit={h => setLayout(l => ({...l, cameraH: h}))}
                onReset={() => setLayout(l => ({...l, cameraH: CAM_H.def}))}
              />
            </>
          )}
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
      {v.driveFull && focused && <DriveFullscreen key={focused} robot={focused} onExit={() => v.set({driveFull: false})} />}
      {isPhone && leftOpen && (
        <BottomSheet key="left" label={missionMode ? "Mission chat" : "Sources and layers"} host={viewerRef} onClose={() => setLeftOpen(false)}>
          {missionMode && leftSwitch}
          {missionMode && !layersInMission ? <ChatPanel missionMode /> : leftPanel}
        </BottomSheet>
      )}
      {isPhone && rightOpen && (
        <BottomSheet key="right" label="Inspector and planning" head={tabs} host={viewerRef} onClose={() => setRightOpen(false)} initial={camPip ? "peek" : "half"}>
          {rightBody}
        </BottomSheet>
      )}
    </div>
  );
}

/** The toolbar's overflow menu when the map is narrow. */
function ToolbarMore({children}: {children: ReactNode}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const away = (e: PointerEvent) => {
      // the menu's own popovers (screenshot, help) render inside it
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    const key = (e: KeyboardEvent) => e.key === "Escape" && setOpen(false);
    window.addEventListener("pointerdown", away);
    window.addEventListener("keydown", key);
    return () => {
      window.removeEventListener("pointerdown", away);
      window.removeEventListener("keydown", key);
    };
  }, [open]);
  return (
    <div className="vtoolbar-more" ref={ref}>
      <button className={`btn btn-sm panel-btn ${open ? "on" : ""}`} aria-expanded={open} aria-label="More map tools" title="Robot focus, follow, drive layout, screenshot, help" onClick={() => setOpen(o => !o)} data-testid="toolbar-more">
        <Icon name="more" size={16} />
      </button>
      {open && (
        <div className="vtoolbar-menu" role="menu">
          {children}
        </div>
      )}
    </div>
  );
}

/** The side panel's "More" tabs on /mission (desktop): Plan, Teleop, Images,
 *  Live link, Camera. Shows the open one's name while it is selected. */
function TabMore({tabs, active, label, onPick}: {tabs: Tab[]; active: Tab | null; label: (t: Tab) => ReactNode; onPick: (t: Tab) => void}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const away = (e: PointerEvent) => ref.current && !ref.current.contains(e.target as Node) && setOpen(false);
    const key = (e: KeyboardEvent) => e.key === "Escape" && setOpen(false);
    window.addEventListener("pointerdown", away);
    window.addEventListener("keydown", key);
    return () => {
      window.removeEventListener("pointerdown", away);
      window.removeEventListener("keydown", key);
    };
  }, [open]);
  return (
    <div className="tab-more" ref={ref}>
      <button className={`btn btn-sm tab-more-btn ${active ? "on" : ""}`} aria-expanded={open} aria-haspopup="menu" onClick={() => setOpen(o => !o)} data-testid="tab-more" title={tabs.map(t => TAB_TITLE[t]).join(", ")}>
        {active ? TAB_TITLE[active] : "More"} ▾
      </button>
      {open && (
        <div className="tab-more-menu" role="menu">
          {tabs.map(t => (
            <button
              key={t}
              role="menuitem"
              className={`btn btn-sm ${t === active ? "on" : ""}`}
              onClick={() => {
                setOpen(false);
                onPick(t);
              }}
            >
              {label(t)}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}
