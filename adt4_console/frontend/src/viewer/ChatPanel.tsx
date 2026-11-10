// Operator chat. Cypher over the planner's Neo4j map is how the assistant reads
// the scene graph (heracles_agents chatdsg). Nothing changes the map or moves a
// robot without the operator: writes, SAM3 objects and goals arrive as cards to
// approve, and each decision goes back to the assistant as a note.
import {useEffect, useRef, useState} from "react";
import {useViewer} from "../stores/viewer";
import {getToken, withToken} from "../lib/auth";
import {errorText, getJSON, postJSON} from "../lib/api";
import {Badge, Spinner} from "../components/ui";
import {sendGoal} from "./PlanPanel";
import {goalSendMessage, type SendTone} from "./PlanRepair";
import {LeaseRefusalNotice} from "./LeaseControl";
import {chatAbort, clearChat, storable, useChat} from "../stores/chat";
import {closeMission, fitMission, missionContext, openMission, saveCurrent, simCmd, useMission} from "../stores/mission";
import {clearPreview, previewPddl, PreviewError, previewRobots, usePlanPreview, type PddlPreview} from "../stores/planPreview";
import {FastDownwardDetails} from "./FastDownward";
import {PlannerSchedule} from "./PlannerSchedule";
import {EnginePicker} from "./EnginePicker";
import {GoalImages} from "./GoalImages";
import {DirectRouteButtons} from "./DirectRoute";
import {autoRoster, onlineRobots, planAutoSend, type AutoRobot, type Fleet, type Skipped} from "../lib/autoSend";
import {setAutoSend, useAutoSend} from "../stores/autoSend";
import {useRobots} from "../stores/robots";
import {useFleet} from "../stores/fleet";
import {rosterOf} from "./robotFocus";
import {agentColors} from "./MissionOverlay";
import {useRobotColor3D} from "./background";
import {ENGINE_LABEL, refreshEngines, useEngines, type EngineId} from "../stores/engines";
import type {FastDownwardInfo} from "../lib/fastDownward";
import {decideMapEdit, MapEditCard, mapEditNote, statusOf, toMapEditItem, type MapEditItem} from "./MapEditCard";
import {decideSceneEdit, SceneEditCard, sceneEditNote, sceneEditStatus, toSceneEditItem, undoSceneEdit, type SceneEditItem} from "./SceneEditCard";
import {refreshEdit} from "../stores/mapEdit";
import {candidateNote, candidateOrdinals, cellText, cypherErrorShort, fmtLeft, lookupSummary, mergeTargetsAfterCommit, scrollAnchor, toolGroups, trimTranscript, writeNote, type TranscriptItem, type WriteOutcome} from "../lib/chatTranscript";

export type ChatItem =
  | {type: "user"; text: string; note?: boolean}
  | {type: "text"; text: string}
  | {type: "tool"; name: string; args: unknown; result: unknown}
  | {type: "goal"; kind: string; body: Record<string, unknown>; robot?: string | null; sent?: {tone: SendTone; text: string}}
  | {type: "error"; text: string}
  | MissionCardItem
  | CypherWrite
  | MapEditItem
  | SceneEditItem
  | ImagesItem
  | ObjectCandidate
  | AutoSendItem;

/** A plan the chat sent by itself ("Auto-send to online robots"): local only. */
export interface AutoSendItem {
  type: "autosend";
  goal: string;
  engine?: string | null;
  /** the robot the goal was sent through (null: not sent, see ``blocked``) */
  via: string | null;
  robots: {name: string; kind: string; leaseHolder?: string}[];
  skipped: Skipped[];
  blocked?: string;
  result?: {tone: SendTone; text: string};
  /** the backend's lease report ({robot: {ok, message}}) */
  lease?: Record<string, {ok?: boolean; message?: string}> | null;
}

/** The console's robots now, for the auto-send rules. */
function fleetNow(): {names: string[]; fleet: Fleet} {
  const r = useRobots.getState();
  const machines = useFleet.getState().machines?.machines;
  const platforms: Record<string, string | undefined> = {};
  for (const m of machines ?? []) platforms[m.name] = (m as {platform_type?: string}).platform_type;
  return {names: rosterOf(machines, Object.keys(r.state), Object.keys(r.link)), fleet: {state: r.state, stateAt: r.stateAt, link: r.link, platforms}};
}

/** Online robots (and the skipped ones) for an engine, now. */
export function autoSendRobots(engine?: string | null): {online: AutoRobot[]; skipped: Skipped[]} {
  const {names, fleet} = fleetNow();
  return onlineRobots(names, fleet, Date.now(), engine);
}

/** A mission the assistant proposed (propose_mission); stored server-side. */
export interface MissionCardItem {
  type: "mission";
  id: string;
  revision: number;
  name: string;
  agents: {id: string; kind: string; binding?: string | null}[];
  tasks: {id: string; agent: string; type: string; target?: unknown; depends_on?: string[]}[];
  /** each agent's goals with where its path there ends (the mission preview's) */
  goals?: Record<string, {sym: string; xy?: number[] | null; yaw?: number | null}[]> | null;
  issues: {level: string; msg: string}[];
  /** what the assistant interpreted or substituted (shown in amber) */
  assumptions: string[];
  makespan: number | null;
  /** omniplanner's report when it planned the mission (plan_ground_mission) */
  planned?: {driven_m: Record<string, number>; planning_s: number; points_of_interest: number; pddl_goal?: string; fast_downward?: FastDownwardInfo | null; engine?: string; makespan_s?: number; planner_log?: FastDownwardInfo | null} | null;
}

/** A database change the assistant proposed; runs only when approved. */
export interface CypherWrite {
  type: "cypher_write";
  id: string;
  query: string;
  summary: string;
  src: string | null;
  status?: {state: "busy" | "done" | "rejected" | "error"; text: string};
}

/** Camera images of mapped nodes (show_object_images). */
export interface ImagesItem {
  type: "images";
  src: string;
  items: {sym: string; label: string; pos?: number[] | null; images: string[]}[];
}

/** A SAM3 detection from the robot's stored images, not yet in the map. */
export interface ObjectCandidate {
  type: "object_candidate";
  pending_id: string;
  index: number;
  label: string;
  score: number;
  image_uri: string | null;
  centroid: number[] | null;
  bbox_min: number[] | null;
  bbox_max: number[] | null;
  num_views?: number;
  would_merge_into?: {symbol: string; distance_m: number | null} | null;
  /** seconds left when it arrived, and when that was (ms) */
  expires_in_s?: number | null;
  received?: number;
  src?: string | null;
  /** UI state after the operator acts on it */
  status?: {state: "busy" | "added" | "discarded" | "error"; text: string};
}

const nums = (v: unknown): number[] | null => (Array.isArray(v) && v.every(x => typeof x === "number") ? (v as number[]) : null);

/** One streamed NDJSON object → a chat item (null: not shown, e.g. ``done``). */
export function toChatItem(o: Record<string, unknown>): ChatItem | null {
  switch (o.type) {
    case "text":
      return {type: "text", text: String(o.text ?? "")};
    case "tool":
      return {type: "tool", name: String(o.name), args: o.args, result: o.result};
    case "goal":
      return {type: "goal", kind: String(o.kind), body: (o.body as Record<string, unknown>) ?? {}, robot: typeof o.robot === "string" ? o.robot : null};
    case "error":
      return {type: "error", text: String(o.text ?? "error")};
    case "mission":
      return {
        type: "mission",
        id: String(o.id),
        revision: Number(o.revision ?? 1),
        name: String(o.name ?? ""),
        agents: Array.isArray(o.agents) ? (o.agents as MissionCardItem["agents"]) : [],
        tasks: Array.isArray(o.tasks) ? (o.tasks as MissionCardItem["tasks"]) : [],
        issues: Array.isArray(o.issues) ? (o.issues as MissionCardItem["issues"]) : [],
        assumptions: Array.isArray(o.assumptions) ? o.assumptions.map(String).filter(Boolean) : [],
        makespan: typeof o.makespan === "number" ? o.makespan : null,
        planned: o.planned && typeof o.planned === "object" ? (o.planned as MissionCardItem["planned"]) : null,
        goals: o.goals && typeof o.goals === "object" ? (o.goals as MissionCardItem["goals"]) : null,
      };
    case "cypher_write":
      return {type: "cypher_write", id: String(o.id), query: String(o.query ?? ""), summary: String(o.summary ?? ""), src: typeof o.src === "string" ? o.src : null};
    case "map_edit":
      return toMapEditItem(o);
    case "scene_edit":
      return toSceneEditItem(o);
    case "images":
      return {type: "images", src: String(o.src ?? ""), items: Array.isArray(o.items) ? (o.items as ImagesItem["items"]) : []};
    case "object_candidate": {
      if (typeof o.pending_id !== "string") return {type: "error", text: "object candidate without pending_id"};
      const m = o.would_merge_into as {symbol?: unknown; distance_m?: unknown} | null | undefined;
      return {
        type: "object_candidate",
        pending_id: o.pending_id,
        index: Number(o.index ?? 0),
        label: String(o.label ?? "object"),
        score: Number(o.score ?? 0),
        image_uri: typeof o.image_uri === "string" && o.image_uri.startsWith("data:image/") ? o.image_uri : null,
        centroid: nums(o.centroid),
        bbox_min: nums(o.bbox_min),
        bbox_max: nums(o.bbox_max),
        num_views: typeof o.num_views === "number" ? o.num_views : undefined,
        would_merge_into: m && typeof m.symbol === "string" ? {symbol: m.symbol, distance_m: typeof m.distance_m === "number" ? m.distance_m : null} : null,
        expires_in_s: typeof o.expires_in_s === "number" ? o.expires_in_s : null,
        received: Date.now(),
        src: typeof o.src === "string" ? o.src : null,
      };
    }
    default:
      return null;
  }
}

/** Chat request failure → message; 503 says why instead of always blaming ROS. */
export function chatHttpError(status: number, detail: string): string {
  if (status === 503) return `Chat unavailable: ${detail || "service unavailable"}`;
  if (status === 401 || status === 403) return "Not authorised: open the console with ?token=...";
  return detail || `HTTP ${status}`;
}

const fmt = (v: number[] | null | undefined) => (v ? `(${v.map(x => x.toFixed(1)).join(", ")})` : "?");

/** Parse complete NDJSON lines out of `buf`; returns [items, remainder]. */
export function parseNdjson(buf: string): [unknown[], string] {
  const lines = buf.split("\n");
  const rest = lines.pop() ?? "";
  const out: unknown[] = [];
  for (const l of lines) {
    const t = l.trim();
    if (!t) continue;
    try {
      out.push(JSON.parse(t));
    } catch {
      out.push({type: "error", text: `bad line: ${t.slice(0, 80)}`});
    }
  }
  return [out, rest];
}

/** Bar charts / aligned tables the assistant draws need a fixed-width font
 *  and must not wrap. */
export function looksTabular(text: string): boolean {
  const lines = text.split("\n");
  return /[█▇▆▅▄▃▂▁■▌]/.test(text) || lines.filter(l => /\S\s*\|/.test(l)).length >= 3;
}

const MODEL_KEY = "adt4.chat.model";
const REASONING_KEY = "adt4.chat.reasoning";
export const REASONING_LEVELS = ["fast", "low", "medium", "high"] as const;
/** GPT-5 / 6 (and o-series) take a reasoning effort. */
export const isReasoningModel = (m: string | null | undefined) => /^(gpt-[5-9]|o\d)/.test(m ?? "");
const loadReasoning = () => {
  try {
    const v = localStorage.getItem(REASONING_KEY);
    return (REASONING_LEVELS as readonly string[]).includes(v ?? "") ? (v as string) : "fast";
  } catch {
    return "fast";
  }
};
const loadModel = () => {
  try {
    return localStorage.getItem(MODEL_KEY) ?? "";
  } catch {
    return "";
  }
};

const setItems = (f: ChatItem[] | ((cur: ChatItem[]) => ChatItem[])) =>
  useChat.setState(s => ({items: typeof f === "function" ? f(s.items) : f}));
const setBusy = (busy: boolean) => useChat.setState({busy});
const transcript = {
  get current(): TranscriptItem[] {
    return useChat.getState().transcript;
  },
  set current(v: TranscriptItem[]) {
    useChat.setState({transcript: v});
  },
};
const busyRef = {
  get current() {
    return useChat.getState().busy;
  },
  set current(_v: boolean) {
    /* setBusy() writes the store */
  },
};
const turnStart = {
  get current() {
    return useChat.getState().turnStart;
  },
  set current(v: number) {
    useChat.setState({turnStart: v});
  },
};
const abort = chatAbort;

export function ChatPanel({missionMode = false}: {missionMode?: boolean} = {}) {
  const {planRobot, order, sources} = useViewer();
  // the conversation lives in the chat store: it outlives this panel (page
  // switches, reloads) and a turn keeps streaming into it while unmounted
  const items = useChat(s => s.items);
  const autoOn = useAutoSend(s => s.on);
  const busy = useChat(s => s.busy);
  const [input, setInput] = useState("");
  // "Re-plan in chat" (Mission tab): the text lands in the message box, unsent
  const prefill = useChat(s => s.prefill);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  useEffect(() => {
    if (prefill === null) return;
    setInput(prefill);
    useChat.setState({prefill: null});
    window.setTimeout(() => inputRef.current?.focus(), 0);
  }, [prefill]);
  const [models, setModels] = useState<string[]>([]);
  const [defaultModel, setDefaultModel] = useState<string | null>(null);
  const [model, setModelRaw] = useState(loadModel);
  const [reasoning, setReasoningRaw] = useState(loadReasoning);
  const setReasoning = (r: string) => {
    setReasoningRaw(r);
    try {
      localStorage.setItem(REASONING_KEY, r);
    } catch {
      /* private window */
    }
  };
  const scroller = useRef<HTMLDivElement>(null);
  // how many items the log last scrolled for (0: scroll to the end on mount)
  const shown = useRef(0);
  // Cypher runs on the planner's Neo4j map; else the first shown source
  const src = order.find(id => sources[id]?.kind === "neo4j") ?? order.find(id => sources[id]?.enabled) ?? undefined;

  useEffect(() => {
    const el = scroller.current;
    const grew = items.length > shown.current;
    shown.current = items.length;
    // a card changing state (approve / discard) leaves the log where the operator put it
    if (!el || !grew) return;
    // a SAM3 search: its first card on top, the others below (bottom-pinned, only the last was in view)
    const a = scrollAnchor(items, turnStart.current);
    const card = a === null ? null : el.querySelector<HTMLElement>(`[data-item="${a}"]`);
    el.scrollTop = card ? el.scrollTop + card.getBoundingClientRect().top - el.getBoundingClientRect().top : el.scrollHeight;
  }, [items]);
  useEffect(() => {
    getJSON<{models?: string[]; model?: string | null}>("api/chat/usage")
      .then(u => {
        setModels(u.models ?? []);
        setDefaultModel(u.model ?? null);
      })
      .catch(() => undefined);
  }, []);
  const setModel = (m: string) => {
    setModelRaw(m);
    try {
      localStorage.setItem(MODEL_KEY, m);
    } catch {
      /* private window */
    }
  };
  const chosenModel = model && models.includes(model) ? model : undefined;
  const ords = candidateOrdinals(items);
  const groups = toolGroups(items);

  const push = (it: ChatItem) =>
    setItems(cur => {
      const last = cur[cur.length - 1];
      // merge streamed text chunks into one bubble
      if (it.type === "text" && last?.type === "text") return [...cur.slice(0, -1), {type: "text", text: last.text + it.text}];
      if (it.type === "text") {
        const text = it.text.replace(/^\s+/, "");
        return text ? [...cur, {type: "text", text}] : cur;
      }
      if (it.type === "user") turnStart.current = cur.length;
      return [...cur, it];
    });

  /** One turn: ``message`` from the operator, or an operator note (``note``)
   *  the assistant should react to (an approval). */
  const ask = async (message: string, note = false) => {
    if (busyRef.current) {
      // the assistant is mid-turn: keep the note for the next request
      transcript.current = [...transcript.current, {role: "user", text: message}];
      return;
    }
    push({type: "user", text: message, note});
    busyRef.current = true;
    setBusy(true);
    const ctl = new AbortController();
    abort.current = ctl;
    let gotTranscript = false;
    const toSend: AutoQueued[] = [];
    try {
      const headers: Record<string, string> = {"Content-Type": "application/json"};
      const t = getToken();
      if (t) headers.Authorization = `Bearer ${t}`;
      // the mission planner's context: agents, drawn areas, the open mission (propose_mission)
      const ms = useMission.getState();
      // auto-send: plan for the robots that are online now, whatever the Mission panel holds
      const auto = useAutoSend.getState().on;
      const engineNow = useEngines.getState().engine;
      const autoFleet = auto ? autoSendRobots(engineNow) : null;
      const withMission = auto || missionMode || (ms.chatOn && (ms.roster.length > 0 || !!ms.current));
      const body = {
        message,
        robot: planRobot ?? undefined,
        src: withMission ? (ms.world?.src ?? src) : src,
        history: trimTranscript(transcript.current),
        model: chosenModel,
        mission_ctx: withMission ? (autoFleet ? {...missionContext(), roster: autoRoster(autoFleet.online), auto_send: true} : missionContext()) : undefined,
        reasoning: isReasoningModel(chosenModel ?? defaultModel) ? reasoning : undefined,
      };
      const res = await fetch("api/chat", {method: "POST", headers, body: JSON.stringify(body), signal: ctl.signal});
      if (!res.ok || !res.body) {
        let detail = `HTTP ${res.status}`;
        try {
          detail = (await res.json()).detail ?? detail;
        } catch {
          /* ignore */
        }
        push({type: "error", text: chatHttpError(res.status, detail)});
        return;
      }
      const reader = res.body.getReader();
      const dec = new TextDecoder();
      let buf = "";
      const handle = (o: Record<string, unknown>) => {
        if (o.type === "transcript" && Array.isArray(o.items)) {
          transcript.current = trimTranscript([...transcript.current, ...(o.items as TranscriptItem[])], 120);
          gotTranscript = true;
          return;
        }
        const it = toChatItem(o);
        if (it) push(it);
        if (auto && it) {
          if (it.type === "goal" && it.kind === "pddl" && typeof it.body.goal === "string")
            toSend.push({goal: it.body.goal, engine: (it.body.engine as string | undefined) ?? engineNow, constraints: it.body.constraints, card: it});
          else if (it.type === "mission" && it.planned?.pddl_goal) toSend.push({goal: it.planned.pddl_goal, engine: it.planned.engine ?? engineNow});
        }
        // a proposed mission opens in the Mission tab at once (on a phone the
        // chat sheet stays up: the tab is only selected for when the panel opens)
        if (it?.type === "mission")
          void openMission(it.id, {fit: true}).then(() => {
            if (missionMode) window.dispatchEvent(new CustomEvent("adt4-show-tab", {detail: {tab: "mission", soft: true}}));
          });
      };
      for (;;) {
        const {value, done} = await reader.read();
        if (done) break;
        buf += dec.decode(value, {stream: true});
        const [objs, rest] = parseNdjson(buf);
        buf = rest;
        for (const o of objs as Record<string, unknown>[]) handle(o);
      }
      if (buf.trim()) for (const o of parseNdjson(buf + "\n")[0] as Record<string, unknown>[]) handle(o);
      for (const q of toSend) await autoSendGoal(q);
    } catch (e) {
      if (!ctl.signal.aborted) push({type: "error", text: errorText(e)});
    } finally {
      // offline assistant / errors: still remember what the operator said
      if (!gotTranscript) transcript.current = [...transcript.current, {role: "user", text: message}];
      busyRef.current = false;
      setBusy(false);
    }
  };

  const send = () => {
    const message = input.trim();
    if (!message || busyRef.current) return;
    setInput("");
    void ask(message);
  };

  const newChat = () => {
    clearChat();
    // the mission planner's Clear also starts the next mission from scratch
    if (missionMode) closeMission();
  };
  const current = useMission(s => s.current);
  const [saveName, setSaveName] = useState<string | null>(null);
  const [savedNote, setSavedNote] = useState<string | null>(null);
  const doSave = async () => {
    const {items: its, transcript: tr} = useChat.getState();
    const ok = await saveCurrent(saveName ?? "", {items: storable(its), transcript: tr});
    setSaveName(null);
    setSavedNote(ok ? "saved" : null);
    window.setTimeout(() => setSavedNote(null), 2500);
  };

  const patch = (idx: number, f: (it: ChatItem) => ChatItem) => setItems(cur => cur.map((c, i) => (i === idx ? f(c) : c)));

  const decideWrite = async (idx: number, approve: boolean) => {
    const it = items[idx];
    if (it.type !== "cypher_write") return;
    patch(idx, c => ({...(c as CypherWrite), status: {state: "busy", text: approve ? "running ..." : "rejecting ..."}}));
    try {
      const r = await postJSON<WriteOutcome>("api/chat/cypher_write", {id: it.id, approve});
      const counters = Object.entries(r.counters ?? {})
        .map(([k, v]) => `${k.replace(/_/g, " ")}: ${v}`)
        .join(", ");
      const status: CypherWrite["status"] = r.rejected
        ? {state: "rejected", text: "rejected; nothing ran"}
        : r.ok
          ? {state: "done", text: `ran · ${counters || "no changes"}`}
          : {state: "error", text: r.error ?? "failed"};
      patch(idx, c => ({...(c as CypherWrite), status}));
      // the assistant learns the outcome; an approval lets it carry on
      if (approve) void ask(writeNote(it.summary, r), true);
      else transcript.current = [...transcript.current, {role: "user", text: writeNote(it.summary, r)}];
    } catch (e) {
      patch(idx, c => ({...(c as CypherWrite), status: {state: "error", text: errorText(e)}}));
    }
  };

  const decideEdit = async (idx: number, approve: boolean) => {
    const it = items[idx];
    if (it.type !== "map_edit") return;
    patch(idx, c => ({...(c as MapEditItem), status: {state: "busy", text: approve ? "applying ..." : "rejecting ..."}}));
    try {
      const r = await decideMapEdit(it.id, approve);
      patch(idx, c => ({...(c as MapEditItem), status: statusOf(r)}));
      void refreshEdit();
      if (approve && r.ok) void ask(mapEditNote(it, r), true);
      else transcript.current = [...transcript.current, {role: "user", text: mapEditNote(it, r)}];
    } catch (e) {
      patch(idx, c => ({...(c as MapEditItem), status: {state: "error", text: errorText(e)}}));
    }
  };

  const decideScene = async (idx: number, approve: boolean) => {
    const it = items[idx];
    if (it.type !== "scene_edit") return;
    patch(idx, c => ({...(c as SceneEditItem), status: {state: "busy", text: approve ? "applying ..." : "rejecting ..."}}));
    try {
      const r = await decideSceneEdit(it.id, approve);
      patch(idx, c => ({...(c as SceneEditItem), status: sceneEditStatus(r), undoable: Boolean(r.ok && r.undoable)}));
      if (approve && r.ok) void ask(sceneEditNote(it, r), true);
      else transcript.current = [...transcript.current, {role: "user", text: sceneEditNote(it, r)}];
    } catch (e) {
      patch(idx, c => ({...(c as SceneEditItem), status: {state: "error", text: errorText(e)}}));
    }
  };

  const undoScene = async (idx: number) => {
    const it = items[idx];
    if (it.type !== "scene_edit") return;
    patch(idx, c => ({...(c as SceneEditItem), undoable: false, status: {state: "busy", text: "undoing ..."}}));
    try {
      const r = await undoSceneEdit(it.src);
      const text = r.ok ? `undone: ${r.result ?? "the map is rebuilt without the latest chat edit"}` : `undo failed: ${r.result ?? r.error ?? "unknown error"}`;
      patch(idx, c => ({...(c as SceneEditItem), undoable: !r.ok, status: {state: r.ok ? "undone" : "error", text}}));
      transcript.current = [...transcript.current, {role: "user", text: `[operator] ${r.ok ? "Undid" : "Tried to undo"} the scene edit "${it.summary || it.text}": ${r.result ?? r.error ?? ""}`}];
    } catch (e) {
      patch(idx, c => ({...(c as SceneEditItem), undoable: true, status: {state: "error", text: errorText(e)}}));
    }
  };

  const actOnCandidate = async (idx: number, add: boolean) => {
    const it = items[idx];
    if (it.type !== "object_candidate") return;
    patch(idx, c => ({...(c as ObjectCandidate), status: {state: "busy", text: add ? "adding ..." : "discarding ..."}}));
    try {
      if (add) {
        const r = await postJSON<{symbol?: string; created?: boolean; merged?: boolean | string}>("api/sam3/commit", {
          pending_id: it.pending_id,
          index: it.index,
          src: it.src ?? undefined,
        });
        const how = r.merged ? `merged into ${r.symbol ?? "an existing object"}` : `added as ${r.symbol ?? "a new object"}`;
        setItems(cur => {
          const next = cur.map((c, i) => (i === idx ? {...(c as ObjectCandidate), status: {state: "added" as const, text: how}} : c));
          if (!r.symbol) return next;
          // the other undecided cards now merge into it when close enough
          const cands = next.filter((c): c is ObjectCandidate => c.type === "object_candidate");
          const targets = mergeTargetsAfterCommit(cands, it.label, it.centroid, r.symbol);
          let k = 0;
          return next.map(c => (c.type === "object_candidate" ? {...c, would_merge_into: targets[k++]} : c));
        });
        void ask(candidateNote(it.label, r), true);
      } else {
        await postJSON("api/sam3/discard", {pending_id: it.pending_id});
        // one pending id covers every candidate of that search
        setItems(cur =>
          cur.map(c =>
            c.type === "object_candidate" && c.pending_id === it.pending_id && c.status?.state !== "added" ? {...c, status: {state: "discarded", text: "discarded"}} : c,
          ),
        );
        transcript.current = [...transcript.current, {role: "user", text: `[operator] Discarded the "${it.label}" candidates; they are not in the map.`}];
      }
    } catch (e) {
      patch(idx, c => ({...(c as ObjectCandidate), status: {state: "error", text: errorText(e)}}));
    }
  };

  /** Send a plan the chat made, as the PDDL goal omniplanner routes (never a
   *  straight goto), through one robot it names (-by forms cover the others). */
  const autoSendGoal = async (q: AutoQueued) => {
    const {online, skipped} = autoSendRobots(q.engine);
    const plan = planAutoSend(q.goal, online, skipped);
    const named = plan.robots.map(n => online.find(r => r.name === n) ?? {name: n, kind: "spot"});
    const card: AutoSendItem = {type: "autosend", goal: q.goal, engine: q.engine, via: plan.via, robots: named, skipped: plan.skipped, blocked: plan.blocked};
    if (!plan.via) {
      push(card);
      return;
    }
    let result: AutoSendItem["result"];
    let lease: AutoSendItem["lease"] = null;
    try {
      const body: Record<string, unknown> = {goal: q.goal};
      if (q.constraints) body.constraints = q.constraints;
      if (q.engine === "grstaps") body.engine = "grstaps";
      const r = await sendGoal(plan.via, "pddl", body);
      result = goalSendMessage(`Sent through ${plan.via}`, r);
      lease = ((r as unknown as {lease?: AutoSendItem["lease"]}).lease ?? null) || null;
    } catch (e) {
      result = {tone: "err", text: `failed: ${errorText(e)}`};
    }
    push({...card, result, lease});
    if (q.card) setItems(cur => cur.map(c => (c === q.card ? {...(c as Extract<ChatItem, {type: "goal"}>), sent: {tone: result!.tone, text: `auto-sent: ${result!.text}`}} : c)));
    // the assistant should know the plan is underway
    transcript.current = [...transcript.current, {role: "user", text: `[operator] auto-sent ${q.goal} through ${plan.via}: ${result!.text}`}];
  };

  const sendProposal = async (idx: number) => {
    const it = items[idx];
    if (it.type !== "goal") return;
    const robot = it.robot ?? planRobot;
    if (!robot) {
      patch(idx, c => ({...(c as typeof it), sent: {tone: "err", text: "no robot selected"}}));
      return;
    }
    try {
      // a PDDL goal goes to the engine the card shows (GRSTAPS-X: its plugin or goal_manager)
      const engine = it.kind === "pddl" ? ((it.body as {engine?: string}).engine ?? useEngines.getState().engine) : undefined;
      const r = await sendGoal(robot, it.kind, engine === "grstaps" ? {...it.body, engine} : it.body);
      patch(idx, c => ({...(c as typeof it), sent: goalSendMessage(`Sent to ${robot}`, r)}));
    } catch (e) {
      patch(idx, c => ({...(c as typeof it), sent: {tone: "err", text: `failed: ${errorText(e)}`}}));
    }
  };

  return (
    <div className="chat">
      <div className="chat-bar row gap-sm">
        {models.length > 0 && (
          <select className="input input-sm" value={chosenModel ?? ""} onChange={e => setModel(e.target.value)} aria-label="Chat model" title="Model the assistant runs on">
            <option value="">{defaultModel ? `${defaultModel} (default)` : "default model"}</option>
            {models
              .filter(m => m !== defaultModel)
              .map(m => (
                <option key={m} value={m}>
                  {m}
                </option>
              ))}
          </select>
        )}
        {isReasoningModel(chosenModel ?? defaultModel) && (
          <select
            className="input input-sm"
            value={reasoning}
            onChange={e => setReasoning(e.target.value)}
            aria-label="Reasoning effort"
            title="How hard the model thinks before answering: fast = the least it allows (quickest); high = slowest, best for hard lookups and plans"
            data-testid="reasoning-effort"
          >
            {REASONING_LEVELS.map(r => (
              <option key={r} value={r}>
                {r === "fast" ? "reasoning: fast" : `reasoning: ${r}`}
              </option>
            ))}
          </select>
        )}
        <label className="row gap-xs small chat-autosend" title="Plans the chat makes are sent at once to the robots that are online (fleetlink up, executor status, not E-stopped); sending takes a Spot's lease">
          <input type="checkbox" checked={autoOn} onChange={e => setAutoSend(e.target.checked)} data-testid="auto-send-toggle" />
          Auto-send to online robots
        </label>
        {autoOn && (
          <span data-testid="auto-send-on">
            <Badge tone="warn">AUTO-SEND ON</Badge>
          </span>
        )}
        <span className="grow" />
        {missionMode && current && saveName === null && (
          <button className="btn btn-xs" onClick={() => setSaveName(current.name || "")} title="Keep this mission with its chat and the approvals of its simulated run, to replay or continue later">
            {savedNote ? "✓ Saved" : current.saved ? "Save again" : "Save mission"}
          </button>
        )}
        {missionMode && saveName !== null && (
          <span className="row gap">
            <input
              className="input input-xs"
              autoFocus
              placeholder="mission name"
              value={saveName}
              onChange={e => setSaveName(e.target.value)}
              onKeyDown={e => {
                if (e.key === "Enter") void doSave();
                if (e.key === "Escape") setSaveName(null);
              }}
              aria-label="Mission name"
            />
            <button className="btn btn-xs btn-primary" onClick={() => void doSave()}>
              Save
            </button>
          </span>
        )}
        <button
          className="btn btn-xs"
          onClick={newChat}
          disabled={!items.length && !(missionMode && current)}
          title={missionMode ? "Start over: clear the chat and close the mission (saved missions stay in the Mission tab)" : "Start over: the assistant forgets this conversation"}
        >
          {missionMode ? "Clear" : "New chat"}
        </button>
      </div>
      {missionMode && (
        <div className="chat-engine">
          <EnginePicker label="planner" />
        </div>
      )}
      <div className="chat-log" ref={scroller}>
        {items.length === 0 && (
          <div className="muted small pad">
            Ask about the map, e.g. <span className="mono">what is O7?</span>, <span className="mono">how many trees are near the robot?</span>, or give a task. The assistant reads the
            map with Cypher; map changes, new objects and goals wait for your approval.
          </div>
        )}
        {items.map((it, i) => {
          // a turn's lookups: one disclosure where the first one was
          const grp = groups.get(i);
          if (grp === null) return null;
          if (grp) return <LookupGroup key={i} tools={grp.map(k => items[k] as ToolItem)} />;
          if (it.type === "user")
            return (
              <div key={i} className={`msg msg-user ${it.note ? "msg-note" : ""}`}>
                {it.note ? it.text.replace(/^\[operator\]\s*/, "") : it.text}
              </div>
            );
          if (it.type === "text") return <div key={i} className={`msg msg-bot ${looksTabular(it.text) ? "msg-pre" : ""}`}>{it.text}</div>;
          if (it.type === "error") return <div key={i} className="msg msg-err">{it.text}</div>;
          if (it.type === "object_candidate") return <CandidateCard key={i} idx={i} ord={ords[i]} it={it} onAct={add => void actOnCandidate(i, add)} />;
          if (it.type === "cypher_write") return <WriteCard key={i} it={it} onDecide={ok => void decideWrite(i, ok)} />;
          if (it.type === "map_edit") return <MapEditCard key={i} it={it} onDecide={ok => void decideEdit(i, ok)} />;
          if (it.type === "scene_edit") return <SceneEditCard key={i} it={it} onDecide={ok => void decideScene(i, ok)} onUndo={() => void undoScene(i)} />;
          if (it.type === "images") return <ImagesCard key={i} it={it} />;
          if (it.type === "tool") return <ToolCard key={i} it={it} />;
          if (it.type === "mission") return <MissionCard key={i} it={it} />;
          if (it.type === "autosend") return <AutoSendCard key={i} it={it} />;
          return <GoalCard key={i} it={it} robot={it.robot ?? planRobot} src={src ?? null} onSend={() => void sendProposal(i)} />;
        })}
        {busy && <Spinner label="thinking" />}
      </div>
      <LeaseRefusalNotice robot={planRobot} />
      <div className="chat-input">
        <textarea
          ref={inputRef}
          className="input"
          rows={2}
          value={input}
          placeholder={planRobot ? `Message (robot ${planRobot})` : "Message"}
          onChange={e => setInput(e.target.value)}
          onKeyDown={e => {
            if (e.key === "Enter" && !e.shiftKey) {
              e.preventDefault();
              send();
            }
          }}
          aria-label="Chat message"
        />
        <button className="btn btn-primary" onClick={send} disabled={busy || !input.trim()}>
          Send
        </button>
      </div>
    </div>
  );
}

/** {agent: [node symbol, ...]}: where each agent's tasks go (map symbols only, in order). */
export function missionGoals(tasks: {agent: string; type: string; target?: unknown}[]): Record<string, string[]> {
  const out: Record<string, string[]> = {};
  for (const t of tasks) {
    if (typeof t.target !== "string" || t.target.startsWith("$") || !/^[A-Za-z]+\(?\d+\)?$/.test(t.target)) continue;
    const l = (out[t.agent] ??= []);
    if (!l.includes(t.target)) l.push(t.target);
  }
  return out;
}

export function MissionCard({it}: {it: MissionCardItem}) {
  const mine = useMission(s => s.current?.id === it.id);
  // the newest revision: the open mission's, else the mission list's
  const latest = useMission(s => (s.current?.id === it.id ? s.current.revision : (s.list.find(m => m.id === it.id)?.revision ?? it.revision)));
  const mapNow = useMission(s => (s.current?.id === it.id ? s.preview?.map : null));
  const run = useMission(s => (s.current?.id === it.id ? s.run : null));
  const superseded = latest > it.revision;
  const running = cardRunState(run);
  const errors = it.issues.filter(i => i.level === "error");
  const warnings = it.issues.filter(i => i.level === "warning");
  const open = async (simulate: boolean) => {
    await openMission(it.id, {fit: true});
    window.dispatchEvent(new CustomEvent("adt4-show-tab", {detail: "mission"}));
    if (simulate) {
      await simCmd("new");
      await simCmd("play");
      fitMission();
    }
  };
  const byAgent = it.agents.map(a => ({a, tasks: it.tasks.filter(t => t.agent === a.id)}));
  const robotColor = useRobotColor3D();
  const colors = agentColors(it.agents as Parameters<typeof agentColors>[0], robotColor);
  // where each goal's path ends (places get the nearest view): the card's, else the open
  // mission's preview of this revision (cards from before goals were sent), else symbols only
  const openGoals = useMission(s => (s.current?.id === it.id && s.current.revision === it.revision ? s.preview?.goals : null));
  const goals = it.goals ?? openGoals ?? missionGoals(it.tasks);
  return (
    <div className={`card chat-card mission-card ${superseded ? "superseded" : ""}`}>
      <div className="row gap">
        <b>🗺 {it.name || "Mission"}</b>
        <Badge tone="info">r{it.revision}</Badge>
        {it.makespan != null && <span className="muted small">~{Math.round(it.makespan)} s</span>}
        {superseded && <Badge tone="off">superseded</Badge>}
        {!superseded && mine && <Badge tone="ok">open</Badge>}
      </div>
      <ul className="small mission-card-agents">
        {byAgent.map(({a, tasks}) => (
          <li key={a.id}>
            <span className="mono">{a.id}</span> <span className="muted">({a.kind}{a.binding ? ` → ${a.binding}` : ""})</span>:{" "}
            {tasks.map(t => `${t.type.replace("_", " ")}${t.target ? ` ${typeof t.target === "string" ? t.target : JSON.stringify(t.target)}` : ""}`).join(" → ") || "idle"}
          </li>
        ))}
      </ul>
      <GoalImages goals={goals} colors={colors} />
      {it.planned && (
        <div className="small muted" data-testid="mission-planned">
          omniplanner{it.planned.engine ? ` (${ENGINE_LABEL[it.planned.engine as EngineId] ?? it.planned.engine})` : ""}:{" "}
          {Object.entries(it.planned.driven_m)
            .map(([a, m]) => `${a} ${Math.round(m)} m`)
            .join(" · ")}{" "}
          · planned in {it.planned.planning_s.toFixed(1)} s{it.planned.makespan_s != null ? ` · makespan ${Math.round(it.planned.makespan_s)} s` : ""}
          {it.planned.pddl_goal && <pre className="mono small">{it.planned.pddl_goal}</pre>}
          <FastDownwardDetails fd={it.planned.planner_log ?? it.planned.fast_downward} />
        </div>
      )}
      {(it.assumptions ?? []).length > 0 && (
        <div className="mission-assumptions small" data-testid="mission-assumptions">
          <b>Assumed</b>
          <ul>
            {it.assumptions.map((a, k) => (
              <li key={k}>{a}</li>
            ))}
          </ul>
        </div>
      )}
      {errors.length > 0 && <div className="msg-err small">✖ {errors.map(e => e.msg).join("; ")}</div>}
      {warnings.length > 0 && <div className="small muted">⚠ {warnings.length} warning(s)</div>}
      {mapNow?.stale && !superseded && (
        <div className="small tone-warn" role="status" data-testid="mission-map-stale">
          ⚠ {mapNow.message ?? "map changed since planning"}{" "}
          <button className="btn btn-xs" onClick={() => void open(false)} title="Re-preview on the current map in the Mission tab">
            Re-preview
          </button>
        </div>
      )}
      <div className="row gap wrap">
        {superseded ? (
          <button className="btn btn-xs" onClick={() => void open(false)} data-testid="card-open-latest" title={`This is r${it.revision}; the mission is at r${latest} now (edited in the Mission tab or by a later proposal)`}>
            Superseded by r{latest} — open latest
          </button>
        ) : (
          <>
            <button
              className="btn btn-xs btn-primary"
              disabled={errors.length > 0 || !!running}
              onClick={() => void open(true)}
              title={running ? `${running}: pause or reset it in the Mission tab` : errors.length ? "Fix the errors first" : "Simulate it (Mission tab)"}
            >
              {running ? `▶ ${running}…` : "▶ Simulate"}
            </button>
            <button className="btn btn-xs" onClick={() => void open(false)}>
              {mine ? "Show in Mission tab" : "Open in Mission tab"}
            </button>
          </>
        )}
      </div>
    </div>
  );
}

/** Why a mission card cannot start a simulation now (null: it can). */
export function cardRunState(run: {kind: string; playing: boolean; finished: boolean} | null | undefined): string | null {
  if (!run || run.finished) return null;
  if (run.kind === "live") return "Running live";
  return run.playing ? "Simulating" : null;
}

type ToolItem = Extract<ChatItem, {type: "tool"}>;

/** One turn's map lookups (Cypher queries, searches) behind one disclosure:
 *  "Looked up the map · 3 queries, 2 retried"; failed ones show a short reason. */
function LookupGroup({tools}: {tools: ToolItem[]}) {
  const sum = lookupSummary(tools);
  return (
    <details className="msg msg-tool msg-lookups" data-testid="chat-lookups">
      <summary>
        <span className="lookup-title">Looked up the map</span> <span className="muted small chip-nowrap">{sum.text}</span>
      </summary>
      <div className="lookup-list">
        {tools.map((t, k) => (
          <ToolCard key={k} it={t} />
        ))}
      </div>
    </details>
  );
}

function ToolCard({it}: {it: Extract<ChatItem, {type: "tool"}>}) {
  if (it.name === "plan_ground_mission") {
    // only failures are tool cards (a plan is a mission card)
    const a = (it.args ?? {}) as {pddl_goal?: string; engine?: string};
    const r = (it.result ?? {}) as {error?: string; fast_downward?: FastDownwardInfo | null; planner_log?: FastDownwardInfo | null};
    return (
      <div className="msg msg-tool msg-err" data-testid="omniplanner-failure">
        <Badge tone="err">omniplanner{a.engine ? ` · ${ENGINE_LABEL[a.engine as EngineId] ?? a.engine}` : ""}</Badge> <span className="small">{r.error ?? "no plan"}</span>
        {a.pddl_goal && <pre className="mono small">{a.pddl_goal}</pre>}
        <FastDownwardDetails fd={r.planner_log ?? r.fast_downward} />
      </div>
    );
  }
  if (it.name === "run_cypher") {
    const q = String((it.args as {query?: string})?.query ?? "");
    const r = (it.result ?? {}) as {columns?: string[]; rows?: Record<string, unknown>[]; row_count?: number; truncated?: boolean; error?: string};
    const cols = r.columns ?? (r.rows?.[0] ? Object.keys(r.rows[0]) : []);
    const label = r.error ? "error" : `${r.row_count ?? 0}${r.truncated ? "+" : ""} row${r.row_count === 1 ? "" : "s"}`;
    return (
      <details className="msg msg-tool msg-cypher">
        <summary>
          <Badge tone={r.error ? "err" : "info"}>Cypher</Badge> <span className="muted small chip-nowrap">{label}</span> <span className="mono small cypher-peek">{q.replace(/\s+/g, " ")}</span>
        </summary>
        <pre className="mono small cypher-query">{q}</pre>
        {r.error ? (
          <div className="small tone-err cypher-err">
            {cypherErrorShort(r.error)}
            <details className="cypher-err-full">
              <summary className="muted">full error</summary>
              <pre className="mono small">{r.error}</pre>
            </details>
          </div>
        ) : r.rows && r.rows.length > 0 ? (
          <div className="table-wrap cypher-table">
            <table className="table small">
              <thead>
                <tr>
                  {cols.map(c => (
                    <th key={c} className="mono">
                      {c}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {r.rows.map((row, k) => (
                  <tr key={k}>
                    {cols.map(c => (
                      <td key={c} className="mono">
                        {cellText(row[c])}
                      </td>
                    ))}
                  </tr>
                ))}
              </tbody>
            </table>
            {(r.row_count ?? 0) > r.rows.length && <div className="muted small">showing {r.rows.length} of {r.row_count}{r.truncated ? "+" : ""}</div>}
          </div>
        ) : (
          <div className="muted small">no rows</div>
        )}
      </details>
    );
  }
  if (it.name === "search_robot_images") {
    const r = (it.result ?? {}) as {num_candidates?: number; error?: string};
    const q = String((it.args as {query?: string})?.query ?? "");
    return (
      <div className="msg msg-tool small">
        <Badge tone={r.error ? "err" : "info"}>SAM3</Badge> searched the robot's images for <strong>{q}</strong>: {r.error ? <span className="tone-err">{r.error}</span> : `${r.num_candidates ?? 0} candidate(s)`}
      </div>
    );
  }
  return (
    <details className="msg msg-tool">
      <summary>
        <span className="mono">{it.name}</span> <span className="muted small">tool call</span>
      </summary>
      <pre className="mono small">{JSON.stringify({args: it.args, result: it.result}, null, 2)}</pre>
    </details>
  );
}

function WriteCard({it, onDecide}: {it: CypherWrite; onDecide: (approve: boolean) => void}) {
  const st = it.status;
  const decided = st && st.state !== "error" && st.state !== "busy";
  return (
    <div className="msg msg-write">
      <div className="row gap between">
        <span>
          <Badge tone="warn">map change</Badge> <strong>{it.summary || "Database write"}</strong>
        </span>
      </div>
      <pre className="mono small cypher-query">{it.query}</pre>
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(true)}>
          Approve &amp; run
        </button>
        <button className="btn btn-sm" disabled={Boolean(decided) || st?.state === "busy"} onClick={() => onDecide(false)}>
          Reject
        </button>
        {st && (
          <span className={`small ${st.state === "done" ? "tone-ok" : st.state === "error" ? "tone-err" : "muted"}`} role="status">
            {st.text}
          </span>
        )}
      </div>
    </div>
  );
}

function ImagesCard({it}: {it: ImagesItem}) {
  const [big, setBig] = useState<string | null>(null);
  return (
    <div className="msg msg-images">
      {it.items.map(n => (
        <div key={n.sym} className="img-node">
          <div className="small">
            <strong className="mono">{n.sym}</strong> {n.label && <span>{n.label}</span>} {n.pos && <span className="muted mono">{fmt(n.pos)}</span>}
          </div>
          {n.images.length === 0 ? (
            <div className="muted small">no images of this node</div>
          ) : (
            <div className="img-strip">
              {n.images.map(u => (
                <button key={u} className="img-thumb" onClick={() => setBig(u)} aria-label={`Enlarge image of ${n.sym}`}>
                  <img src={withToken(u)} alt={`${n.sym} ${n.label}`} loading="lazy" />
                </button>
              ))}
            </div>
          )}
        </div>
      ))}
      {big && (
        <button className="img-big" onClick={() => setBig(null)} aria-label="Close image">
          <img src={withToken(big.replace(/w=\d+/, "w=1280"))} alt="enlarged" />
        </button>
      )}
    </div>
  );
}


function GoalCard({it, robot, src, onSend}: {it: Extract<ChatItem, {type: "goal"}>; robot: string | null; src: string | null; onSend: () => void}) {
  const b = it.body as {goal?: string; points?: string[]; command?: string; engine?: string; constraints?: {predicate: string; symbols: string[]}[]};
  const sent = it.sent;
  const {engines, engine: chosen, loaded} = useEngines();
  useEffect(() => {
    if (!loaded && it.kind === "pddl") void refreshEngines();
  }, [loaded, it.kind]);
  // the model's choice (engine in the proposal) wins; else the operator's
  const engine = (b.engine as EngineId | undefined) ?? chosen;
  const [pv, setPv] = useState<{busy?: boolean; err?: string; fd?: FastDownwardInfo | null; p?: PddlPreview} | null>(null);
  const shown = usePlanPreview(s => s.goal !== null && s.goal === b.goal);
  const preview = async () => {
    if (!b.goal) return;
    if (shown) return clearPreview();
    const robots = previewRobots(robot);
    if (robots.length === 0) return setPv({err: "no robot poses: connect the robots, or add agents with starts in the mission planner"});
    setPv({busy: true});
    try {
      setPv({p: await previewPddl(b.goal, robots, src, b.constraints, engine)});
    } catch (e) {
      setPv({err: errorText(e), fd: e instanceof PreviewError ? e.fd : null});
    }
  };
  return (
    <div className="msg msg-goal">
      <div className="row gap between">
        <span>
          <Badge tone="info">{it.kind}</Badge> goal proposal
          {it.kind === "pddl" && engines.length > 1 && (
            <>
              {" "}
              <Badge tone={engine === "grstaps" ? "info" : "off"} title={b.engine ? "the chat chose this planner" : "the planner chosen in the mission panel"}>
                {ENGINE_LABEL[engine]}
              </Badge>
            </>
          )}
        </span>
        <span className="row gap-sm">
          {it.kind === "pddl" && b.goal && (
            <button className="btn btn-sm" disabled={pv?.busy} onClick={() => void preview()} title="Plan it here with omniplanner and draw each robot's route on the map">
              {pv?.busy ? "planning…" : shown ? "Hide preview" : "Preview"}
            </button>
          )}
          <button className="btn btn-sm btn-primary" disabled={Boolean(sent && sent.tone !== "err")} onClick={onSend}>
            Send to {robot ?? "robot"}
          </button>
        </span>
      </div>
      {b.goal && <pre className="mono small">{b.goal}</pre>}
      {b.points && <div className="mono small">{b.points.join(" → ")}</div>}
      {b.command && <div className="small">“{b.command}”</div>}
      {b.constraints && b.constraints.length > 0 && (
        <div className="small">
          constraints:{" "}
          {b.constraints.map((c, k) => (
            <span key={k} className="chip mono">
              ({c.predicate} {c.symbols.join(" ")})
            </span>
          ))}
        </div>
      )}
      {pv?.err && (
        <div className="small tone-err">
          preview: {pv.err}
          <FastDownwardDetails fd={pv.fd} />
        </div>
      )}
      {pv?.p && shown && (
        <div className="small muted" data-testid="goal-preview">
          {Object.entries(pv.p.driven_m)
            .map(([r, m]) => `${r} ${Math.round(m)} m`)
            .join(" · ") || "no driving"}{" "}
          · {pv.p.actions.length} actions · planned in {(pv.p.ground_s + pv.p.solve_s).toFixed(1)} s
          {pv.p.schedule ? ` · makespan ${Math.round(pv.p.schedule.makespan)} s` : ""}
        </div>
      )}
      {pv?.p?.goals && shown && <GoalImages goals={pv.p.goals} src={src} />}
      {pv?.p && shown && <DirectRouteButtons routes={pv.p.routes} />}
      {pv?.p?.schedule && shown && <PlannerSchedule schedule={pv.p.schedule} compact />}
      {sent && <div className={`small tone-${sent.tone}`}>{sent.text}</div>}
    </div>
  );
}

function CandidateCard({it, idx, ord, onAct}: {it: ObjectCandidate; idx: number; ord: {k: number; n: number} | null; onAct: (add: boolean) => void}) {
  const st = it.status;
  const done = st && st.state !== "error";
  const [view, setView] = useState(0);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    if (done || it.expires_in_s == null) return;
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, [done, it.expires_in_s]);
  const left = it.expires_in_s != null ? it.expires_in_s - (now - (it.received ?? now)) / 1000 : null;
  const expired = left !== null && left <= 0 && !done;
  const size = it.bbox_min && it.bbox_max ? it.bbox_max.map((v, k) => Math.abs(v - (it.bbox_min as number[])[k])) : null;
  const views = Math.max(1, it.num_views ?? 1);
  const src =
    view === 0 && it.image_uri
      ? it.image_uri
      : withToken(`api/sam3/frame.jpg?pending_id=${encodeURIComponent(it.pending_id)}&index=${it.index}&view=${view}&max_dim=640`);
  return (
    <div className="msg msg-candidate" data-item={idx}>
      <div className="row gap between">
        <span>
          <Badge tone="warn">not in map</Badge> <strong>{it.label}</strong> <span className="muted small">score {it.score.toFixed(2)}</span>
          {ord && ord.n > 1 && (
            <span className="muted small">
              {" "}
              · {ord.k} of {ord.n}
            </span>
          )}
        </span>
        {left !== null && !done && <span className={`small mono ${expired ? "tone-err" : "muted"}`} title="The candidate is held this long; search again after">{fmtLeft(left)}</span>}
      </div>
      <img className="candidate-img" src={src} alt={`SAM3 mask of ${it.label}, view ${view + 1}`} />
      {views > 1 && (
        <div className="row gap-sm candidate-views" role="group" aria-label="Views">
          {Array.from({length: views}, (_, k) => (
            <button key={k} className={`btn btn-xs ${k === view ? "btn-primary" : ""}`} onClick={() => setView(k)} aria-pressed={k === view}>
              view {k + 1}
            </button>
          ))}
        </div>
      )}
      <div className="small mono">
        at {fmt(it.centroid)}
        {size && <span className="muted"> · size {fmt(size)} m</span>}
      </div>
      <div className="small">
        {it.would_merge_into ? (
          <span className="tone-warn">
            would merge into mapped {it.would_merge_into.symbol}
            {it.would_merge_into.distance_m != null ? ` (${it.would_merge_into.distance_m} m away)` : ""}
          </span>
        ) : (
          <span className="muted">new object</span>
        )}
      </div>
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={Boolean(done) || st?.state === "busy" || expired} onClick={() => onAct(true)}>
          {it.would_merge_into ? "Approve (merge)" : "Approve (add to map)"}
        </button>
        <button className="btn btn-sm" disabled={Boolean(done) || st?.state === "busy" || expired} onClick={() => onAct(false)}>
          Discard
        </button>
        {st && (
          <span className={`small ${st.state === "added" ? "tone-ok" : st.state === "error" ? "tone-err" : "muted"}`} role="status">
            {st.text}
          </span>
        )}
      </div>
    </div>
  );
}

interface AutoQueued {
  goal: string;
  engine?: string | null;
  constraints?: unknown;
  /** the goal card it came from (marked auto-sent) */
  card?: ChatItem;
}

function AutoSendCard({it}: {it: AutoSendItem}) {
  const [msg, setMsg] = useState<Record<string, string>>({});
  const act = async (r: {name: string; kind: string}) => {
    const spot = r.kind !== "husky";
    try {
      await postJSON(spot ? `api/robots/${encodeURIComponent(r.name)}/executor/pause` : `api/robots/${encodeURIComponent(r.name)}/teleop/stop`);
      setMsg(m => ({...m, [r.name]: spot ? "paused" : "stopped (manual)"}));
    } catch (e) {
      setMsg(m => ({...m, [r.name]: `failed: ${errorText(e)}`}));
    }
  };
  return (
    <div className={`msg msg-goal ${it.via ? "" : "msg-err"}`} data-testid="auto-send-card">
      <div className="row gap between">
        <span>
          <Badge tone={it.via ? "warn" : "err"}>auto-send</Badge> {it.via ? <>sent through <b>{it.via}</b>{it.engine === "grstaps" ? " · GRSTAPS-X" : ""}</> : "not sent"}
        </span>
      </div>
      <pre className="mono small">{it.goal}</pre>
      {it.result && <div className={`small tone-${it.result.tone}`}>{it.result.text}</div>}
      {it.blocked && <div className="small tone-err">{it.blocked}</div>}
      {it.via &&
        it.robots.map(r => (
          <div key={r.name} className="row gap-sm small">
            <span className="mono">{r.name}</span>
            {r.leaseHolder && <span className="muted">lease was held by {r.leaseHolder}{it.lease?.[r.name] ? (it.lease[r.name].ok ? " · taken" : ` · not taken: ${it.lease[r.name].message ?? ""}`) : ""}</span>}
            <button className="btn btn-xs" onClick={() => void act(r)} data-testid={`auto-send-stop-${r.name}`}>
              {r.kind === "husky" ? "Stop (manual)" : "Pause"}
            </button>
            {msg[r.name] && <span className="muted">{msg[r.name]}</span>}
          </div>
        ))}
      {it.skipped.length > 0 && (
        <div className="small muted" data-testid="auto-send-skipped">
          skipped: {it.skipped.map(s => `${s.name} (${s.reason})`).join(", ")}
        </div>
      )}
    </div>
  );
}
