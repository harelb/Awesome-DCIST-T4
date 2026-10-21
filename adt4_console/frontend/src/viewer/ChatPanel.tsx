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
import {candidateNote, cellText, fmtLeft, mergeTargetsAfterCommit, trimTranscript, writeNote, type TranscriptItem, type WriteOutcome} from "../lib/chatTranscript";

export type ChatItem =
  | {type: "user"; text: string; note?: boolean}
  | {type: "text"; text: string}
  | {type: "tool"; name: string; args: unknown; result: unknown}
  | {type: "goal"; kind: string; body: Record<string, unknown>; robot?: string | null; sent?: {tone: SendTone; text: string}}
  | {type: "error"; text: string}
  | CypherWrite
  | ImagesItem
  | ObjectCandidate;

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
    case "cypher_write":
      return {type: "cypher_write", id: String(o.id), query: String(o.query ?? ""), summary: String(o.summary ?? ""), src: typeof o.src === "string" ? o.src : null};
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
const loadModel = () => {
  try {
    return localStorage.getItem(MODEL_KEY) ?? "";
  } catch {
    return "";
  }
};

export function ChatPanel() {
  const {planRobot, order, sources} = useViewer();
  const [items, setItems] = useState<ChatItem[]>([]);
  const [input, setInput] = useState("");
  const [busy, setBusy] = useState(false);
  const [models, setModels] = useState<string[]>([]);
  const [defaultModel, setDefaultModel] = useState<string | null>(null);
  const [model, setModelRaw] = useState(loadModel);
  const transcript = useRef<TranscriptItem[]>([]);
  const scroller = useRef<HTMLDivElement>(null);
  const abort = useRef<AbortController | null>(null);
  const busyRef = useRef(false);
  // Cypher runs on the planner's Neo4j map; else the first shown source
  const src = order.find(id => sources[id]?.kind === "neo4j") ?? order.find(id => sources[id]?.enabled) ?? undefined;

  useEffect(() => {
    scroller.current?.scrollTo({top: scroller.current.scrollHeight});
  }, [items]);
  useEffect(() => () => abort.current?.abort(), []);
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

  const push = (it: ChatItem) =>
    setItems(cur => {
      const last = cur[cur.length - 1];
      // merge streamed text chunks into one bubble
      if (it.type === "text" && last?.type === "text") return [...cur.slice(0, -1), {type: "text", text: last.text + it.text}];
      if (it.type === "text") {
        const text = it.text.replace(/^\s+/, "");
        return text ? [...cur, {type: "text", text}] : cur;
      }
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
    try {
      const headers: Record<string, string> = {"Content-Type": "application/json"};
      const t = getToken();
      if (t) headers.Authorization = `Bearer ${t}`;
      const body = {message, robot: planRobot ?? undefined, src, history: trimTranscript(transcript.current), model: chosenModel};
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
    abort.current?.abort();
    transcript.current = [];
    setItems([]);
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

  const sendProposal = async (idx: number) => {
    const it = items[idx];
    if (it.type !== "goal") return;
    const robot = it.robot ?? planRobot;
    if (!robot) {
      patch(idx, c => ({...(c as typeof it), sent: {tone: "err", text: "no robot selected"}}));
      return;
    }
    try {
      const r = await sendGoal(robot, it.kind, it.body);
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
        <span className="grow" />
        <button className="btn btn-xs" onClick={newChat} disabled={!items.length} title="Start over: the assistant forgets this conversation">
          New chat
        </button>
      </div>
      <div className="chat-log" ref={scroller}>
        {items.length === 0 && (
          <div className="muted small pad">
            Ask about the map, e.g. <span className="mono">what is O7?</span>, <span className="mono">how many trees are near the robot?</span>, or give a task. The assistant reads the
            map with Cypher; map changes, new objects and goals wait for your approval.
          </div>
        )}
        {items.map((it, i) => {
          if (it.type === "user")
            return (
              <div key={i} className={`msg msg-user ${it.note ? "msg-note" : ""}`}>
                {it.note ? it.text.replace(/^\[operator\]\s*/, "") : it.text}
              </div>
            );
          if (it.type === "text") return <div key={i} className={`msg msg-bot ${looksTabular(it.text) ? "msg-pre" : ""}`}>{it.text}</div>;
          if (it.type === "error") return <div key={i} className="msg msg-err">{it.text}</div>;
          if (it.type === "object_candidate") return <CandidateCard key={i} it={it} onAct={add => void actOnCandidate(i, add)} />;
          if (it.type === "cypher_write") return <WriteCard key={i} it={it} onDecide={ok => void decideWrite(i, ok)} />;
          if (it.type === "images") return <ImagesCard key={i} it={it} />;
          if (it.type === "tool") return <ToolCard key={i} it={it} />;
          return <GoalCard key={i} it={it} robot={it.robot ?? planRobot} onSend={() => void sendProposal(i)} />;
        })}
        {busy && <Spinner label="thinking" />}
      </div>
      <LeaseRefusalNotice robot={planRobot} />
      <div className="chat-input">
        <textarea
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

function ToolCard({it}: {it: Extract<ChatItem, {type: "tool"}>}) {
  if (it.name === "run_cypher") {
    const q = String((it.args as {query?: string})?.query ?? "");
    const r = (it.result ?? {}) as {columns?: string[]; rows?: Record<string, unknown>[]; row_count?: number; truncated?: boolean; error?: string};
    const cols = r.columns ?? (r.rows?.[0] ? Object.keys(r.rows[0]) : []);
    const label = r.error ? "error" : `${r.row_count ?? 0}${r.truncated ? "+" : ""} row${r.row_count === 1 ? "" : "s"}`;
    return (
      <details className="msg msg-tool msg-cypher" open={Boolean(r.error)}>
        <summary>
          <Badge tone={r.error ? "err" : "info"}>Cypher</Badge> <span className="muted small">{label}</span> <span className="mono small cypher-peek">{q.replace(/\s+/g, " ")}</span>
        </summary>
        <pre className="mono small cypher-query">{q}</pre>
        {r.error ? (
          <div className="small tone-err mono">{r.error}</div>
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

function GoalCard({it, robot, onSend}: {it: Extract<ChatItem, {type: "goal"}>; robot: string | null; onSend: () => void}) {
  const b = it.body as {goal?: string; points?: string[]; command?: string; constraints?: {predicate: string; symbols: string[]}[]};
  const sent = it.sent;
  return (
    <div className="msg msg-goal">
      <div className="row gap between">
        <span>
          <Badge tone="info">{it.kind}</Badge> goal proposal
        </span>
        <button className="btn btn-sm btn-primary" disabled={Boolean(sent && sent.tone !== "err")} onClick={onSend}>
          Send to {robot ?? "robot"}
        </button>
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
      {sent && <div className={`small tone-${sent.tone}`}>{sent.text}</div>}
    </div>
  );
}

function CandidateCard({it, onAct}: {it: ObjectCandidate; onAct: (add: boolean) => void}) {
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
    <div className="msg msg-candidate">
      <div className="row gap between">
        <span>
          <Badge tone="warn">not in map</Badge> <strong>{it.label}</strong> <span className="muted small">score {it.score.toFixed(2)}</span>
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
