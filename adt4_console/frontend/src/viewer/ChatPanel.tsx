import {useEffect, useRef, useState} from "react";
import {useViewer} from "../stores/viewer";
import {getToken} from "../lib/auth";
import {errorText, postJSON} from "../lib/api";
import {Badge, Spinner} from "../components/ui";
import {sendGoal} from "./PlanPanel";

export type ChatItem =
  | {type: "user"; text: string}
  | {type: "text"; text: string}
  | {type: "tool"; name: string; args: unknown; result: unknown}
  | {type: "goal"; kind: string; body: Record<string, unknown>; sent?: string}
  | {type: "error"; text: string}
  | ObjectCandidate;

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
      return {type: "goal", kind: String(o.kind), body: (o.body as Record<string, unknown>) ?? {}};
    case "error":
      return {type: "error", text: String(o.text ?? "error")};
    case "object_candidate":
      if (typeof o.pending_id !== "string") return {type: "error", text: "object candidate without pending_id"};
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
        src: typeof o.src === "string" ? o.src : null,
      };
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

const fmt = (v: number[] | null) => (v ? `(${v.map(x => x.toFixed(1)).join(", ")})` : "?");

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

export function ChatPanel() {
  const {planRobot, order, sources} = useViewer();
  const [items, setItems] = useState<ChatItem[]>([]);
  const [input, setInput] = useState("");
  const [busy, setBusy] = useState(false);
  const scroller = useRef<HTMLDivElement>(null);
  const abort = useRef<AbortController | null>(null);
  const src = order.find(id => sources[id]?.enabled) ?? undefined;

  useEffect(() => {
    scroller.current?.scrollTo({top: scroller.current.scrollHeight});
  }, [items]);
  useEffect(() => () => abort.current?.abort(), []);

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

  const send = async () => {
    const message = input.trim();
    if (!message || busy) return;
    setInput("");
    push({type: "user", text: message});
    setBusy(true);
    const ctl = new AbortController();
    abort.current = ctl;
    try {
      const headers: Record<string, string> = {"Content-Type": "application/json"};
      const t = getToken();
      if (t) headers.Authorization = `Bearer ${t}`;
      const res = await fetch("api/chat", {method: "POST", headers, body: JSON.stringify({message, robot: planRobot ?? undefined, src}), signal: ctl.signal});
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
      for (;;) {
        const {value, done} = await reader.read();
        if (done) break;
        buf += dec.decode(value, {stream: true});
        const [objs, rest] = parseNdjson(buf);
        buf = rest;
        for (const o of objs as Record<string, unknown>[]) {
          const it = toChatItem(o);
          if (it) push(it);
        }
      }
      if (buf.trim()) for (const o of parseNdjson(buf + "\n")[0] as Record<string, unknown>[]) if (o.type === "text") push({type: "text", text: String(o.text)});
    } catch (e) {
      if (!ctl.signal.aborted) push({type: "error", text: errorText(e)});
    } finally {
      setBusy(false);
    }
  };

  const setCandidate = (idx: number, status: ObjectCandidate["status"]) =>
    setItems(cur => cur.map((c, i) => (i === idx && c.type === "object_candidate" ? {...c, status} : c)));

  const actOnCandidate = async (idx: number, add: boolean) => {
    const it = items[idx];
    if (it.type !== "object_candidate") return;
    setCandidate(idx, {state: "busy", text: add ? "adding ..." : "discarding ..."});
    try {
      if (add) {
        const r = await postJSON<{symbol?: string; created?: boolean; merged?: boolean | string}>("api/sam3/commit", {
          pending_id: it.pending_id,
          index: it.index,
          src: it.src ?? undefined,
        });
        const how = r.merged ? `merged into ${r.symbol ?? "an existing object"}` : `added as ${r.symbol ?? "a new object"}`;
        setCandidate(idx, {state: "added", text: how});
      } else {
        await postJSON("api/sam3/discard", {pending_id: it.pending_id});
        // one pending id covers every candidate of that search
        setItems(cur =>
          cur.map(c =>
            c.type === "object_candidate" && c.pending_id === it.pending_id && c.status?.state !== "added"
              ? {...c, status: {state: "discarded", text: "discarded"}}
              : c,
          ),
        );
      }
    } catch (e) {
      setCandidate(idx, {state: "error", text: errorText(e)});
    }
  };

  const sendProposal = async (idx: number) => {
    const it = items[idx];
    if (it.type !== "goal") return;
    const robot = (it.body.robot as string) ?? planRobot;
    if (!robot) {
      setItems(cur => cur.map((c, i) => (i === idx ? {...it, sent: "no robot selected"} : c)));
      return;
    }
    try {
      const r = await sendGoal(robot, it.kind, it.body);
      setItems(cur => cur.map((c, i) => (i === idx ? {...it, sent: `sent to ${robot} on ${r.topic ?? "?"}`} : c)));
    } catch (e) {
      setItems(cur => cur.map((c, i) => (i === idx ? {...it, sent: `failed: ${errorText(e)}`} : c)));
    }
  };

  return (
    <div className="chat">
      <div className="chat-log" ref={scroller}>
        {items.length === 0 && (
          <div className="muted small pad">
            Ask about the map or give a task, e.g. <span className="mono">find the fire extinguisher</span>. Goals are proposed, never sent without your click.
          </div>
        )}
        {items.map((it, i) => {
          if (it.type === "user") return <div key={i} className="msg msg-user">{it.text}</div>;
          if (it.type === "text") return <div key={i} className="msg msg-bot">{it.text}</div>;
          if (it.type === "error") return <div key={i} className="msg msg-err">{it.text}</div>;
          if (it.type === "object_candidate") return <CandidateCard key={i} it={it} onAct={add => void actOnCandidate(i, add)} />;
          if (it.type === "tool")
            return (
              <details key={i} className="msg msg-tool">
                <summary>
                  <span className="mono">{it.name}</span> <span className="muted small">tool call</span>
                </summary>
                <pre className="mono small">{JSON.stringify({args: it.args, result: it.result}, null, 2)}</pre>
              </details>
            );
          return (
            <div key={i} className="msg msg-goal">
              <div className="row gap between">
                <span>
                  <Badge tone="info">{it.kind}</Badge> goal proposal
                </span>
                <button className="btn btn-sm btn-primary" disabled={Boolean(it.sent && it.sent.startsWith("sent"))} onClick={() => sendProposal(i)}>
                  Send to {(it.body.robot as string) ?? planRobot ?? "robot"}
                </button>
              </div>
              <pre className="mono small">{JSON.stringify(it.body, null, 1)}</pre>
              {it.sent && <div className={`small ${it.sent.startsWith("sent") ? "tone-ok" : "tone-err"}`}>{it.sent}</div>}
            </div>
          );
        })}
        {busy && <Spinner label="thinking" />}
      </div>
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
              void send();
            }
          }}
          aria-label="Chat message"
        />
        <button className="btn btn-primary" onClick={() => void send()} disabled={busy || !input.trim()}>
          Send
        </button>
      </div>
    </div>
  );
}

function CandidateCard({it, onAct}: {it: ObjectCandidate; onAct: (add: boolean) => void}) {
  const st = it.status;
  const done = st && st.state !== "error";
  const size = it.bbox_min && it.bbox_max ? it.bbox_max.map((v, k) => Math.abs(v - (it.bbox_min as number[])[k])) : null;
  return (
    <div className="msg msg-candidate">
      <div className="row gap between">
        <span>
          <Badge tone="warn">not in map</Badge> <strong>{it.label}</strong> <span className="muted small">score {it.score.toFixed(2)}</span>
        </span>
        <span className="muted small mono">#{it.index}</span>
      </div>
      {it.image_uri ? (
        <img className="candidate-img" src={it.image_uri} alt={`SAM3 mask of ${it.label}`} />
      ) : (
        <div className="candidate-img candidate-noimg muted small">no image</div>
      )}
      <div className="small mono">
        at {fmt(it.centroid)}
        {size && <span className="muted"> · size {fmt(size)} m</span>}
        {it.num_views ? <span className="muted"> · {it.num_views} view{it.num_views === 1 ? "" : "s"}</span> : null}
      </div>
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={Boolean(done) || st?.state === "busy"} onClick={() => onAct(true)}>
          Add to Neo4j
        </button>
        <button className="btn btn-sm" disabled={Boolean(done) || st?.state === "busy"} onClick={() => onAct(false)}>
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
