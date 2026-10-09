import {useEffect, useRef, useState} from "react";
import {useViewer} from "../stores/viewer";
import {getToken} from "../lib/auth";
import {errorText} from "../lib/api";
import {Badge, Spinner} from "../components/ui";
import {sendGoal} from "./PlanPanel";

export type ChatItem =
  | {type: "user"; text: string}
  | {type: "text"; text: string}
  | {type: "tool"; name: string; args: unknown; result: unknown}
  | {type: "goal"; kind: string; body: Record<string, unknown>; sent?: string}
  | {type: "error"; text: string};

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
        push({type: "error", text: res.status === 503 ? "Chat unavailable (ROS not connected)" : detail});
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
          if (o.type === "done") continue;
          if (o.type === "text") push({type: "text", text: String(o.text ?? "")});
          else if (o.type === "tool") push({type: "tool", name: String(o.name), args: o.args, result: o.result});
          else if (o.type === "goal") push({type: "goal", kind: String(o.kind), body: (o.body as Record<string, unknown>) ?? {}});
          else if (o.type === "error") push({type: "error", text: String(o.text ?? "error")});
        }
      }
      if (buf.trim()) for (const o of parseNdjson(buf + "\n")[0] as Record<string, unknown>[]) if (o.type === "text") push({type: "text", text: String(o.text)});
    } catch (e) {
      if (!ctl.signal.aborted) push({type: "error", text: errorText(e)});
    } finally {
      setBusy(false);
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
