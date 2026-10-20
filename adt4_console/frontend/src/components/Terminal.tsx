import {useEffect, useRef, useState} from "react";
import {withToken} from "../lib/auth";
import {Badge, Modal, type Tone} from "./ui";

/** Close codes the server sends before accepting (routes/terminal.py). */
const CLOSE_TEXT: Record<number, string> = {
  4401: "not authorised: open the console with ?token=...",
  4403: "terminals are off: start the console with --terminal",
  4404: "unknown machine or tmux session",
};

type State = "connecting" | "open" | "closed";
const STATE_TONE: Record<State, Tone> = {connecting: "info", open: "ok", closed: "off"};

export function terminalUrl(machine: string, session: string | null, cols: number, rows: number): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  const base = new URL(`api/fleet/${encodeURIComponent(machine)}/terminal`, new URL(".", window.location.href)).pathname;
  const q = new URLSearchParams({cols: String(cols), rows: String(rows)});
  if (session) q.set("session", session);
  return withToken(`${proto}//${window.location.host}${base}?${q}`);
}

/** A shell on a fleet machine (ssh from the console host), or `tmux attach` to one of its sessions. */
export function TerminalModal({machine, session, onClose}: {machine: string; session: string | null; onClose: () => void}) {
  const host = useRef<HTMLDivElement>(null);
  const [state, setState] = useState<State>("connecting");
  const [note, setNote] = useState("");
  const [attempt, setAttempt] = useState(0);

  useEffect(() => {
    let disposed = false;
    let cleanup = () => {};
    (async () => {
      // xterm is only loaded when a terminal is opened
      const [{Terminal}, {FitAddon}] = await Promise.all([import("@xterm/xterm"), import("@xterm/addon-fit")]);
      await import("@xterm/xterm/css/xterm.css");
      if (disposed || !host.current) return;
      const xterm = new Terminal({
        cursorBlink: true,
        fontFamily: getComputedStyle(document.documentElement).getPropertyValue("--mono") || "monospace",
        fontSize: 13,
        scrollback: 5000,
        theme: {background: "#0b0f14"},
      });
      const fit = new FitAddon();
      xterm.loadAddon(fit);
      xterm.open(host.current);
      fit.fit();
      xterm.focus();

      setState("connecting");
      setNote("");
      const ws = new WebSocket(terminalUrl(machine, session, xterm.cols, xterm.rows));
      ws.binaryType = "arraybuffer";
      const enc = new TextEncoder();
      ws.onopen = () => setState("open");
      ws.onmessage = e => xterm.write(typeof e.data === "string" ? e.data : new Uint8Array(e.data as ArrayBuffer));
      ws.onclose = e => {
        setState("closed");
        setNote(CLOSE_TEXT[e.code] ?? (session ? "detached" : "session ended"));
        xterm.write("\r\n\x1b[2m[connection closed]\x1b[0m\r\n");
      };
      const onData = xterm.onData(d => ws.readyState === WebSocket.OPEN && ws.send(enc.encode(d)));
      const onResize = xterm.onResize(({cols, rows}) => ws.readyState === WebSocket.OPEN && ws.send(JSON.stringify({type: "resize", cols, rows})));
      const ro = new ResizeObserver(() => fit.fit());
      ro.observe(host.current);
      cleanup = () => {
        ro.disconnect();
        onData.dispose();
        onResize.dispose();
        ws.onclose = null;
        ws.close();
        xterm.dispose();
      };
    })().catch(e => {
      setState("closed");
      setNote(String(e));
    });
    return () => {
      disposed = true;
      cleanup();
    };
  }, [machine, session, attempt]);

  return (
    <Modal
      wide
      title={
        <span className="row gap">
          {session ? (
            <>
              <span className="mono">{machine}</span> · tmux <span className="mono">{session}</span>
            </>
          ) : (
            <>
              Terminal · <span className="mono">{machine}</span>
            </>
          )}
          <Badge tone={STATE_TONE[state]}>{state}</Badge>
        </span>
      }
      footer={
        <>
          <span className="muted small grow">
            {note || (session ? "Closing this window detaches; the session keeps running (tmux prefix Ctrl-b d also detaches)." : "Closing this window ends the shell.")}
          </span>
          {state === "closed" && (
            <button className="btn" onClick={() => setAttempt(a => a + 1)}>
              Reconnect
            </button>
          )}
          <button className="btn" onClick={onClose}>
            Close
          </button>
        </>
      }
    >
      <div className="terminal-host" ref={host} />
    </Modal>
  );
}
