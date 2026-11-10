// Plan repair's map controls (scene change writer services): undo the last edit,
// clear every edit, or reset the map to its prior. Each asks first; a reset needs
// the operator to type "reset". The writer rebuilds the map and the console reloads
// it. Disabled, with the reason, while the writer is not running on the base.
import {useCallback, useEffect, useState} from "react";
import {errorText, getJSON, postJSON} from "../lib/api";

export type MapAction = "undo" | "clear" | "reset";

export interface SceneStatus {
  writer: string;
  running: boolean;
  reason: string | null;
}

export interface SceneActionResult {
  ok: boolean;
  action: MapAction;
  service?: string;
  message: string;
}

export const RESET_CONFIRM = "reset";

const LABEL: Record<MapAction, string> = {undo: "Undo last edit", clear: "Clear all edits", reset: "Reset map to prior"};
const ASK: Record<MapAction, string> = {
  undo: "Undo the latest map edit? The map is rebuilt without it.",
  clear: "Clear every map edit? The map is rebuilt from the prior plus detected changes.",
  reset: `Reset the map to its prior scene graph? This drops every change. Type "${RESET_CONFIRM}" to confirm.`,
};

export const fetchSceneStatus = () => getJSON<SceneStatus>("api/maps/scene");
export const runMapAction = (action: MapAction, src: string | null, confirm?: string) =>
  postJSON<SceneActionResult>(`api/maps/scene/${action}`, {src, confirm});

export function MapEditControls({src = null, pollMs = 15000}: {src?: string | null; pollMs?: number}) {
  const [status, setStatus] = useState<SceneStatus | null>(null);
  const [asking, setAsking] = useState<MapAction | null>(null);
  const [typed, setTyped] = useState("");
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<{ok: boolean; text: string} | null>(null);

  const refresh = useCallback(() => {
    fetchSceneStatus().then(setStatus, e => setStatus({writer: "", running: false, reason: errorText(e)}));
  }, []);
  useEffect(() => {
    refresh();
    if (!pollMs) return;
    const t = setInterval(refresh, pollMs);
    return () => clearInterval(t);
  }, [refresh, pollMs]);

  const disabledWhy = !status ? "checking plan repair's scene change writer…" : status.running ? null : (status.reason ?? "the scene change writer is not running");
  const confirmOk = asking !== "reset" || typed.trim().toLowerCase() === RESET_CONFIRM;

  const go = async () => {
    if (!asking || !confirmOk) return;
    setBusy(true);
    try {
      const r = await runMapAction(asking, src, asking === "reset" ? typed.trim() : undefined);
      setResult({ok: r.ok, text: r.message || (r.ok ? "done" : "failed")});
    } catch (e) {
      setResult({ok: false, text: errorText(e)});
    } finally {
      setBusy(false);
      setAsking(null);
      setTyped("");
    }
  };

  return (
    <div className="map-edit-controls" data-testid="map-edit-controls">
      <div className="row gap small">
        <span className="muted">Map edits</span>
        <span className="grow" />
        {(["undo", "clear", "reset"] as MapAction[]).map(a => (
          <button
            key={a}
            className={`btn btn-xs${a === "reset" ? " btn-danger" : ""}`}
            data-testid={`map-${a}`}
            disabled={!!disabledWhy || busy}
            title={disabledWhy ?? LABEL[a]}
            onClick={() => {
              setAsking(a);
              setTyped("");
              setResult(null);
            }}
          >
            {LABEL[a]}
          </button>
        ))}
      </div>
      {disabledWhy && status && <div className="small muted" data-testid="map-edit-why">{disabledWhy}</div>}
      {asking && (
        <div className="confirm small" data-testid="map-edit-confirm">
          <div>{ASK[asking]}</div>
          <div className="row gap">
            {asking === "reset" && (
              <input className="input input-xs" aria-label="Type reset to confirm" value={typed} onChange={e => setTyped(e.target.value)} placeholder={RESET_CONFIRM} />
            )}
            <button className="btn btn-xs btn-primary" data-testid="map-edit-go" disabled={!confirmOk || busy} onClick={() => void go()}>
              {busy ? "Working…" : LABEL[asking]}
            </button>
            <button className="btn btn-xs" disabled={busy} onClick={() => setAsking(null)}>
              Cancel
            </button>
          </div>
        </div>
      )}
      {result && (
        <div className={`small ${result.ok ? "ok-msg" : "inline-err"}`} data-testid="map-edit-result">
          {result.text}
        </div>
      )}
    </div>
  );
}
