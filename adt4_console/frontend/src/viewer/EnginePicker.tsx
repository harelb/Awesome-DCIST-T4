// The omniplanner engine choice (Fast Downward | GRSTAPS-X), shared by the mission
// tab's omniplanner planner and the mission chat window (stores/engines).
import {useEffect, useId} from "react";
import {refreshEngines, useEngines, type EngineId} from "../stores/engines";

/** Which omniplanner engine plans: Fast Downward or GRSTAPS-X (greyed out, with
 *  the reason, when it cannot plan here, e.g. no Gurobi license). */
export function EnginePicker({compact = false, label = "planner"}: {compact?: boolean; label?: string}) {
  const {engines, engine, setEngine, loaded} = useEngines();
  const group = useId(); // each picker its own radio group (the tab's and the chat's can both be on screen)
  useEffect(() => {
    if (!loaded) void refreshEngines();
  }, [loaded]);
  if (!engines.length) return null;
  return (
    <div className="engine-pick small" data-testid="engine-picker">
      {!compact && <span className="muted">{label}</span>}
      {engines.map(e => (
        <label key={e.id} className={e.available ? "" : "muted"} title={e.available ? `${e.label}${e.id === "grstaps" ? ": task allocation with a timed schedule, joint inspections, before-ordering, moving objects" : ": shortest routes"}` : `${e.label} cannot plan here: ${e.reason}`}>
          <input type="radio" name={group} disabled={!e.available} checked={engine === e.id} onChange={() => setEngine(e.id as EngineId)} /> {e.label}
          {!e.available && " ⚠"}
        </label>
      ))}
      {engines.some(e => !e.available) && (
        <button className="btn btn-xs" title="Check again (e.g. after copying a Gurobi license in)" onClick={() => void refreshEngines()}>
          ↻
        </button>
      )}
    </div>
  );
}
