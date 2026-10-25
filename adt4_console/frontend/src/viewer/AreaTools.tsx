// Areas (API.md §15.3): one drawing tool and one list. "Draw area" collects
// corners on the map; the popover then names it and picks its purpose (search
// area / no-go zone) and scope: this mission only (the browser + the mission
// JSON) or saved to the map (a Room node, through the map-edit API). The list
// shows both scopes; the chat sees them as one list of areas. Mock objects for
// rehearsals are planted here too.
import {useEffect, useMemo, useRef, useState} from "react";
import {Badge, Segmented} from "../components/ui";
import {uiSym, type Region} from "../lib/mapEdit";
import {nextAreaName, removeMissionArea, takenAreaNames, useMission, type AreaPurpose, type AreaScope} from "../stores/mission";
import {applyOps, editInMapTab, editSource, loadAreas, useMapEdit} from "../stores/mapEdit";
import {armMissionTool, areaNameProblem, mapAreaNames, promoteArea, saveArea} from "../stores/tools";
import {useViewer} from "../stores/viewer";
import {useMedia} from "../lib/media";
import {ws} from "../ws";
import {regionColor} from "./EditOverlay";

export const AREA_BLUE = "#4ea8ff";
export const NOGO_RED = "#ff4040";

export interface AreaRow {
  key: string;
  name: string;
  purpose: AreaPurpose;
  scope: AreaScope;
  color: string;
  /** map scope: the Room */
  region?: Region;
}

/** One list over both scopes. A mission area named like a map area is the same
 *  area (e.g. saved to the map while robots were live): listed once, as the map's. */
export function areaRows(areas: {name: string}[], fences: {id: string}[], regions: Region[]): AreaRow[] {
  const mapNames = new Set(regions.map(r => (r.name ?? "").trim().toLowerCase()).filter(Boolean));
  return [
    ...regions.map((r, i): AreaRow => ({key: `map:${r.symbol}`, name: r.name || "unnamed area", purpose: "search", scope: "map", color: regionColor(i), region: r})),
    ...areas.filter(a => !mapNames.has(a.name.trim().toLowerCase())).map((a): AreaRow => ({key: `mission:${a.name}`, name: a.name, purpose: "search", scope: "mission", color: AREA_BLUE})),
    ...fences.map((f): AreaRow => ({key: `nogo:${f.id}`, name: f.id, purpose: "no_go", scope: "mission", color: NOGO_RED})),
  ];
}

/** The map's areas: loaded with the map's edit state and kept current. */
function useMapAreas() {
  const src = useViewer(v => v.order.find(id => v.sources[id]?.kind === "neo4j" && v.sources[id]?.enabled) ?? v.order.find(id => v.sources[id]?.kind === "neo4j") ?? null);
  const editSrc = useMapEdit(s => s.src);
  useEffect(() => {
    if (!src) return;
    void loadAreas();
    const s = editSrc ?? src;
    return ws.on(`edit/${s}`, () => void loadAreas());
  }, [src, editSrc]);
  return src;
}

function memberText(r: Region): string {
  return Object.entries(r.members)
    .map(([k, v]) => `${v} ${k === "MeshPlace" || k === "Place" ? "places" : k === "Object" ? "objects" : k}`)
    .join(", ");
}

export function AreasSection() {
  const {tool, draft, areas, fences} = useMission();
  const regions = useMapEdit(s => s.regions);
  const busy = useMapEdit(s => s.busy);
  const neo4j = useMapAreas();
  const rows = useMemo(() => areaRows(areas, fences, regions), [areas, fences, regions]);
  const drawing = tool.kind === "area";
  return (
    <section className="vsec" data-testid="areas">
      <header className="vsec-head">
        <h4>
          Areas <span className="muted small">{rows.length}</span>
        </h4>
        <span className="grow" />
        <button
          className={`btn btn-xs ${drawing ? "on" : ""}`}
          aria-pressed={drawing}
          onClick={() => armMissionTool(drawing ? {kind: "none"} : {kind: "area"})}
          title="Click the corners on the map; Enter or double-click finishes, Backspace drops the last corner, Esc cancels"
        >
          {drawing ? `Drawing… ${draft.length}` : "Draw area"}
        </button>
      </header>
      {rows.length === 0 ? (
        <div className="small muted">Draw a search area or a no-go zone. The chat plans with them by name.</div>
      ) : (
        <ul className="area-list small">
          {rows.map(r => (
            <li key={r.key} data-scope={r.scope}>
              <span className={`swatch ${r.purpose === "no_go" ? "swatch-nogo" : ""}`} style={{background: r.color}} />
              <b className="area-name" title={r.region ? `Room ${uiSym(r.region.symbol)}${memberText(r.region) ? ` · ${memberText(r.region)}` : ""}` : undefined}>
                {r.name}
              </b>
              <span className="muted">{r.purpose === "no_go" ? "no-go zone" : "search area"}</span>
              <Badge tone={r.scope === "map" ? "info" : "off"} title={r.scope === "map" ? "Saved in the map: every mission, the chat, Cypher and omniplanner see it" : "This mission only (kept in this browser and in the mission)"}>
                {r.scope === "map" ? "Map" : "Mission"}
              </Badge>
              <span className="grow" />
              {r.scope === "mission" && r.purpose === "search" && (
                <button className="btn btn-xs" disabled={!neo4j || busy} onClick={() => void promoteArea(r.name)} title={neo4j ? "Save to the map as a Room (places and objects inside become its members)" : "Needs a Neo4j map"}>
                  Save to map
                </button>
              )}
              {r.scope === "map" && r.region && (
                <button className="btn btn-xs" onClick={() => editInMapTab(useMapEdit.getState().src ?? editSource() ?? "", uiSym(r.region!.symbol))} title="Select it in the Map tab (rename, focus its members)">
                  Edit
                </button>
              )}
              <button
                className="btn btn-xs"
                disabled={r.scope === "map" && busy}
                aria-label={`Remove ${r.name}`}
                title={r.scope === "map" ? "Remove it from the map (undo from the toast or Map › History)" : "Remove"}
                onClick={() =>
                  r.scope === "map" && r.region
                    ? void applyOps([{op: "retire_node", symbol: r.region.symbol, reason: "area removed by the operator"}], `remove area ${r.name}`)
                    : removeMissionArea(r.name, r.purpose)
                }
              >
                Remove
              </button>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}

export function MockObjects() {
  const {tool, planted, set} = useMission();
  const [label, setLabel] = useState("crate");
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Mock objects</h4>
        <span className="grow" />
        <input className="input input-xs" style={{width: 90}} value={label} onChange={e => setLabel(e.target.value)} aria-label="Mock object label" />
        <button
          className={`btn btn-xs ${tool.kind === "plant" ? "on" : ""}`}
          onClick={() => armMissionTool(tool.kind === "plant" ? {kind: "none"} : {kind: "plant", label: label.trim() || "object"})}
          title="Place mock objects only the simulated quad camera sees (rehearsals); a search finds one when its query matches the label"
        >
          Plant mock
        </button>
      </header>
      {planted.length > 0 && (
        <ul className="mission-chips">
          {planted.map((p, i) => (
            <li key={`p-${i}`} className="chip chip-plant">
              mock {p.label}
              <button aria-label={`Remove mock ${p.label}`} onClick={() => set({planted: planted.filter((_, k) => k !== i)})}>
                ×
              </button>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}

/** After a drawing is finished: name, purpose, scope. */
export function AreaPopover() {
  const poly = useMission(s => s.pendingArea);
  const mode = useMapEdit(s => s.state?.mode ?? null);
  const live = useMapEdit(s => s.state?.live ?? false);
  const canMap = useViewer(() => !!editSource());
  // no keyboard popping up over the map on touch screens: the default name is usually fine
  const coarse = useMedia("(pointer: coarse)");
  const [name, setName] = useState("");
  const [purpose, setPurpose] = useState<AreaPurpose>("search");
  const [scope, setScope] = useState<AreaScope>("mission");
  const [err, setErr] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);
  const touched = useRef(false);
  useEffect(() => {
    if (!poly) return;
    touched.current = false;
    setPurpose("search");
    setScope("mission");
    setErr(null);
    setName(nextAreaName("search", takenAreaNames(mapAreaNames())));
  }, [poly]);
  if (!poly) return null;
  const pickPurpose = (p: AreaPurpose) => {
    setPurpose(p);
    if (p === "no_go") setScope("mission");
    if (!touched.current) setName(nextAreaName(p, takenAreaNames(mapAreaNames())));
  };
  const problem = areaNameProblem(name);
  const save = async () => {
    if (problem || saving) return setErr(problem);
    setSaving(true);
    const e = await saveArea(name, purpose, scope);
    setSaving(false);
    setErr(e);
  };
  const discard = () => useMission.getState().set({pendingArea: null});
  return (
    <div className="area-popover" role="dialog" aria-label="New area" data-testid="area-popover" onKeyDown={e => e.key === "Escape" && discard()}>
      <div className="row gap">
        <b>New area</b>
        <span className="muted small">{poly.length} corners</span>
      </div>
      <label className="area-field">
        <span>Name</span>
        <input
          className="input input-sm"
          autoFocus={!coarse}
          value={name}
          aria-label="Area name"
          onChange={e => {
            touched.current = true;
            setName(e.target.value);
            setErr(null);
          }}
          onKeyDown={e => e.key === "Enter" && void save()}
        />
      </label>
      <div className="area-field">
        <span>Purpose</span>
        <Segmented<AreaPurpose>
          ariaLabel="Purpose"
          value={purpose}
          onChange={pickPurpose}
          options={[
            {value: "search", label: "Search area"},
            {value: "no_go", label: "No-go zone"},
          ]}
        />
      </div>
      <div className="area-field">
        <span>Scope</span>
        <Segmented<AreaScope>
          ariaLabel="Scope"
          value={scope}
          onChange={v => setScope(v)}
          options={[
            {value: "mission", label: "This mission only"},
            {value: "map", label: "Save to map", disabled: purpose === "no_go" || !canMap},
          ]}
        />
      </div>
      {purpose === "no_go" && <div className="small muted">No-go zones belong to the mission: ground paths go around them, quad paths that cross one get a warning.</div>}
      {scope === "map" && (
        <div className="small tone-warn" data-testid="area-map-warning">
          {mode === "staged"
            ? `Robots are live${live ? "" : " (staged edits)"}: the area is staged until you publish it in the Map tab.`
            : "Writes the area into the map now (every mission, the chat and the planners see it). Undo from the toast or Map › History."}
        </div>
      )}
      {(err || (problem && name.trim())) && (
        <div className="small tone-err" role="alert">
          {err ?? problem}
        </div>
      )}
      <div className="row gap">
        <button className="btn btn-sm btn-primary" disabled={!!problem || saving} onClick={() => void save()}>
          {scope === "map" ? "Save to map" : "Save"}
        </button>
        <button className="btn btn-sm" onClick={discard}>
          Discard
        </button>
      </div>
    </div>
  );
}
