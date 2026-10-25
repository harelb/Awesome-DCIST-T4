// The Map tab (API.md §17): fix and annotate the Neo4j map before planning.
// Edit mode turns map clicks and search hits into the selection (Shift-click
// adds, the lasso selects inside a polygon), and the panel edits it: class /
// name in one batch (one node also its position and "blocked"), remove (retire,
// with Undo), connect / disconnect two nodes, group into an area, focus on an
// area's subtree; it adds objects and keeps snapshots. This is the one editor:
// the Inspector only links here. Areas themselves are drawn and listed in the
// Mission tab (AreaTools.tsx). While a launch is active the edits are staged and
// published with a typed confirm.
import {useEffect, useMemo, useState} from "react";
import {Badge, Segmented, Spinner, Toggle} from "../components/ui";
import {withToken} from "../lib/auth";
import {batchUpdateOps, common, edgeOp, editApi, editLayerOf, selectionSummary, uiSummary, type EditLayer} from "../lib/mapEdit";
import {
  applyOps,
  deleteSelection,
  discardStaged,
  editModel,
  finishLasso,
  lastUndoable,
  publishStaged,
  refreshEdit,
  restoreSnapshot,
  setEditMode,
  takeSnapshot,
  undoEdit,
  undoLabel,
  useMapEdit,
  type EditTool,
} from "../stores/mapEdit";
import {nextAreaName, takenAreaNames} from "../stores/mission";
import {areaNameProblem, armEditTool, mapAreaNames} from "../stores/tools";
import {useViewer} from "../stores/viewer";

const LASSO_LAYERS: {value: EditLayer; label: string}[] = [
  {value: "Object", label: "objects"},
  {value: "MeshPlace", label: "places"},
  {value: "Room", label: "areas"},
];

/** Class names of the edited map (object labelspace), for the class pickers. */
export function useClassNames(): string[] {
  const src = useMapEdit(s => s.src);
  const rev = useViewer(s => (src ? s.sources[src]?.rev : 0));
  return useMemo(() => {
    const m = editModel();
    if (!m) return [];
    const names = new Set<string>();
    for (const [k, ls] of Object.entries(m.labelspaces)) if (k.startsWith("2/")) for (const v of Object.values(ls)) names.add(v);
    return [...names].sort();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, rev]);
}

function ClassList({id}: {id: string}) {
  const names = useClassNames();
  return (
    <datalist id={id}>
      {names.map(n => (
        <option key={n} value={n} />
      ))}
    </datalist>
  );
}

function ModeBadge() {
  const st = useMapEdit(s => s.state);
  if (!st) return null;
  return st.mode === "staged" ? (
    <Badge tone="warn" title={st.live ? `Launch ${st.launch?.experiment ?? ""} is active: edits wait for Publish` : "Edits are staged (--map-edits staged)"}>
      staged{st.live ? ` · live: ${(st.launch?.robots ?? []).join(", ")}` : ""}
    </Badge>
  ) : (
    <Badge tone="ok" title="No launch is active: edits apply to the map immediately">
      immediate
    </Badge>
  );
}

function StagedBar() {
  const {state, stagedSummary, busy} = useMapEdit();
  const [typed, setTyped] = useState("");
  const n = state?.staged.length ?? 0;
  if (!n) return null;
  return (
    <div className="edit-staged" role="region" aria-label="Staged edits" data-testid="staged-bar">
      <div className="row gap">
        <b>{n} staged edit(s)</b>
        <span className="muted small">the robots do not see them until you publish</span>
      </div>
      <ul className="map-edit-diff mono small">
        {stagedSummary.map((l, i) => (
          <li key={i}>{uiSummary(l)}</li>
        ))}
      </ul>
      <div className="row gap">
        <input className="input input-xs" placeholder='type "publish"' value={typed} onChange={e => setTyped(e.target.value)} aria-label="Type publish to confirm" />
        <button
          className="btn btn-xs btn-primary"
          disabled={typed !== "publish" || busy}
          onClick={async () => {
            if (await publishStaged(typed)) setTyped("");
          }}
        >
          Publish to robots
        </button>
        <button className="btn btn-xs" onClick={() => void discardStaged()} disabled={busy}>
          Discard all
        </button>
      </div>
    </div>
  );
}

function Selection() {
  const {selection, src, busy, focusRoot} = useMapEdit();
  const set = useMapEdit(s => s.set);
  const rev = useViewer(s => (src ? s.sources[src]?.rev : 0));
  const model = editModel();
  const nodes = useMemo(() => selection.map(s => model?.nodes.get(s)).filter(Boolean) as NonNullable<ReturnType<NonNullable<typeof model>["nodes"]["get"]>>[], [selection, model, rev]);
  const [cls, setCls] = useState("");
  const [name, setName] = useState("");
  const [pos, setPos] = useState<string[]>(["", "", ""]);
  const [blocked, setBlocked] = useState(false);
  const [areaName, setAreaName] = useState("");
  const sameLayer = common(nodes, n => editLayerOf(n) ?? "?");
  const one = nodes.length === 1 ? nodes[0] : null;
  useEffect(() => {
    setCls(model && nodes.length ? (common(nodes, n => model.labelName(n)) ?? "") : "");
    setName(one ? one.name : "");
    setPos(one ? one.pos.map(v => v.toFixed(2)) : ["", "", ""]);
    setBlocked(false);
    setAreaName("");
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selection.join(","), rev]);
  if (!selection.length) return <div className="small muted">Click nodes to select them (Shift-click adds), search for one, or lasso several.</div>;
  const room = one && editLayerOf(one) === "Room" ? one : null;
  const layer = one ? editLayerOf(one) : null;
  const apply = () => {
    const fields: Record<string, unknown> = {};
    if (cls.trim() && (!model || common(nodes, n => model.labelName(n)) !== cls.trim())) fields.class = cls.trim();
    if (one && name.trim() !== one.name && (name.trim() || one.name)) fields.name = name.trim();
    if (one) {
      // the fields show 2 decimals: only an edited field moves the node
      const p = pos.map(Number);
      if (p.every(Number.isFinite) && pos.some((v, i) => v !== one.pos[i].toFixed(2))) fields.center = p;
      if (layer !== "Object" && blocked) fields.blocked = true;
    }
    void applyOps(batchUpdateOps(selection, fields), nodes.length > 1 ? `set ${Object.keys(fields).join(", ")} on ${nodes.length} nodes` : undefined);
  };
  const groupName = areaName.trim() || nextAreaName("search", takenAreaNames(mapAreaNames()));
  const groupProblem = areaNameProblem(groupName);
  return (
    <div className="edit-selection" data-testid="edit-selection">
      <div className="row gap">
        <b data-testid="selection-summary">{selectionSummary(selection, model)}</b>
        <span className="mono small muted">{selection.slice(0, 4).join(" ")}{selection.length > 4 ? " …" : ""}</span>
        <span className="grow" />
        <button className="btn btn-xs" onClick={() => set({selection: []})}>
          Clear
        </button>
      </div>
      {sameLayer && sameLayer !== "?" && (
        <div className="edit-fields">
          {layer !== "Place" && (
            <label>
              class <input className="input input-xs" list="edit-classes" placeholder="class" value={cls} onChange={e => setCls(e.target.value)} aria-label="Class" />
            </label>
          )}
          {one && (
            <label>
              name <input className="input input-xs" placeholder="name" value={name} onChange={e => setName(e.target.value)} aria-label="Name" />
            </label>
          )}
          {one && (
            <label>
              position
              <span className="row gap-sm">
                {pos.map((v, i) => (
                  <input key={i} className="input input-xs mono" style={{width: 64}} value={v} aria-label={"xyz"[i]} onChange={e => setPos(cur => cur.map((x, k) => (k === i ? e.target.value : x)))} />
                ))}
              </span>
            </label>
          )}
          {one && (layer === "MeshPlace" || layer === "Place") && (
            <label className="row gap-sm">
              <input type="checkbox" checked={blocked} onChange={e => setBlocked(e.target.checked)} /> mark blocked
            </label>
          )}
          <div>
            <button className="btn btn-xs btn-primary" disabled={busy} onClick={apply} title={nodes.length > 1 ? `Batch edit: one unit of work for all ${nodes.length}` : "Apply"}>
              {nodes.length > 1 ? `Apply to ${nodes.length}` : "Apply"}
            </button>
          </div>
        </div>
      )}
      <div className="row gap wrap">
        <button className="btn btn-xs" disabled={busy} onClick={() => void deleteSelection()} title="Retire: gone from the map for planners and robots; undoable">
          🗑 Remove
        </button>
        {selection.length === 2 && (
          <>
            <button className="btn btn-xs" disabled={busy} onClick={() => void applyOps([edgeOp("create_edge", selection[0], selection[1])])}>
              Connect
            </button>
            <button className="btn btn-xs" disabled={busy} onClick={() => void applyOps([edgeOp("delete_edge", selection[0], selection[1])])}>
              Disconnect
            </button>
          </>
        )}
        {room && (
          <button className={`btn btn-xs ${focusRoot === room.sym ? "on" : ""}`} onClick={() => set({focusRoot: focusRoot === room.sym ? null : room.sym})} title="Show only this area's places and objects (the rest dims)">
            {focusRoot === room.sym ? "Show all" : "Focus on its members"}
          </button>
        )}
      </div>
      {!room && nodes.some(n => ["Object", "MeshPlace", "Place"].includes(editLayerOf(n) ?? "")) && (
        <details className="edit-sub">
          <summary>Advanced</summary>
          <div className="row gap wrap">
            <input className="input input-xs" placeholder={groupName} value={areaName} onChange={e => setAreaName(e.target.value)} aria-label="Area name" style={{width: 120}} />
            <button
              className="btn btn-xs"
              disabled={busy || !!groupProblem}
              onClick={async () => {
                const r = await applyOps([{op: "group", parent_layer: "Room", class: "region", name: groupName, members: selection}], `area '${groupName}'`);
                if (r) setAreaName("");
              }}
              title="A map area (Room) containing the selected places and objects, like Draw area › Save to map"
            >
              Group selection into area
            </button>
          </div>
          {groupProblem && areaName.trim() && <div className="small tone-err">{groupProblem}</div>}
        </details>
      )}
    </div>
  );
}

function Tools() {
  const {tool, lasso, lassoLayers, addClass} = useMapEdit();
  const set = useMapEdit(s => s.set);
  return (
    <>
      <div className="row gap wrap">
        <Segmented<EditTool>
          ariaLabel="Edit tool"
          value={tool}
          onChange={armEditTool}
          options={[
            {value: "select", label: "Select"},
            {value: "lasso", label: "Lasso"},
            {value: "add", label: "Add object"},
          ]}
        />
        {tool === "add" && <input className="input input-xs" list="edit-classes" placeholder="class, then click the map" value={addClass} onChange={e => set({addClass: e.target.value})} aria-label="New object class" style={{width: 150}} />}
      </div>
      {tool === "lasso" && (
        <div className="edit-lasso">
          <div className="row gap wrap small">
            {LASSO_LAYERS.map(l => (
              <Toggle key={l.value} checked={lassoLayers.includes(l.value)} onChange={on => set({lassoLayers: on ? [...lassoLayers, l.value] : lassoLayers.filter(x => x !== l.value)})} label={l.label} />
            ))}
          </div>
          <div className="row gap wrap">
            <span className="small">{lasso.length} corner(s)</span>
            <button className="btn btn-xs btn-primary" disabled={lasso.length < 3} onClick={finishLasso}>
              Select inside
            </button>
            <button className="btn btn-xs" disabled={!lasso.length} onClick={() => set({lasso: lasso.slice(0, -1)})}>
              Undo corner
            </button>
          </div>
          {lasso.length < 3 && <div className="small muted disabled-why">Click at least 3 corners on the map ({3 - lasso.length} more).</div>}
        </div>
      )}
    </>
  );
}

function fmtTime(iso: string | null): string {
  if (!iso) return "";
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? iso : d.toLocaleTimeString([], {hour: "2-digit", minute: "2-digit"});
}

function History() {
  const {history, historyOpen, busy, state} = useMapEdit();
  const set = useMapEdit(s => s.set);
  const edits = history.filter(h => h.kind === "map_edit" || h.undoable);
  const last = lastUndoable(history);
  return (
    <details className="edit-sub" open={historyOpen} onToggle={e => set({historyOpen: (e.target as HTMLDetailsElement).open})}>
      <summary>
        History <span className="muted">({edits.length})</span>
        {state?.can_undo && (
          <button
            className="btn btn-xs"
            style={{marginLeft: 8}}
            disabled={busy}
            title={last ? `Undo the map's last edit: ${uiSummary(last.summary)}` : "Undo the map's last edit"}
            onClick={e => {
              e.preventDefault();
              void undoEdit(last?.mutation_id);
            }}
          >
            ↶ {last ? undoLabel(uiSummary(last.summary)) : "Undo last"}
          </button>
        )}
      </summary>
      <ul className="edit-list edit-history small" data-testid="edit-history">
        {edits.map(h => (
          <li key={h.mutation_id} className={h.undone_by ? "undone" : ""}>
            <span className="mono muted">{fmtTime(h.committed_at)}</span> <span className="muted">{String(h.provenance.operator ?? "")}{h.provenance.via ? ` · ${String(h.provenance.via)}` : ""}</span>
            <div>{h.summary ? uiSummary(h.summary) : h.mutation_id}</div>
            <span className="grow" />
            {h.undoable && !h.undo_of ? (
              <button className="btn btn-xs" disabled={busy} onClick={() => void undoEdit(h.mutation_id)} title={`Undo: ${h.summary ? uiSummary(h.summary) : h.mutation_id}`}>
                Undo
              </button>
            ) : h.undone_by ? (
              <Badge tone="off">undone</Badge>
            ) : h.undo_of ? (
              <Badge tone="info">undo</Badge>
            ) : null}
          </li>
        ))}
        {!edits.length && <li className="muted">No edits yet.</li>}
      </ul>
    </details>
  );
}

function Snapshots() {
  const {snapshots, src, busy} = useMapEdit();
  const [name, setName] = useState("");
  return (
    <details className="edit-sub">
      <summary>
        Snapshots <span className="muted">({snapshots.length})</span>
      </summary>
      <div className="row gap">
        <input className="input input-xs grow" placeholder="snapshot name" value={name} onChange={e => setName(e.target.value)} aria-label="Snapshot name" />
        <button
          className="btn btn-xs"
          disabled={busy}
          onClick={async () => {
            await takeSnapshot(name);
            setName("");
          }}
        >
          Take snapshot
        </button>
      </div>
      <ul className="edit-list small">
        {snapshots.map(s => (
          <li key={s.id}>
            <b>{s.name}</b> <span className="muted">{new Date(s.created_at * 1000).toLocaleString([], {month: "short", day: "numeric", hour: "2-digit", minute: "2-digit"})}</span>{" "}
            <span className="muted">
              {s.counts.Object ?? 0} obj · {s.counts.Room ?? 0} areas{s.mission ? ` · mission ${s.mission}` : ""}
            </span>
            <span className="grow" />
            {src && (
              <a className="btn btn-xs" href={withToken(editApi.downloadUrl(src, s.id))} download title="Download (zip: dsg.json + extras)">
                ⤓
              </a>
            )}
            <button className="btn btn-xs" onClick={() => void restoreSnapshot(s.id)} title='Replace the map with this snapshot (a job; type "restore" to confirm)'>
              Restore…
            </button>
          </li>
        ))}
      </ul>
    </details>
  );
}

export function MapEditPanel() {
  const {on, src, error, busy} = useMapEdit();
  const set = useMapEdit(s => s.set);
  const label = useViewer(s => (src ? s.sources[src]?.label : null));
  useEffect(() => {
    if (on) void refreshEdit();
  }, [on]);
  return (
    <section className="vsec map-edit" data-testid="map-edit">
      <header className="vsec-head">
        <h4>Edit the map</h4>
        {on && <ModeBadge />}
        {busy && <Spinner />}
        <span className="grow" />
        <Toggle checked={on} onChange={setEditMode} label={on ? "editing" : "edit map"} title="Map clicks and search hits select nodes to edit (Neo4j maps only)" />
      </header>
      <ClassList id="edit-classes" />
      {error && (
        <div className="msg msg-err small" role="alert">
          {error}{" "}
          <button className="btn btn-xs" onClick={() => set({error: null})}>
            ×
          </button>
        </div>
      )}
      {!on ? (
        <div className="small muted">Fix the map before planning: reclassify, move or remove objects, add missing ones, mark places blocked, group places into an area. Areas are drawn in the Mission tab.</div>
      ) : (
        <>
          {label && <div className="small muted">Editing {label}</div>}
          <StagedBar />
          <Tools />
          <Selection />
          <History />
          <Snapshots />
        </>
      )}
    </section>
  );
}

/** The undo toast over the map (the edit-mode banner is the viewer's ToolBanner). */
export function EditCanvasOverlay() {
  const toast = useMapEdit(s => s.toast);
  const set = useMapEdit(s => s.set);
  useEffect(() => {
    if (!toast) return;
    const id = window.setTimeout(() => useMapEdit.getState().set({toast: null}), 9000);
    return () => window.clearTimeout(id);
  }, [toast]);
  if (!toast) return null;
  return (
    <div className="toast edit-toast" role="status" data-testid="edit-toast">
      {toast.mutationId ? <span>Written to the map.</span> : toast.text}
      {toast.mutationId && (
        <button
          className="btn btn-xs"
          title="Undo this edit (Ctrl+Z while it is the newest change)"
          onClick={() => {
            const id = toast.mutationId;
            set({toast: null});
            void undoEdit(id);
          }}
        >
          ↶ {undoLabel(toast.text, 48)}
        </button>
      )}
    </div>
  );
}

/** While editing: the strip over the map (and the frame round it, ViewerPage)
 *  saying where an edit goes: straight into the Neo4j map, or staged until it
 *  is published to the robots. */
export function LiveEditStrip({phone = false}: {phone?: boolean}) {
  const on = useMapEdit(s => s.on);
  const st = useMapEdit(s => s.state);
  if (!on) return null;
  const staged = st?.mode === "staged";
  const n = st?.staged.length ?? 0;
  return (
    <div
      className={`live-edit-strip ${staged ? "staged" : "live"}`}
      role="status"
      data-testid="live-edit-strip"
      title={staged ? "Edits wait here until Publish to robots (Map tab)" : "Every edit is committed to the Neo4j map at once; undo it from its toast, History or Ctrl+Z"}
    >
      {staged ? (
        <span>
          <b>Staged</b> — publish to send{n ? ` (${n} waiting)` : ""}
          {!phone && st?.live ? ` · robots live: ${(st.launch?.robots ?? []).join(", ")}` : ""}
        </span>
      ) : (
        <span>
          <b>{phone ? "LIVE edit" : "Live map editing"}</b> — changes write to Neo4j
        </span>
      )}
      <button className="btn btn-xs" onClick={() => setEditMode(false)} title="Stop editing the map">
        Stop editing
      </button>
    </div>
  );
}

/** The canvas class for edit mode: "map-edit-live" (immediate) / "map-edit-staged". */
export function useEditFrameClass(): string {
  const on = useMapEdit(s => s.on);
  const staged = useMapEdit(s => s.state?.mode === "staged");
  return on ? `map-edit-on ${staged ? "map-edit-staged" : "map-edit-live"}` : "";
}
