// Change detection (relocalize_awcd): each robot's latest report of objects
// added since the prior map (green) and prior objects found removed (red),
// drawn as wireframe boxes with labels like the base station's awcd_visualizer
// in RViz. Reports are complete snapshots (the robot is authoritative), so the
// layer simply redraws the latest one.
import {useMemo} from "react";
import {Html, Line} from "@react-three/drei";
import {frameAlignOf, useRobots} from "../stores/robots";
import {useOverlays} from "../stores/overlays";
import {useViewer} from "../stores/viewer";
import {applyPoint, layerMatrix} from "../lib/frameAlign";
import {boxEdges, changeName, type ChangedObject, type ChangeKind} from "../lib/overlays";
import {AlignedGroup} from "./AlignedGroup";
import {useBackground} from "./background";
import {contrastFor, readableOn} from "./style";

const BASE: Record<ChangeKind, string> = {added: "#22c55e", removed: "#ef4444"};

/** Added / removed colours, darkened on a light 3D background. */
export function changeColors(bg?: string): Record<ChangeKind, string> {
  if (bg && contrastFor(bg).light) return {added: readableOn(BASE.added, bg, 3), removed: readableOn(BASE.removed, bg, 3)};
  return BASE;
}

const sign = (k: ChangeKind) => (k === "added" ? "+" : "−");

function ChangeBox({o, color, label}: {o: ChangedObject; color: string; label: boolean}) {
  const edges = useMemo(() => boxEdges(o.dims), [o.dims]);
  const [qx, qy, qz, qw] = o.quat;
  return (
    <group position={o.center} quaternion={[qx, qy, qz, qw]}>
      <Line points={edges} segments color={color} lineWidth={2.5} />
      {label && (
        <Html position={[0, 0, o.dims[2] / 2 + 0.35]} center zIndexRange={[6, 0]} className="viz-text-wrap" style={{pointerEvents: "none"}}>
          <div className="viz-text change-label" style={{color}}>
            {sign(o.kind)} {changeName(o)} <span className="change-cc">{o.cc.toFixed(2)}</span>
          </div>
        </Html>
      )}
    </group>
  );
}

/** One robot's change boxes (rendered inside its DimGroup by LiveOverlays). */
export function ChangeLayer({robot}: {robot: string}) {
  const ch = useRobots(s => s.changes[robot]);
  const {showAdded, showRemoved, showChangeLabels} = useOverlays();
  const colors = changeColors(useBackground());
  if (!ch) return null;
  const shown = ch.objects.filter(o => o.box && (o.kind === "added" ? showAdded : showRemoved));
  if (!shown.length) return null;
  return (
    <AlignedGroup robot={robot} frame={ch.frame}>
      {shown.map(o => (
        <ChangeBox key={`${o.kind}-${o.id}`} o={o} color={colors[o.kind]} label={showChangeLabels} />
      ))}
    </AlignedGroup>
  );
}

/** Global-frame centre of a changed object (for fly-to). */
function worldCenter(robot: string, frame: string, o: ChangedObject) {
  return applyPoint(layerMatrix(frameAlignOf(robot), frame), o.center);
}

/** Left-panel section: added / removed toggles and the list of changes per robot. */
export function ChangesPanel() {
  const changes = useRobots(s => s.changes);
  const st = useOverlays();
  const fly = useViewer(s => s.fly);
  const colors = changeColors(useBackground());
  const robots = Object.keys(changes).sort();
  if (!robots.length) return null;
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Change detection</h4>
      </header>
      <div className="row gap-sm wrap overlay-path-toggles">
        <label className="mini-check">
          <input type="checkbox" checked={st.showAdded} onChange={e => st.set({showAdded: e.target.checked})} />
          <span className="swatch line" style={{background: colors.added}} /> added
        </label>
        <label className="mini-check">
          <input type="checkbox" checked={st.showRemoved} onChange={e => st.set({showRemoved: e.target.checked})} />
          <span className="swatch line" style={{background: colors.removed}} /> removed
        </label>
        <label className="mini-check">
          <input type="checkbox" checked={st.showChangeLabels} onChange={e => st.set({showChangeLabels: e.target.checked})} />
          labels
        </label>
      </div>
      {robots.map(r => {
        const ch = changes[r];
        const unboxed = ch.objects.filter(o => !o.box).length;
        return (
          <div key={r} className="overlay-robot">
            <div className="small">
              <b>{r}</b>: <span style={{color: colors.added}}>{ch.added} added</span> · <span style={{color: colors.removed}}>{ch.removed} removed</span>
            </div>
            {ch.prior && !ch.prior.ready && (
              <div className="muted small">{ch.prior.error ? `prior map: ${ch.prior.error}` : "loading the prior map for removed objects..."}</div>
            )}
            {unboxed > 0 && ch.prior?.ready && <div className="muted small">{unboxed} removed not in the prior map (not drawn)</div>}
            {ch.objects.length > 0 && (
              <ul className="change-list">
                {ch.objects.map(o => (
                  <li key={`${o.kind}-${o.id}`}>
                    <button
                      type="button"
                      className="change-row"
                      disabled={!o.box}
                      title={o.box ? "Fly to this object" : "No box in the prior map"}
                      onClick={() => fly(worldCenter(r, ch.frame, o))}
                    >
                      <span className="change-kind" style={{color: colors[o.kind]}}>
                        {sign(o.kind)}
                      </span>
                      <span className="change-name">{changeName(o)}</span>
                      <span className="mono small muted">
                        cc {o.cc.toFixed(2)} · n {o.n}
                      </span>
                    </button>
                  </li>
                ))}
              </ul>
            )}
          </div>
        );
      })}
    </section>
  );
}
