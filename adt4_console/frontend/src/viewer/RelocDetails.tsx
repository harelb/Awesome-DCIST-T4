// Live panel: the evidence behind a robot's relocalization (fleetlink C1
// ``reloc``): its latest loop closures against the prior map and how its
// ``map -> odom`` placement moved. Nothing for agents that do not report it.
import {useRobots} from "../stores/robots";
import {isBigJump} from "../lib/frameAlign";

const fmtAge = (s: number | null | undefined) =>
  s === null || s === undefined ? "-" : s < 90 ? `${s.toFixed(0)} s` : s < 5400 ? `${Math.round(s / 60)} min` : `${(s / 3600).toFixed(1)} h`;

export function RelocDetails({robot}: {robot: string}) {
  const reloc = useRobots(s => s.state[robot]?.reloc ?? s.link[robot]?.reloc);
  const at = useRobots(s => s.stateAt[robot]);
  if (!reloc) return null;
  const since = at ? Math.max(0, (Date.now() - at) / 1000) : 0;
  const age = (a: number | null | undefined) => (a === null || a === undefined ? null : a + since);
  const jumps = reloc.recent_jumps ?? (reloc.last_jump ? [reloc.last_jump] : []);
  return (
    <details className="reloc-details small" data-testid={`reloc-${robot}`}>
      <summary>
        relocalization · {reloc.prior_lcs} prior matches · {reloc.jumps} corrections
        {reloc.total ? ` · moved ${reloc.total.dxy.toFixed(1)} m / ${reloc.total.dyaw.toFixed(0)}°` : ""}
      </summary>
      {reloc.lcs.length ? (
        <table className="reloc-table mono">
          <thead>
            <tr>
              <th>ago</th>
              <th>live ↔ prior submap</th>
              <th>objects</th>
            </tr>
          </thead>
          <tbody>
            {reloc.lcs.map((lc, i) => (
              <tr key={i} className={lc.assoc <= 5 ? "muted" : ""} title={lc.assoc <= 5 ? "few matched objects: a weak match" : undefined}>
                <td>{fmtAge(age(lc.age_s))}</td>
                <td>
                  {lc.live_submap} ↔ {lc.prior_submap}
                </td>
                <td>{lc.assoc}</td>
              </tr>
            ))}
          </tbody>
        </table>
      ) : (
        <div className="muted">no loop closure against the prior map yet</div>
      )}
      {jumps.length > 0 && (
        <table className="reloc-table mono">
          <thead>
            <tr>
              <th>ago</th>
              <th>placement moved</th>
              <th>turned</th>
            </tr>
          </thead>
          <tbody>
            {jumps.map((j, i) => (
              <tr key={i} className={isBigJump(j) ? "tone-warn" : ""}>
                <td>{fmtAge(age(j.age_s))}</td>
                <td>{j.dxy.toFixed(2)} m</td>
                <td>{j.dyaw.toFixed(1)}°</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </details>
  );
}
