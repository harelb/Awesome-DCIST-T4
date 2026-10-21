// Per robot, per fleetlink class: what arrives at the base, what the robot puts on
// the link, its budget, what waits on the robot, and what was lost. The numbers
// the comms test plan (docs/FLEETLINK_COMMS_TEST_PLAN.md) reads during a run.
import type {Cls, LinkInfo} from "../types";
import {fmtBytes, fmtRate} from "../lib/fuzzy";
import {Badge, Dot} from "../components/ui";

export const TRAFFIC_CLASSES: {cls: Cls; name: string}[] = [
  {cls: "C1", name: "state"},
  {cls: "C2", name: "graph"},
  {cls: "C3", name: "mesh"},
  {cls: "C4", name: "stills"},
  {cls: "C5", name: "video"},
  {cls: "C6", name: "images"},
];

export type TrafficLink = LinkInfo & {
  robot_rates?: Partial<Record<Cls, number>>;
  robot_queue?: Partial<Record<Cls, {n: number; bytes: number}>>;
  loss?: Partial<Record<Cls, number>>;
  lost?: Partial<Record<Cls, number>>;
  rtt_ms?: number | null;
  state_age_s?: number | null;
};

/** Received bytes/s summed over the classes, as kbit/s. */
export function totalKbps(l: TrafficLink): number {
  const vals = Object.values(l.rates ?? {}) as (number | undefined)[];
  return (8 * vals.reduce((a: number, b) => a + (b ?? 0), 0)) / 1000;
}

/** A class at >= 95 % of its budget is saturated (its data queues on the robot). */
export function saturated(l: TrafficLink, cls: Cls): boolean {
  const b = l.budgets?.[cls];
  const r = l.robot_rates?.[cls] ?? l.rates?.[cls] ?? 0;
  return Boolean(b && r >= 0.95 * b);
}

export function FleetlinkTraffic({links}: {links: Record<string, TrafficLink>}) {
  const robots = Object.keys(links).sort();
  if (!robots.length) return <div className="muted small">fleetlink has not heard from any robot.</div>;
  return (
    <div className="traffic">
      {robots.map(r => {
        const l = links[r];
        const cols = TRAFFIC_CLASSES.filter(c => c.cls !== "C6" || l.budgets?.C6 !== undefined || l.rates?.C6);
        return (
          <div key={r} className="traffic-robot">
            <div className="row gap-sm traffic-head">
              <Dot tone={l.connected ? "ok" : "err"} />
              <strong>{r}</strong>
              <span className="mono small">{totalKbps(l).toFixed(0)} kbit/s in</span>
              <span className="muted small mono">
                state {l.state_age_s != null ? `${l.state_age_s.toFixed(1)} s` : "-"} · RTT {l.rtt_ms != null ? `${l.rtt_ms.toFixed(0)} ms` : "-"} · gaps {l.gaps ?? 0} · drops {l.drops ?? 0} · resyncs{" "}
                {l.resyncs ?? 0}
              </span>
              {l.auto && <Badge tone="info">auto budgets</Badge>}
            </div>
            <div className="table-wrap">
              <table className="table small traffic-table">
                <thead>
                  <tr>
                    <th />
                    {cols.map(c => (
                      <th key={c.cls} className="mono" title={c.name}>
                        {c.cls} <span className="muted">{c.name}</span>
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <th>received</th>
                    {cols.map(c => (
                      <td key={c.cls} className="mono">
                        {fmtRate(l.rates?.[c.cls])}
                      </td>
                    ))}
                  </tr>
                  <tr>
                    <th>robot sent</th>
                    {cols.map(c => (
                      <td key={c.cls} className={`mono ${saturated(l, c.cls) ? "tone-warn" : ""}`} title={saturated(l, c.cls) ? "at its budget: data waits on the robot" : ""}>
                        {fmtRate(l.robot_rates?.[c.cls])}
                      </td>
                    ))}
                  </tr>
                  <tr>
                    <th>budget</th>
                    {cols.map(c => (
                      <td key={c.cls} className="mono muted">
                        {fmtRate(l.budgets?.[c.cls])}
                      </td>
                    ))}
                  </tr>
                  <tr>
                    <th>queued</th>
                    {cols.map(c => {
                      const q = l.robot_queue?.[c.cls];
                      return (
                        <td key={c.cls} className={`mono ${q && q.bytes > 0 ? "" : "muted"}`}>
                          {q ? `${q.n} · ${fmtBytes(q.bytes)}` : "-"}
                        </td>
                      );
                    })}
                  </tr>
                  <tr>
                    <th>lost</th>
                    {cols.map(c => {
                      const loss = l.loss?.[c.cls] ?? 0;
                      return (
                        <td key={c.cls} className={`mono ${loss > 0.02 ? "tone-err" : loss > 0 ? "tone-warn" : "muted"}`}>
                          {l.lost?.[c.cls] ?? 0}
                          {loss > 0 ? ` (${(100 * loss).toFixed(1)} %)` : ""}
                        </td>
                      );
                    })}
                  </tr>
                </tbody>
              </table>
            </div>
          </div>
        );
      })}
    </div>
  );
}
