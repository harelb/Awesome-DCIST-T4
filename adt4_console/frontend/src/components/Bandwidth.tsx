// Bandwidth totals UI: fleetlink link totals (Live panel) and interface totals +
// radio-vs-fleetlink comparison (Network page).
import {useEffect, useRef, useState} from "react";
import {getJSON} from "../lib/api";
import {fmtBytes, fmtRate} from "../lib/fuzzy";
import {
  COMPARE_TOOLTIP,
  compareWindow,
  type CompareSample,
  fleetTotals,
  fmtDuration,
  type InterfacesResponse,
  type Link,
  linkTotal,
  loadRadioChoices,
  radioIface,
  sampleOf,
  saveRadioChoice,
} from "../lib/netstats";
import type {LiveResponse} from "../types";
import {Badge, Card, Dot, Empty, ErrorBox, Loading} from "./ui";
import "./bandwidth.css";

const CLASS_NAMES: Record<string, string> = {C0: "control", C1: "state", C2: "graph", C3: "mesh", C4: "camera", C5: "video", C6: "image store"};

function sinceText(epochS: number | null | undefined): string {
  return epochS ? fmtDuration(Date.now() / 1000 - epochS) : "-";
}

/** Per-direction totals of one fleetlink link + per-class robot->base bytes. */
export function LinkTotalsTable({link}: {link: Link}) {
  const hasTotals = link.rx_total_B !== undefined;
  const classes = Object.entries(link.bytes ?? {}).filter(([, b]) => b > 0);
  return (
    <div className="bw-block">
      <table className="bw-table mono small">
        <thead>
          <tr>
            <th />
            <th>rate</th>
            <th title={link.connected_since ? `link session began ${sinceText(link.connected_since)} ago (connect or agent restart)` : "no link session yet"}>since connect</th>
            <th title={`since the console's fleetlink receiver started (${fmtDuration(link.since_s)} ago)`}>since console start</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <th className="bw-dir" title="robot → base: data (state, graph, mesh, camera, video, image store)">↑ data</th>
            <td>{fmtRate(link.rx_rate_Bps)}</td>
            <td>{fmtBytes(link.rx_session_B)}</td>
            <td>{fmtBytes(linkTotal(link))}</td>
          </tr>
          <tr>
            <th className="bw-dir" title="base → robot: control requests (C0)">↓ control</th>
            <td>{fmtRate(link.tx_rate_Bps)}</td>
            <td>{fmtBytes(link.tx_session_B)}</td>
            <td>{hasTotals ? fmtBytes(link.tx_total_B) : "-"}</td>
          </tr>
        </tbody>
      </table>
      {classes.length > 0 && (
        <div className="bw-classes small">
          {classes.map(([c, b]) => (
            <span key={c} className="bw-chip" title={`${CLASS_NAMES[c] ?? c}: robot → base bytes since console start`}>
              <span className="bw-cls">{c}</span> <span className="mono">{fmtBytes(b)}</span>
            </span>
          ))}
        </div>
      )}
      {!hasTotals && <div className="muted small">This receiver predates direction totals (class bytes only).</div>}
    </div>
  );
}

/** Fleet-wide sum over every robot's fleetlink link. */
export function FleetTotalsRow({links}: {links: Record<string, Link>}) {
  const t = fleetTotals(links);
  if (!t.robots) return null;
  return (
    <section className="live-robot bw-fleet">
      <header className="row gap between">
        <strong>Fleet total</strong>
        <span className="muted small">
          {t.connected}/{t.robots} connected
        </span>
      </header>
      <table className="bw-table mono small">
        <thead>
          <tr>
            <th />
            <th>rate</th>
            <th>since connect</th>
            <th>since console start</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <th className="bw-dir">↑ data</th>
            <td>{fmtRate(t.rx_rate_Bps)}</td>
            <td>{fmtBytes(t.rx_session_B)}</td>
            <td>{fmtBytes(t.rx_total_B)}</td>
          </tr>
          <tr>
            <th className="bw-dir">↓ control</th>
            <td>{fmtRate(t.tx_rate_Bps)}</td>
            <td>{fmtBytes(t.tx_session_B)}</td>
            <td>{fmtBytes(t.tx_total_B)}</td>
          </tr>
        </tbody>
      </table>
    </section>
  );
}

function usePoll<T>(path: string, ms: number): {data: T | undefined; error: unknown} {
  const [data, setData] = useState<T | undefined>(undefined);
  const [error, setError] = useState<unknown>(null);
  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const r = await getJSON<T>(path);
        if (alive) {
          setData(r);
          setError(null);
        }
      } catch (e) {
        if (alive) setError(e);
      }
    };
    void poll();
    const id = window.setInterval(poll, ms);
    return () => {
      alive = false;
      window.clearInterval(id);
    };
  }, [path, ms]);
  return {data, error};
}

/** Network page: interface totals per machine + radio-vs-fleetlink per robot. */
export function BandwidthSection() {
  const ifs = usePoll<InterfacesResponse>("api/network/interfaces", 2000);
  const live = usePoll<LiveResponse>("api/live", 2000);
  const [choices, setChoices] = useState<Record<string, string>>(() => loadRadioChoices());
  const pick = (machine: string, iface: string | null) => setChoices(saveRadioChoice(machine, iface));
  return (
    <>
      <InterfaceTotalsCard data={ifs.data} error={ifs.error} choices={choices} onPick={pick} />
      <RadioCompareCard ifs={ifs.data} links={(live.data?.robots ?? {}) as Record<string, Link>} choices={choices} onPick={pick} />
    </>
  );
}

function InterfaceTotalsCard({data, error, choices, onPick}: {data?: InterfacesResponse; error: unknown; choices: Record<string, string>; onPick: (m: string, i: string | null) => void}) {
  const machines = Object.entries(data?.machines ?? {});
  return (
    <Card title="Interface totals (tx_rx_monitor)">
      {!data && !error && <Loading />}
      {error && !data ? <ErrorBox error={error} compact /> : null}
      {data && machines.length === 0 && <Empty title="No tx_rx_monitor reports">No machine's status table has a tx_rx_monitor entry (launch_tx_rx).</Empty>}
      {machines.length > 0 && (
        <div className="table-wrap">
          <table className="table compact bw-ifaces">
            <thead>
              <tr>
                <th>machine</th>
                <th>interface</th>
                <th>address</th>
                <th className="num">TX rate</th>
                <th className="num">RX rate</th>
                <th className="num">TX total</th>
                <th className="num">RX total</th>
                <th className="num">since</th>
                <th title="the interface carrying the radio link (used for the comparison below)">radio</th>
              </tr>
            </thead>
            <tbody>
              {machines.map(([m, info]) => {
                const radio = radioIface(info, choices[m]);
                const names = Object.keys(info.interfaces).sort();
                return names.map((name, i) => {
                  const s = info.interfaces[name];
                  return (
                    <tr key={`${m}/${name}`} className={name === radio ? "bw-radio-row" : ""}>
                      {i === 0 && (
                        <td rowSpan={names.length} className="bw-machine">
                          <strong>{m}</strong>
                          {!info.has_totals && <div className="muted small">old monitor: rates only</div>}
                        </td>
                      )}
                      <td className="mono">
                        {s.error ? <Dot tone="err" title="counter unreadable" /> : null} {name}
                      </td>
                      <td className="mono small muted">{s.addr || "-"}</td>
                      <td className="num mono">{fmtRate(s.tx_rate_Bps)}</td>
                      <td className="num mono">{fmtRate(s.rx_rate_Bps)}</td>
                      <td className="num mono">{fmtBytes(s.tx_total_B)}</td>
                      <td className="num mono">{fmtBytes(s.rx_total_B)}</td>
                      <td className="num mono small muted">{fmtDuration(s.since_s)}</td>
                      <td>
                        <input
                          type="radio"
                          name={`radio-${m}`}
                          checked={name === radio}
                          onChange={() => onPick(m, name === info.radio_default ? null : name)}
                          aria-label={`${m}: ${name} is the radio interface`}
                          title={name === info.radio_default ? `default: address on ${info.radio_network}` : "use as the radio interface"}
                        />
                      </td>
                    </tr>
                  );
                });
              })}
            </tbody>
          </table>
        </div>
      )}
    </Card>
  );
}

interface Baseline {
  iface: string;
  t: number;
  base: CompareSample;
}

function RadioCompareCard({ifs, links, choices, onPick}: {ifs?: InterfacesResponse; links: Record<string, Link>; choices: Record<string, string>; onPick: (m: string, i: string | null) => void}) {
  const baselines = useRef<Record<string, Baseline>>({});
  const [, force] = useState(0);
  const robots = [...new Set([...Object.keys(links), ...Object.keys(ifs?.machines ?? {}).filter(m => links[m])])].sort();
  return (
    <Card
      title={
        <span className="row gap">
          Radio interface vs fleetlink
          <span className="bw-info" title={COMPARE_TOOLTIP} aria-label={COMPARE_TOOLTIP}>
            i
          </span>
        </span>
      }
      actions={
        <button
          className="btn btn-sm"
          onClick={() => {
            baselines.current = {};
            force(x => x + 1);
          }}
        >
          Reset window
        </button>
      }
    >
      {robots.length === 0 && <Empty title="No fleetlink robots">The comparison needs a robot heard by fleetlink and reporting tx_rx_monitor totals.</Empty>}
      <div className="bw-compare-grid">
        {robots.map(r => {
          const m = ifs?.machines?.[r];
          const l = links[r];
          const iface = radioIface(m, choices[r]);
          const s = iface ? m?.interfaces[iface] : undefined;
          const now = sampleOf(s, l);
          let b = baselines.current[r];
          if (now && (!b || b.iface !== iface)) b = baselines.current[r] = {iface: iface!, t: Date.now() / 1000, base: now};
          let cmp = now && b ? compareWindow(b.base, now) : null;
          if (now && b && !cmp) {
            b = baselines.current[r] = {iface: iface!, t: Date.now() / 1000, base: now}; // a counter restarted
            cmp = compareWindow(b.base, now);
          }
          const flRate = (l?.rx_rate_Bps ?? 0) + (l?.tx_rate_Bps ?? 0);
          const ifRate = (s?.tx_rate_Bps ?? 0) + (s?.rx_rate_Bps ?? 0);
          return (
            <section key={r} className="bw-compare" title={COMPARE_TOOLTIP}>
              <header className="row gap between">
                <span className="row gap">
                  <Dot tone={l?.connected ? "ok" : "err"} />
                  <strong>{r}</strong>
                </span>
                <select className="input input-sm" value={iface ?? ""} onChange={e => onPick(r, e.target.value || null)} aria-label={`${r} radio interface`}>
                  {!iface && <option value="">no interface</option>}
                  {Object.keys(m?.interfaces ?? {})
                    .sort()
                    .map(n => (
                      <option key={n} value={n}>
                        {n}
                        {n === m?.radio_default ? ` (${m?.radio_network ?? "default"})` : ""}
                      </option>
                    ))}
                </select>
              </header>
              {!m ? (
                <div className="muted small">No tx_rx_monitor report from {r}.</div>
              ) : !s || s.tx_total_B === null ? (
                <div className="muted small">{iface ? `${iface} has no totals (old tx_rx_monitor).` : "Pick the radio interface."}</div>
              ) : (
                <>
                  <table className="bw-table mono small">
                    <thead>
                      <tr>
                        <th />
                        <th>interface</th>
                        <th>fleetlink</th>
                        <th>difference</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr>
                        <th className="bw-dir">rate</th>
                        <td>{fmtRate(ifRate)}</td>
                        <td>{fmtRate(flRate)}</td>
                        <td>{fmtRate(ifRate - flRate)}</td>
                      </tr>
                      <tr>
                        <th className="bw-dir" title="robot TX on the radio interface vs fleetlink robot → base">↑ window</th>
                        <td>{fmtBytes(cmp?.ifUp)}</td>
                        <td>{fmtBytes(cmp?.flUp)}</td>
                        <td>{fmtBytes(cmp?.diffUp)}</td>
                      </tr>
                      <tr>
                        <th className="bw-dir" title="robot RX on the radio interface vs fleetlink base → robot">↓ window</th>
                        <td>{fmtBytes(cmp?.ifDown)}</td>
                        <td>{fmtBytes(cmp?.flDown)}</td>
                        <td>{fmtBytes(cmp?.diffDown)}</td>
                      </tr>
                      <tr className="muted">
                        <th className="bw-dir" title="cumulative: interface since the monitor started, fleetlink since the console started">total</th>
                        <td title={`since tx_rx_monitor start (${fmtDuration(s.since_s)})`}>{fmtBytes((s.tx_total_B ?? 0) + (s.rx_total_B ?? 0))}</td>
                        <td title={`since console start (${fmtDuration(l?.since_s)})`}>{fmtBytes(l ? linkTotal(l) + (l.tx_total_B ?? 0) : null)}</td>
                        <td />
                      </tr>
                    </tbody>
                  </table>
                  <div className="row gap between small">
                    <span className="muted">window {sinceText(b?.t)}</span>
                    {cmp?.ratio != null && (
                      <Badge tone={cmp.ratio > 3 ? "warn" : "info"} title="interface bytes / fleetlink bytes over the window">
                        ×{cmp.ratio.toFixed(2)} of fleetlink
                      </Badge>
                    )}
                  </div>
                </>
              )}
            </section>
          );
        })}
      </div>
    </Card>
  );
}
